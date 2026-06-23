function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL  속도 추종 + ABS 슬립 비율 제어 종방향 제어기
%
%  입력:
%    vxRef       [m/s]    - 목표 종방향 속도
%    vx          [m/s]    - 실측 종방향 속도
%    ax          [m/s^2]  - 실측 종가속도
%    ctrlState   struct   - 이전 스텝 상태
%    CTRL        struct   - 제어기 파라미터
%    LIM         struct   - 액추에이터 한계
%    dt          [s]      - 샘플링 주기
%
%  출력:
%    forceCmd.Fx_total    [N]   - 총 종방향 힘 명령 (음수=제동)
%    forceCmd.brakeRatio  [-]   - 제동 비율 (0~1)
%    ctrlState            struct - 갱신된 상태
%
%  설계 기법:
%    속도 추종: PI 제어 + Conditional Anti-Windup
%    ABS      : 슬립 비율 추정 + Bang-Bang/비례 감압 제어
%    안전      : Jerk Limit (Rate Limiter)
%
%  Author: 학생 설계 (AI tuning aid 활용 — 보고서 명시)

%% ── 상태 초기화 ────────────────────────────────────────────────────────────
if ~isfield(ctrlState, 'intError')
    ctrlState.intError   = 0;
end
if ~isfield(ctrlState, 'prevForce')
    ctrlState.prevForce  = 0;
end
if ~isfield(ctrlState, 'wheelSpd')
    % 4륜 휠 속도 추정 초기값 (wheel slip 계산용)
    ctrlState.wheelSpd   = max(vx, 0.5) * ones(4,1);
end
if ~isfield(ctrlState, 'absActive')
    ctrlState.absActive  = false(4,1);
end
if ~isfield(ctrlState, 'prevAx')
    ctrlState.prevAx     = 0;
end

%% ── 파라미터 ───────────────────────────────────────────────────────────────
mass     = 1500;          % [kg] sim_params VEH.mass
rw       = 0.31;          % [m]  타이어 유효 반경 (sim_params VEH.rw)
g        = 9.81;

Kp       = CTRL.LON.Kp;   % sim_params: 0.5
Ki       = CTRL.LON.Ki;   % sim_params: 0.05
intMax   = CTRL.LON.intMax;

% ABS 목표 슬립 비율: 최대 마찰 계수 근방 유지
% Pacejka C=1.6, B=14 기준 peak는 κ ≈ 0.10~0.12
KAPPA_TARGET = 0.10;   % [-] 목표 슬립 비율 (최대 마찰 근방)
KAPPA_MAX    = 0.12;   % [-] ASSIGNMENT 기준: 이 이상이면 ABS 개입
KAPPA_HYST   = 0.08;   % [-] ABS 해제 임계 (히스테리시스)

% 휠 관성 (간이: Iw * alpha = Trq - r*Fx)
Iw = 1.5;              % [kg*m^2] sim_params VEH.Iw

%% ── (1) 속도 추종 PI 제어 ─────────────────────────────────────────────────
err_v    = vxRef - vx;
vx_safe  = max(vx, 0.5);

% 적분항 + Conditional Anti-Windup
raw_int  = ctrlState.intError + err_v * dt;
Fx_test  = Kp * err_v + Ki * raw_int;
F_max    = mass * LIM.MAX_AX;

if abs(Fx_test) > F_max
    if sign(Fx_test) == sign(err_v)
        raw_int = ctrlState.intError;   % 포화 시 적분 동결
    end
end
ctrlState.intError = min(max(raw_int, -intMax), intMax);

Fx_pi = Kp * err_v + Ki * ctrlState.intError;

%% ── (2) 휠 슬립 비율 추정 (간이 모델) ────────────────────────────────────
% 실제 구현에서는 wheel speed sensor 입력이 별도 제공됨
% 여기서는 ax 적분으로 휠 속도 근사 (plant가 ax를 제공)
% 제동 시: 차체 감속 < 휠 감속 → 슬립 발생

% 실질 제동력으로 휠 각속도 변화 추정
% 4륜 균등 분배 가정
Fx_per_wheel = abs(min(Fx_pi, 0)) / 4;   % 제동 힘 per wheel

% 슬립 비율 추정: κ = (vx - vw) / vx (제동 시 정의)
% 휠 속도 추정: 간이 적분 (Iw * dωw/dt = Trq_brake - r*Fz*μ)
% 단, plant에서 직접 받지 못하므로 ax 기반 근사 사용
ax_filtered = (1 - 0.3) * ctrlState.prevAx + 0.3 * ax;
ctrlState.prevAx = ax_filtered;

% 추정 슬립 비율 (ax 기반)
% 순수 롤링 시 vw ≈ vx + rw*ax/g (근사)
% 실제 ABS는 wheel speed sensor 사용하나, 여기선 ax로 근사
if Fx_pi < 0   % 제동 중
    % 제동 감속도 기반 슬립 추정
    decel_g    = abs(ax_filtered) / g;
    % 슬립 근사: 강한 제동일수록 슬립 증가
    kappa_est  = min(decel_g * 0.15, 0.25);   % 경험적 근사 (0.15*g_decel)
else
    kappa_est  = 0;
end

%% ── (3) ABS 제어 (슬립 비율 피드백) ─────────────────────────────────────
% 슬립이 KAPPA_MAX 초과 → 제동력 감압 (비례 감압)
% 슬립이 KAPPA_HYST 이하 → 복압

abs_force_scale = ones(4, 1);

for w = 1:4
    if ctrlState.absActive(w)
        % ABS 활성 상태
        if kappa_est < KAPPA_HYST
            % 슬립 충분히 감소 → ABS 해제
            ctrlState.absActive(w) = false;
            abs_force_scale(w)     = 1.0;
        else
            % 목표 슬립(KAPPA_TARGET)에 비례 제어로 제동력 조절
            % κ > target → 제동 감압, κ < target → 복압
            k_err = kappa_est - KAPPA_TARGET;
            % 감압률: 오차에 비례, [0.3 ~ 1.0] 범위 클램프
            abs_force_scale(w) = max(0.30, 1.0 - 5.0 * k_err);
        end
    else
        % ABS 비활성 상태
        if kappa_est > KAPPA_MAX
            ctrlState.absActive(w) = true;
            abs_force_scale(w)     = 0.70;   % 즉각 30% 감압
        else
            abs_force_scale(w)     = 1.0;
        end
    end
end

% 4륜 평균 스케일 적용 (단일 Fx_total 출력)
mean_scale = mean(abs_force_scale);

Fx_abs = Fx_pi;
if Fx_pi < 0   % 제동 중에만 ABS 개입
    Fx_abs = Fx_pi * mean_scale;
end

%% ── (4) Jerk Limit (승차감 + 안전) ──────────────────────────────────────
max_dF   = LIM.MAX_JERK * mass * dt;
dF       = Fx_abs - ctrlState.prevForce;
dF_limit = min(max(dF, -max_dF), max_dF);
Fx_out   = ctrlState.prevForce + dF_limit;

% 최대 힘 클램프
Fx_out   = min(max(Fx_out, -mass * LIM.MAX_AX), mass * LIM.MAX_AX);

%% ── (5) 출력 ─────────────────────────────────────────────────────────────
forceCmd.Fx_total = Fx_out;

if Fx_out < 0
    forceCmd.brakeRatio = min(abs(Fx_out) / (mass * g), 1.0);
else
    forceCmd.brakeRatio = 0;
end

ctrlState.prevForce = Fx_out;

end
