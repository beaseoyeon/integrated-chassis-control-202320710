function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL  AFS + ESC 통합 횡방향 제어기
%
%  입력:
%    yawRateRef  [rad/s]  - 목표 요 레이트 (driver model 산출)
%    yawRate     [rad/s]  - 실측 요 레이트
%    slipAngle   [rad]    - 차체 슬립 앵글 (beta)
%    vx          [m/s]    - 종방향 속도
%    ctrlState   struct   - 이전 스텝 상태 (적분기, 미분기 등)
%    CTRL        struct   - 제어기 파라미터 (sim_params 정의)
%    LIM         struct   - 액추에이터 한계
%    dt          [s]      - 샘플링 주기
%
%  출력:
%    deltaAdd.steerAngle  [rad]  - AFS 부가 조향각
%    deltaAdd.yawMoment   [Nm]   - ESC 요 모멘트 명령
%    ctrlState            struct - 갱신된 상태
%
%  설계 기법:
%    AFS : PID + 속도 의존 gain scheduling (LPV 근사)
%    ESC : 비례+미분 slip angle 피드백 + yaw rate 피드백 혼합
%
%  Author: 학생 설계 (AI tuning aid 활용 — 보고서 명시)

%% ── 상태 초기화 ────────────────────────────────────────────────────────────
if ~isfield(ctrlState, 'intError')
    ctrlState.intError  = 0;   % AFS 적분 누적 [rad*s]
end
if ~isfield(ctrlState, 'prevErr')
    ctrlState.prevErr   = 0;   % AFS 이전 오차 (D항)
end
if ~isfield(ctrlState, 'dFilt')
    ctrlState.dFilt     = 0;   % D항 저역통과 필터 상태
end
if ~isfield(ctrlState, 'prevSlip')
    ctrlState.prevSlip  = 0;   % ESC 슬립 미분용
end
if ~isfield(ctrlState, 'escFilt')
    ctrlState.escFilt   = 0;   % ESC 저역통과 필터
end

%% ── 파라미터 로드 ──────────────────────────────────────────────────────────
Kp_base = CTRL.LAT.Kp;   % sim_params: 1.0
Ki_base = CTRL.LAT.Ki;   % sim_params: 0.1
Kd_base = CTRL.LAT.Kd;   % sim_params: 0.05
intMax  = CTRL.LAT.intMax;

%% ── (1) 속도 의존 Gain Scheduling (LPV 근사) ──────────────────────────────
% 저속(< 5 m/s)에서 적분기 동결 / 중속 기준 / 고속에서 게인 감소
% 기준 속도: 20 m/s (≈ 72 km/h), 안정 고속 운행 기준
vx_safe = max(vx, 1.0);   % 0 나눗셈 방지

v_ref   = 20.0;            % [m/s] gain scheduling 기준 속도
v_norm  = vx_safe / v_ref;

% 비례 게인: 속도가 높을수록 줄임 (횡력 응답 과민 방지)
% Kp: 저속(5m/s)=1.6x, 기준=1.0x, 고속(40m/s)=0.5x
Kp_sched = Kp_base * (1.0 / max(v_norm, 0.5));
Kp_sched = min(Kp_sched, Kp_base * 2.0);   % 상한 클램프

% 적분 게인: 고속에서 추가 감소 (적분 와인드업 위험)
Ki_sched = Ki_base * (1.0 / max(v_norm, 0.8));
Ki_sched = min(Ki_sched, Ki_base * 1.5);

% 미분 게인: 속도에 무관 (노이즈 증폭 억제 우선)
Kd_sched = Kd_base;

%% ── (2) AFS: Yaw Rate 추종 PID ────────────────────────────────────────────
err_yaw  = yawRateRef - yawRate;

% ── 적분항 + Conditional Anti-Windup ──
% 포화 시 오차와 같은 방향의 적분만 동결 (back-calculation 단순 근사)
raw_int = ctrlState.intError + err_yaw * dt;

% 예비 출력으로 포화 예측
raw_steer_test = Kp_sched * err_yaw + Ki_sched * raw_int;
if abs(raw_steer_test) > LIM.MAX_STEER_ANGLE
    % 포화 방향과 적분 방향이 같으면 적분 동결
    if sign(raw_steer_test) == sign(err_yaw)
        raw_int = ctrlState.intError;   % 동결
    end
end
ctrlState.intError = min(max(raw_int, -intMax), intMax);

% ── 미분항: 1차 저역통과 필터 (τ = 5*dt, 노이즈 억제) ──
tau_d    = 5 * dt;
alpha_d  = dt / (tau_d + dt);
d_raw    = (err_yaw - ctrlState.prevErr) / dt;
ctrlState.dFilt   = (1 - alpha_d) * ctrlState.dFilt + alpha_d * d_raw;
ctrlState.prevErr = err_yaw;

% ── PID 합산 ──
raw_steer = Kp_sched * err_yaw ...
          + Ki_sched * ctrlState.intError ...
          + Kd_sched * ctrlState.dFilt;

deltaAdd.steerAngle = min(max(raw_steer, -LIM.MAX_STEER_ANGLE), LIM.MAX_STEER_ANGLE);

%% ── (3) ESC: Slip Angle 제한 (PD + yaw rate 피드백) ──────────────────────
% 임계값: 3° (ASSIGNMENT 명시), 고속에서 더 엄격
beta_th_base = 0.0524;   % 3° in rad
% 고속일수록 임계값 축소 (안전 마진 확보)
% 30 m/s에서 약 2.5°까지 줄임
beta_th = beta_th_base * max(0.8, 1.0 - 0.007 * (vx_safe - v_ref));
beta_th = max(beta_th, 0.0349);   % 하한: 2°

% 슬립 초과량
beta_exc = abs(slipAngle) - beta_th;

if beta_exc > 0
    % P항: 초과 슬립에 비례
    % D항: 슬립 변화율 (미분 필터 적용)
    tau_esc  = 8 * dt;
    alpha_esc = dt / (tau_esc + dt);
    d_slip_raw = (abs(slipAngle) - abs(ctrlState.prevSlip)) / dt;
    ctrlState.escFilt = (1 - alpha_esc) * ctrlState.escFilt + alpha_esc * d_slip_raw;

    % ESC 게인: 속도 비례 (고속 = 더 강한 모멘트 필요)
    % vx가 높을수록 lateral force 포화 → 모멘트로 교정
    K_P_esc = 55000 * (1.0 + 0.5 * max(v_norm - 1.0, 0));
    K_D_esc = 8000;

    Mz_esc = -(K_P_esc * beta_exc + K_D_esc * ctrlState.escFilt) ...
              * sign(slipAngle);

    % 요 레이트 오차가 동일 방향이면 ESC 모멘트 증폭 (협력 제어)
    % 반대 방향이면 감쇠 (AFS와 충돌 방지)
    yaw_err_sign = sign(yawRateRef - yawRate);
    esc_sign     = sign(Mz_esc);
    if yaw_err_sign == esc_sign
        blend = 1.1;   % 협력: 10% 증폭
    else
        blend = 0.7;   % 경합: 30% 감쇠
    end
    Mz_esc = Mz_esc * blend;

    % 최대 ESC 모멘트 클램프 (차체 전복 방지)
    Mz_max = 8000;   % [Nm]
    deltaAdd.yawMoment = min(max(Mz_esc, -Mz_max), Mz_max);
else
    ctrlState.escFilt  = ctrlState.escFilt * 0.95;   % 서서히 감소
    deltaAdd.yawMoment = 0;
end

ctrlState.prevSlip = slipAngle;

end
