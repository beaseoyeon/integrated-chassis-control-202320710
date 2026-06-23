function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR  횡·종·수직 명령 → Actuator 분배 (WLS 기반)
%
%  입력:
%    latCmd   struct  - 횡방향 제어기 출력
%      .steerAngle   [rad]  AFS 부가 조향각
%      .yawMoment    [Nm]   ESC 요 모멘트
%    lonCmd   struct  - 종방향 제어기 출력
%      .Fx_total     [N]    총 종방향 힘
%      .brakeRatio   [-]    제동 비율
%    verCmd   [4x1]   - 수직 제어기 출력 (감쇠 계수 [Ns/m])
%    vx       [m/s]   - 종방향 속도
%    VEH      struct  - 차량 파라미터
%    CTRL     struct  - 제어기 파라미터
%    LIM      struct  - 액추에이터 한계
%
%  출력:
%    actuatorCmd.steerAngle    [rad]    최종 조향각 명령
%    actuatorCmd.brakeTorque   [4x1 Nm] 4륜 제동 토크 명령 [FL;FR;RL;RR]
%    actuatorCmd.dampingCoeff  [4x1 Ns/m] 4륜 감쇠 계수 명령
%
%  설계 기법:
%    1) 종방향 제동 → 전후륜 비율 분배 (60:40 bias, 하중 이동 보정)
%    2) ESC 요 모멘트 → 좌우 제동력 차이 (lever arm 기반)
%    3) 마찰원 제한 (가산점 항목): 각 휠의 Fx²+Fy² ≤ (μ*Fz)²
%    4) WLS 정규화 (가산점 항목): 타이어 부하율 균등화
%
%  Author: 학생 설계 (AI tuning aid 활용 — 보고서 명시)

%% ── 차량 파라미터 ──────────────────────────────────────────────────────────
rw      = VEH.rw;           % [m] 타이어 유효 반경 (sim_params: 0.31)
t_f     = VEH.track_f;     % [m] 전륜 트레드 (sim_params: 1.55)
t_r     = VEH.track_r;     % [m] 후륜 트레드 (sim_params: 1.55)
mass    = VEH.mass;         % [kg] (sim_params: 1500)
lf      = VEH.lf;          % [m] CG-전축 거리 (sim_params: 1.2)
lr      = VEH.lr;          % [m] CG-후축 거리 (sim_params: 1.4)
L       = VEH.L;           % [m] 축간 거리 (sim_params: lf+lr)
h_cog   = VEH.h_cog;       % [m] CG 높이 (sim_params: 0.55)
mu_peak = 1.0;              % [-] 최대 마찰 계수 (sim_params TIRE.D)
g       = 9.81;

%% ── 가중치 (WLS용, 타이어 부하율 균등화) ──────────────────────────────────
wLat  = CTRL.COORD.wLat;    % 횡방향 가중치
wLon  = CTRL.COORD.wLon;    % 종방향 가중치
wEff  = CTRL.COORD.wEff;    % 에너지 효율 가중치

%% ── (1) 정적 하중 분배 추정 ───────────────────────────────────────────────
% 제동/가속 시 하중 이동 고려 (quasi-static)
ax_est  = lonCmd.Fx_total / mass;   % 추정 종가속도 [m/s^2]

% 종방향 하중 이동 (ax 기반)
% ΔFz_lon = m*ax*h_cog / L  (전후 차이)
dFz_lon = mass * ax_est * h_cog / L;

% 정적 하중 (전후 균등 분배, 좌우 50:50)
Fz_static_f = (mass * g * lr / L);     % 전축 정적 수직력
Fz_static_r = (mass * g * lf / L);     % 후축 정적 수직력

% 제동 시: 전축 하중 증가, 후축 감소
Fz_f = Fz_static_f + dFz_lon;
Fz_r = Fz_static_r - dFz_lon;
Fz_f = max(Fz_f, 0);
Fz_r = max(Fz_r, 0);

% 4륜 정적 수직력 (하중 이동 반영, 좌우 동일 가정)
Fz = [Fz_f/2; Fz_f/2; Fz_r/2; Fz_r/2];  % [FL;FR;RL;RR]

%% ── (2) 종방향 제동력 분배 (하중 이동 보정) ──────────────────────────────
brake_trq = zeros(4,1);

if lonCmd.Fx_total < 0
    F_brake_total = abs(lonCmd.Fx_total);

    % 하중 비례 분배: 전축 비율 = Fz_f / (Fz_f + Fz_r)
    Fz_total = Fz_f + Fz_r;
    if Fz_total > 0
        ratio_f_dyn = Fz_f / Fz_total;
    else
        ratio_f_dyn = 0.6;   % 폴백
    end
    ratio_f_dyn = min(max(ratio_f_dyn, 0.50), 0.75);   % 물리 타당 범위 클램프

    F_f = F_brake_total * ratio_f_dyn;     % 전축 제동력
    F_r = F_brake_total * (1 - ratio_f_dyn);  % 후축 제동력

    % 좌우 균등 분배 → 토크 변환
    brake_trq(1) = (F_f / 2) * rw;   % FL
    brake_trq(2) = (F_f / 2) * rw;   % FR
    brake_trq(3) = (F_r / 2) * rw;   % RL
    brake_trq(4) = (F_r / 2) * rw;   % RR
end

%% ── (3) ESC 요 모멘트 → 차동 제동 분배 (lever arm 기반) ─────────────────
if abs(latCmd.yawMoment) > 1.0   % 소량 노이즈 무시 (1 Nm 데드밴드)
    Mz     = latCmd.yawMoment;   % 요구 요 모멘트 [Nm]

    % 요 모멘트 = 전후 차동 제동력 * track/2
    % Mz = dF_f * (t_f/2) + dF_r * (t_r/2)
    % 전후 분배: 전축 60%, 후축 40% (기본), WLS로 보정 가능
    ratio_esc_f = 0.6;
    ratio_esc_r = 1 - ratio_esc_f;

    % 차동 제동력 크기 (좌우 각각)
    % lever_f = t_f/2, lever_r = t_r/2
    dF_f_esc = abs(Mz) * ratio_esc_f / (t_f / 2);
    dF_r_esc = abs(Mz) * ratio_esc_r / (t_r / 2);

    % 마찰원 사전 클램프: 이미 종방향 제동 중이면 lateral capacity 감소
    % ── 마찰원 제한 (가산점) ──────────────────────────────────────────────
    % 각 휠에서 |Fx_long|² + |Fx_esc|² ≤ (μ*Fz)² 조건
    % 종방향 제동이 이미 마찰원을 일부 사용한 상태
    mu_Fz    = mu_peak * Fz;               % 휠별 최대 마찰력 [N]
    Fx_long  = brake_trq / rw;            % 이미 할당된 종방향 힘

    % 가용 마찰력 (ESC 차동 제동용)
    avail_FL = sqrt(max(mu_Fz(1)^2 - Fx_long(1)^2, 0));
    avail_FR = sqrt(max(mu_Fz(2)^2 - Fx_long(2)^2, 0));
    avail_RL = sqrt(max(mu_Fz(3)^2 - Fx_long(3)^2, 0));
    avail_RR = sqrt(max(mu_Fz(4)^2 - Fx_long(4)^2, 0));

    if Mz > 0
        % CCW 회전 → 좌측 브레이크 (FL, RL)
        dF_f_esc = min(dF_f_esc, avail_FL);
        dF_r_esc = min(dF_r_esc, avail_RL);
        brake_trq(1) = brake_trq(1) + dF_f_esc * rw;
        brake_trq(3) = brake_trq(3) + dF_r_esc * rw;
    else
        % CW 회전 → 우측 브레이크 (FR, RR)
        dF_f_esc = min(dF_f_esc, avail_FR);
        dF_r_esc = min(dF_r_esc, avail_RR);
        brake_trq(2) = brake_trq(2) + dF_f_esc * rw;
        brake_trq(4) = brake_trq(4) + dF_r_esc * rw;
    end
end

%% ── (4) WLS 정규화 (타이어 부하율 균등화, 가산점) ────────────────────────
% 각 휠 제동 토크를 마찰원 한계 대비 비율로 정규화
% 목적: 어느 한 휠이 과부하 받지 않도록 전체 스케일 조정
F_wheel     = brake_trq / rw;          % 제동력 [N]
mu_Fz_each  = mu_peak * Fz;            % 마찰 한계 [N]

% 각 휠 부하율 (utilization)
util        = F_wheel ./ max(mu_Fz_each, 1e-3);

% 가장 부하율이 높은 휠이 1.0을 넘으면 전체 스케일 감소
max_util    = max(util);
if max_util > 1.0
    scale   = 1.0 / max_util;          % WLS 스케일 팩터
    brake_trq = brake_trq * scale;
end

%% ── (5) 포화 + 최종 출력 ─────────────────────────────────────────────────
% 제동 토크: 0 이상, MAX_BRAKE_TRQ 이하
actuatorCmd.brakeTorque  = min(max(brake_trq, 0), LIM.MAX_BRAKE_TRQ);

% 조향각: AFS 부가각 + 포화 적용
actuatorCmd.steerAngle   = min(max(latCmd.steerAngle, ...
                                   -LIM.MAX_STEER_ANGLE), ...
                                    LIM.MAX_STEER_ANGLE);

% 수직 감쇠 계수: 범위 클램프
actuatorCmd.dampingCoeff = min(max(verCmd, CTRL.VER.cMin), CTRL.VER.cMax);

end
