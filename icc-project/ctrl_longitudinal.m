function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%% 상태 초기화
if ~isfield(ctrlState, 'intError')
    ctrlState.intError = 0;
end
if ~isfield(ctrlState, 'prevForce')
    ctrlState.prevForce = 0;
end

%% (1) 속도 추종 PI 제어
err_v = vxRef - vx;

% 적분항 및 Anti-windup
ctrlState.intError = ctrlState.intError + err_v * dt;
ctrlState.intError = min(max(ctrlState.intError, -CTRL.LON.intMax), CTRL.LON.intMax);

Fx_raw = (CTRL.LON.Kp * err_v) + (CTRL.LON.Ki * ctrlState.intError);

%% (2) Jerk Limit (승차감 보호)
mass = 1800; % BMW_5 예상 중량 [kg]
max_dF = LIM.MAX_JERK * mass * dt; % 1 step당 최대 힘 변화량

% 이전 스텝의 힘에서 변화량을 제한 (Rate Limiter)
Fx_limited = ctrlState.prevForce + min(max(Fx_raw - ctrlState.prevForce, -max_dF), max_dF);

%% (3) 출력 할당
forceCmd.Fx_total = Fx_limited;

% 제동 비율 (단순화: 힘이 음수면 브레이크 개입으로 간주)
if Fx_limited < 0
    forceCmd.brakeRatio = min(abs(Fx_limited) / (mass * 9.81), 1);
else
    forceCmd.brakeRatio = 0;
end

ctrlState.prevForce = Fx_limited;
end