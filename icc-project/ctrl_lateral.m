function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
    %% 상태 초기화
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    
    %% (1) 속도 스케줄링 (Gain Scheduling)
    % 고속일수록 조향 민감도가 커지므로 게인을 줄여줍니다.
    v_ref = 15; % 기준 속도 [m/s]
    v_factor = max(min(vx / v_ref, 2), 0.5); 
    
    %% (2) AFS: Yaw Rate 추종 (PI Controller)
    err_yaw = yawRateRef - yawRate;
    
    % 적분항 계산 및 Anti-windup
    ctrlState.intError = ctrlState.intError + err_yaw * dt;
    ctrlState.intError = min(max(ctrlState.intError, -CTRL.LAT.intMax), CTRL.LAT.intMax);
    
    % 스케줄링된 게인 적용
    Kp_afs = CTRL.LAT.Kp / v_factor;
    Ki_afs = CTRL.LAT.Ki / v_factor;
    
    raw_steer = (Kp_afs * err_yaw) + (Ki_afs * ctrlState.intError);
    deltaAdd.steerAngle = min(max(raw_steer, -LIM.MAX_STEER_ANGLE), LIM.MAX_STEER_ANGLE);
    
    %% (3) ESC: Slip Angle 제한 (비례 제어)
    beta_th = 0.052; % 약 3도 (임계 슬립 앵글)
    K_beta = 60000;  % Yaw Moment 게인
    
    if abs(slipAngle) > beta_th
        % 슬립이 임계치를 넘으면 반대 방향으로 모멘트 생성
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * v_factor;
    else
        deltaAdd.yawMoment = 0;
    end
end