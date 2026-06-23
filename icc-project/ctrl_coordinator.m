function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
    % 휠 반경 및 트랙 폭 설정 (BMW 5 계열 근사치)
    r_w = 0.33; 
    t_f = VEH.track_f; 
    t_r = VEH.track_r;
    ratio_f = 0.6; % 종방향 제동 전륜 배분율 (60:40)
    
    brake_trq = zeros(4, 1); % [FL; FR; RL; RR]
    
    %% (1) 종방향 명령 (Longitudinal Allocation)
    if lonCmd.Fx_total < 0
        % 제동 상황: 요구되는 총 제동력(음수)을 4륜에 분배
        F_brake = abs(lonCmd.Fx_total);
        brake_trq(1) = (F_brake * ratio_f / 2) * r_w;     % FL
        brake_trq(2) = (F_brake * ratio_f / 2) * r_w;     % FR
        brake_trq(3) = (F_brake * (1 - ratio_f) / 2) * r_w; % RL
        brake_trq(4) = (F_brake * (1 - ratio_f) / 2) * r_w; % RR
    end
    
    %% (2) 횡방향 명령 (Lateral Allocation - ESC)
    if latCmd.yawMoment ~= 0
        Mz = latCmd.yawMoment;
        % 요구 Mz를 전/후륜 비율에 맞춰 제동력 차이로 변환
        dF_f = abs(Mz) * ratio_f / t_f;
        dF_r = abs(Mz) * (1 - ratio_f) / t_r;
        
        if Mz > 0
            % CCW(반시계) 회전 모멘트 필요 -> 좌측(Left) 브레이크 개입
            brake_trq(1) = brake_trq(1) + dF_f * r_w; % FL
            brake_trq(3) = brake_trq(3) + dF_r * r_w; % RL
        else
            % CW(시계) 회전 모멘트 필요 -> 우측(Right) 브레이크 개입
            brake_trq(2) = brake_trq(2) + dF_f * r_w; % FR
            brake_trq(4) = brake_trq(4) + dF_r * r_w; % RR
        end
    end
    
    %% (3) Saturation 및 최종 출력
    % 최대 브레이크 토크 제한
    actuatorCmd.brakeTorque = min(max(brake_trq, 0), LIM.MAX_BRAKE_TRQ);
    
    % 조향각 그대로 전달 및 제한
    actuatorCmd.steerAngle = min(max(latCmd.steerAngle, -LIM.MAX_STEER_ANGLE), LIM.MAX_STEER_ANGLE);
    
    % 수직 댐핑 계수 전달
    actuatorCmd.dampingCoeff = verCmd;
end