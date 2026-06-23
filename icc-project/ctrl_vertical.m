function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
    % 파라미터 로드
    cMin = CTRL.VER.cMin; % 최소 댐핑 (소프트)
    cMax = CTRL.VER.cMax; % 최대 댐핑 (하드)
    
    dampingCmd = zeros(4, 1);
    
    %% 4륜 개별 Skyhook 제어 (On-Off)
    for i = 1:4
        zs_dot = suspState.zs_dot(i); % 차체 수직 속도
        zu_dot = suspState.zu_dot(i); % 휠 수직 속도
        
        v_rel = zs_dot - zu_dot; % 서스펜션 상대 속도
        
        % Skyhook 조건: 차체 속도와 상대 속도의 부호가 같으면(곱이 양수) 하드 댐핑
        if (zs_dot * v_rel) > 0
            dampingCmd(i) = cMax;
        else
            dampingCmd(i) = cMin;
        end
    end
end