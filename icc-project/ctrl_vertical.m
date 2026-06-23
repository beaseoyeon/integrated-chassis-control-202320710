function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL  CDC (Continuous Damping Control) — Hybrid Skyhook/Groundhook
%
%  입력:
%    suspState   struct   - 서스펜션 상태
%      .zs_dot(4x1) [m/s]   차체 수직 속도 (sprung mass velocity, per wheel)
%      .zu_dot(4x1) [m/s]   휠 수직 속도  (unsprung mass velocity, per wheel)
%      .zdef(4x1)   [m]     서스펜션 변위 (optional, stroke)
%    ctrlState   struct   - 이전 스텝 상태
%    CTRL        struct   - 제어기 파라미터 (sim_params 정의)
%    dt          [s]      - 샘플링 주기
%
%  출력:
%    dampingCmd  [4x1 Ns/m] - 4륜 개별 감쇠 계수 명령
%    ctrlState   struct     - 갱신된 상태
%
%  설계 기법:
%    Hybrid Skyhook–Groundhook (alpha 혼합)
%      c_sky  : body bounce (1~2 Hz) 억제 — 승차감 목적
%      c_gnd  : wheel hop  (10~15 Hz) 억제 — 접지력 목적
%    주파수 분리: 저역통과(LPF) + 고역통과(HPF) 필터로 zs_dot 분해
%
%  Author: 학생 설계 (AI tuning aid 활용 — 보고서 명시)

%% ── 상태 초기화 ────────────────────────────────────────────────────────────
if ~isfield(ctrlState, 'zs_lpf')
    ctrlState.zs_lpf  = zeros(4,1);   % 저역 통과 필터 (body bounce용)
end
if ~isfield(ctrlState, 'zu_lpf')
    ctrlState.zu_lpf  = zeros(4,1);   % 저역 통과 필터 (wheel hop 분리용)
end
if ~isfield(ctrlState, 'cPrev')
    ctrlState.cPrev   = zeros(4,1);   % 이전 댐핑 (Rate Limiter용)
end

%% ── 파라미터 ───────────────────────────────────────────────────────────────
cMin  = CTRL.VER.cMin;     % sim_params: 500  [Ns/m]
cMax  = CTRL.VER.cMax;     % sim_params: 5000 [Ns/m]
cGain = CTRL.VER.skyGain;  % sim_params: 2500 [Ns/m] — skyhook 게인

% Hybrid 혼합 비율 (0=순수 skyhook, 1=순수 groundhook)
% α=0.7: body bounce 중심, 승차감 우선 (ASSIGNMENT 요구)
alpha_sky = 0.7;   % Skyhook 기여 비율
alpha_gnd = 0.3;   % Groundhook 기여 비율

% ── 저역통과 필터 설계 (1차 Butterworth) ──
% Body bounce 대역: 1~2 Hz → 차단 주파수 fc_low = 2.5 Hz
% Wheel hop 대역: 10~15 Hz → 차단 주파수 fc_high = 8 Hz
fc_low   = 2.5;    % [Hz] body bounce LPF 차단
fc_high  = 8.0;    % [Hz] wheel hop HPF 차단

% 1차 RC 필터 계수 (이산 시간 근사: y[k] = α*x[k] + (1-α)*y[k-1])
tau_low   = 1 / (2 * pi * fc_low);
tau_high  = 1 / (2 * pi * fc_high);
alpha_low  = dt / (tau_low  + dt);   % LPF 계수
alpha_high = dt / (tau_high + dt);   % HPF 기반 (1-LPF)

% 댐핑 Rate Limiter (급변 방지 — 소음/충격 억제)
% 최대 변화율: (cMax-cMin) / 0.05s ≈ 90000 Ns/m/s
dc_max = (cMax - cMin) / 0.05 * dt;

dampingCmd = zeros(4,1);

%% ── 4륜 독립 제어 루프 ────────────────────────────────────────────────────
for i = 1:4
    zs  = suspState.zs_dot(i);   % 차체 수직 속도
    zu  = suspState.zu_dot(i);   % 휠 수직 속도
    v_rel = zs - zu;              % 서스펜션 상대 속도 (압축 양수)

    % ── 주파수 분리 ──────────────────────────────────────────────────────
    % Body bounce 성분 (LPF)
    ctrlState.zs_lpf(i) = (1 - alpha_low) * ctrlState.zs_lpf(i) + alpha_low * zs;
    zs_body = ctrlState.zs_lpf(i);           % 저주파 (1~2 Hz body bounce)
    zs_hop  = zs - zs_body;                  % 고주파 (10~15 Hz wheel hop)

    % 휠 속도 LPF
    ctrlState.zu_lpf(i) = (1 - alpha_low) * ctrlState.zu_lpf(i) + alpha_low * zu;
    zu_body = ctrlState.zu_lpf(i);
    zu_hop  = zu - zu_body;

    % ── Skyhook 댐핑 (body bounce 대역) ─────────────────────────────────
    % 조건: zs_body * v_rel > 0 이면 하드 (에너지 흡수)
    %       else 소프트 (접지력 유지)
    if (zs_body * v_rel) > 0
        % 연속 skyhook: c = cGain * |zs| / |v_rel| (포화 방지)
        if abs(v_rel) > 1e-4
            c_sky = cGain * abs(zs_body) / max(abs(v_rel), 1e-3);
        else
            c_sky = cMin;
        end
        c_sky = min(c_sky, cMax);
    else
        c_sky = cMin;
    end

    % ── Groundhook 댐핑 (wheel hop 대역) ─────────────────────────────────
    % 목적: 타이어 접지력 유지 (wheel hop 억제)
    % 조건: zu_hop * v_rel < 0 이면 하드
    if (zu_hop * v_rel) < 0
        if abs(v_rel) > 1e-4
            c_gnd = cGain * abs(zu_hop) / max(abs(v_rel), 1e-3);
        else
            c_gnd = cMin;
        end
        c_gnd = min(c_gnd, cMax);
    else
        c_gnd = cMin;
    end

    % ── Hybrid 혼합 ──────────────────────────────────────────────────────
    c_hybrid = alpha_sky * c_sky + alpha_gnd * c_gnd;

    % ── 범위 클램프 ──────────────────────────────────────────────────────
    c_cmd = min(max(c_hybrid, cMin), cMax);

    % ── Rate Limiter (댐핑 급변 방지) ────────────────────────────────────
    dc = c_cmd - ctrlState.cPrev(i);
    dc = min(max(dc, -dc_max), dc_max);
    c_out = ctrlState.cPrev(i) + dc;
    c_out = min(max(c_out, cMin), cMax);

    ctrlState.cPrev(i)  = c_out;
    dampingCmd(i)       = c_out;
end

end
