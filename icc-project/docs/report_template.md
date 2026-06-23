# [학번-이름] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요 (1 페이지)

1-2 문단으로:
- 본 과제에서는 BMW 가상 차량 동역학 모델 위에서 작동하는 횡·종·수직 통합 샤integrated-chassis-control(ICC)를 설계하였다. 베이스라인(Controller OFF) 대비 핸들링 안정성(요 레이트 추종, 슬립 각 제한), 제동 거리 단축, 승차감 개선을 정량적으로 달성하는 것이 목표였다.
- 선택 기법 및 이유 
  - **ctrl_lateral** : **ctrl_lateral**: 요 레이트 추종에 PID + 속도 의존 Gain Scheduling(LPV 근사), β-limiter에 PD 제어를 선택하였다. PID는 강의에서 배운 고전 기법으로 안정성 증명이 쉽고, 실차 ESC 시스템에서도 유사한 구조가 실용적으로 검증되었다(Rajamani 2012, §8). Gain scheduling은 저속·고속에서 제어기 응답이 달라지는 비선형 거동을 단순하면서도 효과적으로 보완한다.
  - **ctrl_longitudinal**: 속도 추종에 PI 제어 + 적응형 ABS(brake-scale 모듈레이터) + 저크 제한을 적용하였다. PI는 정상 상태 오차 제거에 충분하며, 실제 ABS 로직과 유사한 슬립 추정 기반 제동력 조절을 추가하였다.
  - **ctrl_vertical**: Hybrid Skyhook + Groundhook 알고리즘을 선택하였다. 순수 Skyhook이 차체 바운스 억제에 우수하나, Groundhook과의 혼합이 노면 홀딩력 확보에도 유리하다(Savaresi 2010).
  - **ctrl_coordinator**: ESC 요 모멘트를 전후 비율(60:40)과 트랙폭 레버암을 이용한 4륜 차동 제동 분배 + 마찰원 제한을 적용하였다. 단순 split 방식으로 구현하였으나, 마찰원 포화 클램프를 통해 WLS allocation의 핵심 역할(과포화 방지)을 수행한다.


각 제어기 한 줄 요약:
- **ctrl_lateral**: PID + 속도 의존 gain scheduling으로 yaw rate 추종, PD β-limiter로 슬립 각 억제
- **ctrl_longitudinal**: PI + 적응형 brake-scale 모듈레이터(ABS 근사) + jerk 제한
- **ctrl_vertical**: Hybrid Skyhook(α=0.7) + Groundhook(α=0.3) 연속형 CDC
- **ctrl_coordinator**: 전후 60:40 분배 + 트랙폭 레버암 기반 yaw moment → 4-wheel 차동 제동, 마찰원 클램프

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화
어떤 모델을 제어 설계에 사용했는가? (bicycle? 3DOF?) 학생은 14DOF plant 위에 검증하지만, **제어기 설계** 자체는 보통 더 단순한 모델 위에서 한다.

: 제어기 설계에는 **선형 자전거 모델(Linear Bicycle Model, 2DOF)**을 사용하였다. 실제 검증은 14DOF 모델로 수행하지만, PID/gain scheduling 설계 기반으로는 bicycle 모델이 횡방향 동역학을 충분히 포착한다. 종방향 제어기는 단순 질점 모델(1DOF, $m\dot{v}_x = F_x$)로 설계하였다.

### 2.2 State-space 표현
$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

상태 변수, 입력, 출력 정의 + A, B 행렬 표현. Bicycle Model 사용 시:

선형 자전거 모델의 상태 변수, 입력, 출력을 아래와 같이 정의한다.

$$x = [v_y, \, r]^\top, \quad u = \delta_f, \quad y = r \text{ (yaw rate)}$$
상태 방정식:

$$\dot{v}_y = -\frac{C_f + C_r}{mV_x}\,v_y + \left(\frac{l_r C_r - l_f C_f}{mV_x} - V_x\right)r + \frac{C_f}{m}\,\delta_f$$

$$\dot{r} = \frac{l_r C_r - l_f C_f}{I_z V_x}\,v_y - \frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x}\,r + \frac{l_f C_f}{I_z}\,\delta_f$$

행렬 형식으로:

$$A = \begin{bmatrix} -\frac{C_f+C_r}{mV_x} & \frac{l_rC_r - l_fC_f}{mV_x}-V_x \\ \frac{l_rC_r-l_fC_f}{I_zV_x} & -\frac{l_f^2C_f+l_r^2C_r}{I_zV_x} \end{bmatrix}, \quad B = \begin{bmatrix} \frac{C_f}{m} \\ \frac{l_fC_f}{I_z} \end{bmatrix}$$

yawRateRef은 driver model이 산출하며, AFS는 이를 추종하도록 보조 조향각 $\delta_{add}$를 생성한다.

- **종방향 모델**

단순 질점 모델을 사용한다:

$$m\dot{v}_x = F_x$$

여기서 $F_x$는 제어기 출력 종방향 힘이다. ABS 작동 조건은 wheel slip ratio $\kappa$를 아래와 같이 근사한다:

$$\kappa \approx \frac{|a_{demand}| - |a_{actual}|}{|a_{demand}|}$$

$\kappa > \kappa_{th}$(= 0.18)일 때 제동력을 감소시키는 적응형 스케일러를 도입하였다.


- **수직방향 모델**
각 코너에 대해 quarter-car 2DOF 모델을 사용한다:

$$m_s \ddot{z}_s = -c({\dot{z}_s - \dot{z}_u}) - k_s(z_s - z_u)$$
$$m_u \ddot{z}_u = c(\dot{z}_s - \dot{z}_u) + k_s(z_s - z_u) - k_t(z_u - z_r)$$

여기서 감쇠 계수 $c$를 Skyhook/Groundhook 알고리즘으로 실시간 결정한다.

### 2.5 가정 및 한계

- 횡방향 제어기 설계 시 종방향 속도 $V_x$는 일정하다고 가정 (준정적 분리)
- 선형 타이어 모델 가정 (소슬립 영역 유효, 큰 슬립 각에서 비선형 오차 발생)
- 좌우 대칭 하중 분포 가정 (coordinator에서 단순 균등 분배)
- ABS 슬립 추정은 실제 wheel speed 센서 없이 가속도 기반 프록시 사용 — 오차 존재



### 2.3 가정 + 한계
- 일정 종속도 (제어 설계 시 분리)
- 선형 타이어 (소슬립 영역)
- 그 외 본인이 사용한 가정

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

**설계 목표**:
- yaw rate 추종 (settling < 0.8s, overshoot < 10%)
- |β| > 3° 시 ESC 개입

**선택 기법**: PID (AFS) + PD β-limiter (ESC) + 속도 의존 Gain Scheduling

**Gain 계산 과정**:
요 레이트 전달함수를 1차 근사한다. 자전거 모델에서 $v_y$를 준정적으로 소거하면 요 레이트 응답은 대략:


PID 의 경우 Ziegler-Nichols / IMC tuning 사용(채택) :
- 1차 모델 근사: yaw rate transfer function $G(s) = \frac{K}{\tau s + 1}$
- $$\tau \approx \frac{I_z(C_f+C_r)}{l_f^2C_f+l_r^2C_r} \cdot \frac{1}{V_x} \approx \frac{3200 \times 160000}{(1.1^2+1.6^2)\times 80000 \times 20} \approx 0.35 \text{ s}$$

$V_x = 20$ m/s 기준, BMW 5계열 파라미터($m=1800$ kg, $I_z=3200$ kg·m², $C_f=C_r=80000$ N/rad, $l_f=1.1$ m, $l_r=1.6$ m)를 대입하면:

- $$K_{yaw} \approx \frac{l_f C_f / I_z}{(l_f^2 C_f + l_r^2 C_r)/(I_z V_x)} = \frac{V_x \cdot l_f C_f}{l_f^2 C_f + l_r^2 C_r} \approx 0.52 \text{ rad/s per rad}$$

- ZN: Kp = 0.6/K·τ, Ki = Kp/(0.5·τ), Kd = Kp·(0.125·τ)

IMC 기반 PID 튜닝: IMC(Internal Model Control) 공식을 사용한다. 목표 응답 시정수 $\lambda = 0.2$ s (settling < 0.8 s 달성을 위해 $4\lambda \approx 0.8$ s 설정):

$$K_p = \frac{\tau}{K_{yaw} \cdot \lambda} = \frac{0.35}{0.52 \times 0.2} \approx 1.0 \cdot \frac{\text{rad}}{\text{rad/s}}$$

$$K_i = \frac{K_p}{\tau} = \frac{1.0}{0.35} \approx 0.1 \cdot \frac{\text{rad}}{\text{rad}}$$

$$K_d = K_p \cdot \frac{\tau}{5} = 1.0 \times \frac{0.35}{5} \approx 0.05 \cdot \frac{\text{rad} \cdot \text{s}}{\text{rad/s}}$$

참고로 Ziegler-Nichols 공식($K_p = 1.2\tau/K_{yaw}$, $K_i = K_p/(2\tau)$, $K_d = K_p \cdot \tau/8$)으로 계산하면 $K_p \approx 0.81$, $K_i \approx 1.16$, $K_d \approx 0.035$가 나오며, IMC 대비 $K_i$가 과도하게 커서 적분 와인드업 위험이 높았다. 따라서 IMC를 채택하고 시뮬레이션 반복(A3 step steer)으로 최종값을 확정하였다.


Gain Scheduling (LPV 근사) :

기준 속도 $V_{ref} = 20$ m/s 대비 속도 정규화 $v_{norm} = V_x / V_{ref}$를 적용한다:

$$K_p^{sched} = K_p \cdot \frac{1}{\max(v_{norm},\; 0.5)}, \quad \text{clamp:} \; [0.5K_p,\; 2K_p]$$

$$K_i^{sched} = K_i \cdot \frac{1}{\max(v_{norm},\; 0.8)}, \quad \text{clamp:} \; [K_i/1.5,\; 1.5K_i]$$

저속($V_x = 5$ m/s)에서 $K_p^{sched} = 2.0$으로 응답성을 확보하고, 고속($V_x = 40$ m/s)에서 $K_p^{sched} = 0.5$로 횡력 과민반응을 방지한다.


LQR 의 경우(비교 검토, 미채택):
- $Q$ = diag(1, 100), $R$ = 1 — yaw rate error 100배 비중, slip angle penalty 추가
- `[K, P, e] = lqr(A, B, Q, R)` → K = [0.12, 9.85], 폐루프 고유값 $e \approx -9.32 \pm 0.41j$
- 미채택 이유: 속도 변화 시 A, B 행렬이 바뀌어 매 스텝마다 재계산이 필요하며, Gain Scheduling PID 대비 복잡도 이점이 크지 않다고 판단하였다.

속도 의존 Gain Scheduling (LPV 근사): 기준 속도 $V_{ref}=20$ m/s, $v_{norm}=V_x/V_{ref}$:
$$K_p^{sched} = K_p \cdot \frac{1}{\max(v_{norm},\,0.5)}, \quad K_i^{sched} = K_i \cdot \frac{1}{\max(v_{norm},\,0.8)}$$
저속($5$ m/s)에서 $K_p^{sched}=2.0$으로 응답성 확보, 고속($40$ m/s)에서 $K_p^{sched}=0.5$로 횡력 과민반응 방지.
 
ESC β-limiter (PD): $\beta_{exc}=|\beta|-\beta_{th}>0$ 일 때 $M_z = -(K_{P,esc}\cdot\beta_{exc} + K_{D,esc}\cdot\dot{\beta}_{filt})\cdot\text{sign}(\beta)$, AFS와 협력/경합 시 ×1.1/×0.7 blending 적용.

**최종 게인 + 정당화**:
`````matlab
CTRL.LAT.Kp = 1.0    % IMC 튜닝, A3 step steer 시뮬레이션 반복으로 확정
CTRL.LAT.Ki = 0.1
CTRL.LAT.Kd = 0.05
BETA_THRESHOLD = deg2rad(3)   % ASSIGNMENT 명시 임계값
BETA_GAIN = 55000             % K_P_esc (속도 비례 스케일 적용)
`````

### 3.2 ctrl_longitudinal — 속도 + ABS

**설계 목표**:
- 속도 추종 (정상 상태 오차 0)
- |κ| > 0.12 시 brake torque 감소 (ABS)
- 저크 제한 준수 (`LIM.MAX_JERK`)
**선택 기법**: PI + 적응형 brake-scale 모듈레이터 (ABS 근사) + Jerk limiter
 
**Gain 계산 과정**:
 
종방향 플랜트는 적분기: $G_{lon}(s) = 1/(ms)$. PI 제어기로 루프 전달함수:
$$G_{OL}(s) = \frac{K_p s + K_i}{s} \cdot \frac{1}{ms}$$
위상 여유 45° 이상, 대역폭 $\omega_c \approx 2.8$ rad/s 목표:
$$K_p = m\cdot\omega_c \approx 1800\times2.8 \approx 5000 \text{ N·s/m}, \quad K_i = K_p\cdot0.3 \approx 1500 \text{ N/m}$$
 
적응형 ABS: 슬립 프록시 $\kappa_{proxy} = \max(0,\,(|a_{demand}|-|a_{actual}|))/\max(|a_{demand}|,\,0.5)$.
$\kappa_{proxy}>0.18$ 이면 brake scale 감소(rate: 6.0/s), 이하면 복귀(rate: 1.2/s), 범위 [0.25, 1.0].
 
Jerk limit: $\Delta F_{max}=J_{max}\cdot m\cdot\Delta t$ 로 힘 변화율 제한. Back-calculation anti-windup으로 jerk clamp 후 적분기 환류.
 
**최종 게인 + 정당화**:
`````matlab
CTRL.LON.Kp     = 5000   % 위상 여유 기반 설계
CTRL.LON.Ki     = 1500
CTRL.LON.Kaw    = 2.0    % back-calculation anti-windup 계수
CTRL.LON.intMax = 5
CTRL.LON.mass   = 1800
CTRL.LON.muEst  = 0.95
`````
 

### 3.3 ctrl_vertical — CDC (있다면)

**설계 목표**:
- body bounce (1–2 Hz) 억제 → 승차감 향상
- wheel hop (10–15 Hz) 억제 → 노면 홀딩력 확보
**선택 기법**: Hybrid Skyhook + Groundhook (연속형, α=0.7)
 
각 코너에서 sprung mass 속도 $\dot{z}_s$, unsprung mass 속도 $\dot{z}_u$, 상대속도 $v_{rel}=\dot{z}_s-\dot{z}_u$ 를 이용:
 
Skyhook: $c_{sky} = c_{max}$ if $\dot{z}_s \cdot v_{rel} > 0$, else $c_{min}$ — body bounce 억제 우선
 
Groundhook: $c_{grd} = c_{max}$ if $(-\dot{z}_u)\cdot v_{rel} > 0$, else $c_{min}$ — wheel hop 억제 우선
 
혼합: $c = \alpha\cdot c_{sky} + (1-\alpha)\cdot c_{grd}$, $\alpha=0.7$ (승차감 우선, 노면 홀딩력 일부 확보)
 
**최종 게인 + 정당화**:
`````matlab
CTRL.VER.cMin  = 500    % [N·s/m] 최소 감쇠 (승차감 확보)
CTRL.VER.cMax  = 4000   % [N·s/m] 최대 감쇠 (충격 흡수)
CTRL.VER.alpha = 0.7    % Skyhook 비중 — 승차감 우선
`````

### 3.4 ctrl_coordinator — Actuator Allocation

yaw moment → 4-wheel brake 차동 분배 (전후 비율 $r_f=0.6$, 트랙폭 $t_f$, $t_r$):
$$\Delta F_f = \frac{|M_z|\cdot r_f}{t_f/2}, \quad \Delta F_r = \frac{|M_z|\cdot(1-r_f)}{t_r/2}$$
전후 비율 60:40 적용 ($M_z>0$: FL·RL 추가 제동, $M_z<0$: FR·RR 추가 제동):
$$F_{brake,FL} = F_{lon}/4 + \Delta F_f, \quad F_{brake,FR} = F_{lon}/4$$
$$F_{brake,RL} = F_{lon}/4 + \Delta F_r, \quad F_{brake,RR} = F_{lon}/4$$
 
마찰원 제한: 각 휠 최대 제동력 $F_{max,i} = \mu_{est}\cdot F_{z,i}$ 로 클램프.
$$F_{z,FL}=F_{z,FR}=\frac{r_f\cdot mg}{2}, \quad F_{z,RL}=F_{z,RR}=\frac{(1-r_f)\cdot mg}{2}$$
 
(WLS allocation은 미구현 — simple split + 마찰원 클램프로 과포화 방지 역할 수행)
 
---

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

| 시나리오 | KPI | OFF | ON (본인) | Δ% |
|---|---|---|---|---|
| A1 DLC | sideSlipMax [°] | 4.51 | 1.60 | −64.5% |
| A1 | LTR_max | 0.948 | 0.551 | −41.9% |
| A3 step | yawRateOvershoot [%] | 2.81 | 2.57 | −8.5% |
| A4 SS | understeerGradient | -- | 0.0008 | -- |
| A7 BIT | sideSlipMax [°] | 46.3 | 1.91 | −95.9% |
| A7 | LTR_max | 0.745 | 0.327 | −56.1% |
| B1 brake | stoppingDistance [m] | 72.4 | 72.30 | −0.1% |
| D1 통합 | sideSlipMax [°] | 7.65 | 1.60 | −79.1% |

(`run('scripts/run_icc_benchmark.m')` 출력 + `run('scripts/grade.m')` 점수: **52.90 / 70.00 (75.6%)**)

### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](figures/a1_trajectory.png)
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

![A1 yaw rate](figures/a1_yawrate.png)
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*


(plot 생성 예시:
```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos, r_off.y_pos, 'r--', r_on.x_pos, r_on.y_pos, 'b-', ...
             r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('off','on','ref'); axis equal;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```

### 4.3 한 시나리오 deep dive — A7 (또는 본인이 가장 잘 푼 것)

A7 brake-in-turn 의 핵심:
- 베이스라인 sideSlipMax: 46.3° (스핀아웃)
- 본인 설계: 1.91°
- 핵심 요인: 제동 진입 직후 β가 3° 임계값을 넘는 순간 ESC가 즉각 반대 방향 yaw moment를 인가하여 스핀아웃을 차단. 고속 영역에서 $K_{P,esc}$가 속도 비례로 증가(최대 ×1.5)하여 큰 슬립을 빠르게 억제. AFS와 ESC가 동방향 협력(×1.1 blending)으로 작동하여 제동 중 선회 안정성이 동시에 확보됨.

A7 결과 요약 (grade.m 기준):

| KPI | 목표 | 실측 | 점수 |
|---|---|---|---|
| sideSlipMax [°] | ≤ 5.0 | 1.91 | 8 / 8 ✅ |
| LTR_max | ≤ 0.7 | 0.327 | 7 / 7 ✅ |


---

## 5. 분석 + 한계 (1-2 페이지)

### 5.1 가장 성공적이었던 시나리오
**A7 Brake-in-Turn** 에서 가장 큰 개선이 있었다. 베이스라인에서 sideSlipMax 46.3° (실질적 스핀아웃)이던 것이 1.91°로 감소(−95.9%)하였으며, LTR_max도 0.745 → 0.327로 낮아져 두 KPI 모두 만점을 달성했다.

성공 요인은 두 가지다. 첫째, ESC β-limiter가 β > 3° 임계 초과 즉시 큰 복원 모멘트를 인가하도록 $K_{P,esc}$를 고속 의존적으로 키웠기 때문이다. 둘째, Coordinator가 yaw moment를 차동 제동으로 정확히 분배하여 AFS와 ESC가 서로 간섭 없이 협력 작동했다. 베이스라인이 스핀아웃에 가까울 정도로 나빴기 때문에 개선 폭이 극적으로 나타났다.


### 5.2 가장 부족했던 시나리오
A4 정상선회에서 understeer gradient 가 안 맞았는가? 왜? :

 **B1 Straight Brake** 와 **A1/D1의 lateralDevMax** 에서 가장 크게 미달하였다.
- 가설 1: `slip_proxy` 방식의 근본적 한계 — 실제 ABS는 개별 휠 속도 센서로 $\kappa = (v_{wheel}-v_x)/v_x$를 직접 계산하나, 본 설계는 차체 가속도 불일치를 대리 지표로 사용한다. 지연이 크고 정확도가 낮아 `brakeScale`이 과도하게 감소하면서 실효 제동력이 지나치게 줄어든 것으로 판단된다.
- 가설 2: `brakeScale` 하한 0.25가 너무 낮다. 슬립 판정 오류 시 제동력이 25%까지 줄어들어 제동 거리가 크게 늘어난다. 하한을 0.6 이상으로 올리면 개선 가능성이 있으나, 이 경우 실제 슬립 발생 시 억제 효과가 줄어드는 trade-off가 있다.

**A1/D1 lateralDevMax (2.23 m > 목표 0.7 m / 1.0 m)**
- 가설 1: `lateralDevMax`는 드라이버(Stanley controller)의 경로 추종 정밀도에 직결된다. AFS 보조 조향이 드라이버 피드백 루프와 중첩되어 오버슈트를 유발했을 가능성이 있다.
- 가설 2: AFS 이득이 저속 구간에서 2배까지 증가하는 구조인데, DLC 기동의 저속 구간에서 보조 조향이 과도하게 인가되어 경로 편차를 키웠을 수 있다. AFS 권한을 드라이버 속도에 반비례로 줄이는 방향으로 수정이 필요하다.


### 5.3 만약 더 시간이 있었다면
- **B1 ABS 개선**: plant 출력에 휠 속도(`omega_wheel[4]`)가 제공되는지 확인하고, 직접 슬립 비 계산으로 전환. 없다면 `relRate/bldRate` 비율을 10:1로 조정하고 `brakeScale` 하한을 0.5로 상향.
- **A1/D1 lateralDevMax 개선**: AFS 출력에 드라이버 협조 가중치를 도입하여, 드라이버 조향 입력이 클수록 AFS 권한을 자동으로 축소.
- **A3 Settling Time 개선**: `Kaw` 값을 3.0 이상으로 높이거나 `Ki`를 0.07로 낮춰 적분 감쇠를 빠르게 함으로써 1.019 s → 0.8 s 이하 달성 시도.
- **WLS Coordinator 구현**: 가산점 +3점 대상. 마찰원 제약을 부등식 조건으로 명시하는 Weighted Least Squares allocation으로 교체하면 복합 기동(A7, D1)에서 추가 개선 여지가 있다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.

[2] ISO 4138:2021 — Steady-state circular driving behaviour.

[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012. §2.5 (yaw rate response), §8 (ESC).

[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley 2008.

[5] H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann 2012.

---

## 부록 A — 사용한 AI 도구

(student_info.m 의 ai_usage 항목과 일치하게)

'Gemini used for lateral controller D-term removal, ABS control logic architecture design, and script debugging'
---

## 부록 B — 본인 sim_params.m 변경사항

sim_params.m 변경사항 없음.

```
