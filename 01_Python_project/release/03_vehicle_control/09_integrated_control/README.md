# 과제 — Integrated Control (종+횡 통합, lane keep + adaptive cruise control)

## 목표
01~08 의 모든 구성요소 (속도 PID, time-gap, frame transform + polynomial fit, lateral controller) 를 **하나의 시나리오에서 동시에 동작**시키는 통합 문제. 학생이 짤 본문은 **종 mode 의사결정 + 통합 dispatch**.

> **시나리오**: 직선(50m) → R=200 좌커브(100m, ≈28.6°) → 접선 직선의 2차선 도로. ego 는 lane2 (오른쪽) 중앙에서 vx=10 출발 — **lane2 를 쭉 따라간다 (lane change 없음)**. target 은 lane1 (왼쪽) 50m 앞, vx=8 등속. t=10s 부터 target 이 5초 동안 lane1→lane2 (ego 의 lane) 침범.
>
> **종 mode 전환 trigger** (LongitudinalDecision): target.Y 가 road center 통과 시 → ego 의 종 mode 가 `speed`(vx=10 추종) → `timegap`(constant τ·v_ego 유지). 결과: ego 가 target 의 vx=8 로 감속해 안전 거리 유지하며 추종.

## 인터페이스 계약
```python
# lateral_controller.py
class PurePursuit:
    def __init__(self, L, lookahead_time=1.0): ...
    def step(self, coeff, vx) -> float                  # 07 과 동일 식

@dataclass
class LateralController:                                # adapter — 06/07/08 셋 다 wrap 가능
    ctrl: object                                        # .step(coeff, vx)
    lookahead_x_fn: Callable[[float], float]
    def step(self, coeff, vx) -> float
    def lookahead_x(self, vx) -> float

# longitudinal_controller.py
class LongitudinalController:
    def __init__(self, dt, kp_v, kd_v, kp_g, kd_g, tau_gap=1.5): ...
    def speed_step(self, v_des, v_ego) -> float
    def timegap_step(self, gap, v_ego, v_target) -> float   # gap = ego heading projection

# control_pipeline.py
class LongitudinalDecision:
    def __init__(self, road, y_invasion_offset=0.0): ...
    def long_mode(self, t, ego, target) -> str            # "speed" | "timegap" (latch)

class ControlPipeline:
    def __init__(self, g2l, fitter, ev, lat_ctrl, long_ctrl, decision,
                 ref_y_fn, sample_xs, x_local, v_des): ...
    def step(self, t, ego, target) -> PipelineOutput      # delta, ax, mode, coeff, viz
```

## 구현 위치
1. `01_Python_project/release/03_vehicle_control/09_integrated_control/lateral_controller.py` — `PurePursuit.step`
2. `01_Python_project/release/03_vehicle_control/09_integrated_control/longitudinal_controller.py` — `speed_step`, `timegap_step`
3. `01_Python_project/release/03_vehicle_control/09_integrated_control/control_pipeline.py` — `LongitudinalDecision.long_mode`, `ControlPipeline.step`

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/09_integrated_control/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/09_integrated_control/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/09_integrated_control/
```

## 합격 기준 (`pytest` 통과)
sub-controller 단위 수식 검증은 06/07/08 에 위임 — 09 는 **decision latch + 통합 closed-loop** 만 본다.

1. **LongitudinalDecision latch** — target 침범 감지 후 mode 가 `timegap` 으로 latch (이후 target 이 원 lane 으로 돌아가도 유지)
2. **통합 폐루프 45 초 시뮬**
   - 침범 후 `timegap` mode 진입 (decision.invaded 참)
   - sim 마지막 5 초 ego 가 lane2 추종 (평균 `|Y_ego - lane2| < 0.4 m`)
   - 감속 발생 (vx tail mean `< 9.5`, v_des 10 보다 작아짐)
   - 과감속 X (vx tail mean `> 7.0`, target vx 8 까지 과감속 X)
   - 충돌 X (`min gap > 2 m`)

> sub-controller adapter (PP/Stanley/PIDFF) 가 작동 안 하면 통합 폐루프 spec 미달로 자동 차단.

## 힌트
- **speed PID** (01 챕터와 동일): `err = v_des - v_ego`; 첫 호출 D=0; `ax = kp_v·err + kd_v·d_err`.
- **timegap PD** (03 챕터와 동일): `desired_gap = τ·v_ego`; `gap_err = gap - desired_gap`; `rel_v = v_target - v_ego`; `ax = kp_g·gap_err + kd_g·rel_v`. **gap 은 호출자(ControlPipeline)가 ego heading projection 으로 계산해서 넘김** — controller 안에서 X 차이로 직접 계산하지 말 것.
- **LongitudinalDecision** — `y_in_road = target.Y - road.y_center(target.X)` 가 음수면 invasion. 한 번 invaded=True 가 되면 영원히 timegap.
- **ControlPipeline.step** — 4 단계 시퀀스 (perception → lat → decision → long). perception 의 sampling 은 ego heading projection (`x_global = ego.X + cos(ego.Yaw) * sample_xs`) — 곡선 도로에서도 ref 가 ego 앞에 분포.
- **longitudinal min selection** — mode=timegap 시 `ax = min(ax_speed, ax_timegap)`. ACC 의 timegap 식은 gap 멀면 가속 명령(+) 까지 내지만, ego 의 lane 에 침범한 target 앞에서 가속하면 안 됨. speed 명령(=cruise 평형 0) 으로 capping 해서 가속 차단 — ego 는 절대 v_des 이상으로 가속 안 함.
- **종 mode 3D 시각화** — Rerun viewer 에서: speed mode 시 아무 표시 없음, **timegap mode 시 ego 앞 지면에 와이파이 모양 레이더 파형** (concentric arc 3개, 반경 2/3.5/5m, fan ±16° = 차선폭 80%, z=0.05, 흰색 투명) — ACC 가 앞차 센싱 중. record_gen 의 `_radar_arc_paths` 헬퍼가 호 3개를 `dynamic_paths` 의 별도 entity 로 생성. speed mode step 은 빈 list 보내 simulator 가 entity Clear.
- **다른 lateral controller 사용**: `lateral_controller.py` 의 docstring 의 예시 — Stanley 또는 LatPIDFF 를 sys.path import 후 `LateralController` 로 wrap.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/kff/lookahead_time/tau_gap 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` 안의 게인/파라미터 (PurePursuit lookahead_time, LongitudinalController kp_v/kd_v/kp_g/kd_g/tau_gap) 가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_integrated_control_09.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_combined.py` 수정 금지** — plant (fixture).
- **`frame_transform.py` (05 폴더) 수정 금지** — sys.path 로 import 만.
- **`Road`, `EgoState`, `TargetState`, `PipelineOutput` dataclass 수정 금지** — 인터페이스 fixture.
- `target_state` (test/record_gen 의 hard-coded target 동작) 의 인자 (X0, t_invasion, T_invasion) 수정 금지 — 시나리오 spec.
- ego 는 **lane change 안 함** (lane2 추종 유지). lane change planner 같은 의사결정 모듈은 본 과제 범위 밖.
