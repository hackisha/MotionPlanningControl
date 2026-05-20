# 과제 — Pure Pursuit (기하학적 lateral controller)

## 목표
PID 가 아닌 **순수 기하학적** 횡방향 컨트롤러. 차량 wheelbase 와 lookahead 거리를 이용해 한 번의 atan 으로 조향각 결정. **stateless** — 내부 상태 없음.


> **시나리오**: 차량 시작 `Y0=2` (도로 밖) → 직선 도로 (`Y=0`, 첫 40m) 로 수렴 → sine 도로 (`Y = 2·(cos((x−40)/14)−1)  (R≈100m)`) 진입. 첫 step 응답 + 곡선 추종을 함께 학습.

## 인터페이스 계약
```python
class PurePursuit:
    def __init__(self, L: float, lookahead_time: float = 1.0): ...
    def step(self, coeff: np.ndarray, vx: float) -> float   # 반환: rad
```

- `L` = 차량 wheelbase (`VehicleLat.L` 와 일치).
- **stateless** — 같은 입력 두 번 호출 → 같은 출력 (내부 상태 X).

## 구현 위치
`01_Python_project/release/03_vehicle_control/07_pure_pursuit/pure_pursuit.py` 의 `step` 메소드.

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/07_pure_pursuit/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/07_pure_pursuit/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/07_pure_pursuit/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `03_vehicle_control/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 pure-pursuit / 다른 lookahead 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **곡선 경로 추적 오차** — `L=4, lookahead_time=1.0`, vx=3, 직선(40m)+sin path, Y0=2 (도로 밖 시작), 30 초 pipeline, tail 평균 `|lateral err| < 0.4 m`, peak `< 2.2 m`

## 힌트
- 식: `δ = atan( 2·L·y_lh / (d_lh² + y_lh² + ε) )`
  - `d_lh = lookahead_time · vx`  (lookahead 거리)
  - `y_lh = poly(coeff, d_lh)`     (lookahead 점의 local-frame y)
  - `ε = 1e-3`                      (분모 0 회피)
- atan2 가 아닌 1-arg atan 으로 충분.
- 내부 state 안 만들어도 됨 — `__init__` 에 L, lookahead_time 만 저장.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_lat_pursuit.py`, `frame_transform.py` 수정 금지**.
