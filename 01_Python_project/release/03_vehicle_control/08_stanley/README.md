# 과제 — Stanley (heading + cross-track 합성 lateral controller)

## 목표
PurePursuit 와 다른 두 번째 기하학적 컨트롤러. **현재 위치의 두 가지 오차** (heading error + cross-track error) 를 합성. DARPA Grand Challenge 2005 우승팀의 method. **stateless**.


> **시나리오**: 차량 시작 `Y0=2` (도로 밖) → 직선 도로 (`Y=0`, 첫 40m) 로 수렴 → sine 도로 (`Y = 2·(cos((x−40)/14)−1)  (R≈100m)`) 진입. 첫 step 응답 + 곡선 추종을 함께 학습.

## 인터페이스 계약
```python
class Stanley:
    def __init__(self, k: float = 1.0, epsilon: float = 1e-3): ...
    def step(self, coeff: np.ndarray, vx: float) -> float   # 반환: rad
```

- `k` = cross-track gain (보통 1.0).
- **stateless** — 내부 상태 X.

## 구현 위치
`01_Python_project/release/03_vehicle_control/08_stanley/stanley.py` 의 `step` 메소드.

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/08_stanley/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/08_stanley/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/08_stanley/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `03_vehicle_control/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 Stanley / 다른 heading+cross 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **곡선 경로 추적 오차** — `k=1.0`, vx=3, 직선(40m)+sin path, Y0=2 (도로 밖 시작), 30 초 pipeline, tail 평균 `|lateral err| < 0.4 m`, peak `< 2.2 m`

## 힌트
- 식: `δ = ψ_e + atan( k · e_y / (vx + ε) )`
  - `ψ_e = coeff[-2]` (1차 계수: heading error)
  - `e_y = coeff[-1]` (상수항: cross-track error = local frame 의 y at x=0)
- 인덱스 헷갈리지 말 것: 마지막이 cross-track, 마지막에서 둘째가 heading.
- ε 는 저속(vx≈0) 분모 회피.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_lat_stanley.py`, `frame_transform.py` 수정 금지**.
