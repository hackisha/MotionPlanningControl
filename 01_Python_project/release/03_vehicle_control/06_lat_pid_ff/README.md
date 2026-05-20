# 과제 — Lateral PID with curvature Feedforward (PID+FF)

## 목표
04 의 직선 Y-tracking 을 곡선 path 로 일반화. local-frame 다항식의 lookahead 점에서 y-error 를 PID 로 보정 + 곡률 항을 feedforward 로 사전 보정. **kff=0 이면 순수 PID, kff>0 이면 FF 추가** — 저속/고속 데모로 효과 비교.


> **시나리오**: 차량 시작 `Y0=1` (도로 밖) → 직선 도로 (`Y=0`, 첫 40m) 로 수렴 → sine 도로 (`Y = 2·(cos((x−40)/14)−1)  (R≈100m)`) 진입. 첫 step 응답 + 곡선 추종을 함께 학습.

## 인터페이스 계약
```python
class LatPIDFF:
    def __init__(self, kp: float, kd: float, ki: float, kff: float,
                 dt: float, lookahead_time: float = 0.5): ...
    def step(self, coeff: np.ndarray, vx: float) -> float   # 반환: rad
```

- `coeff` shape: `(degree+1, 1)` column (05 의 PolynomialFitting 출력 그대로)
- 첫 호출 D=0 정책

## 구현 위치
`01_Python_project/release/03_vehicle_control/06_lat_pid_ff/lat_pid_ff.py` 의 `step` 메소드.

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/06_lat_pid_ff/ -v
```

두 시나리오 실행 → `record_low_speed.json` / `record_high_speed.json` 생성 + Rerun viewer 자동 띄움 (단일 ego; viewer 의 `lateral_error` 시계열을 보고 FF (kff) 도입 여부/강도 직접 판단. 같은 viewer 에 두 시나리오 누적):
```bash
uv run python 01_Python_project/release/03_vehicle_control/06_lat_pid_ff/record_gen_low_speed.py
uv run python 01_Python_project/release/03_vehicle_control/06_lat_pid_ff/record_gen_high_speed.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생 (두 시나리오 모두 한 viewer 에 별도 recording 으로 로드 — 좌측 Recordings 패널에서 클릭 전환):
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/06_lat_pid_ff/
```

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID + FF / 다른 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **저속 (vx=3) PID-only 추적 오차** — kff=0, pipeline 30 초, tail 평균 `|lateral err| < 0.3 m`, peak `< 1.2 m`
2. **고속 (vx=10) FF 효과 입증** — kff=0.1 평균 error 가 kff=0 대비 **15% 이상 감소** — FF 무효/약한 구현 차단

## 힌트
- `error = poly(coeff, d_lh)` where `d_lh = lookahead_time · vx` (lookahead 점의 local-frame y)
- `ff_term = vx² · 2 · coeff[-3]` ← `coeff[-3]` 은 다항식 2차 계수 (≈ y''(0)/2 = 곡률/2)
- 인덱스 헷갈리지 말 것: `coeff[-1]` = 상수항, `coeff[-2]` = 1차 (heading), `coeff[-3]` = 2차 (곡률)
- `coeff[-3]` 은 column 이라 `coeff[-3][0]` 또는 `float(coeff[-3])` 로 scalar 변환

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_lat_pid.py`, `frame_transform.py` 수정 금지** — 모두 fixture.
- legacy `kff=0.4` 는 high-fidelity 모델 기준. 본 과제 kinematic plant 에선 `kff=0.1` 이 적정 (테스트가 이 값 사용).
