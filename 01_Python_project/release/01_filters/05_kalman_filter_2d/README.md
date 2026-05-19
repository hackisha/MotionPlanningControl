# 과제 — Kalman Filter 2D (matrix state-space)

## 목표
04 의 scalar 칼만을 **2 차원 상태 (위치 + 속도)** 로 확장한다. 행렬 산술이 도입되며, 측정은 여전히 scalar (위치만) 인 상황에서 위치와 속도를 동시에 추정한다.

같은 모듈로 **constant velocity (등속)** 과 **spring-mass-damper** 두 시스템 모두 처리 — system matrices (A, B, C) 만 바뀜. 알고리즘은 04 와 동일 6줄.

## 시스템 모델
```
state    : x_{k+1} = A · x_k + B · u_k + w_k,    w ~ N(0, Q)
measure  : z_k     = C · x_k + v_k,              v ~ N(0, R)
```
- x ∈ ℝ² (위치, 속도)
- A 는 2×2, B/C 는 길이 2 1D 벡터, Q 는 2×2, R 은 scalar
- 측정 z 는 scalar (위치만)

## 알고리즘 (구현해야 할 것)
**Predict:**
```
x_pred = A @ x + B * u
P_pred = A @ P @ A.T + Q
```
**Update:**
```
S = C @ P_pred @ C + R                   (scalar)
K = (P_pred @ C) / S                     (1D 길이 2)
innovation = measurement - C @ x_pred    (scalar)
x = x_pred + K * innovation
P = (np.eye(2) - np.outer(K, C)) @ P_pred
```

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class KalmanFilter2D:
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                 Q: np.ndarray, R: float,
                 x0: np.ndarray | None = None,
                 P0: np.ndarray | None = None): ...
    def step(self, measurement: float, control_input: float) -> np.ndarray
```

- `__init__` 은 이미 작성됨. `x0` 미지정 시 `np.zeros(2)`, `P0` 미지정 시 `10·I`.
- `step` 반환은 길이 2 numpy 배열 (1D, shape `(2,)`). `state[0]` = 위치, `state[1]` = 속도.
- **column vector `(2, 1)` 로 짜지 마세요** — 인터페이스가 1D 라 인덱싱이 어긋나 테스트 fail.

## 구현 위치
`01_Python_project_refactored/release/01_filters/05_kalman_filter_2d/kalman_filter_2d.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project/release/01_filters/05_kalman_filter_2d/ -v
```

CV 데모 (등속 운동, 시각 확인 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/05_kalman_filter_2d/cv_demo.py
```
→ truth = v·t (등속), 위치만 노이즈 측정, 위치 + 속도 동시 추정 plotly 그래프 (subplot 2개).

SMD 데모 (스프링-매스-댐퍼, 자유 진동 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/05_kalman_filter_2d/smd_demo.py
```
→ m=10, k=100, b=2 자유 진동 시스템, 위치만 노이즈 측정.

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 Kalman / 단순 추정기 / 다른 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **CV 추적 RMS** — 등속 truth (v=2.0 m/s), 위치만 N(0, 0.5) 노이즈, 300 step, warm-up 이후 위치 RMS `< 0.5`, 속도 RMS `< 0.3`

> RMS = √(bias² + variance). 상수/패스스루 류 trivial 구현은 임계값 초과로 차단.

## 힌트
- 04 (1D Kalman) 의 6줄을 행렬 형태로 옮기는 게 핵심 — 표준 알고리즘이라 식 자체는 정답 노출 OK.
- 1D 배열 (`(2,)`) 사용 — `np.array([0.0, 0.0])` 로 2-원소 평면 벡터. column vector 아님.
- `(np.eye(2) - np.outer(K, C)) @ P_pred` — `np.outer(K, C)` 가 (2,2) 행렬. `K @ C` (inner product, scalar) 와 혼동 주의.
- `A.T` (transpose) 빼먹기 흔함
- `B * u` (broadcasting) 와 `B @ u` (차원 mismatch) 구분
- `S = C @ P_pred @ C + R` 에서 `C @ P_pred @ C` 는 1D · 2D · 1D = scalar (numpy 가 자동)

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조)

이 문제에 한정한 추가 사항은 없습니다.
