# 과제 — Frame Transform + Polynomial Path Fitting

## 목표
횡방향 곡선 제어 (06/07/08) 의 사전 단계. global path 를 ego 의 local frame 으로 변환 → 다항식 fitting → 임의 x 에서 y 평가. 알고리즘 자체는 회전 행렬 + LSQ 정상방정식.

## 인터페이스 계약
**세 클래스 모두 구현 필요** (06/07/08 에서 사용):

```python
class Global2Local:
    def __init__(self, num_points: int): ...
    def convert(self, points, yaw_ego, x_ego, y_ego) -> np.ndarray
        # (num_points, 2) local frame points, also stores .local_points

class PolynomialFitting:
    def __init__(self, degree: int, num_points: int): ...
    def fit(self, points) -> np.ndarray
        # (degree+1, 1) coeff column (고차→저차 순), also stores .coeff

class PolynomialValue:
    def __init__(self, degree: int, num_x: int): ...
    def calculate(self, coeff, x) -> np.ndarray
        # (num_x, 1) y values, also stores .y, .points (= [x, y] pairs)
```

## 구현 위치
`01_Python_project/release/03_vehicle_control/05_frame_transform/frame_transform.py` 의 세 클래스 본문 `# TODO`.

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/05_frame_transform/ -v
```

데모 (plotly 2-패널 — global / local 변환 전후 직관 확인):
```bash
uv run python 01_Python_project/release/03_vehicle_control/05_frame_transform/demo.py
```

> 본 모듈은 시간축 없는 정적 데모라 `record_gen.py` + Rerun 시뮬레이터 흐름을 쓰지 않는다 (3D 차량 박스 한 개 띄우는 게 본질을 흐림). 다른 03_vehicle_control 모듈과 다른 점.

## 합격 기준 (`pytest` 통과)
이 모듈은 06/07/08/09 의 pipeline 빌딩 블록 (순수 수학 유틸리티) — **수치 정확성** 으로 검증. 구현 형태 (numpy / scipy / pure python) 는 제약 X.

1. **회전 + 평행이동 결합** — yaw=π/2 + ego at (10, 5) 환경에서 global (11, 5) 점 → local (0, -1) (tol `1e-12`)
2. **다항식 fit** — 알려진 cubic 의 sample 들로 fit → 계수 정확 (tol `1e-9`)
3. **다항식 평가** — `coeff = [0, 0, 2, -1]` (degree 3) → `y = 2x - 1` 모든 x 에서 일치

## 힌트
- 회전 행렬: `θ = -yaw_ego` (음수!), `R(θ) = [[cos, -sin], [sin, cos]]`
- LSQ: `coeff = (Aᵀ A)⁻¹ Aᵀ b` (정상방정식; `np.linalg.inv` 사용)
- **계수 순서: 고차 → 저차** (`coeff[0]` 가 x^d 의 계수, `coeff[-1]` 이 상수항). 후속 모듈이 이 순서에 의존.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- 시그니처 (특히 `coeff` shape `(d+1, 1)` column) 변경 X — 06/07/08 호환.
