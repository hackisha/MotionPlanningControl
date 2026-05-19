# 과제 — PID + 2D Kalman Filter (모델 기반 추정 결합)

## 목표
LPF 대신 **2D Kalman Filter** 로 위치를 추정한다.
LPF 와의 핵심 차이: KF 는 **plant 모델 + 직전 제어 입력 (`prev_u`)** 을 사용해 한 스텝 예측한 뒤 측정값으로 보정 → 위상 지연이 줄고 추정이 더 정확.

학생의 작업: 닫힌 루프 결합 함수 — **`prev_u` 를 KF 에 정확히 전달**하는 것이 핵심.

## Plant 조건
- 외란 0.1, 측정 노이즈 표준편차 0.25 (시드 42)
- 알고리즘(KF, PID) + KF 모델 행렬은 이미 fixture 로 제공

## 인터페이스 계약 (학생 작성 위치)
**파일**: `01_Python_project_refactored/release/02_pid/06_pid_with_kf/closed_loop_kf.py`

```python
def closed_loop_step(
    plant, estimator, controller, target: float, prev_u: float,
) -> tuple[float, float, float, float]:
    """반환: (y_true, y_measure, y_estimate_position, u)"""
```

**LPF 과제와의 시그니처 차이**: `prev_u` 인자가 추가됨. driver loop 에서 매 스텝 갱신 (처음엔 0).

호출 순서:
1. `y_measure = plant.measure()`
2. `state = estimator.step(y_measure, prev_u)` — **`prev_u` 를 두 번째 인자로 전달**
3. `y_estimate = float(state[0])` — `state[0]` = 위치, `state[1]` = 속도
4. `u = controller.step(target, y_estimate)`
5. `plant.step(u)`

## 구현 위치
`01_Python_project_refactored/release/02_pid/06_pid_with_kf/closed_loop_kf.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project/release/02_pid/06_pid_with_kf/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/02_pid/06_pid_with_kf/record_gen.py
```
→ ego (파랑) 가 true Y, 노란 점 = noisy measurement, 청록 점 = KF estimate. LPF (05) 보다 위상 지연 적은 model-based 추정의 3D 시각화.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/02_pid/simulator_pid.py 01_Python_project/release/02_pid/06_pid_with_kf/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `02_pid/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 짠 `closed_loop_step` 내부 순서·`prev_u` 전파 형태는 제약 X — 인터페이스 + behavioral spec 만 본다.

1. **KF + PID 폐루프 추적 오차** — 외란 0.1, 노이즈 std 0.25, seed 42, 60 초 시뮬, tail 평균 `|y_true| < 0.15`, peak < 1.5
2. **KF 추정 정확도** — 추정 위치의 tail 평균 절대 오차 `< 0.10` (raw 노이즈 ≈ 0.2 의 절반 이하)

> `prev_u` 가 KF 에 잘못 전달되거나 누락되면 추정/추적 모두 임계값 초과로 차단.

## 힌트
- KF 의 `step` 시그니처: `step(measurement, control_input)` — control 입력 누락 시 `TypeError`.
- driver loop 에서 `prev_u` 추적: 처음 0 으로 시작, 매 스텝 후 방금 계산된 `u` 로 갱신.
- `state[0]` 은 numpy scalar — `float(state[0])` 로 명시적 변환 (테스트의 타입 검증 통과).
- 위치는 `plant.y` (true) 또는 `plant.step(u)` 의 반환값.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`plant_kf.py`, `kalman_filter_2d.py`, `pid_controller.py` 수정 금지**.
- KF 모델 행렬 (A, B, C, Q, R) 은 테스트/데모에서 fixture 로 제공 — 수정 X.
- 노이즈 시드(42) 변경 금지.
