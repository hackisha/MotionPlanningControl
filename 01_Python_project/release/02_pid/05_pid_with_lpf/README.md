# 과제 — PID + Low-Pass Filter (노이즈 측정 결합)

## 목표
노이즈가 섞인 측정값을 PID 에 그대로 넣으면 D 항이 노이즈를 증폭해 액추에이터가 격렬히 출렁인다.
LPF 로 측정값을 평활한 뒤 PID 에 넣는 **닫힌 루프 결합 함수**를 학생이 직접 작성한다.

> **핵심 학습 포인트**: 추적 정확도가 좋아지는 게 아니라 **제어 입력의 출렁임이 한 자리수 이상 감소** — 액추에이터 보호.

## Plant 조건
- 외란 0.1, 측정 노이즈 표준편차 0.25 (노이즈 시드 42 로 결정론적)
- 알고리즘(LPF, PID)은 이미 학습한 모듈 — 이번엔 **결합 글루** 만 작성

## 인터페이스 계약 (학생 작성 위치)
**파일**: `01_Python_project_refactored/release/02_pid/05_pid_with_lpf/closed_loop_lpf.py`

```python
def closed_loop_step(
    plant, estimator, controller, target: float,
) -> tuple[float, float, float, float]:
    """반환: (y_true, y_measure, y_estimate, u) — 모두 float"""
```

호출 순서 (의도):
1. `y_measure = plant.measure()` — 노이즈 포함 관측
2. `y_estimate = estimator.step(y_measure)` — LPF 평활
3. `u = controller.step(target, y_estimate)` — **PID 는 추정값 사용 (raw 노이즈 ❌)**
4. `plant.step(u)` — actuator → 동역학 한 스텝

## 구현 위치
`01_Python_project_refactored/release/02_pid/05_pid_with_lpf/closed_loop_lpf.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project/release/02_pid/05_pid_with_lpf/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/02_pid/05_pid_with_lpf/record_gen.py
```
→ ego (파랑) 가 true Y, 노란 점 = noisy measurement, 청록 점 = LPF estimate. 필터가 noise 를 얼마나 부드럽게 만드는지 3D + scalar 양쪽으로 관찰.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/02_pid/simulator_pid.py 01_Python_project_refactored/release/02_pid/05_pid_with_lpf/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `02_pid/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 짠 `closed_loop_step` 의 내부 순서 (measure → filter → control → actuate) 형태는 제약 X — 인터페이스 + behavioral spec 만 본다.

1. **필터 폐루프 추적 오차** — 외란 0.1, 노이즈 std 0.25, seed 42, 60 초 시뮬, tail 평균 `|y_true| < 0.15`, peak < 1.5
2. **LPF 가 raw 대비 actuator 부드러움 우수** — 동일 plant 에 raw 측정값을 PID 에 직접 넣은 베이스라인 대비 **제어 입력 std 가 1/5 이하** (5 배 이상 부드러움)

> 필터 우회/약한 필터링 구현은 std ratio < 5 로 차단.

## 힌트
- 4 단계 호출 순서를 정확히 — **estimator 갱신이 controller 앞서야** 추정값으로 제어 가능.
- 반환값의 `y_true` 는 `plant.y` 속성 (true 위치). `plant.step(u)` 의 반환값과 같음 — 어느 쪽을 써도 OK.
- LPF 알고리즘 자체는 `low_pass_filter.py` 에 이미 들어 있음 (수정 X).
- PID 알고리즘은 `pid_controller.py` 에 이미 들어 있음 (수정 X).

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`plant_lpf.py`, `low_pass_filter.py`, `pid_controller.py` 수정 금지**.
- 노이즈 시드(42) 변경 금지 — 테스트 재현성에 의존.
- LPF 의 `α=0.9` 변경 금지 — 테스트 fixture 일부.
