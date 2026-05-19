# 과제 — PID Tuning (게인 튜닝 실습)

## 목표
**알고리즘은 이미 구현되어 있고**, 학생은 더 어려운 plant 에 대해 PID 게인 `KP, KD, KI` 를 직접 결정한다.
"데모를 돌려보고 응답을 보고 게인을 조절" 사이클 — 실무 컨트롤 튜닝의 축소판.

## Plant 조건 (이 과제의 환경)
- `actuation_gain = 0.5` — 제어 입력이 plant 에 절반만 전달됨 (액추에이터 권한 약함)
- `disturbance = 0.3` — 상수 외란
- 초기 위치 1.0, 목표 0.0

→ 이전 과제(03)의 plant 보다 명확히 어려움. 외란 보상 + 약한 액추에이터를 함께 다뤄야 함.

## 인터페이스 계약 (학생 작성 위치)
**파일**: `01_Python_project/release/02_pid/04_pid_tuning/tuning.py`

```python
KP: float = 0.0   # ← 학생이 결정
KD: float = 0.0
KI: float = 0.0
```

- `pid_controller.py` 는 정답이 이미 들어 있음 (수정 X).
- 학생의 작업은 위 세 값 결정 한 가지뿐.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project/release/02_pid/04_pid_tuning/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움 (**튜닝 보조 — 반복 실행 권장**):
```bash
uv run python 01_Python_project/release/02_pid/04_pid_tuning/record_gen.py
```
→ 학생이 채운 `tuning.py` 의 KP/KD/KI 로 폐루프 응답. 차량이 차로 중앙으로 수렴하는 모양 + control 시계열을 보고 게인 재조정.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project_refactored/release/02_pid/simulator_pid.py 01_Python_project_refactored/release/02_pid/04_pid_tuning/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `02_pid/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 채운 `KP/KD/KI` 값으로 평가 — 게인 부호/크기 제약 X, **behavioral spec** 만 본다.

1. **폐루프 추적 오차** — 외란 0.3 + actuation_gain 0.5 plant, 60 초 시뮬, tail 평균 `|error| < 0.1`, peak `|error| < 1.5`
2. **제어 입력 boundedness** — 시뮬 동안 `max|u| < 50` (발산형 튜닝/극단 게인 차단)

## 힌트
- 모두 작은 양수에서 시작 (예: `KP=KD=KI=0.5`) → demo 실행 → 응답 보고 키우기.
- **KP**: 응답 속도 / 정상상태 게인. 너무 크면 진동.
- **KD**: 진동/오버슈트 억제. 너무 크면 응답 둔해짐.
- **KI**: 외란/정상상태 오차 보정. 너무 크면 적분 windup 으로 진동.
- 권장 탐색 범위: `KP ∈ [1, 5]`, `KD ∈ [1, 3]`, `KI ∈ [0.3, 1]`.
- 데모 패널의 **제어 입력 u** 가 너무 출렁이면 KD 가 큼 / 너무 둔하면 KD 가 작음.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`pid_controller.py` 수정 금지** — 이번 과제는 알고리즘 재구현이 아니라 게인 튜닝.
- **`plant_pid_tuning.py` 수정 금지** — 검증 환경.
- 정답 게인은 한 가지가 아님 — 합격 기준을 만족하는 어떤 조합이라도 통과.
