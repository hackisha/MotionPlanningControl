# 과제 — Time-Gap PID (시간 간격 추종)

## 목표
간격이 ego 속도에 비례 (`gap = ego_vx · time_gap`) — 실차 ACC 의 표준 정책. **같은 알고리즘이 두 시나리오 (정지 / 기동 target) 에서 모두 작동**해야 함.

## 인터페이스 계약
```python
class TimeGapPID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float,
                 time_gap: float = 1.0): ...
    def step(self, target_x: float, ego_x: float, ego_vx: float) -> float
```

- `step` 가 ego_vx 도 받음 — target_space 매 스텝 재계산용.
- 첫 호출 D=0 정책.

## 구현 위치
`01_Python_project/release/03_vehicle_control/03_time_gap/time_gap_pid.py` 의 `step` 메소드.

## 실행

테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/03_time_gap/ -v
```

두 시나리오 실행 → `record_stationary.json` / `record_maneuvering.json` 생성 + Rerun viewer 자동 띄움 (같은 viewer 에 두 시나리오 누적):
```bash
uv run python 01_Python_project/release/03_vehicle_control/03_time_gap/record_gen_stationary.py
uv run python 01_Python_project/release/03_vehicle_control/03_time_gap/record_gen_maneuvering.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생 (두 시나리오 모두 한 viewer 에 별도 recording 으로 로드 — 좌측 Recordings 패널에서 클릭 전환):
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/03_time_gap/
```

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID / 다른 gap 정의) 는 제약 X — **behavioral spec** 만 본다.

1. **정속 target 시간 간격 정확도** — 50 초 시뮬, tail 평균 `|time_gap - 1.0| < 0.1 s`, peak `< 0.3 s`
2. **기동 target 안전 + 재수렴** — 가/감속 시나리오 80 초, 충돌 없음 (`min gap > 0`) + 정속 복귀 후 `|time_gap - 1.0| < 0.5 s` 재수렴

> 같은 게인이 두 시나리오 모두 통과해야 — 한쪽만 작동하는 튜닝은 합격 X.

## 힌트
- `target_space = ego_vx · time_gap` 을 **매 step 호출 시 다시 계산** (생성자에서 한 번만 X)
- 기동 시나리오는 정지보다 진동·overshoot 위험 ↑ → D 항 필수
- 충돌 회피: error 부호 반대인지 (`measure - reference` 가 아닌 `reference - measure`) 점검
- **초기 조건은 ACC 인계 시점** — `ego_vx == target_vx`, 이미 정상 time-gap 거리 (`gap0 = ego_vx · time_gap = 10m`) 에서 시작. cold start (큰 gap 좁히기) 는 본 과제 범위 밖.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_long_tg.py` 수정 금지**.
- 두 데모 모두 통과해야 함 — 한쪽만 작동하는 게인은 합격 X.
