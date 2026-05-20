# 과제 — Constant-Space PID (일정 간격 추종)

## 목표
앞 차량과 **일정한 거리** (기본 20m) 를 유지하는 종방향 PID. 두 대의 차량 (target + ego) 시뮬레이션 구조 도입.

## 인터페이스 계약
```python
class ConstantSpacePID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float,
                 target_space: float = 20.0): ...
    def step(self, target_x: float, ego_x: float) -> float
```

- `step` 가 두 위치 (target 의 절대 x, ego 의 절대 x) 를 직접 받음.
- 컨트롤러 내부에서 `(target_x - ego_x) - target_space` 형태로 error 계산.
- 첫 호출 D=0 정책.

## 구현 위치
`01_Python_project/release/03_vehicle_control/02_constant_space/constant_space_pid.py` 의 `step` 메소드.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조.

테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/02_constant_space/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/02_constant_space/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/02_constant_space/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `03_vehicle_control/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID / 다른 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **PD-only 추종 안전성·boundedness** — 80 초 시뮬:
   - 충돌 없음 (`min gap > 5 m`)
   - 정상상태 gap 이 `target_space` 근방 (tail MAE `< 3 m`)
   - 발산 없음 (peak `|gap - target_space| < 12 m`)

> PD-only 의 잔류 offset (drag 외란) 자체가 학습 포인트 — 충돌·발산만 막으면 합격. I 항은 본 과제 범위 밖.

## 힌트
- error 의 핵심은 부호: `(target_x - ego_x) - space`. 멀면 양수 → 가속, 가까우면 음수 → 감속.
- drag 가 일정 외란 역할 → PD-only 는 잔류 오차 ≈ `drag/Kp` 를 못 없앰 (합격 기준 #3 가 이를 입증). 03_time_gap 의 time-gap 정책은 gap_target 자체가 `ego_vx` 와 함께 동적이라 외란을 흡수 → PD-only 로도 0 수렴.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_long_space.py` 수정 금지**.
