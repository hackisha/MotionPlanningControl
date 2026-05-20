# 과제 — Kinematic Lateral PID (직선 Y-tracking)

## 목표
첫 횡방향 (조향) 제어. kinematic bicycle 차량을 PID 로 Y reference 추종. 출력은 조향각 δ (라디안).

## 인터페이스 계약
```python
class KinematicLateralPID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float): ...
    def step(self, reference_Y: float, ego_Y: float) -> float   # 반환: rad
```

- 02_pid 의 PID 와 동일 구조. error 가 위치 → Y 좌표.
- 첫 호출 D=0 정책.

## 구현 위치
`01_Python_project/release/03_vehicle_control/04_kinematic_lateral/kinematic_lateral_pid.py` 의 `step` 메소드.

## 실행
테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/04_kinematic_lateral/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/04_kinematic_lateral/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생:
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/04_kinematic_lateral/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `03_vehicle_control/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID / 다른 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **Y 추적 오차** — `vx=3, Y_ref=4, kp=0.2, kd=0.8, ki=0.0`, 30 초 시뮬, tail 평균 `|Y 오차| < 0.2 m`, peak `< 4.2 m`

## 힌트
- 조향각 단위 = 라디안 (degree 아님). plant 가 `±0.5 rad ≈ ±28.6°` 로 clip.
- 직선 추종은 PID 가 가장 잘하는 시나리오 — 곡선은 다음 모듈 (06/07/08) 에서.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_lat_kinematic.py` 수정 금지**.
