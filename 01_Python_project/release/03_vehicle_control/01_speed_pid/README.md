# 과제 — Speed PID (종방향 속도 PID)

## 목표
실차 dynamics (drag · grade · 가속도 한계) 가 적용된 종방향 점질량 모델에서 PID 로 reference 속도 추종.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
class SpeedPID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float): ...
    def step(self, reference: float, measure: float) -> float
```

- 02_pid/03_pid_controller 의 PIDController 와 동일 구조 (속도 reference 만 다름).
- 첫 호출 D=0 정책 (PD/PID 모듈과 동일).

## 구현 위치
`01_Python_project/release/03_vehicle_control/01_speed_pid/speed_pid.py` 의 `step` 메소드.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project/release/03_vehicle_control/01_speed_pid/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project/release/03_vehicle_control/01_speed_pid/record_gen.py
```

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.


Rerun viewer 로 재생 (시간축 ×1 = 실제 `dt`):
```bash
uv run python 01_Python_project/release/03_vehicle_control/simulator_vehicle_control.py 01_Python_project/release/03_vehicle_control/01_speed_pid/
```

시뮬레이터는 챕터 전체용 — 인자 없이 실행하면 `03_vehicle_control/` 하위 모든 `record*.json` 을 **한 viewer 에 별도 recording 으로 로드**, viewer 좌측 Recordings 패널에서 클릭으로 시나리오 전환 (터미널 번호 선택 불필요). 폴더 인자는 그 폴더만, 파일 인자는 그 record 하나만 로드. `--camera follow|fixed` 로 초기 카메라 모드 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (정통 PID / 다른 처리) 는 제약 X — **behavioral spec** 만 본다.

1. **속도 추적 오차** — `vx0=0, v_ref=30 m/s, kp=1.0, kd=0.0, ki=0.005`, 50 초 시뮬, tail 평균 `|vx 오차| < 0.5`, peak `|vx 오차| < 31`

> KI 가 drag 외란을 보상해 tail MAE 작음. trivial 구현은 tail/peak 모두 초과로 차단.

## 힌트
- 02_pid 의 PID 와 거의 동일 — 그대로 재사용 가능 (시그니처 일치)
- `error = reference - measure` (속도 차이)
- drag (C·v²) 가 외란 역할 → I 항이 잔류 보상

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
- **`vehicle_long_speed.py` 수정 금지** — 검증 환경의 일부.
