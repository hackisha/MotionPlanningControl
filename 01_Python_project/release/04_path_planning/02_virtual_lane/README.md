# 과제 — Virtual Lane Path Planning (한쪽 차선만 valid 한 구간 처리)

## 목표
실제 차선 인식은 항상 양쪽 둘 다 valid 가 아니라, **한쪽이 끊기는 구간이 흔하다**. 이번 과제는 그런 시나리오 (`X ∈ [20, 40]` 왼쪽 invalid, `X ∈ [60, 80]` 오른쪽 invalid) 에서 valid 한 쪽 차선 + 추정된 차선폭으로 **가상 차선** 을 만들어 path 를 끊기지 않게 한다.

> **핵심 아이디어**: 양쪽 valid 일 때 차선폭을 측정해 기억해두고, 한쪽만 valid 인 시점에 그 기억된 폭으로 반대쪽 가상 차선을 추정.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
class LaneWidthEstimator:
    def __init__(self, Lw_init: float = 4.0): ...
    def update(self, coeff_L, coeff_R, valid_L: bool, valid_R: bool) -> None
        # 양쪽 valid 일 때만 self.Lw 갱신

def either_lane_to_path(coeff_L, coeff_R,
                        valid_L: bool, valid_R: bool, Lw: float) -> np.ndarray
    # valid 상태에 따라 path 계수 결정
```

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/02_virtual_lane/virtual_lane_planner.py` 의 클래스·함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/02_virtual_lane/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/02_virtual_lane/record_gen.py
```
→ 차선이 끊기는 구간 (`X ∈ [20, 40]` 좌, `X ∈ [60, 80]` 우) 에서 한쪽 차선만 보이는데, ego 가 가상 차선으로 중앙선을 추정해 끊김 없이 주행. 우측 panel 에 `Lw_est` 시계열 → 양쪽 valid 구간에서 갱신, 한쪽만 valid 구간에서 유지.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_path_planning.py 01_Python_project_refactored/release/04_path_planning/02_virtual_lane/
```

## 합격 기준 (`pytest` 통과)
알고리즘 형태 (가상 차선 보정 / 차선폭 추정 디테일) 는 제약 X — **behavioral spec** 만 본다.

1. **인터페이스 sanity** — `either_lane_to_path` 반환 shape 이 입력과 동일 column
2. **`LaneWidthEstimator` latch 동작** — 양쪽 valid 일 때만 갱신, 한쪽만 valid 면 직전 값 유지
3. **invalid 구간 폐루프** — 30 s 시뮬, tail 평균 `|lateral err| < 0.4 m`, peak `< 1.5 m`

> trivial 구현 (lane width 무시, 한쪽만 valid 시 zero coeff 등) 은 invalid 구간에서 큰 peak 으로 차단.

## 힌트
- `update`: `valid_L AND valid_R` 일 때만 `self.Lw = coeff_L[-1][0] - coeff_R[-1][0]` (마지막=상수 항)
- `either_lane_to_path` 4 case:
  - 양쪽 valid → 평균
  - 왼쪽만 → `coeff_L.copy()` 후 `coeff[-1][0] -= Lw / 2` (오른쪽 방향 = -y)
  - 오른쪽만 → `coeff_R.copy()` 후 `coeff[-1][0] += Lw / 2`
  - 둘 다 invalid → `np.zeros_like(coeff_L)` (직진)

## 게인/파라미터 튜닝 위치
(01_both_lane README 참조)

## 문제별 추가 제약
- **`lane_virtual.py`, `vehicle_lat_virtual.py`, `record_gen.py` 수정 금지**.
- `LaneWidthEstimator.__init__(Lw_init=4.0)` 기본값 변경 X — 테스트 fixture.
