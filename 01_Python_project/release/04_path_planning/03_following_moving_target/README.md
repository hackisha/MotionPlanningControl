# 과제 — Following Moving Target (이동 타겟 추종 path)

## 목표
앞 차량 (leading) 이 차로 중앙을 따라가는 동안, 뒤따르는 ego 가 leading 의 **ego-local frame 기준 위치 history** 를 누적해 그 자취를 cubic path 로 추종한다.

> **핵심 아이디어**: leading 의 위치를 매 step ego 의 local frame 으로 측정해서 history 로 쌓고, ego 자신이 움직일 때마다 그 history 점들을 새 ego frame 좌표계로 갱신 (역회전 + 역시프트). 충분히 쌓이면 origin(ego) 통과 + last-point heading 일치하는 3차 다항식으로 path 생성.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
class LeadingTargetTracker:
    def __init__(self, max_history: int = 5): ...
    def update(self, target_local_xy: list[float],
               vx: float, yaw_rate: float, dt: float) -> None

def target_following_path(history: list[list[float]]) -> np.ndarray
    # 반환: shape (4, 1) column. degree 3, [c3, c2, 0, 0]
```

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/03_following_moving_target/target_following_planner.py` 의 클래스·함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/03_following_moving_target/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/03_following_moving_target/record_gen.py
```
→ 회색 leading 이 sinusoidal 차로 중앙을 따라가고, 파란 ego 가 10 m 뒤에서 leading 의 자취를 cubic path (노란 곡선) 로 추종.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_path_planning.py 01_Python_project_refactored/release/04_path_planning/03_following_moving_target/
```

## 합격 기준 (`pytest` 통과)
알고리즘 형태 (history rotation/shift / cubic 합성 디테일) 는 제약 X — **behavioral spec** 만 본다.

1. **`target_following_path` 인터페이스** — 4 점 이상 history 일 때 `(4, 1)` column 반환, 4 미만이면 zero coeff
2. **`LeadingTargetTracker` 윈도우** — `max_history` 초과 시 가장 오래된 항목 제거
3. **폐루프 추종** — leading 이 lane keep, ego 가 leading 의 자취 추종, 30 s, tail 평균 `|follow_err| < 0.5 m`, peak `< 1.5 m`, 충돌 X (`min gap > 0`)

> history rotation/shift 가 망가지면 path 가 빗나가 임계값 초과로 차단.

## 힌트

**`LeadingTargetTracker.update`**:
1. ego 가 직전 step 에 `theta = yaw_rate * dt` 회전 + `vx * dt` 전진
2. 회전 행렬: `rot = [[cos θ, sin θ], [-sin θ, cos θ]]`
3. shift:
   - `|yaw_rate| < 1e-10` (직진) → `[vx*dt, 0]`
   - 아니면 → `[vx*dt, -vx*(1 - cos θ) / yaw_rate]`
4. 새 측정 `target_local_xy` 를 `self.history` 에 append → 길이 초과 시 `pop(0)`
5. 모든 history 점에 `rot` 적용 후 `shift` 빼기

**`target_following_path`**:
- `len(history) < 4` → `np.zeros((4, 1))`
- numpy polyfit (3차) → 마지막 점에서의 slope = `3·a3·xf² + 2·a2·xf + a1`
- `tan_h = tan(heading)`
- `c3 = -(2·xf·yf - xf²·tan_h) / xf⁴`
- `c2 = (3·xf²·yf - xf³·tan_h) / xf⁴`
- 반환: `np.array([[c3], [c2], [0.0], [0.0]])` — ego origin (0,0) 통과 + zero heading

## 게인/파라미터 튜닝 위치
(01_both_lane README 참조)

## 문제별 추가 제약
- **`lane_following.py`, `vehicle_lat_following.py`, `record_gen.py` 수정 금지**.
- chapter 3 의 `frame_transform`, `pure_pursuit` 와 같은 폴더의 `01_both_lane/both_lane_planner.py` 도 sys.path import 만 (leading 의 lane keep 용).
