# 과제 — Hybrid A* (kinematic 경로 탐색 + Pure Pursuit 추종)

## 목표
Hybrid A* 로 kinematic vehicle 의 연속 상태 (x, y, yaw) 공간에서 최단 경로를 탐색하고, 결과 path 를 chapter 3 의 PurePursuit 으로 따라간다.

A* (05) 와 차이:
- 상태: (x, y) 격자 → **(x, y, yaw) 연속**
- Action: 8-connected 이산 이동 → **motion primitives** (yaw_rate 기반 원호 5종)
- 충돌 검사: 한 점 → **action sweep arc vs 원형 obstacle**
- 동등성: 좌표 같음 → **discretized bucket key** (위치 + yaw 양자화)

## Map 구조 (legacy `map_3`)
- 탐색 공간: x ∈ [-2, 15], y ∈ [-2, 15]
- start = (10, 0, π) — 좌측 바라봄
- goal  = (10, 10, 0) — 우측 바라봄
- 장애물: 반지름 1 의 원형 18 개 (vertical wall x=3, y=5..10 + horizontal wall x=3..15, y=5). L-shape 벽 → 좌측 위로 우회 필요.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
def hybrid_a_star(
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    space: tuple[float, float, float, float],
    obstacles: list[tuple[float, float, float]],
    R: float = 5.0,
    vx: float = 2.0,
    dt: float = 0.5,
    weight: float = 1.1,
    epsilon_goal: float = 0.6,
    on_step: Callable[[pose, set[key]], None] | None = None,
) -> list[tuple[float, float, float]]
```

- `space`: (x_min, x_max, y_min, y_max).
- `obstacles`: (x, y, radius) 튜플 리스트 (원형 충돌).
- `on_step`: 매 expand 직후 호출 — `(current_pose, frontier_keys)`.
- 반환: pose list (start → goal). 미발견 시 `[]`.

## 사용 가능 helper (`vehicle_kinematics.py` fixture)
- `vehicle_move(pose, yaw_rate, dt, vx) → new_pose`: kinematic step.
- `motion_primitives(R, vx, dt) → list[(yaw_rate, dt, cost)]`: 5 가지 action.
- `arc_collision(pose, yaw_rate, dt, vx, obstacles) → bool`: sweep 충돌 검사.
- `in_space(pose, space) → bool`.
- `discretize_pose(pose) → (int, int, int)`: bucket key.
- `euclid_xy(a, b) → float`.

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/06_hybrid_a_star/hybrid_a_star.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조.

테스트:
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/06_hybrid_a_star/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/06_hybrid_a_star/record_gen.py
```
→ 3D viewer 에 갈색 box 18개 (장애물, 높이 2m) + 파란 start marker + 빨간 goal marker + ego 차량. timeline `sim_time` scrubber:
- **t = 0 ~ T_search**: ego 가 expand 되는 노드로 teleport (search 시각화). 우측 패널의 `visited_count` 가 누적, `frontier_count` 가 진동.
- **t = T_search**: 핑크 곡선 (최종 path) 등장.
- **t > T_search**: ego 가 PurePursuit 으로 path 를 매끄럽게 따라가 goal 에 도착.

> `--no-viewer` 로 JSON 만 생성, `--skip N` 으로 search frame 묶기 (기본 1).

Rerun viewer 로 재생 (chapter 4 path planning 통합 — 01~03 의 vehicle simulator 와 06 모두 로드):
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_path_planning.py 01_Python_project_refactored/release/04_path_planning/06_hybrid_a_star/
```

## 합격 기준 (`pytest` 통과)
알고리즘 형태는 자유 — **behavioral spec** 만 본다.

1. **goal 도달** — path 의 마지막 pose 가 goal 에서 `epsilon_goal=0.6` 이내
2. **start 일치** — `path[0]` 가 start 와 0.3 m 이내
3. **장애물 회피** — 모든 path pose 가 어떤 obstacle 반경 밖
4. **kinematic 일관성** — 연속 pose 간 거리 ≤ `vx·dt·1.05` (motion primitive 이동 거리)
5. **on_step 콜백 호출** — 호출 수 ≥ path 길이, 첫 호출 pose ≈ start

## 힌트

**자료 구조**:
- `open_dict: dict[bucket_key, (f, g, pose, parent_key)]`
- `closed: dict[bucket_key, (pose, parent_key)]`
- `bucket_key = discretize_pose(pose)` — 양자화된 (x, y, yaw) 정수 튜플

**핵심 루프** (legacy A* 와 같은 구조, 단 state 가 연속):
1. open 비면 미발견 종료 (`[]`)
2. f 최소 노드 → closed 로
3. `on_step(pose_cur, set(open.keys()))` 호출
4. `euclid_xy(pose_cur, goal) < epsilon_goal` → parent chain 따라 path 복원
5. 아니면 motion_primitives 5 개 모두 시도:
   - `vehicle_move` 로 child_pose 계산
   - `in_space` + `arc_collision` 검사
   - bucket key 가 closed 에 있으면 skip
   - open 에 더 작은 f 있으면 skip, 아니면 갱신

## 게인/파라미터 튜닝 위치
이 과제는 알고리즘 구현 — 파라미터 (R, vx, dt, weight) 는 record_gen 의 module-level 상수. 변경하면 search 결과/속도 달라짐 (단 test 의 값은 변경 X).

## 문제별 추가 제약
- **`map_hybrid.py`, `vehicle_kinematics.py`, `record_gen.py` 수정 금지** — 검증 환경.
- chapter 3 의 `pure_pursuit.py` 도 sys.path import 만 — 수정 X.
- 순수 Python 구현 (numpy/scipy path-finder 라이브러리 사용 X).
