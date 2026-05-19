# 과제 — A* (heuristic 기반 8-connected 최단 경로)

## 목표
A* 알고리즘으로 격자 미로에서 start `(10, 30)` 부터 goal `(50, 30)` 까지 최단 경로를 탐색한다. Dijkstra (04) 와 같은 8-connected / Rerun 2D viewer 패턴이지만, **heuristic** 을 도입해 expand 횟수를 줄인다.

## Map 구조 (`map_2` 스타일)
- 외곽 border + 중앙 vertical wall (x=30, y=10..50)
- box 의 상·하 면 (y=10 / y=50, x=20..40)
- box 의 좌·우 면은 open (x=20 / x=40)
- start = (10, 30), goal = (50, 30) — 둘 다 box 밖, 중앙 벽 우회 필수

## Dijkstra 와의 차이
- Dijkstra: `f = g` (시작부터 누적 cost). 모든 방향 균등 expand.
- A*: `f = g + weight · h`. h = goal 까지 직선거리. goal 방향 우선 expand → 빠름.
- `weight_heuristic = 1.0` (admissible): 최단 경로 보장.
- `weight_heuristic > 1`: greedy bias — expand 수 ↓, 최적성 ↓.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.**

```python
def a_star(
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacles: set[tuple[int, int]],
    weight_heuristic: float = 1.0,
    on_step: Callable[[tuple[int, int], set[tuple[int, int]]], None] | None = None,
) -> list[tuple[int, int]]
```

- `obstacles`: O(1) 검사용 set.
- `weight_heuristic`: heuristic 가중치 (기본 1.0).
- `on_step`: 매 노드 expand 직후 호출되는 viz 콜백. None 이면 호출 안 함.
- 반환: 경로 list (start → goal). 미발견 시 `[]`.

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/05_a_star/a_star.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/05_a_star/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/05_a_star/record_gen.py
```
→ 2D viewer 에 격자 + 장애물 + start + goal. timeline scrubber `step` 으로 expand 진행 재생. 우측 status 패널에 step / visited / frontier 카운트 실시간.

> JSON 만 만들고 viewer 안 띄우려면 `--no-viewer` 옵션 추가.
> `--skip N` 으로 frame subsample (기본 10).
> `--weight W` 로 heuristic 가중치 조정 (기본 1.0, `--weight 10` = greedy).

Rerun viewer 로 재생 (chapter 전체 search records 멀티 로드):
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_search.py 01_Python_project_refactored/release/04_path_planning/05_a_star/
```

## 합격 기준 (`pytest` 통과)
알고리즘 형태 (정통 A* / heapq priority queue / 다른 변종) 는 제약 X — **behavioral spec** 만 본다.

1. **시작·끝 일치** — `path[0] == start`, `path[-1] == goal`
2. **장애물 회피** — 모든 path 노드가 obstacles 밖
3. **8-connectivity**
4. **admissible 최단성** — `weight_heuristic=1.0` 에서 path cost 가 reference 1.05× 이내
5. **on_step 콜백 호출** — 첫 호출의 current 가 start, 호출 횟수 ≥ path 길이

> trivial 구현 (heuristic 잘못 적용해 발산하거나 path 미발견) 은 임계값 초과로 차단.

## 힌트
- **자료 구조**:
  - `open_dict: dict[(x,y), (f_cost, g_cost, parent)]` — f = g + w·h
  - `closed: dict[(x,y), parent]`
- **Loop**:
  1. open_dict 비면 [] 반환 (미발견)
  2. `current = min(open_dict, key=lambda n: open_dict[n][0])` (f 최소)
  3. closed 로 옮김
  4. `on_step(current, set(open_dict.keys()))` 호출 (있으면)
  5. `current == goal` 이면 path 복원
  6. 아니면 `_ACTIONS` 이웃 갱신: `new_g = g_cur + ac`, `new_f = new_g + weight·euclid(child, goal)`
- **Heuristic 도우미**: `_euclid(a, b)` 가 이미 정의됨 (`math.hypot`).

## 게인/파라미터 튜닝 위치
이 과제는 알고리즘 + heuristic 가중치 하나. `record_gen.py --weight` 로 viewer 에서 실험 가능, test 의 weight=1.0 은 변경 X.

## 문제별 추가 제약
- **`map_astar.py`, `record_gen.py` 수정 금지** — 검증 환경.
- `simulator_search.py` 도 수정 X (chapter 4 공용).
- 순수 Python 구현 — numpy/scipy path-finder 라이브러리 사용 X.
