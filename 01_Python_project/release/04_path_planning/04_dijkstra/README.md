# 과제 — Dijkstra (8-connected grid 최단 경로)

## 목표
60×60 격자 미로에서 start `(5, 5)` 부터 goal `(55, 5)` 까지의 8-connected 최단 경로를 Dijkstra 알고리즘으로 탐색한다. 단계별 expansion 과정을 Rerun 2D viewer 로 실시간 재생.

> 본 과제는 **차량 제어 없음**. 격자 탐색 알고리즘에만 집중 — 결과는 2D viewer 에서 노드 expansion 과 최종 경로로만 본다.

## Map 구조
- 외곽 border (x=0,60 / y=0,60): 완전 폐쇄
- 중앙 vertical wall (x=30, y=0..50): 좌·우 분리. y=51..59 만 개방
- y=20 horizontal wall: x=0..20 ∪ x=40..60 폐쇄 (x=21..39 개방)
- y=35 horizontal wall: x=10..50 폐쇄 (x=0..9 ∪ x=51..59 개방)

start 와 goal 모두 y=5 (벽 아래쪽) — 직진 경로 없음. 중앙 벽 위 (y≥51) 로 우회해야 함.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
def dijkstra(
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacles: set[tuple[int, int]],
    on_step: Callable[[tuple[int, int], set[tuple[int, int]]], None] | None = None,
) -> list[tuple[int, int]]
```

- `obstacles`: O(1) 검사를 위해 set.
- `on_step`: 매 노드 expand 직후 호출되는 viz 콜백 — `(current_node, frontier)` 로 시각화 데이터 수집. None 이면 호출 안 함 (test 의 일부 호출에서 무시).
- 반환: 경로 list (start → goal 순서). 미발견 시 `[]`.

## 8-connected actions
직진 비용 1, 대각 비용 √2. 이 module 의 `_ACTIONS` 상수에 이미 정의됨.

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/04_dijkstra/dijkstra.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트:
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/04_dijkstra/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/04_dijkstra/record_gen.py
```
→ Rerun 2D viewer 에 격자 + 장애물 (검정) + start (파랑) + goal (빨강) 표시. 좌측 timeline scrubber `step` 슬라이드 → 매 step 의 current (주황), 누적 visited (옅은 노랑), 현재 frontier (초록) 가 갱신됨. 마지막 step 후 최종 path (마젠타) 가 그려진다.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.
> Frame 수가 너무 많아 scrubber 가 답답하면 `--skip N` 으로 N step 마다 frame 1 개로 묶을 수 있음 (기본 10, `--skip 1` = subsample 없음). visited 누적은 batch 내 모든 expand 노드를 한꺼번에 반영해 정확.

Rerun viewer 로 재생 (chapter 전체 search records 멀티 로드):
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_search.py 01_Python_project_refactored/release/04_path_planning/04_dijkstra/
```

> **`simulator_search.py` 는 chapter 의 search 알고리즘 전용** — 인자 없이 실행하면 `04_path_planning/` 하위 모든 search record (kind=='search') 를 한 viewer 에 멀티 로드. 01~03 은 vehicle 시뮬레이션이라 `simulator_path_planning.py` 를 사용.

## 합격 기준 (`pytest` 통과)
알고리즘 형태 (정통 Dijkstra / heapq priority queue / 다른 변종) 는 제약 X — **behavioral spec** 만 본다.

1. **시작·끝 일치** — `path[0] == start`, `path[-1] == goal`
2. **장애물 회피** — 모든 path 노드가 obstacles set 밖
3. **8-connectivity** — 연속 노드 간 Δx, Δy ∈ {-1, 0, 1} (이동량 0 아님)
4. **최단 cost** — path 총 cost (`Σ √(Δx² + Δy²)`) 가 reference 1.05× 이내
5. **on_step 콜백 호출** — 콜백 전달 시 첫 호출의 current 가 start, 호출 횟수 ≥ path 길이

> trivial 구현 (BFS step-수 최소화) 은 8-connected 가중치 cost 가 다르기 때문에 4 번에서 차단될 수 있음.

## 힌트
- **자료 구조**:
  - `open_dict: dict[(x,y), (g_cost, parent)]` — 후보 노드들. `min(open_dict, key=lambda n: open_dict[n][0])` 으로 매 step 최소 cost 노드 선택.
  - `closed: dict[(x,y), parent | None]` — 이미 expand 한 노드들.
- **Loop**:
  1. open_dict 비면 미발견 종료 → 빈 list 반환.
  2. current = open_dict 의 최소 cost 노드. closed 로 옮김.
  3. `on_step(current, set(open_dict.keys()))` 호출 (있으면).
  4. `current == goal` 이면 parent 체인 따라 거꾸로 path 복원, 뒤집어서 반환.
  5. 아니면 `_ACTIONS` 8 방향에 대해 child 후보 만들기 → obstacles / closed 검사 → cost 갱신 (기존 < 새 cost 면 skip).
- **Path 복원**: `node = current; while node is not None: path.append(node); node = closed[node]; return path[::-1]`

## 게인/파라미터 튜닝 위치
이 과제는 알고리즘만 — 튜닝 파라미터 없음. `record_gen.py` 도 시각화 viewer 만 띄우면 끝 (게인 변경 X).

## 문제별 추가 제약
- **`map_data.py`, `record_gen.py` 수정 금지** — 검증 환경.
- `chapter 4` 의 `simulator_search.py` 도 수정 X.
- 알고리즘은 **순수 Python 으로** 구현 (numpy / scipy 의 path-finder 라이브러리 사용 X — 학습 목적 위반).
