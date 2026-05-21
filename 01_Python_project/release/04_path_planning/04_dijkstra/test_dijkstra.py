"""Dijkstra regression — behavioral spec (requirements level).

알고리즘 형태 (정통 Dijkstra / heapq priority queue / 다른 변종) 는 제약 X.
인터페이스 + 경로의 도달성·8-connectivity·최단성으로 합격 판정.
"""
import math

from dijkstra import dijkstra
from map_dijkstra import GOAL, OBSTACLES, START

# Reference 최적 cost (solution 실측): 약 139.62. 1.05× 까지 허용 → tie-breaking
# 차이 / heuristic 변종도 통과.
_OPTIMAL_COST_BOUND = 147.0


def _path_cost(path: list[tuple[int, int]]) -> float:
    cost = 0.0
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:], strict=False):
        cost += math.hypot(x2 - x1, y2 - y1)
    return cost


def test_path_starts_and_ends_correctly():
    """경로의 시작·끝 노드가 start / goal 과 일치."""
    path = dijkstra(START, GOAL, OBSTACLES)
    assert path, "path empty — 경로 미발견"
    assert path[0] == START, f"path[0] = {path[0]}, expected {START}"
    assert path[-1] == GOAL, f"path[-1] = {path[-1]}, expected {GOAL}"


def test_path_obstacle_free():
    """모든 path 노드가 장애물 set 밖."""
    path = dijkstra(START, GOAL, OBSTACLES)
    for node in path:
        assert node not in OBSTACLES, f"path 가 장애물 통과: {node}"


def test_path_8_connected():
    """연속 노드 간 Δx, Δy ∈ {-1, 0, 1} 이고 (0, 0) 아님."""
    path = dijkstra(START, GOAL, OBSTACLES)
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:], strict=False):
        dx, dy = x2 - x1, y2 - y1
        assert dx in (-1, 0, 1) and dy in (-1, 0, 1) and (dx, dy) != (0, 0), \
            f"비 8-connected 이동: ({x1},{y1}) → ({x2},{y2})"


def test_path_near_optimal():
    """경로 cost 가 optimal bound 이내."""
    path = dijkstra(START, GOAL, OBSTACLES)
    cost = _path_cost(path)
    assert cost < _OPTIMAL_COST_BOUND, (
        f"path cost {cost:.2f} 가 임계값 {_OPTIMAL_COST_BOUND} 초과 — "
        f"Dijkstra 가 최단 경로를 찾지 못함")


def test_on_step_callback_invoked():
    """on_step 콜백이 expand 횟수만큼 호출됨 (viz/디버깅 인터페이스 sanity).

    콜백 시그니처는 `(current, open_set, g_cost, f_cost)` — g_cost 는 누적 비용,
    Dijkstra 는 heuristic 이 없어 f_cost = g_cost.
    """
    calls: list[tuple] = []

    def cb(current, open_set, g_cost, f_cost) -> None:
        calls.append((current, len(open_set), g_cost, f_cost))

    path = dijkstra(START, GOAL, OBSTACLES, on_step=cb)
    assert path, "callback 모드에서도 path 발견되어야 함"
    assert len(calls) >= len(path), (
        f"callback 호출 수 ({len(calls)}) 가 path 길이 ({len(path)}) 보다 작음")
    # 첫 호출의 current 는 start, 그 g_cost 는 0
    assert calls[0][0] == START, f"첫 expand 는 start 여야 함: got {calls[0][0]}"
    assert calls[0][2] == 0.0, f"start 의 g_cost 는 0 이어야 함: got {calls[0][2]}"
    # Dijkstra: heuristic 이 없으므로 f_cost == g_cost
    for current, _, g_cost, f_cost in calls:
        assert f_cost == g_cost, (
            f"Dijkstra 는 f = g 여야 함 — {current}: g={g_cost}, f={f_cost}")
