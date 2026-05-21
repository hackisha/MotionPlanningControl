"""Dijkstra — 8-connected grid 의 최단 경로 탐색.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import math
from typing import Callable

# 8-connected actions: (dx, dy, cost). 직진=1, 대각=√2.
_ACTIONS: list[tuple[int, int, float]] = [
    (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def dijkstra(
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacles: set[tuple[int, int]],
    on_step: Callable[
        [tuple[int, int], set[tuple[int, int]], float, float], None
    ] | None = None,
) -> list[tuple[int, int]]:
    """start 부터 goal 까지 8-connected 최단 path 탐색.

    Args:
        start, goal: (x, y) 격자 좌표.
        obstacles: 충돌 노드 set (O(1) 검사).
        on_step: 매 노드 expand 직후 호출되는 viz 콜백 —
                 `(current_node, open_set, g_cost, f_cost)`. open_set 은 그 시점의
                 open 키 set, g_cost 는 current 의 누적 비용. Dijkstra 는
                 heuristic 이 없어 f_cost = g_cost (동일값). None 이면 호출 안 함.

    Returns:
        path: [(x0, y0), ..., (xn, yn)] start → goal 순서. 미발견 시 [].
    """
    # TODO: Dijkstra 알고리즘으로 8-connected 최단 경로를 구하시오.
    # 힌트:
    #   - open_dict: {node: (g_cost, parent)} — 아직 expand 안 한 후보
    #   - closed:    {node: parent}            — 이미 expand 한 노드
    #   - 매 loop: open_dict 에서 g_cost 최소인 current 선택 → pop → closed 에 추가
    #     → on_step(current, set(open_dict.keys()), g_cost, g_cost) 호출 (있으면 —
    #       Dijkstra 는 heuristic 이 없어 f = g 라 g_cost 를 두 번 넘김)
    #     → goal 이면 closed 의 parent 체인으로 path 복원해 반환
    #     → 아니면 _ACTIONS 8 방향 child 검사 (obstacles / closed 제외 / cost 갱신)
    #   - open_dict 비면 [] 반환 (미발견)
    raise NotImplementedError
