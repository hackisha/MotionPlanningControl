"""A* — 8-connected grid 의 heuristic 기반 최단 경로 탐색.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import math
from typing import Callable

_ACTIONS: list[tuple[int, int, float]] = [
    (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _euclid(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def a_star(
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacles: set[tuple[int, int]],
    weight_heuristic: float = 1.0,
    on_step: Callable[
        [tuple[int, int], set[tuple[int, int]], float, float], None
    ] | None = None,
) -> list[tuple[int, int]]:
    """start 부터 goal 까지 8-connected A* 경로 탐색.

    Args:
        start, goal: (x, y) 격자 좌표.
        obstacles: 충돌 노드 set.
        weight_heuristic: heuristic 가중치. 1.0 = admissible (최적). >1 = greedy bias.
        on_step: 매 노드 expand 직후 호출되는 viz 콜백 —
                 `(current, open_set, g_cost, f_cost)`. g_cost 는 current 의
                 누적 비용, f_cost = g + weight·h.

    Returns:
        path: [(x0, y0), ..., (xn, yn)] start → goal. 미발견 시 [].
    """
    # TODO: A* 알고리즘으로 8-connected heuristic 기반 최단 경로를 구하시오.
    # 힌트:
    #   - open_dict: {node: (f_cost, g_cost, parent)} — f = g + weight·h
    #     h(node) = _euclid(node, goal)
    #   - closed: {node: parent}
    #   - 매 loop: open_dict 에서 f 최소인 current 선택 → pop → closed 추가
    #     → on_step(current, set(open_dict.keys()), g_cost, f_cost) 호출 (있으면)
    #     → goal 이면 closed 의 parent 체인으로 path 복원 반환
    #     → 아니면 _ACTIONS 8 방향 child 검사:
    #         child in obstacles or closed → skip
    #         new_g = g_cur + action_cost
    #         new_f = new_g + weight_heuristic · _euclid(child, goal)
    #         기존 open 의 f 가 더 작거나 같으면 skip, 아니면 갱신
    #   - open_dict 비면 [] 반환
    raise NotImplementedError
