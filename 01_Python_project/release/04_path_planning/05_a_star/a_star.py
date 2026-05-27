"""A* search on an 8-connected grid."""
from __future__ import annotations

import math
from collections.abc import Callable

_ACTIONS: list[tuple[int, int, float]] = [
    (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _euclid(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct(
    closed: dict[tuple[int, int], tuple[int, int] | None],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    path: list[tuple[int, int]] = []
    node: tuple[int, int] | None = goal
    while node is not None:
        path.append(node)
        node = closed[node]
    path.reverse()
    return path


def a_star(
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacles: set[tuple[int, int]],
    weight_heuristic: float = 1.0,
    on_step: Callable[
        [tuple[int, int], set[tuple[int, int]], float, float], None
    ] | None = None,
) -> list[tuple[int, int]]:
    """Return an 8-connected path using f = g + weight*h."""
    h0 = weight_heuristic * _euclid(start, goal)
    open_dict: dict[tuple[int, int], tuple[float, float, tuple[int, int] | None]] = {
        start: (h0, 0.0, None),
    }
    closed: dict[tuple[int, int], tuple[int, int] | None] = {}

    while open_dict:
        current = min(open_dict, key=lambda node: open_dict[node][0])
        f_cur, g_cur, parent = open_dict.pop(current)
        if current in closed:
            continue

        closed[current] = parent
        if on_step is not None:
            on_step(current, set(open_dict.keys()), g_cur, f_cur)

        if current == goal:
            return _reconstruct(closed, goal)

        cx, cy = current
        for dx, dy, step_cost in _ACTIONS:
            child = (cx + dx, cy + dy)
            if child in obstacles or child in closed:
                continue

            new_g = g_cur + step_cost
            new_f = new_g + weight_heuristic * _euclid(child, goal)
            old = open_dict.get(child)
            if old is not None and old[0] <= new_f:
                continue
            open_dict[child] = (new_f, new_g, current)

    return []
