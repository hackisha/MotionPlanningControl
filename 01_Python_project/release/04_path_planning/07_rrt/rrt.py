"""Rapidly-exploring Random Tree path planner."""
from __future__ import annotations

import math
import random
from collections.abc import Callable


def _euclid(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _steer(node_from: tuple[float, float], node_to: tuple[float, float],
           eta: float) -> tuple[float, float]:
    dx = node_to[0] - node_from[0]
    dy = node_to[1] - node_from[1]
    mag = math.hypot(dx, dy)
    if mag <= eta:
        return (float(node_to[0]), float(node_to[1]))
    return (node_from[0] + eta * dx / mag,
            node_from[1] + eta * dy / mag)


def _is_collision_free(node_from: tuple[float, float],
                       node_to: tuple[float, float],
                       obstacles: set[tuple[int, int]],
                       step: float = 0.3) -> bool:
    dx = node_to[0] - node_from[0]
    dy = node_to[1] - node_from[1]
    mag = math.hypot(dx, dy)
    if mag < 1e-9:
        return True
    n = max(1, int(math.ceil(mag / step)))
    for k in range(n + 1):
        t = k / n
        x = node_from[0] + t * dx
        y = node_from[1] + t * dy
        cell = (int(round(x)), int(round(y)))
        if cell in obstacles:
            return False
    return True


def _reconstruct(
    nodes: list[tuple[float, float]],
    parents: dict[int, int | None],
    end_idx: int,
) -> list[tuple[float, float]]:
    path: list[tuple[float, float]] = []
    idx: int | None = end_idx
    while idx is not None:
        path.append(nodes[idx])
        idx = parents[idx]
    path.reverse()
    return path


def rrt(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: set[tuple[int, int]],
    grid_size: int,
    max_iter: int = 2000,
    eta: float = 3.0,
    goal_sample_rate: float = 0.05,
    goal_range: float = 1.5,
    seed: int | None = 0,
    on_step: Callable[[tuple[float, float], tuple[float, float], int],
                      None] | None = None,
    dbg=None,
) -> list[tuple[float, float]]:
    """Grow a random tree and return the first branch near the goal."""
    rng = random.Random(seed)
    nodes: list[tuple[float, float]] = [start]
    parents: dict[int, int | None] = {0: None}

    for it in range(max_iter):
        sample = goal if rng.random() < goal_sample_rate else (
            rng.uniform(0.0, float(grid_size)),
            rng.uniform(0.0, float(grid_size)),
        )

        nearest_i = min(range(len(nodes)), key=lambda i: _euclid(nodes[i], sample))
        nearest = nodes[nearest_i]
        new_pt = _steer(nearest, sample, eta)

        rejected = False
        if _euclid(nearest, new_pt) < 1e-6:
            rejected = True
        elif not _is_collision_free(nearest, new_pt, obstacles):
            rejected = True

        if rejected:
            if dbg is not None:
                dbg.add(goal_dist=_euclid(nodes[-1], goal),
                        rejected=1.0, tree_size=float(len(nodes)))
            continue

        new_i = len(nodes)
        nodes.append(new_pt)
        parents[new_i] = nearest_i
        if on_step is not None:
            on_step(new_pt, nearest, it)

        if dbg is not None:
            dbg.add(goal_dist=_euclid(new_pt, goal),
                    rejected=0.0, tree_size=float(len(nodes)))

        if _euclid(new_pt, goal) < goal_range:
            return _reconstruct(nodes, parents, new_i)

    return []
