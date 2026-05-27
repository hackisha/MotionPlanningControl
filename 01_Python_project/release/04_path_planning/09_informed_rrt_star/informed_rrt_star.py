"""Informed RRT* path planner in 2D."""
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
                       step: float = 0.15) -> bool:
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
        if (int(round(x)), int(round(y))) in obstacles:
            return False
    return True


def _path_length(path: list[tuple[float, float]]) -> float:
    return sum(_euclid(path[i], path[i + 1]) for i in range(len(path) - 1))


def _sample_in_ellipse(rng: random.Random,
                       x_start: tuple[float, float],
                       x_goal: tuple[float, float],
                       c_best: float) -> tuple[float, float]:
    c_min = _euclid(x_start, x_goal)
    cx = 0.5 * (x_start[0] + x_goal[0])
    cy = 0.5 * (x_start[1] + x_goal[1])
    theta = math.atan2(x_goal[1] - x_start[1], x_goal[0] - x_start[0])
    a = 0.5 * c_best
    b = 0.5 * math.sqrt(max(c_best * c_best - c_min * c_min, 0.0))
    rad = math.sqrt(rng.random())
    phi = 2.0 * math.pi * rng.random()
    ex = a * rad * math.cos(phi)
    ey = b * rad * math.sin(phi)
    return (cx + ex * math.cos(theta) - ey * math.sin(theta),
            cy + ex * math.sin(theta) + ey * math.cos(theta))


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


def informed_rrt_star(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: set[tuple[int, int]],
    grid_size: int,
    max_iter: int = 900,
    eta: float = 6.0,
    goal_range: float = 8.0,
    goal_range_min: float = 1.5,
    eta_decay: float = 0.72,
    eta_min: float = 2.0,
    goal_decay: float = 1.0,
    improve_eps: float = 0.3,
    search_radius: float = 8.0,
    goal_sample_rate: float = 0.05,
    seed: int | None = 0,
    on_step: Callable[[tuple[float, float], tuple[float, float], int],
                      None] | None = None,
    dbg=None,
) -> list[tuple[float, float]]:
    """Run RRT* for the full budget and focus samples after a first solution."""
    rng = random.Random(seed)
    nodes: list[tuple[float, float]] = [start]
    parents: dict[int, int | None] = {0: None}
    children: dict[int, list[int]] = {0: []}
    cost: dict[int, float] = {0: 0.0}
    c_best = math.inf
    best_path: list[tuple[float, float]] = []
    inform_round = 0

    def update_subtree(idx: int, delta: float) -> None:
        for child in children[idx]:
            cost[child] += delta
            update_subtree(child, delta)

    for it in range(max_iter):
        eta_r = max(eta * (eta_decay ** inform_round), eta_min)
        goal_range_r = max(goal_range * (goal_decay ** inform_round), goal_range_min)

        if rng.random() < goal_sample_rate:
            sample = goal
        elif math.isfinite(c_best):
            sample = _sample_in_ellipse(rng, start, goal, c_best)
        else:
            sample = (rng.uniform(0.0, float(grid_size)),
                      rng.uniform(0.0, float(grid_size)))

        nearest_i = min(range(len(nodes)), key=lambda i: _euclid(nodes[i], sample))
        nearest = nodes[nearest_i]
        new_pt = _steer(nearest, sample, eta_r)
        near = [i for i, node in enumerate(nodes)
                if _euclid(node, new_pt) <= search_radius]

        rejected = False
        if _euclid(nearest, new_pt) < 1e-6:
            rejected = True
        elif not _is_collision_free(nearest, new_pt, obstacles):
            rejected = True

        if rejected:
            if dbg is not None:
                dbg.add(goal_dist=_euclid(nodes[-1], goal), rejected=1.0,
                        tree_size=float(len(nodes)), rewire_count=0.0,
                        inform_round=float(inform_round),
                        best_cost=0.0 if not math.isfinite(c_best) else c_best)
            continue

        best_i = nearest_i
        best_cost = cost[nearest_i] + _euclid(nearest, new_pt)
        for i in near:
            edge_cost = _euclid(nodes[i], new_pt)
            cand_cost = cost[i] + edge_cost
            if cand_cost < best_cost and _is_collision_free(nodes[i], new_pt, obstacles):
                best_i = i
                best_cost = cand_cost

        new_i = len(nodes)
        nodes.append(new_pt)
        parents[new_i] = best_i
        children[new_i] = []
        children[best_i].append(new_i)
        cost[new_i] = best_cost
        if on_step is not None:
            on_step(new_pt, nodes[best_i], it)

        rewire_count = 0
        for i in near:
            if i == best_i:
                continue
            new_cost = cost[new_i] + _euclid(new_pt, nodes[i])
            if new_cost + 1e-9 >= cost[i]:
                continue
            if not _is_collision_free(new_pt, nodes[i], obstacles):
                continue
            old_parent = parents[i]
            if old_parent is not None:
                children[old_parent].remove(i)
            parents[i] = new_i
            children[new_i].append(i)
            delta = new_cost - cost[i]
            cost[i] = new_cost
            update_subtree(i, delta)
            rewire_count += 1
            if on_step is not None:
                on_step(nodes[i], new_pt, it)

        candidate_indices = [
            i for i, node in enumerate(nodes)
            if _euclid(node, goal) < goal_range_r
        ]
        if candidate_indices:
            cand_i = min(candidate_indices,
                         key=lambda i: cost[i] + _euclid(nodes[i], goal))
            eff = cost[cand_i] + _euclid(nodes[cand_i], goal)
            if eff < c_best - improve_eps:
                c_best = eff
                best_path = _reconstruct(nodes, parents, cand_i)
                inform_round += 1

        if dbg is not None:
            dbg.add(goal_dist=_euclid(new_pt, goal), rejected=0.0,
                    tree_size=float(len(nodes)), rewire_count=float(rewire_count),
                    inform_round=float(inform_round),
                    best_cost=0.0 if not math.isfinite(c_best) else c_best)

    return best_path
