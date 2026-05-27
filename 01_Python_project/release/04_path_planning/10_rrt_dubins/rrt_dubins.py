"""Informed RRT* with Dubins steering."""
from __future__ import annotations

import math
import random
from collections.abc import Callable

from dubins import dubins_plan, truncate_path

State = tuple[float, float, float]
ConnectFn = Callable[[State, State], "tuple[list[State], float]"]
StepFn = Callable[[State, State, "list[State]", int], None]


def _euclid_xy(a: State, b: State) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_collision_free(samples: list[State],
                         obstacles: set[tuple[int, int]]) -> bool:
    for x, y, _ in samples:
        if (int(round(x)), int(round(y))) in obstacles:
            return False
    return True


def _sample_state(rng: random.Random, grid_size: int) -> State:
    return (rng.uniform(0.0, float(grid_size)),
            rng.uniform(0.0, float(grid_size)),
            rng.uniform(-math.pi, math.pi))


def _make_dubins_connect(kappa: float, ds: float) -> ConnectFn:
    def _connect(s1: State, s2: State) -> tuple[list[State], float]:
        return dubins_plan(s1, s2, kappa, ds=ds)
    return _connect


def _steer_to(
    s_from: State,
    s_to: State,
    connect_fn: ConnectFn,
    eta: float,
    ds: float,
) -> tuple[list[State], float]:
    full_path, full_len = connect_fn(s_from, s_to)
    if not full_path or full_len == math.inf:
        return [], 0.0
    if full_len <= eta:
        return full_path, full_len
    return truncate_path(full_path, ds, eta)


def _reconstruct_path(
    nodes: list[State],
    parents: dict[int, int | None],
    edge_samples: dict[int, list[State]],
    end_idx: int,
) -> list[State]:
    chain: list[int] = []
    idx: int | None = end_idx
    while idx is not None:
        chain.append(idx)
        idx = parents[idx]
    chain.reverse()

    path: list[State] = [nodes[chain[0]]]
    for child_idx in chain[1:]:
        path.extend(edge_samples[child_idx])
    return path


def _sample_in_ellipse(rng: random.Random,
                       x_start: State, x_goal: State,
                       c_best: float) -> tuple[float, float]:
    c_min = math.hypot(x_goal[0] - x_start[0], x_goal[1] - x_start[1])
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


def informed_rrt_star(
    start: State,
    goal: State,
    obstacles: set[tuple[int, int]],
    grid_size: int,
    kappa: float = 1.0 / 3.0,
    max_iter: int = 1500,
    eta: float = 6.0,
    goal_range: float = 4.0,
    goal_range_min: float = 1.5,
    eta_decay: float = 0.8,
    eta_min: float = 2.5,
    improve_eps: float = 0.3,
    search_radius: float = 10.0,
    goal_sample_rate: float = 0.05,
    ds: float = 0.2,
    seed: int | None = 0,
    on_step: StepFn | None = None,
    on_improve: Callable[[int, float, list[State]], None] | None = None,
    dbg=None,
) -> list[State]:
    """Return the best Dubins-feasible branch found within max_iter."""
    connect = _make_dubins_connect(kappa, ds)
    rng = random.Random(seed)

    nodes: list[State] = [start]
    parents: dict[int, int | None] = {0: None}
    children: dict[int, list[int]] = {0: []}
    cost: dict[int, float] = {0: 0.0}
    edge_cost: dict[int, float] = {0: 0.0}
    edge_samples: dict[int, list[State]] = {0: []}
    best_idx: int | None = None
    c_best = math.inf
    inform_round = 0

    def update_subtree(idx: int, delta: float) -> None:
        for child in children[idx]:
            cost[child] += delta
            update_subtree(child, delta)

    def is_ancestor(candidate: int, node_idx: int) -> bool:
        idx: int | None = node_idx
        while idx is not None:
            if idx == candidate:
                return True
            idx = parents[idx]
        return False

    for it in range(max_iter):
        eta_r = max(eta * (eta_decay ** inform_round), eta_min)
        goal_range_r = max(goal_range, goal_range_min)

        if rng.random() < goal_sample_rate:
            sample = goal
        elif math.isfinite(c_best):
            sx, sy = _sample_in_ellipse(rng, start, goal, c_best)
            sample = (sx, sy, rng.uniform(-math.pi, math.pi))
        else:
            sample = _sample_state(rng, grid_size)

        nearest_i = min(range(len(nodes)), key=lambda i: _euclid_xy(nodes[i], sample))
        steer_samples, steer_len = _steer_to(nodes[nearest_i], sample, connect, eta_r, ds)

        rejected = False
        if not steer_samples or steer_len < 1e-9:
            rejected = True
        elif not _path_collision_free(steer_samples, obstacles):
            rejected = True

        if rejected:
            if dbg is not None:
                dbg.add(goal_dist=_euclid_xy(nodes[-1], goal), rejected=1.0,
                        tree_size=float(len(nodes)), rewire_count=0.0,
                        inform_round=float(inform_round),
                        best_cost=0.0 if not math.isfinite(c_best) else c_best)
            continue

        new_pt = steer_samples[-1]
        near = [i for i, node in enumerate(nodes)
                if _euclid_xy(node, new_pt) <= search_radius]

        best_parent = nearest_i
        best_parent_samples = steer_samples
        best_edge_len = steer_len
        best_new_cost = cost[nearest_i] + steer_len

        for i in near:
            samples_i, len_i = connect(nodes[i], new_pt)
            if not samples_i or len_i == math.inf:
                continue
            if cost[i] + len_i >= best_new_cost:
                continue
            if not _path_collision_free(samples_i, obstacles):
                continue
            best_parent = i
            best_parent_samples = samples_i
            best_edge_len = len_i
            best_new_cost = cost[i] + len_i

        new_i = len(nodes)
        nodes.append(new_pt)
        parents[new_i] = best_parent
        children[new_i] = []
        children[best_parent].append(new_i)
        cost[new_i] = best_new_cost
        edge_cost[new_i] = best_edge_len
        edge_samples[new_i] = best_parent_samples
        if on_step is not None:
            on_step(new_pt, nodes[best_parent], best_parent_samples, it)

        rewire_count = 0
        for i in near:
            if i == best_parent or is_ancestor(i, new_i):
                continue
            samples_i, len_i = connect(new_pt, nodes[i])
            if not samples_i or len_i == math.inf:
                continue
            new_cost = cost[new_i] + len_i
            if new_cost + 1e-9 >= cost[i]:
                continue
            if not _path_collision_free(samples_i, obstacles):
                continue

            old_parent = parents[i]
            if old_parent is not None:
                children[old_parent].remove(i)
            parents[i] = new_i
            children[new_i].append(i)
            edge_samples[i] = samples_i
            edge_cost[i] = len_i
            delta = new_cost - cost[i]
            cost[i] = new_cost
            update_subtree(i, delta)
            rewire_count += 1
            if on_step is not None:
                on_step(nodes[i], new_pt, samples_i, it)

        goal_candidates = [
            i for i, node in enumerate(nodes)
            if _euclid_xy(node, goal) < goal_range_r
        ]
        if goal_candidates:
            cand_i = min(goal_candidates,
                         key=lambda i: cost[i] + _euclid_xy(nodes[i], goal))
            eff = cost[cand_i] + _euclid_xy(nodes[cand_i], goal)
            if eff < c_best - improve_eps:
                c_best = eff
                best_idx = cand_i
                inform_round += 1
                snap = _reconstruct_path(nodes, parents, edge_samples, best_idx)
                if on_improve is not None:
                    on_improve(it, c_best, snap)

        if dbg is not None:
            dbg.add(goal_dist=_euclid_xy(new_pt, goal), rejected=0.0,
                    tree_size=float(len(nodes)), rewire_count=float(rewire_count),
                    inform_round=float(inform_round),
                    best_cost=0.0 if not math.isfinite(c_best) else c_best)

    if best_idx is None:
        return []
    return _reconstruct_path(nodes, parents, edge_samples, best_idx)
