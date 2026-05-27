"""Hybrid A* in continuous vehicle-pose space."""
from __future__ import annotations

from collections.abc import Callable

from vehicle_kinematics import (
    arc_collision,
    discretize_pose,
    euclid_xy,
    in_space,
    motion_primitives,
    vehicle_move,
)


def _reconstruct(
    closed: dict[
        tuple[int, int, int],
        tuple[tuple[float, float, float], tuple[int, int, int] | None],
    ],
    end_key: tuple[int, int, int],
) -> list[tuple[float, float, float]]:
    path: list[tuple[float, float, float]] = []
    key: tuple[int, int, int] | None = end_key
    while key is not None:
        pose, parent_key = closed[key]
        path.append(pose)
        key = parent_key
    path.reverse()
    return path


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
    vehicle_radius: float = 1.7,
    on_step: Callable[[tuple[float, float, float],
                       tuple[float, float, float] | None,
                       set[tuple[int, int, int]], float, float],
                      None] | None = None,
) -> list[tuple[float, float, float]]:
    """Return a kinematically feasible path from start to goal."""
    actions = motion_primitives(R, vx, dt)
    start_key = discretize_pose(start)
    f0 = weight * euclid_xy(start, goal)
    open_dict: dict[
        tuple[int, int, int],
        tuple[float, float, tuple[float, float, float], tuple[int, int, int] | None],
    ] = {start_key: (f0, 0.0, start, None)}
    closed: dict[
        tuple[int, int, int],
        tuple[tuple[float, float, float], tuple[int, int, int] | None],
    ] = {}

    while open_dict:
        cur_key = min(open_dict, key=lambda key: open_dict[key][0])
        f_cur, g_cur, pose_cur, parent_key = open_dict.pop(cur_key)
        if cur_key in closed:
            continue

        closed[cur_key] = (pose_cur, parent_key)
        parent_pose = closed[parent_key][0] if parent_key is not None else None
        if on_step is not None:
            on_step(pose_cur, parent_pose, set(open_dict.keys()), g_cur, f_cur)

        if euclid_xy(pose_cur, goal) < epsilon_goal:
            return _reconstruct(closed, cur_key)

        for yaw_rate, action_dt, action_cost in actions:
            child_pose = vehicle_move(pose_cur, yaw_rate, action_dt, vx)
            if not in_space(child_pose, space):
                continue
            if arc_collision(
                pose_cur, yaw_rate, action_dt, vx, obstacles,
                vehicle_radius=vehicle_radius,
            ):
                continue

            child_key = discretize_pose(child_pose)
            if child_key in closed:
                continue

            new_g = g_cur + action_cost
            new_f = new_g + weight * euclid_xy(child_pose, goal)
            old = open_dict.get(child_key)
            if old is not None and old[0] <= new_f:
                continue
            open_dict[child_key] = (new_f, new_g, child_pose, cur_key)

    return []
