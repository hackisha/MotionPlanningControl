"""Hybrid A* — kinematic vehicle 의 연속 상태 공간 최단 경로 탐색.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

from typing import Callable

from vehicle_kinematics import (
    arc_collision,
    discretize_pose,
    euclid_xy,
    in_space,
    motion_primitives,
    vehicle_move,
)


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
    """start 에서 goal 까지 Hybrid A* 경로 탐색.

    Args:
        start, goal: (x, y, yaw) 연속 pose.
        space: (x_min, x_max, y_min, y_max) 탐색 영역.
        obstacles: (x, y, radius) 원형 장애물 리스트.
        R, vx, dt: motion primitive 파라미터 (최소 회전 반경 / 차속 / 시간 step).
        weight: heuristic 가중치 (1.0=admissible, >1=greedy).
        epsilon_goal: goal 도달 판정 거리 (xy euclidean).
        vehicle_radius: 차량 effective radius (m) — Minkowski-inflation 으로
            arc_collision 의 obstacle 반경에 더해 검사. 1.7 은 차체 + 안전계수.
        on_step: 매 expand 직후 호출되는 viz 콜백 —
            `(current_pose, parent_pose_or_None, open_set, g_cost, f_cost)`.
            start 노드 expand 시 parent_pose 는 None.
            g_cost 는 current 의 누적 비용, f_cost = g + weight·h.

    Returns:
        path: [(x, y, yaw), ...]. 미발견 시 [].
    """
    # TODO: Hybrid A* 알고리즘으로 kinematic 경로를 구하시오.
    # 힌트:
    #   - open_dict: {bucket_key: (f_cost, g_cost, pose, parent_key)}
    #     bucket_key = discretize_pose(pose) — (x, y, yaw) 양자화 → 같은 bucket = 같은 상태.
    #   - closed: {bucket_key: (pose, parent_key)}
    #   - actions = motion_primitives(R, vx, dt) → 5 가지 (yaw_rate, dt, cost)
    #
    # 매 loop:
    #   1. open_dict 비면 [] 반환 (미발견)
    #   2. f 최소 노드 선택: min(open_dict, key=lambda k: open_dict[k][0])
    #   3. open_dict.pop → closed[cur_key] = (pose_cur, parent_key)
    #   4. on_step 호출 (있으면) — parent_pose 는 closed[parent_key][0] (없으면 None):
    #        parent_pose = closed[parent_key][0] if parent_key is not None else None
    #        on_step(pose_cur, parent_pose, set(open_dict.keys()), g_cur, f_cur)
    #   5. euclid_xy(pose_cur, goal) < epsilon_goal → path 복원 후 반환
    #      (closed[key] 의 parent_key 따라가며 pose 누적, 뒤집어 반환)
    #   6. 아니면 actions 마다:
    #      - child_pose = vehicle_move(pose_cur, yaw_rate, dt, vx)
    #      - in_space(child_pose, space) 가 False 면 skip
    #      - arc_collision(pose_cur, yaw_rate, dt, vx, obstacles,
    #                      vehicle_radius=vehicle_radius) 면 skip
    #        (vehicle_radius>0 이면 차량 크기 + 안전계수까지 고려한 Minkowski check)
    #      - child_key = discretize_pose(child_pose); in closed → skip
    #      - new_g = g_cur + cost; new_f = new_g + weight·euclid_xy(child_pose, goal)
    #      - open_dict 에 같은 key 있는데 f 가 더 작거나 같으면 skip
    #      - open_dict[child_key] = (new_f, new_g, child_pose, cur_key)
    raise NotImplementedError
