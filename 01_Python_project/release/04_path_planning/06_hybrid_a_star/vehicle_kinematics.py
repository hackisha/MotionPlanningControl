"""Vehicle kinematics + bookkeeping helpers for Hybrid A*.

이 모듈은 **fixture** 입니다 — Hybrid A* 알고리즘을 구현할 때 학생이 직접 사용
하라고 미리 제공된 도구 모음입니다. 수정하지 마세요 (검증 환경의 일부).

----------------------------------------------------------------------
어떤 도구가 어디서 쓰이나
----------------------------------------------------------------------

자전거 운동학과 격자 / 충돌 / 거리 계산을 분리해 두면, 학생은
*search 알고리즘 자체* (open / closed dict 관리, f-cost 선택, parent 추적,
path 복원) 에만 집중하면 됩니다. 아래 함수들은 hybrid_a_star.py 의 루프 안에서
**적극적으로 import 해서 호출** 하세요.

다음 mapping 이 hybrid_a_star.py 의 한 expand step 의 흐름을 그대로 따라갑니다:

    1) motion_primitives(R, vx, dt)
         → 5 개의 action (yaw_rate, dt, cost). 매 expand 에서 시도할 후보들.

    2) for (yaw_rate, dt_k, cost) in actions:
         child_pose = vehicle_move(pose_cur, yaw_rate, dt_k, vx)
             → kinematic 한 step 후의 새 pose. 직선/원호 둘 다 처리.

         if not in_space(child_pose, space):  continue
             → 탐색 공간 밖이면 후보 탈락.

         if arc_collision(pose_cur, yaw_rate, dt_k, vx, obstacles,
                          vehicle_radius=vehicle_radius):
             continue
             → action sweep arc 중 한 점이라도 장애물 충돌이면 탈락.

         child_key = discretize_pose(child_pose)
             → open / closed dict 의 key. 같은 bucket = 같은 상태.

         if child_key in closed:  continue
             → 이미 expand 된 상태면 skip.

         new_f = new_g + weight * euclid_xy(child_pose, goal)
             → heuristic = goal 까지의 2D 직선 거리.

       (open dict 갱신은 알고리즘 책임 — 이 fixture 의 영역 밖.)

POS_RES / YAW_RES 두 상수는 discretize_pose 의 양자화 해상도. 학생이 직접 만질
일은 없지만, 같은 (x, y, yaw) 셀의 노드들이 어떻게 하나의 bucket 으로 묶이는지
이해할 때 참고하세요.
"""
from __future__ import annotations

import math

# ── 이산화 해상도 ──────────────────────────────────────────────────────
# 같은 bucket 의 노드는 '같은 상태' 로 취급 (open/closed dict dedup).
# POS_RES 가 작을수록 정밀 탐색 / 메모리 ↑, 클수록 빠른 탐색 / 정밀도 ↓.
POS_RES: float = 0.5            # 위치 격자 (m) — 0.5m 단위로 같은 bucket
YAW_RES: float = math.pi / 8.0  # yaw 격자 (rad) — 22.5° 단위로 같은 bucket


def vehicle_move(pose: tuple[float, float, float],
                 yaw_rate: float, dt: float, vx: float
                 ) -> tuple[float, float, float]:
    """Kinematic bicycle 의 한 step 적분 — `pose` 에서 `(yaw_rate, dt)` action 으로
    이동한 새 pose `(nx, ny, n_yaw)` 반환.

    사용처: Hybrid A* 의 expand 에서 child pose 계산.

        child_pose = vehicle_move(pose_cur, yaw_rate, dt_k, vx)

    수식:
      yaw_rate ≠ 0  →  signed turn radius R_signed = vx / yaw_rate
                       (yaw_rate > 0 : left turn, < 0 : right turn) 의 원호.
      yaw_rate = 0  →  vx·dt 만큼 직진.

    Args:
        pose: (x, y, yaw) — 현재 pose. yaw 단위 rad.
        yaw_rate: 각속도 (rad/s). 부호로 좌/우 회전, 0 이면 직진.
        dt: 적분 구간 (s).
        vx: 종방향 속도 (m/s).

    Returns:
        (nx, ny, n_yaw) — 새 pose. n_yaw 는 `[0, 2π)` 로 정규화.
    """
    x, y, yaw = pose
    if abs(yaw_rate) > 1e-9:
        R_signed = vx / yaw_rate
        d_yaw = yaw_rate * dt
        nx = x + R_signed * (math.sin(yaw + d_yaw) - math.sin(yaw))
        ny = y - R_signed * (math.cos(yaw + d_yaw) - math.cos(yaw))
        n_yaw = yaw + d_yaw
    else:
        nx = x + vx * dt * math.cos(yaw)
        ny = y + vx * dt * math.sin(yaw)
        n_yaw = yaw
    # yaw 를 [0, 2π) 로 정규화 → discretize_pose 와 호환.
    n_yaw = n_yaw % (2.0 * math.pi)
    return (nx, ny, n_yaw)


def motion_primitives(R: float, vx: float, dt: float
                      ) -> list[tuple[float, float, float]]:
    """Expand 시 시도할 5 가지 action 후보 반환 — `(yaw_rate, dt, cost)` 튜플 list.

    사용처: Hybrid A* 의 main loop 진입 직전에 한 번 만들어 두고, 매 expand 마다
    이 list 를 순회.

        actions = motion_primitives(R, vx, dt)
        ...
        for yaw_rate, dt_k, cost in actions:
            child_pose = vehicle_move(pose_cur, yaw_rate, dt_k, vx)
            ...

    5 가지 후보:
        - (+yaw_rate_max, dt, travel)   : 최대 좌회전 원호
        - (-yaw_rate_max, dt, travel)   : 최대 우회전 원호
        - (+yaw_rate_max/2, dt, travel) : 절반 좌회전 원호
        - (-yaw_rate_max/2, dt, travel) : 절반 우회전 원호
        - (0, dt, travel)               : 직진

    Args:
        R: 최소 회전 반경 (m) → yaw_rate_max = vx / R.
        vx: 종방향 속도 (m/s).
        dt: 한 step 의 시간 (s).

    Returns:
        5-원소 list. 각 원소의 `cost = vx·dt` (이동 거리, g-cost 증분).
    """
    yaw_rate_max = vx / R
    travel = vx * dt
    return [
        (yaw_rate_max, dt, travel),
        (-yaw_rate_max, dt, travel),
        (yaw_rate_max / 2.0, dt, travel),
        (-yaw_rate_max / 2.0, dt, travel),
        (0.0, dt, travel),
    ]


def arc_collision(pose: tuple[float, float, float],
                  yaw_rate: float, dt: float, vx: float,
                  obstacles: list[tuple[float, float, float]],
                  n_samples: int = 20,
                  vehicle_radius: float = 0.0) -> bool:
    """Sweep arc 충돌 검사 — `pose` 에서 `(yaw_rate, dt)` action 으로 움직일 때
    그 궤적 위의 어느 sample 점이라도 장애물과 충돌하면 `True`.

    사용처: Hybrid A* 의 expand 에서 후보 action 의 안전성 검증.

        if arc_collision(pose_cur, yaw_rate, dt_k, vx, obstacles,
                         vehicle_radius=vehicle_radius):
            continue  # 충돌이라 이 action 은 폐기

    원리: action 전체 dt 구간을 `n_samples + 1` 점으로 등분해, 각 시점의 차량
    중심 위치를 `vehicle_move` 로 계산한 뒤 모든 obstacle disk 와 거리 비교.
    한 점이라도 disk 안에 있으면 충돌.

    `vehicle_radius > 0` 이면 Minkowski-inflation: 차량을 점이 아닌 원으로
    근사해, 충돌 판정에 `orad + vehicle_radius` 의 effective 반경을 사용.
    → 차체 크기 + 안전계수 까지 고려한 보수적 회피.

    Args:
        pose: 현재 pose (x, y, yaw) — action 의 시작점.
        yaw_rate, dt, vx: 검사할 action 의 파라미터 (`vehicle_move` 와 동일).
        obstacles: `(ox, oy, orad)` 튜플 list. 빈 list 면 항상 False.
        n_samples: arc 위 sample 점 개수 (기본 20). 크면 정밀도 ↑ / 비용 ↑.
        vehicle_radius: 차량 effective radius (m). 0 = 점 충돌 검사.

    Returns:
        충돌하면 True, 아니면 False.
    """
    if not obstacles:
        return False
    for k in range(n_samples + 1):
        t_k = dt * k / n_samples
        cx, cy, _ = vehicle_move(pose, yaw_rate, t_k, vx)
        for ox, oy, orad in obstacles:
            eff_r = orad + vehicle_radius
            if (ox - cx) ** 2 + (oy - cy) ** 2 <= eff_r ** 2:
                return True
    return False


def in_space(pose: tuple[float, float, float],
             space: tuple[float, float, float, float]) -> bool:
    """탐색 공간 (axis-aligned rect) 안에 있는지 검사.

    사용처: expand 직후 새 child pose 가 search bound 를 벗어났는지 확인.

        if not in_space(child_pose, space):
            continue  # 공간 밖이면 폐기

    Args:
        pose: (x, y, yaw). yaw 는 사용 안 함.
        space: (x_min, x_max, y_min, y_max).

    Returns:
        `x_min ≤ x ≤ x_max` 이고 `y_min ≤ y ≤ y_max` 면 True.
    """
    x, y, _ = pose
    x_min, x_max, y_min, y_max = space
    return x_min <= x <= x_max and y_min <= y <= y_max


def discretize_pose(pose: tuple[float, float, float]) -> tuple[int, int, int]:
    """연속 pose `(x, y, yaw)` → 정수 bucket key `(ix, iy, iyaw)`.

    사용처: open / closed dict 의 key. 같은 bucket = 같은 상태로 dedup.

        cur_key   = discretize_pose(pose_cur)
        child_key = discretize_pose(child_pose)
        if child_key in closed:   # 이미 expand 된 상태
            continue
        open_dict[child_key] = (f, g, child_pose, cur_key)

    양자화 해상도:
        - `POS_RES = 0.5 m` (모듈 상수)  → x, y 0.5 m 단위로 묶음
        - `YAW_RES = π/8 (22.5°)`        → yaw 22.5° 단위로 묶음
    yaw 는 modular — 0 과 2π 가 같은 bucket 이 되도록 wrap.

    Args:
        pose: (x, y, yaw) — yaw 단위 rad.

    Returns:
        `(ix, iy, iyaw)` 정수 튜플. dict / set key 로 사용 가능.
    """
    x, y, yaw = pose
    yaw_n = yaw % (2.0 * math.pi)
    return (
        int(round(x / POS_RES)),
        int(round(y / POS_RES)),
        int(round(yaw_n / YAW_RES)) % int(round(2.0 * math.pi / YAW_RES)),
    )


def euclid_xy(a: tuple[float, float, float] | tuple[float, float],
              b: tuple[float, float, float] | tuple[float, float]) -> float:
    """2D Euclidean 거리 (yaw 무시) — heuristic / goal 거리 검사용.

    사용처:
      - Heuristic `h(n) = euclid_xy(child_pose, goal)` (A* 의 admissible 추정).
      - Goal 도달 판정 `euclid_xy(pose_cur, goal) < epsilon_goal`.

    `a`, `b` 는 2 원소 또는 3 원소 튜플 — 앞 두 좌표만 사용.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])
