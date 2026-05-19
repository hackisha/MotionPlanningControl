"""Vehicle kinematics for Hybrid A* — motion primitives + 충돌 검사.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

함수:
- vehicle_move(pose, yaw_rate, dt, vx): kinematic 한 step.
  yaw_rate ≠ 0 일 때 원호 운동, =0 일 때 직진.
- motion_primitives(R, vx, dt): expand 시 시도할 5 가지 action.
- arc_collision(pose, yaw_rate, dt, vx, obstacles, n=20): n 등분 한 sweep 충돌 검사.
- in_space(pose, space): 탐색 공간 내부 검사.
- discretize_pose(pose): bucket key — closed/open dict 의 hash 키.
- euclid_xy(a, b): 2D Euclidean.
"""
from __future__ import annotations

import math

# 이산화 해상도 — 같은 bucket 의 노드는 '같은 상태' 로 취급 (closed list dedup).
POS_RES: float = 0.5            # 위치 격자 (m)
YAW_RES: float = math.pi / 8.0  # yaw 격자 (rad) — 22.5°


def vehicle_move(pose: tuple[float, float, float],
                 yaw_rate: float, dt: float, vx: float
                 ) -> tuple[float, float, float]:
    """pose 에서 (yaw_rate, dt) action 으로 한 step 이동한 새 pose 반환."""
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
    # yaw 를 [0, 2π) 로 정규화
    n_yaw = n_yaw % (2.0 * math.pi)
    return (nx, ny, n_yaw)


def motion_primitives(R: float, vx: float, dt: float
                      ) -> list[tuple[float, float, float]]:
    """5 가지 action: (yaw_rate, dt, cost). cost = vx·dt (이동 거리).

    R = 최소 회전 반경 → yaw_rate_max = vx / R.
    Actions: [Left full, Right full, Left half, Right half, Straight].
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
                  n_samples: int = 20) -> bool:
    """pose 에서 action(yaw_rate, dt) 의 sweep arc 가 어느 장애물과 충돌하면 True.

    n_samples 등분된 시점에 vehicle_move 로 pose 계산 후 모든 (ox, oy, or) 와 거리 검사.
    """
    if not obstacles:
        return False
    for k in range(n_samples + 1):
        t_k = dt * k / n_samples
        cx, cy, _ = vehicle_move(pose, yaw_rate, t_k, vx)
        for ox, oy, orad in obstacles:
            if (ox - cx) ** 2 + (oy - cy) ** 2 <= orad ** 2:
                return True
    return False


def in_space(pose: tuple[float, float, float],
             space: tuple[float, float, float, float]) -> bool:
    x, y, _ = pose
    x_min, x_max, y_min, y_max = space
    return x_min <= x <= x_max and y_min <= y <= y_max


def discretize_pose(pose: tuple[float, float, float]) -> tuple[int, int, int]:
    """pose → 정수 bucket key. POS_RES / YAW_RES 단위로 양자화."""
    x, y, yaw = pose
    yaw_n = yaw % (2.0 * math.pi)
    return (
        int(round(x / POS_RES)),
        int(round(y / POS_RES)),
        int(round(yaw_n / YAW_RES)) % int(round(2.0 * math.pi / YAW_RES)),
    )


def euclid_xy(a: tuple[float, float, float] | tuple[float, float],
              b: tuple[float, float, float] | tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
