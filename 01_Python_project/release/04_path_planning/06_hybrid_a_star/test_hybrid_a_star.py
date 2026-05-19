"""Hybrid A* regression — behavioral spec (requirements level).

알고리즘 형태 (linear search / heapq / 다른 bucket 전략) 는 제약 X.
인터페이스 + 경로의 도달성·장애물 회피·kinematic 일관성으로 합격 판정.
"""
import math

from hybrid_a_star import hybrid_a_star
from map_hybrid import GOAL, OBSTACLES, SPACE, START

R = 5.0
VX = 2.0
DT = 0.5
WEIGHT = 1.1
EPSILON_GOAL = 0.6


def test_path_reaches_goal():
    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R, vx=VX, dt=DT, weight=WEIGHT)
    assert path, "Hybrid A* 가 경로를 찾지 못함"
    end = path[-1]
    dist = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1])
    assert dist < EPSILON_GOAL, (
        f"path 끝 ({end[0]:.2f}, {end[1]:.2f}) 가 goal ({GOAL[0]}, {GOAL[1]}) 에서 "
        f"{dist:.2f} m 떨어짐 — epsilon_goal {EPSILON_GOAL} 초과")


def test_path_starts_at_start():
    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R, vx=VX, dt=DT, weight=WEIGHT)
    assert path
    s = path[0]
    dist = math.hypot(s[0] - START[0], s[1] - START[1])
    assert dist < 0.3, f"path[0] 가 START 와 {dist:.3f} m 차이"


def test_path_obstacle_free():
    """모든 path pose 가 어떤 obstacle 반경 밖."""
    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R, vx=VX, dt=DT, weight=WEIGHT)
    for pose in path:
        for ox, oy, orad in OBSTACLES:
            d2 = (pose[0] - ox) ** 2 + (pose[1] - oy) ** 2
            assert d2 >= orad ** 2 * 0.99, (
                f"path pose ({pose[0]:.2f}, {pose[1]:.2f}) 가 obstacle "
                f"({ox}, {oy}, r={orad}) 와 충돌 — d²={d2:.3f}")


def test_consecutive_poses_kinematically_consistent():
    """연속 pose 간 거리 ≈ vx·dt (motion primitive 의 이동 거리).

    원호 vs 직선이라 정확히 같진 않지만, |Δ| ≤ travel·1.05 + 작은 margin.
    """
    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R, vx=VX, dt=DT, weight=WEIGHT)
    travel = VX * DT  # 1.0 m
    for a, b in zip(path[:-1], path[1:], strict=False):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        assert d <= travel * 1.05 + 1e-6, (
            f"연속 pose 간 거리 {d:.3f} 가 motion primitive travel {travel} 초과 "
            f"— Hybrid A* expand 결과가 아닌 점이 끼어 있음")


def test_on_step_callback_invoked():
    """on_step 콜백 호출 횟수 ≥ path 길이, 첫 호출의 current 는 start 근방."""
    calls: list[tuple[tuple[float, float, float], int]] = []

    def cb(pose, frontier):
        calls.append((pose, len(frontier)))

    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R, vx=VX, dt=DT, weight=WEIGHT, on_step=cb)
    assert path, "callback 모드에서도 path 발견되어야 함"
    assert len(calls) >= len(path), (
        f"callback 호출 수 ({len(calls)}) < path 길이 ({len(path)})")
    first_pose = calls[0][0]
    d = math.hypot(first_pose[0] - START[0], first_pose[1] - START[1])
    assert d < 0.3, f"첫 expand 가 start 와 {d:.3f} m 차이"
