"""Smoke test for Informed RRT* + Dubins.

알고리즘 형태 자유. 인터페이스 + Dubins 호 위에서의 도달성·연속성·곡률 제약·
재현성으로 합격 판정.
"""
import math

from map_rrt_dubins import GOAL, GRID_SIZE, OBSTACLES, START
from rrt_dubins import informed_rrt_star

KAPPA = 1.0 / 3.0   # min turning radius R = 3 m
DS = 0.2
SEED = 0
GOAL_RANGE = 2.5


def _check_basic(path):
    """공통 합격 조건: start 일치, goal 근접, 모든 sample 충돌 없음, 연속성."""
    assert path, "경로 미발견"
    assert path[0] == START, f"path[0] {path[0]} != START {START}"
    end = path[-1]
    d = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1])
    assert d < GOAL_RANGE, f"path 끝 goal 거리 {d:.2f} > {GOAL_RANGE}"
    for x, y, _ in path:
        cell = (int(round(x)), int(round(y)))
        assert cell not in OBSTACLES, f"sample {cell} 가 obstacle 충돌"
    # segment 연속성 — sample 간 거리 ≤ 2·ds (Dubins 적분 step 이 ds, 다만 첫
    # segment 는 start state 다음 sample 까지 ds).
    for a, b in zip(path[:-1], path[1:], strict=False):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        assert d <= DS * 2.0 + 1e-6, (
            f"sample 간 거리 {d:.3f} > 2·ds — Dubins 적분 불연속")


def test_informed_rrt_star_dubins_reaches_goal():
    path = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE, kappa=KAPPA,
                             max_iter=1500, eta=6.0, goal_range=GOAL_RANGE,
                             search_radius=10.0, ds=DS, seed=SEED)
    _check_basic(path)


def test_deterministic_with_seed():
    """같은 seed → 같은 path. 재현성 보장."""
    kw = dict(kappa=KAPPA, max_iter=1500, eta=6.0, goal_range=GOAL_RANGE,
              search_radius=10.0, ds=DS, seed=42)
    p1 = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE, **kw)
    p2 = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE, **kw)
    assert p1 == p2, "같은 seed 에서 결과가 달라짐 — 재현성 깨짐"


def test_dubins_curvature_constraint():
    """Dubins 호 sample 의 yaw 변화 |dyaw| 가 ds·kappa 이하 — 곡률 제약."""
    path = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE, kappa=KAPPA,
                             max_iter=1500, eta=6.0, goal_range=GOAL_RANGE,
                             search_radius=10.0, ds=DS, seed=SEED)
    assert path
    for a, b in zip(path[:-1], path[1:], strict=False):
        dyaw = abs((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)
        assert dyaw <= DS * KAPPA + 1e-6, (
            f"yaw 변화 {dyaw:.4f} > ds·κ {DS*KAPPA:.4f} — 곡률 제약 위반")
