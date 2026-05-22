"""Informed RRT* regression — behavioral spec (requirements level).

알고리즘 형태 자유 (sampling 전략 / 자료 구조 제약 X).
인터페이스 + 경로의 도달성·연속성·연결 반경·재현성·budget 단조성으로 합격 판정.
"""
import math

from informed_rrt_star import informed_rrt_star
from map_informed import GOAL, GRID_SIZE, OBSTACLES, START

ETA = 6.0
SEED = 0
MAX_ITER = 900
MAX_ITER_SHORT = 200  # budget 단조성 비교용 — 첫 경로는 찾되 덜 다듬어진 budget.
GOAL_RANGE = 8.0
SEARCH_RADIUS = 8.0


def _run(max_iter=MAX_ITER, seed=SEED):
    return informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE,
                             max_iter=max_iter, eta=ETA, goal_range=GOAL_RANGE,
                             search_radius=SEARCH_RADIUS, seed=seed)


def _path_length(path):
    return sum(math.hypot(path[i + 1][0] - path[i][0],
                          path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


def test_path_starts_at_start():
    path = _run()
    assert path, "Informed RRT* 가 경로를 찾지 못함 (seed/max_iter 확인)"
    assert path[0] == START, f"path[0] = {path[0]}, expected {START}"


def test_path_ends_near_goal():
    path = _run()
    assert path
    end = path[-1]
    d = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1])
    assert d < GOAL_RANGE, (
        f"path 끝 ({end[0]:.2f}, {end[1]:.2f}) 가 goal 에서 {d:.2f} m 떨어짐 "
        f"— goal_range {GOAL_RANGE} 초과")


def test_path_segments_collision_free():
    """연속 두 점을 잇는 선분의 fine-sample 어느 cell 도 obstacle 가 아님."""
    path = _run()
    assert path
    for a, b in zip(path[:-1], path[1:], strict=False):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        mag = math.hypot(dx, dy)
        n = max(1, int(math.ceil(mag / 0.2)))
        for k in range(n + 1):
            t = k / n
            x = a[0] + t * dx
            y = a[1] + t * dy
            cell = (int(round(x)), int(round(y)))
            assert cell not in OBSTACLES, (
                f"segment {a} → {b} 의 sample cell {cell} 가 obstacle 충돌")


def test_path_segment_length_within_radius():
    """연속 두 점 거리 ≤ search_radius + tol (choose-parent·rewire 연결 반경)."""
    path = _run()
    assert path
    for a, b in zip(path[:-1], path[1:], strict=False):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        assert d <= SEARCH_RADIUS + 1e-6, (
            f"segment 길이 {d:.3f} > search_radius {SEARCH_RADIUS} — 연결 반경 위반")


def test_on_step_callback_invoked():
    """on_step 콜백 호출 수 ≥ path edge 수, 첫 호출 parent 가 start.

    iteration 은 단일 loop 의 연속 카운터 — 단조 증가.
    """
    calls: list[tuple[tuple[float, float], tuple[float, float], int]] = []

    def cb(child, parent, iteration):
        calls.append((child, parent, iteration))

    path = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE,
                             max_iter=MAX_ITER, eta=ETA, goal_range=GOAL_RANGE,
                             search_radius=SEARCH_RADIUS, seed=SEED, on_step=cb)
    assert path
    assert len(calls) >= len(path) - 1, (
        f"callback 호출 수 ({len(calls)}) < path edge 수 ({len(path) - 1})")
    assert calls[0][1] == START, (
        f"첫 callback 의 parent {calls[0][1]} != START {START}")
    iters = [c[2] for c in calls]
    assert iters == sorted(iters), "iteration 이 단조 증가하지 않음 (연속성 깨짐)"


def test_deterministic_with_seed():
    """같은 seed → 같은 path. 재현성 보장."""
    assert _run(seed=42) == _run(seed=42), "같은 seed 에서 결과가 달라짐 — 재현성 깨짐"


def test_budget_monotonic_not_worse():
    """max_iter 가 클수록 goal 도달 추정비용 ≤ 작을 때 — budget 단조성.

    채택 기준이 (경로 길이 + 마지막 node→goal 거리) 이므로 그 값으로 비교한다.
    best_path 는 strict 개선일 때만 갱신하므로 budget 이 늘어도 악화되지 않는다.
    """
    def _eff(p):
        return (_path_length(p)
                + math.hypot(p[-1][0] - GOAL[0], p[-1][1] - GOAL[1]))

    path_long = _run(max_iter=MAX_ITER)
    path_short = _run(max_iter=MAX_ITER_SHORT)
    assert path_long and path_short
    assert _eff(path_long) <= _eff(path_short) + 1e-6, (
        f"budget 을 늘렸는데 경로가 악화됨 — "
        f"{MAX_ITER}회 {_eff(path_long):.2f} > {MAX_ITER_SHORT}회 "
        f"{_eff(path_short):.2f}")
