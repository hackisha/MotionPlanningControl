"""RRT regression — behavioral spec (requirements level).

알고리즘 형태 자유 (sampling 전략 / nearest 구현 / 자료 구조 제약 X).
인터페이스 + 경로의 도달성·연속성·재현성으로 합격 판정.
"""
import math

from map_rrt import GOAL, GRID_SIZE, OBSTACLES, START
from rrt import rrt

ETA = 3.0
SEED = 0
MAX_ITER = 2000
GOAL_RANGE = 1.5


def test_path_starts_at_start():
    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=MAX_ITER, eta=ETA, seed=SEED)
    assert path, "RRT 가 경로를 찾지 못함 (seed/max_iter 확인)"
    assert path[0] == START, f"path[0] = {path[0]}, expected {START}"


def test_path_ends_near_goal():
    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=MAX_ITER, eta=ETA, seed=SEED)
    assert path
    end = path[-1]
    d = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1])
    assert d < GOAL_RANGE, (
        f"path 끝 ({end[0]:.2f}, {end[1]:.2f}) 가 goal 에서 {d:.2f} m 떨어짐 "
        f"— goal_range {GOAL_RANGE} 초과")


def test_path_segments_collision_free():
    """연속 두 점을 잇는 선분의 fine-sample 어느 cell 도 obstacle 가 아님."""
    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=MAX_ITER, eta=ETA, seed=SEED)
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


def test_path_segment_length_within_eta():
    """연속 두 점 거리 ≤ eta + tol (steer 의 제약)."""
    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=MAX_ITER, eta=ETA, seed=SEED)
    assert path
    for a, b in zip(path[:-1], path[1:], strict=False):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        assert d <= ETA + 1e-6, (
            f"segment 길이 {d:.3f} > eta {ETA} — steer 제약 위반")


def test_on_step_callback_invoked():
    """on_step 콜백 호출 수 ≥ path edge 수, 첫 호출 parent 가 start."""
    calls: list[tuple[tuple[float, float], tuple[float, float], int]] = []

    def cb(new_node, parent_node, iteration):
        calls.append((new_node, parent_node, iteration))

    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=MAX_ITER, eta=ETA, seed=SEED, on_step=cb)
    assert path
    assert len(calls) >= len(path) - 1, (
        f"callback 호출 수 ({len(calls)}) < path edge 수 ({len(path) - 1})")
    assert calls[0][1] == START, (
        f"첫 callback 의 parent {calls[0][1]} != START {START}")


def test_deterministic_with_seed():
    """같은 seed → 같은 path. 재현성 보장."""
    path1 = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
                max_iter=MAX_ITER, eta=ETA, seed=42)
    path2 = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
                max_iter=MAX_ITER, eta=ETA, seed=42)
    assert path1 == path2, "같은 seed 에서 결과가 달라짐 — 재현성 깨짐"
