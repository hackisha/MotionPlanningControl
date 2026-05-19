"""Map fixture — 60×60 grid with obstacles (legacy `map_1` 의 wall pattern).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

레이아웃:
- 외곽 border (x∈{0,60}, y∈{0,60}): 완전 폐쇄
- 중앙 vertical wall (x=30, y∈[0,50]): 좌·우 분리, y=51..59 만 개방
- y=20 horizontal wall: x∈[0,20] ∪ [40,60] 폐쇄 (x=21..39 개방)
- y=35 horizontal wall: x∈[10,50] 폐쇄 (x=0..9, x=51..59 만 개방)
- start = (5, 5), goal = (55, 5)

start 와 goal 사이 직진 경로는 없음 — 중앙 벽 위로 우회해야 하는 미로.
"""
from __future__ import annotations

START: tuple[int, int] = (5, 5)
GOAL: tuple[int, int] = (55, 5)
GRID_SIZE: int = 60


def make_obstacles() -> set[tuple[int, int]]:
    """legacy map_1 의 wall pattern 을 set 으로 반환 — O(1) collision 검사 가능."""
    obs: set[tuple[int, int]] = set()
    # 외곽 border
    for i in range(GRID_SIZE + 1):
        obs.add((i, 0))
        obs.add((0, i))
        obs.add((i, GRID_SIZE))
        obs.add((GRID_SIZE, i))
    # 중앙 vertical wall (y=0..50)
    for i in range(51):
        obs.add((30, i))
    # y=20 horizontal wall: x=0..20 ∪ 40..60
    for i in range(21):
        obs.add((i, 20))
        obs.add((40 + i, 20))
    # y=35 horizontal wall: x=10..50
    for i in range(41):
        obs.add((10 + i, 35))
    return obs


OBSTACLES: set[tuple[int, int]] = make_obstacles()
