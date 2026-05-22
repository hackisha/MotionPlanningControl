"""Map fixture for RRT* — 60×60 grid (legacy `map_2`, 05 A* / 07 RRT 와 동일).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

05 A* / 07 RRT 와 같은 obstacle 배치를 그대로 사용해 세 알고리즘
(deterministic A* / random RRT / optimal-rewiring RRT*) 의 경로를 viewer 에서
직접 비교할 수 있게 함.

레이아웃:
- 외곽 border (x∈{0,60}, y∈{0,60})
- 중앙 vertical wall (x=30, y=10..50)
- box 상·하 면 (y=10 / y=50, x=20..40), 좌·우 면은 open
- start = (10, 30), goal = (50, 30) — 중앙 벽 우회 필요
"""
from __future__ import annotations

START: tuple[float, float] = (10.0, 30.0)
GOAL: tuple[float, float] = (50.0, 30.0)
GRID_SIZE: int = 60


def make_obstacles() -> set[tuple[int, int]]:
    obs: set[tuple[int, int]] = set()
    # 외곽 border
    for i in range(GRID_SIZE + 1):
        obs.add((i, 0))
        obs.add((0, i))
        obs.add((i, GRID_SIZE))
        obs.add((GRID_SIZE, i))
    # 중앙 vertical wall (y=10..50)
    for i in range(41):
        obs.add((30, 10 + i))
    # box 상·하 면 (y=10, y=50 에서 x=20..40)
    for i in range(21):
        obs.add((20 + i, 10))
        obs.add((20 + i, 50))
    return obs


OBSTACLES: set[tuple[int, int]] = make_obstacles()
