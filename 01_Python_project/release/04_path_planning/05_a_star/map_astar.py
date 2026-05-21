"""Map fixture — 60×60 grid with box-shaped obstacles (legacy `map_2`).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

레이아웃:
- 외곽 border (x∈{0,60}, y∈{0,60})
- 중앙 vertical wall (x=30, y=10..50): 좌·우 chamber 분리
- 상·하 horizontal walls: y=10, y=50 에서 x=20..40 만 차지 (box 의 위·아래 면)
- box 좌·우 면은 open (x=20 / x=40 에 wall 없음)
- start = (10, 30), goal = (50, 30) — 둘 다 box 밖, 중앙 벽 사이에 둠

A* heuristic 이 잘 작동하는 클래식 케이스: 직진 방향이 막혀 있어 우회해야 함.
"""
from __future__ import annotations

START: tuple[int, int] = (10, 30)
GOAL: tuple[int, int] = (50, 30)
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
