"""Map fixture for Hybrid A* — L-shape circular 장애물 배치 (legacy `map_3` 동일).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

레이아웃:
- 탐색 공간: x ∈ [-2, 15], y ∈ [-2, 15]
- start = (10, 0, π) — 우측 하단, 좌측을 바라봄
- goal  = (10, 10, 0) — 우측 중앙, 우측을 바라봄
- 장애물: 반지름 1 인 17 개의 원형 — vertical (x=3, y=5..10) + horizontal (x=4..14, y=5)
  → L-shape 벽. start 와 goal 사이 직진 불가, 우회 (좌측 위로 돌아) 필요.
  (탐색 공간 우측 경계 x=15 에 걸치는 (15, 5) 는 미관상 제외.)
"""
from __future__ import annotations

import math

START: tuple[float, float, float] = (10.0, 0.0, math.pi)
GOAL: tuple[float, float, float] = (10.0, 10.0, 0.0)
SPACE: tuple[float, float, float, float] = (-2.0, 15.0, -2.0, 15.0)


def make_obstacles() -> list[tuple[float, float, float]]:
    """(x, y, radius) 튜플 리스트 반환 — 원형 충돌 영역."""
    obs: list[tuple[float, float, float]] = []
    # vertical wall (x=3, y=5..10)
    for y in range(5, 11):
        obs.append((3.0, float(y), 1.0))
    # horizontal wall (x=4..14, y=5) — x=15 disk 는 space 경계에 걸쳐 제외.
    for x in range(4, 15):
        obs.append((float(x), 5.0, 1.0))
    return obs


OBSTACLES: list[tuple[float, float, float]] = make_obstacles()
