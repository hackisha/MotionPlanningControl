"""Map fixture for Informed RRT* — 60×60 grid, S-커브 슬라럼 장애물 배치.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

05~08 의 box-maze 와 달리, start→goal 일직선(길이 40)을 막는 두 개의 세로 벽을
두고 통과 구멍(gap)을 위·아래로 엇갈리게 배치한다. 경로는 1번 벽을 위로, 2번
벽을 아래로 한 번씩 감아 도는 완만한 S-커브가 된다. 첫 경로는 빨리 찾히되
(~60 iter) 다듬어지지 않아 길고(~54), 남은 budget 동안 informed 타원이 좁아지며
~44 로 수렴한다 — 타원 수렴을 관찰하기 좋은 맵.

- 외곽 border (x∈{0,60}, y∈{0,60})
- start = (10, 30), goal = (50, 30) — 같은 높이, 직선은 슬라럼 벽이 차단
- 세로 벽 2 개(폭 4) + 모서리 장식 사각형 4 개 — 손으로 배치, 항상 같은 맵
"""
from __future__ import annotations

START: tuple[float, float] = (10.0, 30.0)
GOAL: tuple[float, float] = (50.0, 30.0)
GRID_SIZE: int = 60

# 슬라럼 벽: (x0, width, gap_lo, gap_hi)
# 폭 width 의 세로 벽을 x0 부터 세우되, y∈[gap_lo, gap_hi] 구간만 비워 둔다.
# 1번 벽 구멍은 위(34~54), 2번 벽 구멍은 아래(6~24) — 경로가 S 자로 누빈다.
_WALLS: tuple[tuple[int, int, int, int], ...] = (
    (22, 4, 34, 54),   # 1번 벽 — 구멍 위쪽
    (37, 4, 6, 24),    # 2번 벽 — 구멍 아래쪽
)

# 장식 사각형: (x0, y0, width, height) — 네 모서리, 경로 회랑과 무관.
_DECOYS: tuple[tuple[int, int, int, int], ...] = (
    (8, 8, 5, 5),
    (8, 47, 5, 5),
    (48, 9, 5, 5),
    (48, 46, 5, 5),
)


def make_obstacles() -> set[tuple[int, int]]:
    """외곽 border + 슬라럼 벽 + 장식 사각형 cell 집합 (손 배치 → 항상 동일)."""
    obs: set[tuple[int, int]] = set()
    # 외곽 border
    for i in range(GRID_SIZE + 1):
        obs.add((i, 0))
        obs.add((0, i))
        obs.add((i, GRID_SIZE))
        obs.add((GRID_SIZE, i))
    # 슬라럼 벽 — gap 구간을 제외한 모든 y 를 채운다
    for x0, width, gap_lo, gap_hi in _WALLS:
        for dx in range(width):
            for y in range(1, GRID_SIZE):
                if not (gap_lo <= y <= gap_hi):
                    obs.add((x0 + dx, y))
    # 장식 사각형
    for x0, y0, width, height in _DECOYS:
        for dx in range(width):
            for dy in range(height):
                obs.add((x0 + dx, y0 + dy))
    return obs


OBSTACLES: set[tuple[int, int]] = make_obstacles()
