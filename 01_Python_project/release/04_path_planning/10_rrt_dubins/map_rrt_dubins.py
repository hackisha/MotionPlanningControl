"""Map fixture for Informed RRT* + Dubins — 60×60 grid, S-커브 슬라럼 장애물.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

09 Informed RRT* 의 슬라럼 맵을 그대로 사용 — start→goal 일직선(길이 40)을 막는
두 개의 세로 벽이 위·아래로 엇갈린 gap 을 두고 서 있어, 경로는 두 번 감기는 완만한
S-커브가 된다. **차이는 state 가 (x, y) → (x, y, yaw) 로 늘어났다는 점뿐**:
- start = (10, 30, 0), goal = (50, 30, 0) — 둘 다 동쪽(yaw=0) 진행
- Dubins 의 최대 곡률 제약(|κ|≤1/3, R=3) 아래에서 두 슬라럼 게이트를 누비는 경로

이 맵은 Dubins 의 강점을 가장 잘 드러낸다:
- S-커브 자체가 연속곡률 path 의 자연스러운 형태 → Dubins steer 가 분절선 grid
  search 보다 부드럽고 짧은 경로를 만든다
- 두 게이트 사이의 좁은 회랑이 곡률 제약을 강제 — kappa 를 키우면 못 빠져나간다
"""
from __future__ import annotations

START: tuple[float, float, float] = (10.0, 30.0, 0.0)
GOAL: tuple[float, float, float] = (50.0, 30.0, 0.0)
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
