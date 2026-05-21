"""폐루프 트랙 — Frenet 궤적 계획용 2 차선 stadium 트랙.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

레거시 ex02 의 1/10 RC 스케일 짧은 트랙(둘레 ≈ 10 m)을, ch3·ch4 리팩터본과 동일한
차량 스케일의 약 200 m 폐루프 트랙으로 재구성했다. 두 직선 + 두 반원으로 이뤄진
stadium 형태.

차선 레이아웃 (중심선 = Frenet d=0):
  d = 0            도로 중앙 (차선 구분선)
  d = ±LANE_WIDTH/2  두 차선의 중앙
  d = ±LANE_WIDTH    도로 양쪽 경계
"""
from __future__ import annotations

import math

import numpy as np
from frenet_frame import build_maps, cartesian_to_frenet, frenet_to_cartesian, get_dist

# --- 트랙 형상 파라미터 (차량 스케일) ---
LANE_WIDTH: float = 4.0       # 한 차선 폭 [m]
_ARC_RADIUS: float = 20.0     # 양 끝 반원 반지름 [m]
_STRAIGHT: float = 37.0       # 직선 구간 길이 [m]
_SPACING: float = 1.0         # 중심선 점 간격 [m]
# → 둘레 = 2·_STRAIGHT + 2·π·_ARC_RADIUS ≈ 199.7 m


def _build_centerline() -> tuple[np.ndarray, np.ndarray]:
    """반시계 방향 stadium 중심선 — 끝점 중복 없는 폐루프 점열."""
    r, s, ds = _ARC_RADIUS, _STRAIGHT, _SPACING
    xs: list[float] = []
    ys: list[float] = []

    def straight(x0, y0, x1, y1):
        n = max(2, int(round(math.hypot(x1 - x0, y1 - y0) / ds)))
        for i in range(n):
            xs.append(x0 + (x1 - x0) * i / n)
            ys.append(y0 + (y1 - y0) * i / n)

    def arc(cx, cy, a0, a1):
        n = max(2, int(round(abs(a1 - a0) * r / ds)))
        for i in range(n):
            ang = a0 + (a1 - a0) * i / n
            xs.append(cx + r * math.cos(ang))
            ys.append(cy + r * math.sin(ang))

    straight(-s / 2, -r, s / 2, -r)                       # 하단 직선 (+x)
    arc(s / 2, 0.0, -math.pi / 2, math.pi / 2)            # 우측 반원
    straight(s / 2, r, -s / 2, r)                         # 상단 직선 (-x)
    arc(-s / 2, 0.0, math.pi / 2, 3 * math.pi / 2)        # 좌측 반원
    return np.asarray(xs), np.asarray(ys)


def _offset_line(cx: np.ndarray, cy: np.ndarray, d: float
                 ) -> tuple[list[float], list[float]]:
    """중심선을 횡 offset d 만큼 평행 이동한 폴리라인 (d>0 = 왼쪽)."""
    n = len(cx)
    ox: list[float] = []
    oy: list[float] = []
    for i in range(n):
        tx = cx[(i + 1) % n] - cx[(i - 1) % n]
        ty = cy[(i + 1) % n] - cy[(i - 1) % n]
        norm = math.hypot(tx, ty)
        nx, ny = -ty / norm, tx / norm           # tangent 를 +90° 회전 = 왼쪽 법선
        ox.append(float(cx[i] + d * nx))
        oy.append(float(cy[i] + d * ny))
    return ox, oy


class TrackMap:
    """폐루프 stadium 트랙 — 중심선 + Frenet 변환 + 시각화용 차선 폴리라인."""

    def __init__(self) -> None:
        self.center_x, self.center_y = _build_centerline()
        self.maps = build_maps(self.center_x, self.center_y)
        self.length = float(
            self.maps[-1]
            + get_dist(self.center_x[-1], self.center_y[-1],
                       self.center_x[0], self.center_y[0])
        )

    def to_cartesian(self, s: float, d: float) -> tuple[float, float, float]:
        """Frenet (s, d) → 전역 (x, y, heading)."""
        return frenet_to_cartesian(s, d, self.center_x, self.center_y,
                                   self.maps, self.length)

    def to_frenet(self, x: float, y: float) -> tuple[float, float]:
        """전역 (x, y) → Frenet (s, d)."""
        return cartesian_to_frenet(x, y, self.center_x, self.center_y, self.maps)

    def lanes_for_record(self) -> list[dict]:
        """simulator record 의 `lanes` 필드용 차선 폴리라인 목록."""
        lanes: list[dict] = []
        dotted_color = [255, 255, 255, 150]               # 차선 중앙선 — 반투명 흰색
        specs = [
            (LANE_WIDTH, "edge", None), (-LANE_WIDTH, "edge", None),
            (0.0, "center", None),                        # 중앙 구분선 — dashed
            (LANE_WIDTH / 2, "dotted", dotted_color),     # 차선 중앙 — dotted (반투명)
            (-LANE_WIDTH / 2, "dotted", dotted_color),
        ]
        for d, kind, color in specs:
            xs, ys = _offset_line(self.center_x, self.center_y, d)
            xs.append(xs[0])                      # 폐루프 닫기
            ys.append(ys[0])
            lane: dict = {"X": xs, "Y": ys, "kind": kind}
            if color is not None:
                lane["color"] = color
            lanes.append(lane)
        return lanes
