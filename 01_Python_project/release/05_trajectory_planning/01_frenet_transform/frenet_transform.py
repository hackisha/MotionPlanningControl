"""Conversions between Cartesian and Frenet coordinates for an open road."""
from __future__ import annotations

import math

import numpy as np


def cartesian_to_frenet(x: float, y: float,
                        cx: np.ndarray, cy: np.ndarray, cs: np.ndarray
                        ) -> tuple[float, float]:
    """Project a Cartesian point onto the centerline and return (s, d).

    ``s`` is the accumulated arc length to the closest projection foot.
    ``d`` is the signed lateral offset: positive means the point is on the
    left side of the centerline tangent.
    """
    px, py = float(x), float(y)
    best_s = float(cs[0])
    best_d = 0.0
    best_dist_sq = math.inf

    for i in range(len(cx) - 1):
        x0, y0 = float(cx[i]), float(cy[i])
        x1, y1 = float(cx[i + 1]), float(cy[i + 1])
        vx, vy = x1 - x0, y1 - y0
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq < 1e-12:
            continue

        wx, wy = px - x0, py - y0
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
        foot_x = x0 + t * vx
        foot_y = y0 + t * vy
        dx, dy = px - foot_x, py - foot_y
        dist_sq = dx * dx + dy * dy

        if dist_sq < best_dist_sq:
            seg_len = math.sqrt(seg_len_sq)
            best_dist_sq = dist_sq
            best_s = float(cs[i]) + t * seg_len
            best_d = (vx * (py - foot_y) - vy * (px - foot_x)) / seg_len

    return best_s, best_d


def frenet_to_cartesian(s: float, d: float,
                        cx: np.ndarray, cy: np.ndarray, cs: np.ndarray
                        ) -> tuple[float, float, float]:
    """Convert Frenet (s, d) to Cartesian (x, y, heading).

    This road is open, so ``s`` is clamped to the valid centerline interval.
    """
    s_clamped = float(np.clip(s, float(cs[0]), float(cs[-1])))
    if s_clamped >= float(cs[-1]):
        i = len(cs) - 2
    else:
        i = int(np.searchsorted(cs, s_clamped, side="right")) - 1
        i = min(max(i, 0), len(cs) - 2)

    x0, y0 = float(cx[i]), float(cy[i])
    x1, y1 = float(cx[i + 1]), float(cy[i + 1])
    vx, vy = x1 - x0, y1 - y0
    heading = math.atan2(vy, vx)
    ds = s_clamped - float(cs[i])

    base_x = x0 + ds * math.cos(heading)
    base_y = y0 + ds * math.sin(heading)
    normal_x = -math.sin(heading)
    normal_y = math.cos(heading)

    return base_x + float(d) * normal_x, base_y + float(d) * normal_y, heading
