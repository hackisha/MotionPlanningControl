"""Path planner that follows a leading vehicle's ego-local trace."""
from __future__ import annotations

import math

import numpy as np


class LeadingTargetTracker:
    """Maintain recent leading-vehicle positions in the current ego frame."""

    def __init__(self, max_history: int = 5):
        self.max_history = max_history
        self.history: list[list[float]] = []

    def update(self, target_local_xy: list[float],
               vx: float, yaw_rate: float, dt: float) -> None:
        """Append the current target point and express history in the new ego frame.

        During one control step the ego vehicle moves forward and may rotate.
        Points measured in the previous ego frame must therefore be rotated by
        the ego yaw change and shifted by the ego displacement so that all
        stored points remain comparable in the latest ego-local coordinate.
        """
        self.history.append([float(target_local_xy[0]), float(target_local_xy[1])])
        if len(self.history) > self.max_history:
            self.history.pop(0)

        theta = float(yaw_rate) * float(dt)
        c, s = math.cos(theta), math.sin(theta)
        rot = np.array([[c, s], [-s, c]], dtype=float)

        if abs(yaw_rate) < 1e-9:
            shift = np.array([float(vx) * float(dt), 0.0], dtype=float)
        else:
            shift = np.array([
                float(vx) * float(dt),
                -float(vx) * (1.0 - math.cos(theta)) / float(yaw_rate),
            ], dtype=float)

        updated: list[list[float]] = []
        for point in self.history:
            xy = rot @ np.asarray(point, dtype=float) - shift
            updated.append([float(xy[0]), float(xy[1])])
        self.history = updated


def target_following_path(history: list[list[float]]) -> np.ndarray:
    """Fit a cubic path through the ego origin toward the leading trace.

    The returned polynomial has the form y = c3*x^3 + c2*x^2. The zero
    constant and first-order terms force the path to pass through the ego
    origin with zero initial lateral slope, while the last history point and
    fitted terminal slope shape the curve toward the leading vehicle.
    """
    if len(history) < 4:
        return np.zeros((4, 1))

    pts = np.asarray(history, dtype=float)
    order = np.argsort(pts[:, 0])
    xs = pts[order, 0]
    ys = pts[order, 1]

    xf = float(xs[-1])
    yf = float(ys[-1])
    if abs(xf) < 1e-9:
        return np.zeros((4, 1))

    fit = np.polyfit(xs, ys, 3)
    heading_slope = 3.0 * fit[0] * xf ** 2 + 2.0 * fit[1] * xf + fit[2]
    tan_h = math.tan(math.atan(heading_slope))

    c3 = (xf * tan_h - 2.0 * yf) / (xf ** 3)
    c2 = (3.0 * yf - xf * tan_h) / (xf ** 2)
    return np.array([[c3], [c2], [0.0], [0.0]], dtype=float)
