"""Virtual-lane path planner for partially missing lane boundaries."""
from __future__ import annotations

import numpy as np


class LaneWidthEstimator:
    """Keep the most recent reliable lane width estimate."""

    def __init__(self, Lw_init: float = 4.0):
        self.Lw = float(Lw_init)

    def update(self, coeff_L: np.ndarray, coeff_R: np.ndarray,
               valid_L: bool, valid_R: bool) -> None:
        """Update lane width only when both lane boundaries are valid.

        In the local vehicle frame, the constant term of a lane polynomial is
        the lateral intercept at x=0. The width estimate is therefore the
        lateral distance between the left and right intercepts.
        """
        if valid_L and valid_R:
            self.Lw = float(coeff_L[-1, 0] - coeff_R[-1, 0])


def either_lane_to_path(coeff_L: np.ndarray, coeff_R: np.ndarray,
                        valid_L: bool, valid_R: bool, Lw: float) -> np.ndarray:
    """Return a center path using both lanes or a virtual missing lane.

    Cases:
    - both valid: average left and right lanes
    - left only: shift the left boundary right by half the lane width
    - right only: shift the right boundary left by half the lane width
    - neither valid: return a straight local path
    """
    if valid_L and valid_R:
        return (coeff_L + coeff_R) / 2.0

    if valid_L:
        coeff = coeff_L.copy()
        coeff[-1, 0] -= float(Lw) / 2.0
        return coeff

    if valid_R:
        coeff = coeff_R.copy()
        coeff[-1, 0] += float(Lw) / 2.0
        return coeff

    return np.zeros_like(coeff_L)
