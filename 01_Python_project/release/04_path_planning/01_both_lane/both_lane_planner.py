"""Both-lane center path planner."""
from __future__ import annotations

import numpy as np


def both_lane_to_path(coeff_L: np.ndarray, coeff_R: np.ndarray) -> np.ndarray:
    """Return the centerline polynomial from left and right lane polynomials.

    The lane polynomial is represented as a column vector ordered from the
    highest-degree coefficient to the constant term. If both lane boundaries
    share the same degree, the point halfway between them is obtained by
    averaging every coefficient.
    """
    return (coeff_L + coeff_R) / 2.0
