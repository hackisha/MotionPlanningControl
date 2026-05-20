"""LatPIDFF — local-frame polynomial 기반 lateral PID + 곡률 feedforward.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import numpy as np


def _polyval_at(coeff: np.ndarray, x: float) -> float:
    """coeff (degree+1, 1) column 을 x 에서 평가 (계수 순서: 고차 → 저차)."""
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


class LatPIDFF:
    def __init__(self, kp: float, kd: float, ki: float, kff: float,
                 dt: float, lookahead_time: float = 0.5):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.kff = kff
        self.dt = dt
        self.lookahead_time = lookahead_time
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, coeff: np.ndarray, vx: float) -> float:
        # TODO: local-frame 다항식 기반 lateral PID + curvature feedforward.
        # 1) d_lh = lookahead_time · vx   (lookahead 거리)
        # 2) error = _polyval_at(coeff, d_lh)   (lookahead 점의 local-frame y)
        # 3) 첫 호출 D=0
        # 4) error_sum += error * dt
        # 5) ff_term = vx² · 2 · coeff[-3]   (곡률 ≈ y''(0) = 2·coeff[-3])
        # 6) δ = kp*error + kd*d_error + ki*error_sum + kff*ff_term
        # 7) prev_error 갱신
        raise NotImplementedError
