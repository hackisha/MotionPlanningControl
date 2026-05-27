"""PurePursuit — 기하학적 lateral controller (stateless).

과제 명세는 README.md 참조.
"""
from __future__ import annotations

import numpy as np

_EPS = 1e-3


def _polyval_at(coeff: np.ndarray, x: float) -> float:
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


class PurePursuit:
    def __init__(self, L: float, lookahead_time: float = 1.0):
        self.L = L
        self.lookahead_time = lookahead_time

    def step(self, coeff: np.ndarray, vx: float) -> float:
        d_lh = self.lookahead_time * vx
        y_lh = _polyval_at(coeff, d_lh)
        delta = np.arctan2(2 * self.L * y_lh, d_lh ** 2 + y_lh ** 2 + _EPS)
        return float(delta)
    
        # TODO: Pure Pursuit 식을 구현하시오 (stateless — 내부 상태 없음).
        # 1) d_lh = lookahead_time · vx
        # 2) y_lh = _polyval_at(coeff, d_lh)
        # 3) δ = atan( 2 · L · y_lh / (d_lh² + y_lh² + ε) )
        #    ε(=_EPS) 는 0 분모 회피 (직진 시점 y_lh=0 안전)
