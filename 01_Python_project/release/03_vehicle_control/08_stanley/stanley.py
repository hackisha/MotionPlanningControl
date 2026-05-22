"""Stanley — heading + cross-track 합성 lateral controller (stateless).

과제 명세는 README.md 참조.
"""

from __future__ import annotations

import numpy as np


class Stanley:
    def __init__(self, k: float = 1.0, epsilon: float = 1e-3):
        self.k = k
        self.epsilon = epsilon

    def step(self, coeff: np.ndarray, vx: float) -> float:
        coeff = coeff.reshape(-1)

        heading_error = coeff[-2]
        cross_track = coeff[-1]

        delta = heading_error + np.arctan(self.k * cross_track / (vx + self.epsilon))

        return float(delta)
        # TODO: Stanley 식을 구현하시오 (stateless — 내부 상태 없음).
        # 1) heading_error = coeff[-2]   (1차 계수: ψ_e)
        # 2) cross_track   = coeff[-1]   (상수항: e_y, ego ↔ path 의 y 거리)
        # 3) δ = heading_error + atan( k · cross_track / (vx + ε) )
        # ε 는 저속 0 분모 회피
        raise NotImplementedError
