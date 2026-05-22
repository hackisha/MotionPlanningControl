"""LateralPipeline (Stanley 용) — perception (sampling + Global→Local) → state
estimation (poly fit) → control (controller.step) 의 통합 모듈.

06/07/08 각 폴더에 controller 별 사본 (`lateral_pipeline_<controller>.py`) 존재 —
파일명으로 어느 controller 와 짝지어지는지 명시. per-problem self-contained.
과제 명세는 README.md 참조.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


def _polyval_at(coeff: np.ndarray, x: float) -> float:
    """coeff (degree+1, 1) column 을 x 에서 평가 (계수 순서: 고차 → 저차)."""
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


@dataclass
class PipelineOutput:
    delta: float                          # controller 출력 (rad)
    coeff: np.ndarray                     # (degree+1, 1) local-frame poly 계수
    fit_local_points: np.ndarray          # (len(x_local), 2) viz: 곡선
    lookahead_local: tuple[float, float]  # viz: (x, y) local frame


class LateralPipeline:
    def __init__(
        self,
        g2l,
        fitter,
        ev,
        controller,
        sample_xs: np.ndarray,
        x_local: np.ndarray,
    ):
        self.g2l = g2l
        self.fitter = fitter
        self.ev = ev
        self.controller = controller
        self.sample_xs = np.asarray(sample_xs, dtype=float)
        self.x_local = np.asarray(x_local, dtype=float)

    def step(
        self,
        x_ego: float,
        y_ego: float,
        yaw_ego: float,
        vx: float,
        ref_y_fn: Callable[[np.ndarray], np.ndarray],
        lookahead_x: float,
    ) -> PipelineOutput:
        global_xs = x_ego + self.sample_xs
        global_ys = ref_y_fn(global_xs)

        points = np.column_stack([global_xs, global_ys])

        local_points = self.g2l.convert(points, yaw_ego, x_ego, y_ego)

        coeff = self.fitter.fit(local_points)

        self.ev.calculate(coeff, self.x_local.reshape(-1))
        fit_local_points = self.ev.points.copy()

        delta = self.controller.step(coeff, vx)

        y_lh = _polyval_at(coeff, lookahead_x)

        return PipelineOutput(
            delta=float(delta),
            coeff=coeff,
            fit_local_points=fit_local_points,
            lookahead_local=(float(lookahead_x), float(y_lh)),
        )
