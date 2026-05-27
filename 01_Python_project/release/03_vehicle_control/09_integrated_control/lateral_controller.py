"""09 lateral controller — Pure Pursuit + 통일 adapter.

PurePursuit 솔루션은 본 폴더 self-contained. 학생이 Stanley / LatPIDFF 로 교체하고
싶으면 본 파일을 수정하지 않고도 adapter 만 갈아끼우면 됨:

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "08_stanley"))
    from stanley import Stanley
    s = Stanley(k=1.0)
    lat = LateralController(s, lookahead_x_fn=lambda vx: 0.0)

    # 또는 06 의 LatPIDFF:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "06_lat_pid_ff"))
    from lat_pid_ff import LatPIDFF
    pid = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.1, dt=0.1)
    lat = LateralController(pid, lookahead_x_fn=lambda vx: vx * pid.lookahead_time)

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

_EPS = 1e-3


def _polyval_at(coeff: np.ndarray, x: float) -> float:
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


class PurePursuit:
    """09 의 default lateral controller (07 사본 — self-contained)."""
    def __init__(self, L: float, lookahead_time: float = 1.0):
        self.L = L
        self.lookahead_time = lookahead_time

    def step(self, coeff: np.ndarray, vx: float) -> float:
        d_lh = self.lookahead_time * vx
        y_lh = _polyval_at(coeff, d_lh)
        delta = np.arctan2(2 * self.L * y_lh, d_lh ** 2 + y_lh ** 2 + _EPS)
        return float(delta)


@dataclass
class LateralController:
    """통일 adapter — control_pipeline 이 controller-specific 분기 없이 호출.

    `ctrl` 은 `.step(coeff, vx) -> float` 시그니처만 만족하면 됨 (06/07/08 의 셋 다 호환).
    `lookahead_x_fn` 은 viz 용 lookahead point 의 local frame x 좌표
    (PP/PIDFF: `vx*lookahead_time`, Stanley: `0`).
    """
    ctrl: object
    lookahead_x_fn: Callable[[float], float]

    def step(self, coeff: np.ndarray, vx: float) -> float:
        return float(self.ctrl.step(coeff, vx))

    def lookahead_x(self, vx: float) -> float:
        return float(self.lookahead_x_fn(vx))
