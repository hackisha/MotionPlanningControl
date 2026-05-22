"""Both Lane Planner — 좌·우 차선 polynomial 계수로부터 중앙선 path 계수 생성.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import numpy as np


def both_lane_to_path(coeff_L: np.ndarray, coeff_R: np.ndarray) -> np.ndarray:
    """좌·우 차선의 polynomial 계수를 결합해 중앙선 path 의 계수를 반환.

    coeff_L, coeff_R: shape (degree+1, 1) column. 계수 순서 고차 → 저차.
    반환: 같은 shape.
    """
    # TODO: 양쪽 차선 계수로부터 중앙선 path 계수를 생성하시오.
    # 힌트: 같은 차수의 계수끼리 평균을 내면 두 차선의 중앙선이 된다.
    raise NotImplementedError
