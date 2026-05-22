"""Virtual Lane Planner — 한쪽 차선만 valid 한 구간에서 가상 차선 + 차선폭 추정.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import numpy as np


class LaneWidthEstimator:
    """양쪽이 모두 valid 한 시점에 차선폭을 갱신, 그 외엔 보관 값 유지."""

    def __init__(self, Lw_init: float = 4.0):
        self.Lw = float(Lw_init)

    def update(self, coeff_L: np.ndarray, coeff_R: np.ndarray,
               valid_L: bool, valid_R: bool) -> None:
        """둘 다 valid 일 때만 self.Lw 를 갱신, 그 외엔 기존 값 유지.

        힌트: coeff_L/R 의 마지막(상수) 항이 local frame 0 위치의 Y. 차이가 차선폭.
        """
        # TODO: valid_L AND valid_R 일 때만 self.Lw 를 갱신.
        raise NotImplementedError


def either_lane_to_path(coeff_L: np.ndarray, coeff_R: np.ndarray,
                        valid_L: bool, valid_R: bool, Lw: float) -> np.ndarray:
    """valid 상태에 따라 path 계수 결정.

    - 양쪽 valid: 평균 (01_both_lane 과 동일)
    - 왼쪽만 valid: 왼쪽 계수를 그대로 쓰되 상수항을 -Lw/2 (오른쪽 방향)
    - 오른쪽만 valid: 오른쪽 계수에 +Lw/2 (왼쪽 방향)
    - 둘 다 invalid: 0 계수 반환 (직진)

    coeff_*: shape (degree+1, 1) column.
    """
    # TODO: 네 가지 case 분기를 구현하시오.
    # 힌트: 한쪽만 valid 일 때는 그쪽 계수의 마지막 (상수) 항만 ±Lw/2 만큼 보정.
    raise NotImplementedError
