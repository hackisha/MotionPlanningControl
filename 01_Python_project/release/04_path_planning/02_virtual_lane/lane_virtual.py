"""Lane fixture — 좌·우 차선이 구간별로 invalid 한 sinusoidal 도로 (lane_2 style).

invalid 구간:
- X ∈ [20, 40] : 왼쪽 차선 invalid (오른쪽만 사용 → 가상 좌측 추정 필요)
- X ∈ [60, 80] : 오른쪽 차선 invalid (왼쪽만 사용 → 가상 우측 추정 필요)
- 그 외:        양쪽 모두 valid

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.
"""
from __future__ import annotations

import numpy as np

LANE_WIDTH = 4.0


def lane(X_ref: np.ndarray, lanewidth: float = LANE_WIDTH
         ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """좌·우 차선 Y 와 valid 마스크 반환.

    invalid 위치의 Y 는 임의값(0.0) 으로 채워지지만 valid_* 가 False —
    학생은 valid 마스크로 어느 쪽 lane 을 신뢰할지 판단해야 함.
    """
    X = np.asarray(X_ref, dtype=float)
    Y_center = 2.0 - 2.0 * np.cos(X / 10.0)
    Y_L = Y_center + lanewidth / 2.0
    Y_R = Y_center - lanewidth / 2.0
    valid_L = np.ones(X.shape, dtype=bool)
    valid_R = np.ones(X.shape, dtype=bool)
    # X ∈ [20, 40] : 왼쪽 invalid
    mask_L_invalid = (X > 20.0) & (X < 40.0)
    valid_L[mask_L_invalid] = False
    Y_L[mask_L_invalid] = 0.0
    # X ∈ [60, 80] : 오른쪽 invalid
    mask_R_invalid = (X > 60.0) & (X < 80.0)
    valid_R[mask_R_invalid] = False
    Y_R[mask_R_invalid] = 0.0
    return Y_L, Y_R, valid_L, valid_R


def lane_center(X_ref: np.ndarray) -> np.ndarray:
    X = np.asarray(X_ref, dtype=float)
    return 2.0 - 2.0 * np.cos(X / 10.0)
