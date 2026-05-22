"""Lane fixture — 좌·우 차선이 양쪽 모두 valid 한 sinusoidal 도로 (lane_1 style).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.
"""
from __future__ import annotations

import numpy as np

LANE_WIDTH = 4.0


def lane(X_ref: np.ndarray, lanewidth: float = LANE_WIDTH) -> tuple[np.ndarray, np.ndarray]:
    """sinusoidal 도로의 좌·우 차선 Y 좌표 반환.

    center: Y = 2.0 - 2·cos(X / 10)  (대략 한 곡률 주기 60 m, 진폭 4 m)
    """
    X = np.asarray(X_ref, dtype=float)
    Y_center = 2.0 - 2.0 * np.cos(X / 10.0)
    return Y_center + lanewidth / 2.0, Y_center - lanewidth / 2.0


def lane_center(X_ref: np.ndarray) -> np.ndarray:
    X = np.asarray(X_ref, dtype=float)
    return 2.0 - 2.0 * np.cos(X / 10.0)
