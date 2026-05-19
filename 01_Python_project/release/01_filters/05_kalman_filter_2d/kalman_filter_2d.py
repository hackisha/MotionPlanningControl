"""Kalman Filter 2D — matrix state-space (위치 + 속도).

과제 명세는 README.md 참조.
"""
from __future__ import annotations

import numpy as np


class KalmanFilter2D:
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                 Q: np.ndarray, R: float,
                 x0: np.ndarray | None = None,
                 P0: np.ndarray | None = None):
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R
        self.x = x0 if x0 is not None else np.zeros(2)
        self.P = P0 if P0 is not None else 10.0 * np.eye(2)

    def step(self, measurement: float, control_input: float) -> np.ndarray:
        x_pred = self.A @ self.x + self.B * control_input
        P_pred = self.A @ self.P @ self.A.T + self.Q
        
        S = self.C @ P_pred @ self.C + self.R
        K = (P_pred @ self.C) / S
        innovation = measurement - self.C @ x_pred
        x_new = x_pred + K * innovation
        P_new = (np.eye(2) - np.outer(K, self.C)) @ P_pred
        self.x = x_new
        self.P = P_new
        return self.x
    


        # TODO: 행렬 형태의 Kalman Predict + Update 를 작성하시오.
        # Predict:
        #   x_pred = A @ x + B * u
        #   P_pred = A @ P @ A.T + Q
        # Update:
        #   S = C @ P_pred @ C + R           (scalar — C 가 1D 라 inner product)
        #   K = (P_pred @ C) / S             (1D 길이 2)
        #   innovation = measurement - C @ x_pred
        #   x_new = x_pred + K * innovation
        #   P_new = (np.eye(2) - np.outer(K, C)) @ P_pred
        # self.x, self.P 갱신 후 self.x 반환.
        raise NotImplementedError
