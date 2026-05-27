"""Kalman Filter — scalar 1D linear system.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class KalmanFilter:
    def __init__(self, m: float = 1.0, dt: float = 0.01,
                 q: float = 0.1, r: float = 0.9, p0: float = 10.0):
        self.A = 1.0 + dt
        self.B = dt / m
        self.C = 1.0
        self.Q = q
        self.R = r
        self.x = 0.0
        self.P = p0

    def step(self, measurement: float, control_input: float) -> float:
        # TODO: 한 번의 Predict + Update 수행 후 갱신된 추정값을 반환.
        # Predict:
        #   x_pred = A · x + B · u
        #   P_pred = A^2 · P + Q
        # Update:
        #   K = P_pred · C / (C^2 · P_pred + R)
        #   x_new = x_pred + K · (measurement - C · x_pred)
        #   P_new = (1 - K · C) · P_pred
        # self.x, self.P 갱신 후 self.x 반환.
        x_pred = self.A * self.x + self.B * control_input
        p_pred = self.A * self.A * self.P + self.Q

        innovation = measurement - self.C * x_pred
        s = self.C * self.C * p_pred + self.R
        k = p_pred * self.C / s

        self.x = x_pred + k * innovation
        self.P = (1.0 - k * self.C) * p_pred
        return float(self.x)
