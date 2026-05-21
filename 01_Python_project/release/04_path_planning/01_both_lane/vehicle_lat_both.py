"""VehicleLat — kinematic bicycle 차량 모델 (chapter 3 동일 사본).

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.
"""
from __future__ import annotations

import numpy as np


class VehicleLat:
    def __init__(
        self,
        dt: float,
        vx: float,
        m: float = 500.0,
        L: float = 4.0,
        kv: float = 0.005,
        X0: float = 0.0,
        Y0: float = 0.0,
        Yaw0: float = 0.0,
    ):
        self.dt = dt
        self.m = m
        self.L = L
        self.kv = kv
        self.vx = vx
        self.X = X0
        self.Y = Y0
        self.Yaw = Yaw0
        self.yawrate = 0.0
        self.delta = 0.0

    def step(self, delta: float, vx: float) -> None:
        self.vx = vx
        self.delta = float(np.clip(delta, -0.5, 0.5))
        self.yawrate = self.vx / (self.L + self.kv * self.vx**2) * self.delta
        self.Yaw = self.Yaw + self.dt * self.yawrate
        self.X = self.X + self.vx * self.dt * np.cos(self.Yaw)
        self.Y = self.Y + self.vx * self.dt * np.sin(self.Yaw)
