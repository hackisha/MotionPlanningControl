"""PID Controller — 03_pid_controller 와 동일 정답.

본 모듈(04_pid_tuning)에서는 학생이 알고리즘을 다시 구현하지 않습니다.
이 파일은 fixture 로 제공되며, 학생은 tuning.py 의 게인 값만 결정합니다.
"""
from __future__ import annotations


class PIDController:
    def __init__(self, kp: float, kd: float, ki: float, dt: float):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, reference: float, measure: float) -> float:
        error = reference - measure
        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / self.dt
        self.error_sum += error * self.dt
        u = self.kp * error + self.kd * d_error + self.ki * self.error_sum
        self.prev_error = error
        return u
