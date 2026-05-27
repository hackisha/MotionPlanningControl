"""Speed PID — 종방향 속도 추종 PID 제어기.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class SpeedPID:
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
        return float(u)
        # TODO: PID 식을 구현하시오 (02_pid/03_pid_controller 와 동일 패턴).
        # - error = reference - measure
        # - 첫 호출 D=0 (prev_error 가 None 이면)
        # - error_sum += error * dt
        # - u = kp*error + kd*d_error + ki*error_sum
        # - prev_error 갱신
