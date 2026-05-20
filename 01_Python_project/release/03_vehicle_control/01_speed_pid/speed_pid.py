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
        # TODO: PID 식을 구현하시오 (02_pid/03_pid_controller 와 동일 패턴).
        # - error = reference - measure
        # - 첫 호출 D=0 (prev_error 가 None 이면)
        # - error_sum += error * dt
        # - u = kp*error + kd*d_error + ki*error_sum
        # - prev_error 갱신
        raise NotImplementedError
