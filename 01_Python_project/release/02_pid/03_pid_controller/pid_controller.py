"""PID Controller — proportional + integral + derivative feedback.

과제 명세는 problem.html 참조.
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
        # TODO: PID 제어 식을 구현하시오.
        # - error = reference - measure
        # - 첫 호출 시 D 기여 0 (PD 모듈과 동일 정책)
        # - error_sum 누적: error_sum += error * dt   (dt 곱 잊지 말 것)
        # - u = kp*error + kd*d_error + ki*error_sum
        # - 호출 끝에 prev_error 갱신
        raise NotImplementedError
