"""ConstantSpacePID — 일정 간격 유지 종방향 PID.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class ConstantSpacePID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float,
                 target_space: float = 20.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.target_space = target_space
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, target_x: float, ego_x: float) -> float:
        # TODO: 상대 거리 기반 PID 를 구현하시오.
        # - error = (target_x - ego_x) - target_space   (부호 주의)
        # - 첫 호출 D=0
        # - error_sum += error * dt
        # - u = kp*error + kd*d_error + ki*error_sum
        # - prev_error 갱신
        raise NotImplementedError
