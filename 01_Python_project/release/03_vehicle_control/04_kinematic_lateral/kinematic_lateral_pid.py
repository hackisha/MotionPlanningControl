"""KinematicLateralPID — PID 로 Y reference 추종, 출력 = 조향각 δ (rad).

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class KinematicLateralPID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, reference_Y: float, ego_Y: float) -> float:
        # TODO: Y reference 추종 PID 를 구현하시오.
        # - error = reference_Y - ego_Y
        # - 첫 호출 D=0
        # - error_sum += error * dt
        # - 반환값 단위: 라디안 (조향각 δ)
        raise NotImplementedError
