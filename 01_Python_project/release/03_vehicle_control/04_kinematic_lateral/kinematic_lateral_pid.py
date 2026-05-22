"""KinematicLateralPID — PID 로 Y reference 추종, 출력 = 조향각 δ (rad).

과제 명세는 README.md 참조.
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
        error = reference_Y - ego_Y
        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / self.dt
        self.error_sum += error * self.dt
        u = self.kp * error + self.kd * d_error + self.ki
        self.prev_error = error
        return u
            
        # TODO: Y reference 추종 PID 를 구현하시오.
        # - error = reference_Y - ego_Y
        # - 첫 호출 D=0
        # - error_sum += error * dt
        # - 반환값 단위: 라디안 (조향각 δ)
        raise NotImplementedError
