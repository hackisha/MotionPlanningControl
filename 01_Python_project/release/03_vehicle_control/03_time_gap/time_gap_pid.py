"""TimeGapPID — 시간 간격 (`gap = ego_vx · time_gap`) 유지 종방향 PID.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class TimeGapPID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float,
                 time_gap: float = 1.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.time_gap = time_gap
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, target_x: float, ego_x: float, ego_vx: float) -> float:
        target_space = ego_vx * self.time_gap
        error = (target_x - ego_x) - target_space
        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / self.dt
        self.error_sum += error * self.dt
        u = self.kp * error + self.kd * d_error + self.ki
        self.prev_error = error
        return u
    

    
    
        # TODO: 시간 간격 기반 PID 를 구현하시오.
        # - target_space = ego_vx * time_gap   (매 스텝 재계산 — ego_vx 변화 반영)
        # - error = (target_x - ego_x) - target_space
        # - 첫 호출 D=0
        # - error_sum += error * dt
        # - u = kp*error + kd*d_error + ki*error_sum
        raise NotImplementedError
