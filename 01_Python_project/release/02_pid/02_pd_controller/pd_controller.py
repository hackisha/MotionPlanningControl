"""PD Controller — proportional + derivative feedback.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class PDController:
    def __init__(self, kp: float, kd: float, dt: float):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.prev_error: float | None = None

    def step(self, reference: float, measure: float) -> float:
        # TODO: PD 제어 식을 구현하시오.
        # 인터페이스: reference (목표), measure (현재) 를 받아 제어 입력 u 를 반환
        # - error = reference - measure  (부호 주의)
        # - 첫 호출 (prev_error 가 None) 시: D 항 기여를 0 으로 (분모 0 / spike 회피)
        # - 두 번째 호출부터: d_error = (error - prev_error) / dt
        # - u = kp * error + kd * d_error
        # - 호출 끝에 prev_error 갱신 잊지 말 것
        raise NotImplementedError
