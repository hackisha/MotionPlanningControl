"""P Controller — proportional feedback (memoryless).

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class PController:
    def __init__(self, kp: float):
        self.kp = kp

    def step(self, reference: float, measure: float) -> float:
        # TODO: 비례 제어 식을 구현하시오.
        # 인터페이스: reference (목표), measure (현재) 를 받아 제어 입력 u 를 반환
        # 부호 주의: error = reference - measure 형태로 작성할 것
        error = reference - measure
        return float(self.kp * error)
