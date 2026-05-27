"""Low-Pass Filter — 1차 IIR (Exponential Moving Average).

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class LowPassFilter:
    def __init__(self, alpha: float = 0.9):
        self.alpha = alpha
        self.y: float | None = None

    def step(self, x: float) -> float:
        # TODO: 1차 IIR 형태의 저역 필터를 작성하시오.
        # 인터페이스: 매 호출마다 입력 x 를 받아 평활된 출력을 반환
        # 첫 호출에서는 self.y 가 None — x 자체를 그대로 반환 (초기값 보호)
        # 이후에는 y_new = α · y_prev + (1 - α) · x 로 갱신
        if self.y is None:
            self.y = float(x)
        else:
            self.y = self.alpha * self.y + (1.0 - self.alpha) * x
        return float(self.y)
