"""Moving Average Filter — sliding window arithmetic mean.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

from collections import deque


class MovingAverageFilter:
    def __init__(self, window: int = 10):
        self.window = window
        self.buffer: deque[float] = deque(maxlen=window)

    def step(self, x: float) -> float:
        # TODO: 슬라이딩 윈도우 평균을 작성하시오.
        # 인터페이스: 매 호출마다 입력 x 를 받아 갱신된 평균을 반환
        # 부분 채움 단계 (n < window) 에서는 들어온 만큼만 평균 (분모 = n)
        # 권장: self.buffer (deque, maxlen=window) 에 x 를 append 하고, buffer 의 평균 반환
        raise NotImplementedError
