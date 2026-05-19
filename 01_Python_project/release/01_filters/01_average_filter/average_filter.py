"""Average Filter — recursive cumulative mean.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class AverageFilter:
    def __init__(self):
        self.n = 0
        self.avg = 0.0

    def step(self, x: float) -> float:
        self.n += 1
        self.avg += (x - self.avg) / self.n
        return self.avg
    
        # TODO: 누적 평균 갱신식을 작성하시오. (재귀 형태)
        # 인터페이스: 매 호출마다 입력 x 를 받아 갱신된 평균을 반환
        # 첫 호출에서는 x 자체가 반환되어야 함 (n=1 일 때 avg = x)
        raise NotImplementedError