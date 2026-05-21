"""Target Following Planner — leading vehicle 의 ego-local 위치 history 로부터
ego 가 따라갈 path 의 polynomial 계수 생성.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import numpy as np


class LeadingTargetTracker:
    """ego frame 기준 leading vehicle 의 직전 N 위치를 저장. 매 step 회전 + 시프트로 갱신."""

    def __init__(self, max_history: int = 5):
        self.max_history = max_history
        self.history: list[list[float]] = []

    def update(self, target_local_xy: list[float],
               vx: float, yaw_rate: float, dt: float) -> None:
        """현재 step 의 leading local 좌표를 history 에 추가하고, 직전 history
        포인트들은 새 ego frame 좌표계로 갱신 (역회전 + 역시프트).

        Args:
            target_local_xy: 현재 step ego local frame 의 leading 좌표 [x, y].
            vx, yaw_rate, dt: ego 의 직전 step 운동.
        """
        # TODO: 회전 + 시프트 후 윈도우 trim.
        # 힌트:
        #   - theta = yaw_rate * dt
        #   - rot = [[cos, sin], [-sin, cos]]
        #   - |yaw_rate| 가 매우 작으면 직진 근사: shift = [vx*dt, 0]
        #     아니면: shift = [vx*dt, -vx*(1-cos(theta))/yaw_rate]
        #   - history.append → (필요시 pop(0)) → 모든 점에 rot 적용 후 shift 빼기.
        raise NotImplementedError


def target_following_path(history: list[list[float]]) -> np.ndarray:
    """history 가 ego origin 을 통과하고 마지막 점의 heading 과 일치하는 3차 path 생성.

    반환: shape (4, 1) column. history 길이 4 미만이면 zero coeff 반환 (직진).

    제약 조건 3개:
      - y(0) = 0      (ego origin 통과)
      - y(xf) = yf    (last history 점 통과)
      - y'(xf) = tan(heading)  (그 점에서 history slope 추정치와 heading 일치)
    이 셋을 만족하는 path 형태: y = c3·x³ + c2·x² + 0·x + 0
    """
    # TODO: 부족 history → 0 계수. 충분하면 polyfit 으로 heading 추정 → c3, c2 풀이.
    # 힌트:
    #   - polyfit (3차) → coeff [a3, a2, a1, a0] → heading = 3·a3·xf² + 2·a2·xf + a1
    #   - tan_h = tan(heading)
    #   - c3 = -(2·xf·yf − xf²·tan_h) / xf⁴
    #   - c2 =  (3·xf²·yf − xf³·tan_h) / xf⁴
    raise NotImplementedError
