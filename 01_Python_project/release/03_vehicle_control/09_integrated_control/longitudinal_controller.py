"""09 longitudinal controller — speed PID + constant time-gap PD.

평시는 speed mode (목표 vx 추종 PID). 외부 의사결정 (LongitudinalDecision) 이
target invasion 감지 시 timegap mode 로 전환 — 앞차와 일정 time-gap (τ·v_ego) 유지.

dispatch 는 ControlPipeline 이 mode 보고 speed_step / timegap_step 을 직접 호출.
timegap_step 의 `gap` 은 ego heading 방향 종방향 projection (곡선 도로에서도 안전한 정의).

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


class LongitudinalController:
    def __init__(
        self,
        dt: float,
        kp_v: float,
        kd_v: float,
        kp_g: float,
        kd_g: float,
        tau_gap: float = 1.5,
    ):
        self.dt = dt
        self.kp_v = kp_v
        self.kd_v = kd_v
        self.kp_g = kp_g
        self.kd_g = kd_g
        self.tau_gap = tau_gap
        self.prev_v_err: float | None = None

    def speed_step(self, v_des: float, v_ego: float) -> float:
        err = v_des - v_ego
        if self.prev_v_err is None:
            d_err = 0.0
        else:
            d_err = (err - self.prev_v_err) / self.dt

        ax = self.kp_v * err + self.kd_v * d_err
        self.prev_v_err = err
        return float(ax)

    def timegap_step(self, gap: float, v_ego: float, v_target: float) -> float:
        """gap = ego heading 방향 종방향 projection (m). desired = τ·v_ego."""
        constant_gap = self.tau_gap * v_ego
        gap_err = gap - constant_gap
        rel_v = v_target - v_ego
        ax = self.kp_g * gap_err + self.kd_g * rel_v
        return float(ax)
