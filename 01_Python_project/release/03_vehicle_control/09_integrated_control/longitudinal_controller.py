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
        # TODO: 속도 PID.
        # 1) err = v_des - v_ego
        # 2) 첫 호출 D=0; 이후 d_err = (err - prev_err) / dt
        # 3) ax = kp_v · err + kd_v · d_err
        # 4) prev_v_err 갱신
        raise NotImplementedError

    def timegap_step(self, gap: float, v_ego: float, v_target: float) -> float:
        """gap = ego heading 방향 종방향 projection (m). desired = τ·v_ego."""
        # TODO: constant time-gap PD.
        # 1) desired_gap = tau_gap · v_ego
        # 2) gap_err = gap - desired_gap        (양수: 멀어진 상태)
        # 3) rel_v = v_target - v_ego           (양수: target 이 멀어지는 중)
        # 4) ax = kp_g · gap_err + kd_g · rel_v
        raise NotImplementedError
