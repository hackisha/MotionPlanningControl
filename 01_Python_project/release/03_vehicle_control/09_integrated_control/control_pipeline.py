"""09 control pipeline — perception + lateral ctrl + longitudinal ctrl + 종 mode 의사결정.

ego 는 자기 lane (lane2) 을 쭉 따라가되, target 이 ego 의 lane 으로 침범하면
종 제어가 speed mode → constant time-gap mode 로 전환되어 더 느린 속도로 추종.

4-단계 통합 step:
  1) perception (sample global ref → Global→Local → polynomial fit)
  2) lateral ctrl (LateralController adapter — 06/07/08 controller 누구든 호환)
  3) longitudinal mode 결정 (LongitudinalDecision — target invasion 감지 후 latch)
  4) longitudinal ctrl (speed PID / timegap PD dispatch)

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


def _polyval_at(coeff: np.ndarray, x: float) -> float:
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


# -- 도로 기하 (직선 → R=200 좌커브 → 접선 직선) ---------------------------------

@dataclass
class Road:
    """직선 → R=200 좌커브 (100m, ≈28.6°) → 접선 직선 의 도로.

    학생 시나리오 단순화를 위해 곡선 구간을 한정 — 무한 R 원호로 두면 ego 가
    sim 후반에 90° 가까이 회전해서 ACC/PP 의 단순 ego-X 기반 지표가 깨진다.
    """
    R: float = 200.0
    x_curve_start: float = 50.0
    x_curve_end: float = 150.0
    lane_offset: float = 1.75   # 도로 center 로부터 lane center 까지 (반차로폭)

    def y_center(self, x):
        x_curve_len = self.x_curve_end - self.x_curve_start
        y_end = self.R - float(np.sqrt(self.R**2 - x_curve_len**2))
        slope_end = x_curve_len / float(np.sqrt(self.R**2 - x_curve_len**2))
        if np.isscalar(x):
            xs = float(x)
            if xs < self.x_curve_start:
                return 0.0
            if xs <= self.x_curve_end:
                dx = xs - self.x_curve_start
                return float(self.R - np.sqrt(self.R**2 - dx**2))
            return float(y_end + slope_end * (xs - self.x_curve_end))
        x = np.asarray(x, dtype=float)
        out = np.zeros_like(x)
        m_curve = (x >= self.x_curve_start) & (x <= self.x_curve_end)
        dx = x[m_curve] - self.x_curve_start
        out[m_curve] = self.R - np.sqrt(self.R**2 - dx**2)
        m_after = x > self.x_curve_end
        out[m_after] = y_end + slope_end * (x[m_after] - self.x_curve_end)
        return out

    def lane1_center(self, x):
        return self.y_center(x) + self.lane_offset

    def lane2_center(self, x):
        return self.y_center(x) - self.lane_offset


# -- ego/target state ----------------------------------------------------------

@dataclass
class EgoState:
    X: float
    Y: float
    Yaw: float
    vx: float


@dataclass
class TargetState:
    X: float
    Y: float
    vx: float


# -- 종 mode 의사결정 ----------------------------------------------------------

class LongitudinalDecision:
    """target 의 road-frame Y 가 invasion threshold 통과 시 timegap mode latch.

    ego 의 lane (lane2, road frame y<0) 으로 target 이 진입하면 trigger. 한 번 latch
    되면 이후 sim 길이 동안 timegap 유지 (단순화 — 실제 ACC 는 mode hold/release 정책).
    """
    def __init__(self, road: Road, y_invasion_offset: float = 0.0):
        self.road = road
        self.y_invasion_offset = y_invasion_offset
        self.invaded = False

    def long_mode(self, t: float, ego: EgoState, target: TargetState) -> str:
        if not self.invaded:
            y_in_road = target.Y - self.road.y_center(target.X)
            self.invaded = y_in_road < self.y_invasion_offset
        return "timegap" if self.invaded else "speed"


# -- 통합 pipeline ------------------------------------------------------------

@dataclass
class PipelineOutput:
    delta: float                          # 조향 (rad)
    ax: float                             # 종가속 (m/s²)
    long_mode: str                        # "speed" | "timegap"
    coeff: np.ndarray                     # (degree+1, 1) local-frame poly 계수
    fit_local_points: np.ndarray          # (len(x_local), 2) viz: 곡선
    lookahead_local: tuple[float, float]  # viz: (x, y) local frame


class ControlPipeline:
    def __init__(
        self,
        g2l,
        fitter,
        ev,
        lat_ctrl,                          # .step(coeff, vx)->float, .lookahead_x(vx)->float
        long_ctrl,                         # .speed_step / .timegap_step
        decision: LongitudinalDecision,
        ref_y_fn: Callable[[np.ndarray], np.ndarray],   # 호출자가 주입 (보통 road.lane2_center)
        sample_xs: np.ndarray,
        x_local: np.ndarray,
        v_des: float,
    ):
        self.g2l = g2l
        self.fitter = fitter
        self.ev = ev
        self.lat_ctrl = lat_ctrl
        self.long_ctrl = long_ctrl
        self.decision = decision
        self.ref_y_fn = ref_y_fn
        self.sample_xs = np.asarray(sample_xs, dtype=float)
        self.x_local = np.asarray(x_local, dtype=float)
        self.v_des = v_des

    def step(self, t: float, ego: EgoState, target: TargetState) -> PipelineOutput:
        cos_y, sin_y = np.cos(ego.Yaw), np.sin(ego.Yaw)
        perception_x_global = ego.X + cos_y * self.sample_xs
        perception_y_global = self.ref_y_fn(perception_x_global)
        points_global = np.column_stack([perception_x_global, perception_y_global])
        self.g2l.convert(points_global, ego.Yaw, ego.X, ego.Y)
        self.fitter.fit(self.g2l.local_points)
        coeff = self.fitter.coeff
        self.ev.calculate(coeff, self.x_local)
        fit_local_points = self.ev.points.copy()
        delta = self.lat_ctrl.step(coeff, ego.vx)
        lookahead_x = self.lat_ctrl.lookahead_x(ego.vx)
        y_lh = _polyval_at(coeff, lookahead_x)
        mode = self.decision.long_mode(t, ego, target)
        ax_speed = self.long_ctrl.speed_step(self.v_des, ego.vx)
        if mode == "speed":
            ax = ax_speed
        else:
            gap = cos_y * (target.X - ego.X) + sin_y * (target.Y - ego.Y)
            ax_timegap = self.long_ctrl.timegap_step(gap, ego.vx, target.vx)
            ax = min(ax_speed, ax_timegap)
        return PipelineOutput(
            delta=float(delta),
            ax=float(ax),
            long_mode=mode,
            coeff=coeff,
            fit_local_points=fit_local_points,
            lookahead_local=(lookahead_x, y_lh),
        )

        # 1) perception (ego heading projection sampling — 곡선 도로 OK):
        #    cos_y, sin_y = cos(ego.Yaw), sin(ego.Yaw)
        #    x_global = ego.X + cos_y · self.sample_xs
        #    y_global = self.ref_y_fn(x_global)
        #    points   = np.column_stack([x_global, y_global])
        #    self.g2l.convert(points, ego.Yaw, ego.X, ego.Y)
        #    self.fitter.fit(self.g2l.local_points)
        #    coeff = self.fitter.coeff
        #    self.ev.calculate(coeff, self.x_local)
        #    fit_local_points = self.ev.points.copy()
        # 2) lateral ctrl:
        #    delta = self.lat_ctrl.step(coeff, ego.vx)
        #    lookahead_x = self.lat_ctrl.lookahead_x(ego.vx)
        #    y_lh = _polyval_at(coeff, lookahead_x)
        # 3) mode = self.decision.long_mode(t, ego, target)
        # 4) longitudinal ctrl — speed PID 는 항상 평가 (state 유지), timegap 은 mode 시만.
        #    ax_speed = self.long_ctrl.speed_step(self.v_des, ego.vx)
        #    if mode == "speed":   ax = ax_speed
        #    else (timegap):       gap = cos_y·(target.X-ego.X) + sin_y·(target.Y-ego.Y)
        #                          ax_timegap = self.long_ctrl.timegap_step(gap, ego.vx, target.vx)
        #                          ax = min(ax_speed, ax_timegap)   # 가속 capping
        # (ACC 의 timegap 식은 gap 멀면 양수 가속 명령까지 내지만, ego lane 침범 target 앞에서
        #  가속하면 안 됨 → speed 명령 (=0 평형) 으로 capping. 작은 쪽 = 더 보수적인 ax.)
        # return PipelineOutput(delta, ax, mode, coeff, fit_local_points, (lookahead_x, y_lh))
