"""Target Following Planner regression — behavioral spec (requirements level).

알고리즘 형태 (history rotation/shift + cubic 합성 / 다른 누적·fit 방식) 는 자유.
인터페이스 + leading vehicle 추종의 평균·최대 lateral 오차로 합격 판정.
"""
import sys
from pathlib import Path

import numpy as np
from lane_following import lane, lane_center
from target_following_planner import LeadingTargetTracker, target_following_path
from vehicle_lat_following import VehicleLat

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "07_pure_pursuit"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "01_both_lane"))
from both_lane_planner import both_lane_to_path  # noqa: E402
from frame_transform import Global2Local, PolynomialFitting  # noqa: E402
from pure_pursuit import PurePursuit  # noqa: E402


DT = 0.1
SIM_TIME = 30.0
VX = 10.0
NUM_SAMPLE = 5
DEGREE = 3
SAMPLE_XS = np.arange(NUM_SAMPLE) * 1.0


def test_target_following_path_returns_4_by_1():
    history = [[1.0, 0.1], [3.0, 0.3], [5.0, 0.5], [7.0, 0.7]]
    out = target_following_path(history)
    assert out.shape == (4, 1)


def test_target_following_path_short_history_zero():
    out = target_following_path([[1.0, 0.1], [2.0, 0.2]])
    assert np.allclose(out, np.zeros((4, 1)))


def test_tracker_max_history_window():
    tracker = LeadingTargetTracker(max_history=3)
    for i in range(5):
        tracker.update([float(i), 0.0], vx=0.0, yaw_rate=0.0, dt=0.1)
    assert len(tracker.history) == 3


def _g2l_single(pt: list[float], yaw: float, x: float, y: float) -> list[float]:
    theta = -yaw
    c, s = float(np.cos(theta)), float(np.sin(theta))
    dx, dy = pt[0] - x, pt[1] - y
    return [c * dx - s * dy, s * dx + c * dy]


def _run_closed_loop() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    leading = VehicleLat(dt=DT, vx=VX, X0=0.0, Y0=0.0)
    ego = VehicleLat(dt=DT, vx=VX, X0=-10.0, Y0=0.0)
    pp_lead = PurePursuit(L=4.0, lookahead_time=1.0)
    pp_ego = PurePursuit(L=4.0, lookahead_time=1.0)
    tracker = LeadingTargetTracker(max_history=5)
    g2l_L = Global2Local(NUM_SAMPLE)
    g2l_R = Global2Local(NUM_SAMPLE)
    fitter_L = PolynomialFitting(DEGREE, NUM_SAMPLE)
    fitter_R = PolynomialFitting(DEGREE, NUM_SAMPLE)
    steps = int(SIM_TIME / DT)
    follow_err = np.zeros(steps)
    X_lead = np.zeros(steps); X_ego = np.zeros(steps)
    for i in range(steps):
        # leading lane keep (chapter 01 의 both_lane_to_path 재사용)
        X_ref = leading.X + SAMPLE_XS
        Y_L, Y_R = lane(X_ref)
        g2l_L.convert(np.column_stack([X_ref, Y_L]), leading.Yaw, leading.X, leading.Y)
        g2l_R.convert(np.column_stack([X_ref, Y_R]), leading.Yaw, leading.X, leading.Y)
        cL = fitter_L.fit(g2l_L.local_points)
        cR = fitter_R.fit(g2l_R.local_points)
        c_lead = both_lane_to_path(cL, cR)
        leading.step(pp_lead.step(c_lead, VX), VX)
        # ego follow
        leading_local = _g2l_single([leading.X, leading.Y], ego.Yaw, ego.X, ego.Y)
        tracker.update(leading_local, VX, ego.yawrate, DT)
        c_ego = target_following_path(tracker.history)
        ego.step(pp_ego.step(c_ego, VX), VX)
        follow_err[i] = abs(ego.Y - lane_center(np.array([ego.X]))[0])
        X_lead[i] = leading.X; X_ego[i] = ego.X
    return follow_err, X_lead, X_ego


def test_closed_loop_following_within_spec():
    """leading 이 lane keep, ego 가 leading 의 자취 추종, 30 s:
       tail 평균 |follow_err| < 0.5 m, peak < 1.5 m, 충돌 X (ego.X < leading.X 유지).
    """
    follow_err, X_lead, X_ego = _run_closed_loop()
    n = len(follow_err)
    tail_mae = float(np.mean(follow_err[n // 2:]))
    peak = float(np.max(follow_err))
    assert tail_mae < 0.5, f"tail MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 1.5, f"peak |follow_err| {peak:.4f} m 임계값 초과"
    gap = X_lead - X_ego
    assert float(gap.min()) > 0.0, f"ego 가 leading 을 추월·충돌 (min gap = {gap.min():.2f})"
