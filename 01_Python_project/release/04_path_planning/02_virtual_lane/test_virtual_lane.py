"""Virtual Lane Planner regression — behavioral spec (requirements level).

알고리즘 형태 (가상 차선 보정 / 차선폭 추정) 는 자유.
인터페이스 + invalid 구간 포함 폐루프 평균·최대 lateral 오차로 합격 판정.
"""
import sys
from pathlib import Path

import numpy as np
from lane_virtual import lane, lane_center
from vehicle_lat_virtual import VehicleLat
from virtual_lane_planner import LaneWidthEstimator, either_lane_to_path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "07_pure_pursuit"))
from frame_transform import Global2Local, PolynomialFitting  # noqa: E402
from pure_pursuit import PurePursuit  # noqa: E402


DT = 0.1
SIM_TIME = 30.0
VX = 3.0
NUM_SAMPLE = 5
DEGREE = 3
SAMPLE_XS = np.arange(NUM_SAMPLE) * 1.0


def test_either_lane_shape():
    coeff_L = np.array([[0.0], [0.0], [0.0], [2.0]])
    coeff_R = np.array([[0.0], [0.0], [0.0], [-2.0]])
    out = either_lane_to_path(coeff_L, coeff_R, True, True, 4.0)
    assert out.shape == coeff_L.shape


def test_lane_width_estimator_updates_only_when_both_valid():
    est = LaneWidthEstimator(Lw_init=4.0)
    coeff_L = np.array([[0.0], [0.0], [0.0], [3.0]])
    coeff_R = np.array([[0.0], [0.0], [0.0], [-3.0]])
    est.update(coeff_L, coeff_R, True, True)
    assert abs(est.Lw - 6.0) < 1e-9
    est.update(coeff_L, coeff_R, True, False)
    assert abs(est.Lw - 6.0) < 1e-9


def _run_closed_loop() -> np.ndarray:
    plant = VehicleLat(dt=DT, vx=VX)
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    lw_est = LaneWidthEstimator(Lw_init=4.0)
    g2l_L = Global2Local(NUM_SAMPLE)
    g2l_R = Global2Local(NUM_SAMPLE)
    fitter_L = PolynomialFitting(DEGREE, NUM_SAMPLE)
    fitter_R = PolynomialFitting(DEGREE, NUM_SAMPLE)
    steps = int(SIM_TIME / DT)
    lat = np.zeros(steps)
    for i in range(steps):
        X_ref = plant.X + SAMPLE_XS
        Y_L, Y_R, vL_arr, vR_arr = lane(X_ref)
        valid_L = bool(np.all(vL_arr))
        valid_R = bool(np.all(vR_arr))
        g2l_L.convert(np.column_stack([X_ref, Y_L]), plant.Yaw, plant.X, plant.Y)
        g2l_R.convert(np.column_stack([X_ref, Y_R]), plant.Yaw, plant.X, plant.Y)
        cL = fitter_L.fit(g2l_L.local_points)
        cR = fitter_R.fit(g2l_R.local_points)
        lw_est.update(cL, cR, valid_L, valid_R)
        coeff_path = either_lane_to_path(cL, cR, valid_L, valid_R, lw_est.Lw)
        plant.step(pp.step(coeff_path, VX), VX)
        lat[i] = abs(plant.Y - lane_center(np.array([plant.X]))[0])
    return lat


def test_closed_loop_through_invalid_sections():
    """X ∈ [20, 40] 좌 invalid, X ∈ [60, 80] 우 invalid 구간 통과:
       tail 평균 |lateral err| < 0.4 m, peak < 1.5 m.

    가상 차선이 작동해야 invalid 구간을 지나가는 동안에도 path 가 끊기지 않음.
    trivial 구현 (lane width 무시 등) 은 invalid 구간에서 큰 peak 으로 차단.
    """
    lat = _run_closed_loop()
    n = len(lat)
    tail_mae = float(np.mean(lat[n // 2:]))
    peak = float(np.max(lat))
    assert tail_mae < 0.4, f"tail MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 1.5, f"peak |lateral err| {peak:.4f} m 임계값 초과"
