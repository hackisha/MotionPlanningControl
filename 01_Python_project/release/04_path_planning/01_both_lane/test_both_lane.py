"""Both Lane Planner regression — behavioral spec (requirements level).

알고리즘 형태 (정통 좌·우 평균 / 다른 결합 방식) 는 제약 X.
인터페이스 + 폐루프 평균·최대 lateral 오차로 합격 판정.
"""
import sys
from pathlib import Path

import numpy as np
from both_lane_planner import both_lane_to_path
from lane_both import lane, lane_center
from vehicle_lat_both import VehicleLat

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


def test_returns_same_shape_column():
    coeff_L = np.array([[0.001], [0.01], [0.1], [2.0]])
    coeff_R = np.array([[0.001], [0.01], [0.1], [-2.0]])
    out = both_lane_to_path(coeff_L, coeff_R)
    assert out.shape == coeff_L.shape, f"shape mismatch: got {out.shape}"


def _run_closed_loop() -> np.ndarray:
    """폐루프 시뮬 후 |lateral_err| 시계열 반환."""
    plant = VehicleLat(dt=DT, vx=VX)
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    g2l_L = Global2Local(NUM_SAMPLE)
    g2l_R = Global2Local(NUM_SAMPLE)
    fitter_L = PolynomialFitting(DEGREE, NUM_SAMPLE)
    fitter_R = PolynomialFitting(DEGREE, NUM_SAMPLE)
    steps = int(SIM_TIME / DT)
    lat = np.zeros(steps)
    for i in range(steps):
        X_ref = plant.X + SAMPLE_XS
        Y_L, Y_R = lane(X_ref)
        g2l_L.convert(np.column_stack([X_ref, Y_L]), plant.Yaw, plant.X, plant.Y)
        g2l_R.convert(np.column_stack([X_ref, Y_R]), plant.Yaw, plant.X, plant.Y)
        cL = fitter_L.fit(g2l_L.local_points)
        cR = fitter_R.fit(g2l_R.local_points)
        coeff_path = both_lane_to_path(cL, cR)
        plant.step(pp.step(coeff_path, VX), VX)
        lat[i] = abs(plant.Y - lane_center(np.array([plant.X]))[0])
    return lat


def test_closed_loop_tracking_within_spec():
    """sinusoidal both-lane 도로 30 s 추종:
       tail 평균 |lateral err| < 0.3 m, peak |lateral err| < 1.0 m.

    trivial 구현 (coeff_path = 0) 은 도로 곡선 못 따라가 임계값 초과로 차단.
    """
    lat = _run_closed_loop()
    n = len(lat)
    tail_mae = float(np.mean(lat[n // 2:]))
    peak = float(np.max(lat))
    assert tail_mae < 0.3, f"tail MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 1.0, f"peak |lateral err| {peak:.4f} m 임계값 초과"
