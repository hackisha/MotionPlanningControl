"""PurePursuit regression — behavioral spec (requirements level).

알고리즘 형태 (정통 pure-pursuit / 다른 lookahead 처리) 는 자유.
인터페이스만 맞으면 OK — pipeline 폐루프 lateral 추종의 평균·최대 오차로 합격 판정.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_pipeline_pure_pursuit import LateralPipeline
from pure_pursuit import PurePursuit
from vehicle_lat_pursuit import VehicleLat

DT = 0.1


def test_curved_path_tracking_within_spec():
    """직선(40m) + sin path, vx=3, Y0=2 (도로 밖 시작), 30 s:
       tail 평균 |lateral err| < 0.4, peak < 2.2.
    """
    sim_time = 30.0
    vx = 3.0
    L_vehicle = 4.0
    DEGREE = 3
    NUM_POINT = 5
    X_LOCAL = np.arange(0.0, 5.0, 0.5)
    SAMPLE_XS = np.arange(NUM_POINT) * 1.0

    plant = VehicleLat(dt=DT, vx=vx, L=L_vehicle, Y0=2.0)
    pp = PurePursuit(L=L_vehicle, lookahead_time=0.8)
    pipe = LateralPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, np.size(X_LOCAL)),
        controller=pp,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
    )

    def _ref_y(x):
        return np.where(x < 40.0, 0.0, 2.0 * (np.cos((x - 40.0) / 14.0) - 1.0))

    lookahead_x = vx * pp.lookahead_time
    errs = []
    for _ in range(int(sim_time / DT)):
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=lookahead_x)
        plant.step(out.delta, vx)
        ref_at_ego = 0.0 if plant.X < 40.0 else 2.0 * (np.cos((plant.X - 40.0) / 14.0) - 1.0)
        errs.append(abs(plant.Y - ref_at_ego))
    errs = np.array(errs)

    tail_mae = float(np.mean(errs[len(errs) // 2:]))
    peak = float(np.max(errs))
    assert tail_mae < 0.4, f"tail lat MAE {tail_mae:.4f} m 임계값 초과"
    assert peak < 2.2, f"peak |lat error| {peak:.4f} m 임계값 초과"
