"""Both Lane path planning — 좌·우 차선 평균으로 path 생성, pure pursuit 추종.

3D 시각: ego (파랑 차량) + 양쪽 차선 (white edge) + 중앙선 (dashed). 차선 좌우
samples 가 dynamic point trail, 매 step 생성되는 path 가 dynamic_paths 로 보임.
재생: 같은 폴더 ../simulator_path_planning.py.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# chapter 3 frame_transform + pure_pursuit 재사용 (intent: 05_frame_transform 결과물)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "07_pure_pursuit"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from both_lane_planner import both_lane_to_path
from debug_signals import DebugSignals
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lane_both import LANE_WIDTH, lane, lane_center
from pure_pursuit import PurePursuit
from vehicle_lat_both import VehicleLat

DT = 0.1
SIM_TIME = 30.0
VX = 3.0
NUM_SAMPLE = 5
DEGREE = 3
SAMPLE_XS = np.arange(NUM_SAMPLE) * 1.0   # 전방 0..4m, 1m 간격


def _polyval(coeff: np.ndarray, x: float) -> float:
    """coeff (degree+1, 1) column 을 x 에서 평가 (고차→저차 순)."""
    n = coeff.shape[0]
    return float(sum(coeff[j][0] * x ** (n - 1 - j) for j in range(n)))


def run_sim() -> dict:
    """폐루프 시뮬레이션 후 record dict 반환."""
    steps = int(SIM_TIME / DT)
    plant = VehicleLat(dt=DT, vx=VX)
    # [튜닝] PurePursuit lookahead_time 변경 시 응답 변화 확인 — test 의 값은 변경 X
    pp = PurePursuit(L=4.0, lookahead_time=0.0)
    g2l_L = Global2Local(NUM_SAMPLE)
    g2l_R = Global2Local(NUM_SAMPLE)
    fitter_L = PolynomialFitting(DEGREE, NUM_SAMPLE)
    fitter_R = PolynomialFitting(DEGREE, NUM_SAMPLE)
    x_local = np.arange(0.0, 5.0, 0.5)
    ev_path = PolynomialValue(DEGREE, np.size(x_local))

    t = np.zeros(steps)
    X_ego = np.zeros(steps); Y_ego = np.zeros(steps); Yaw_ego = np.zeros(steps)
    lateral_err = np.zeros(steps)
    delta_arr = np.zeros(steps)
    path_paths: list[list[list[float]]] = []  # 매 step 의 global frame path (viz)
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄

    for i in range(steps):
        t[i] = i * DT
        X_ego[i] = plant.X; Y_ego[i] = plant.Y; Yaw_ego[i] = plant.Yaw
        # 전방 lane sampling
        X_ref = plant.X + SAMPLE_XS
        Y_ref_L, Y_ref_R = lane(X_ref)
        points_L = np.column_stack([X_ref, Y_ref_L])
        points_R = np.column_stack([X_ref, Y_ref_R])
        # global → local
        g2l_L.convert(points_L, plant.Yaw, plant.X, plant.Y)
        g2l_R.convert(points_R, plant.Yaw, plant.X, plant.Y)
        # 좌·우 차선 polyfit (local frame)
        coeff_L = fitter_L.fit(g2l_L.local_points)
        coeff_R = fitter_R.fit(g2l_R.local_points)
        # 학생 task: 양쪽 차선 → 중앙 path 계수
        coeff_path = both_lane_to_path(coeff_L, coeff_R)
        # pure pursuit
        delta = pp.step(coeff_path, VX)
        delta_arr[i] = delta
        plant.step(delta, VX)
        # 측정: ego.X 에서의 lane center 와 실제 ego.Y 차이
        lateral_err[i] = float(plant.Y - lane_center(np.array([plant.X]))[0])
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )
        # path 시각화 — local frame 의 path 곡선을 ego 의 global pose 로 변환해 그림
        ev_path.calculate(coeff_path, x_local)
        local_pts = ev_path.points  # (N, 2)
        cos_y, sin_y = np.cos(plant.Yaw), np.sin(plant.Yaw)
        global_pts: list[list[float]] = []
        for lx, ly in local_pts:
            gx = plant.X + cos_y * lx - sin_y * ly
            gy = plant.Y + sin_y * lx + cos_y * ly
            global_pts.append([float(gx), float(gy)])
        path_paths.append(global_pts)

    # static lane reference (전체 X 범위)
    X_lane_all = np.arange(0.0, float(X_ego.max()) + 10.0, 0.5)
    Y_lane_L_all, Y_lane_R_all = lane(X_lane_all)
    Y_lane_C_all = lane_center(X_lane_all)

    return {
        "schema_version": 2,
        "module": "04_path_planning/01_both_lane",
        "dt": DT,
        "actors": [{
            "name": "ego",
            "L": 4.0, "W": 2.0,
            "color": [50, 100, 220, 120],
            "t": t.tolist(),
            "X": X_ego.tolist(),
            "Y": Y_ego.tolist(),
            "Yaw": Yaw_ego.tolist(),
        }],
        "lanes": [
            {"X": X_lane_all.tolist(), "Y": Y_lane_L_all.tolist(), "kind": "edge"},
            {"X": X_lane_all.tolist(), "Y": Y_lane_R_all.tolist(), "kind": "edge"},
            {"X": X_lane_all.tolist(), "Y": Y_lane_C_all.tolist(), "kind": "dotted"},
        ],
        "scalars": [
            {"name": "lateral_err", "unit": "m", "t": t.tolist(),
             "value": lateral_err.tolist()},
            {"name": "delta", "unit": "rad", "t": t.tolist(), "value": delta_arr.tolist()},
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
        "dynamic_paths": [
            {"name": "planner_path", "color": [255, 230, 80, 200], "radius": 0.08,
             "t": t.tolist(), "points_per_t": path_paths},
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Both Lane 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    record = run_sim()
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_path_planning.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_path_planning import replay_records  # type: ignore[no-redef]
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
