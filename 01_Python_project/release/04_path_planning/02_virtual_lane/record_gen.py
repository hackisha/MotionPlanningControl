"""Virtual Lane path planning — 한쪽 차선만 valid 한 구간에서 가상 차선 사용.

3D 시각: ego (파랑 차량) + 양쪽 차선 (valid 구간만 그려짐 — invalid 는 gap) +
중앙선 (dashed). 매 step 의 path 가 dynamic_paths 로 시각화 — invalid 구간에서
가상 차선으로 중앙 추정이 어떻게 이뤄지는지 직관적으로 보임.
재생: 같은 폴더 ../simulator_path_planning.py.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "07_pure_pursuit"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lane_virtual import lane, lane_center
from pure_pursuit import PurePursuit
from vehicle_lat_virtual import VehicleLat
from virtual_lane_planner import LaneWidthEstimator, either_lane_to_path

DT = 0.1
SIM_TIME = 30.0
VX = 3.0
NUM_SAMPLE = 5
DEGREE = 3
SAMPLE_XS = np.arange(NUM_SAMPLE) * 1.0


def run_sim() -> dict:
    steps = int(SIM_TIME / DT)
    plant = VehicleLat(dt=DT, vx=VX)
    # [튜닝] PurePursuit lookahead_time / LaneWidthEstimator init — test 의 값은 변경 X
    pp = PurePursuit(L=4.0, lookahead_time=0.0)
    lw_est = LaneWidthEstimator(Lw_init=4.0)
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
    lw_arr = np.zeros(steps)
    path_paths: list[list[list[float]]] = []
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄

    for i in range(steps):
        t[i] = i * DT
        X_ego[i] = plant.X; Y_ego[i] = plant.Y; Yaw_ego[i] = plant.Yaw
        X_ref = plant.X + SAMPLE_XS
        Y_L, Y_R, valid_L_arr, valid_R_arr = lane(X_ref)
        valid_L = bool(np.all(valid_L_arr))
        valid_R = bool(np.all(valid_R_arr))
        points_L = np.column_stack([X_ref, Y_L])
        points_R = np.column_stack([X_ref, Y_R])
        g2l_L.convert(points_L, plant.Yaw, plant.X, plant.Y)
        g2l_R.convert(points_R, plant.Yaw, plant.X, plant.Y)
        coeff_L = fitter_L.fit(g2l_L.local_points)
        coeff_R = fitter_R.fit(g2l_R.local_points)
        # 학생 task: lane width 추정 + path 계수 결정
        lw_est.update(coeff_L, coeff_R, valid_L, valid_R)
        lw_arr[i] = lw_est.Lw
        coeff_path = either_lane_to_path(coeff_L, coeff_R, valid_L, valid_R, lw_est.Lw)
        delta = pp.step(coeff_path, VX)
        delta_arr[i] = delta
        plant.step(delta, VX)
        lateral_err[i] = float(plant.Y - lane_center(np.array([plant.X]))[0])
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )
        # path 시각화 (local → global)
        ev_path.calculate(coeff_path, x_local)
        cos_y, sin_y = np.cos(plant.Yaw), np.sin(plant.Yaw)
        path_paths.append([
            [float(plant.X + cos_y * lx - sin_y * ly),
             float(plant.Y + sin_y * lx + cos_y * ly)]
            for lx, ly in ev_path.points
        ])

    # static lane reference — invalid 구간은 split 으로 gap 표현
    X_lane_all = np.arange(0.0, float(X_ego.max()) + 10.0, 0.5)
    Y_L_all, Y_R_all, vL_all, vR_all = lane(X_lane_all)
    Y_C_all = lane_center(X_lane_all)

    def _valid_segments(X: np.ndarray, Y: np.ndarray, mask: np.ndarray
                        ) -> list[tuple[list[float], list[float]]]:
        """mask 가 True 인 연속 구간만 잘라 (X_list, Y_list) tuple 들로 반환."""
        segs: list[tuple[list[float], list[float]]] = []
        cur_X: list[float] = []
        cur_Y: list[float] = []
        for x, y, ok in zip(X, Y, mask, strict=True):
            if ok:
                cur_X.append(float(x))
                cur_Y.append(float(y))
            else:
                if len(cur_X) >= 2:
                    segs.append((cur_X, cur_Y))
                cur_X, cur_Y = [], []
        if len(cur_X) >= 2:
            segs.append((cur_X, cur_Y))
        return segs

    lanes_out: list[dict] = []
    for X_seg, Y_seg in _valid_segments(X_lane_all, Y_L_all, vL_all):
        lanes_out.append({"X": X_seg, "Y": Y_seg, "kind": "edge"})
    for X_seg, Y_seg in _valid_segments(X_lane_all, Y_R_all, vR_all):
        lanes_out.append({"X": X_seg, "Y": Y_seg, "kind": "edge"})
    lanes_out.append({"X": X_lane_all.tolist(), "Y": Y_C_all.tolist(), "kind": "dotted"})

    return {
        "schema_version": 2,
        "module": "04_path_planning/02_virtual_lane",
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
        "lanes": lanes_out,
        "scalars": [
            {"name": "lateral_err", "unit": "m", "t": t.tolist(),
             "value": lateral_err.tolist()},
            {"name": "delta", "unit": "rad", "t": t.tolist(), "value": delta_arr.tolist()},
            {"name": "Lw_est", "unit": "m", "t": t.tolist(), "value": lw_arr.tolist()},
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
        description="Virtual Lane 시나리오 실행 → record.json 생성")
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
