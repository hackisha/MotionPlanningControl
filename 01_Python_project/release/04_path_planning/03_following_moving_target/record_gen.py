"""Following Moving Target — leading vehicle 가 lane keep 하고, ego 가 leading 의
local-frame 위치 history 로부터 cubic path 만들어 추종.

3D 시각: leading (회색 차량) + ego (파랑 차량) + 양쪽 차선 (white edge) + 중앙선.
ego 의 매 step planner_path (dynamic_paths) 가 leading 의 자취를 향해 cubic 모양으로 뻗음.
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
# 01_both_lane 의 planner 도 leading vehicle 의 lane keep 용으로 재사용.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "01_both_lane"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from both_lane_planner import both_lane_to_path
from debug_signals import DebugSignals
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lane_following import lane, lane_center
from pure_pursuit import PurePursuit
from target_following_planner import LeadingTargetTracker, target_following_path
from vehicle_lat_following import VehicleLat

DT = 0.1
SIM_TIME = 30.0
VX = 10.0
NUM_SAMPLE = 5
DEGREE = 3
SAMPLE_XS = np.arange(NUM_SAMPLE) * 1.0


def _global_to_local_single(point_xy: list[float], yaw_ego: float,
                            x_ego: float, y_ego: float) -> list[float]:
    """한 점의 global→local 좌표 변환 (LeadingTargetTracker 의 입력용)."""
    theta = -yaw_ego
    cos_t, sin_t = float(np.cos(theta)), float(np.sin(theta))
    dx = point_xy[0] - x_ego
    dy = point_xy[1] - y_ego
    return [cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy]


def run_sim() -> dict:
    steps = int(SIM_TIME / DT)
    leading = VehicleLat(dt=DT, vx=VX, X0=0.0, Y0=0.0)
    ego = VehicleLat(dt=DT, vx=VX, X0=-10.0, Y0=0.0)
    pp_lead = PurePursuit(L=4.0, lookahead_time=1.0)
    pp_ego = PurePursuit(L=4.0, lookahead_time=0.0)
    tracker = LeadingTargetTracker(max_history=5)

    # leading lane keep 파이프라인
    g2l_L = Global2Local(NUM_SAMPLE)
    g2l_R = Global2Local(NUM_SAMPLE)
    fitter_L = PolynomialFitting(DEGREE, NUM_SAMPLE)
    fitter_R = PolynomialFitting(DEGREE, NUM_SAMPLE)
    # ego 시각화용 path
    x_local = np.arange(0.0, 12.0, 0.5)
    ev_path = PolynomialValue(DEGREE, np.size(x_local))

    t = np.zeros(steps)
    X_lead = np.zeros(steps); Y_lead = np.zeros(steps); Yaw_lead = np.zeros(steps)
    X_ego = np.zeros(steps); Y_ego = np.zeros(steps); Yaw_ego = np.zeros(steps)
    follow_err = np.zeros(steps)  # ego 의 ego-lateral 오차 (leading 의 path 기준)
    delta_arr = np.zeros(steps)
    path_paths: list[list[list[float]]] = []
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄

    for i in range(steps):
        t[i] = i * DT
        X_lead[i] = leading.X; Y_lead[i] = leading.Y; Yaw_lead[i] = leading.Yaw
        X_ego[i] = ego.X; Y_ego[i] = ego.Y; Yaw_ego[i] = ego.Yaw

        # 1) leading lane keep — chapter 01 의 both_lane_to_path 재사용
        X_ref_lead = leading.X + SAMPLE_XS
        Y_L, Y_R = lane(X_ref_lead)
        g2l_L.convert(np.column_stack([X_ref_lead, Y_L]),
                      leading.Yaw, leading.X, leading.Y)
        g2l_R.convert(np.column_stack([X_ref_lead, Y_R]),
                      leading.Yaw, leading.X, leading.Y)
        cL = fitter_L.fit(g2l_L.local_points)
        cR = fitter_R.fit(g2l_R.local_points)
        c_lead_path = both_lane_to_path(cL, cR)
        delta_lead = pp_lead.step(c_lead_path, VX)
        leading.step(delta_lead, VX)

        # 2) ego: leading 의 ego-local 위치를 measurement 로 누적
        leading_in_ego_local = _global_to_local_single(
            [leading.X, leading.Y], ego.Yaw, ego.X, ego.Y)
        tracker.update(leading_in_ego_local, VX, ego.yawrate, DT)
        c_ego_path = target_following_path(tracker.history)
        delta_ego = pp_ego.step(c_ego_path, VX)
        delta_arr[i] = delta_ego
        ego.step(delta_ego, VX)

        # 추종 오차: leading 이 이전에 지나간 X 의 Y 와 ego.Y 차이를 보고 싶지만 단순화 —
        # ego 의 X 에서의 lane center 와의 차이 (lane 내 유지 여부)
        follow_err[i] = float(ego.Y - lane_center(np.array([ego.X]))[0])
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )

        # 시각화: ego 의 planner_path 를 ego frame 에서 global frame 으로
        ev_path.calculate(c_ego_path, x_local)
        cos_y, sin_y = np.cos(ego.Yaw), np.sin(ego.Yaw)
        path_paths.append([
            [float(ego.X + cos_y * lx - sin_y * ly),
             float(ego.Y + sin_y * lx + cos_y * ly)]
            for lx, ly in ev_path.points
        ])

    # 본 시나리오는 주변 차선 정보를 ego 가 사용하지 않음 — viz 에 차선 표시 X.
    # (leading 은 내부적으로 lane keep 하지만 시각 강조는 'ego 가 앞차만 보고 간다'.)

    return {
        "schema_version": 2,
        "module": "04_path_planning/03_following_moving_target",
        "dt": DT,
        "actors": [
            {"name": "target", "L": 4.0, "W": 2.0, "color": [150, 150, 150, 120],
             "t": t.tolist(), "X": X_lead.tolist(), "Y": Y_lead.tolist(),
             "Yaw": Yaw_lead.tolist()},
            {"name": "ego", "L": 4.0, "W": 2.0, "color": [50, 100, 220, 120],
             "t": t.tolist(), "X": X_ego.tolist(), "Y": Y_ego.tolist(),
             "Yaw": Yaw_ego.tolist()},
        ],
        "scalars": [
            {"name": "follow_err", "unit": "m", "t": t.tolist(),
             "value": follow_err.tolist()},
            {"name": "delta_ego", "unit": "rad", "t": t.tolist(),
             "value": delta_arr.tolist()},
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
        "dynamic_paths": [
            {"name": "ego_planner_path", "color": [255, 230, 80, 200], "radius": 0.08,
             "t": t.tolist(), "points_per_t": path_paths},
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Following Moving Target 시나리오 실행 → record.json 생성")
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
