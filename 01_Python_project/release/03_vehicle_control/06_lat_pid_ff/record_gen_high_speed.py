"""LatPIDFF — 고속(vx=10): 단일 ego 가 cos path 추종.

scalars 에 `lateral_error` 포함 — 학생이 viewer 에서 error 시계열 보고
FF (kff>0) 도입 효과를 직접 판단/실험.

본 driver 는 LateralPipeline 을 호출만 — perception+fit+control 흐름은
lateral_pipeline_pid_ff.py 책임.
재생: 03_vehicle_control/simulator_vehicle_control.py 참조.

실행 전 `lat_pid_ff.py` 와 `lateral_pipeline_pid_ff.py` 의 `# TODO` 를 구현해야 동작합니다 —
구현 전이면 NotImplementedError.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# 05_frame_transform 의 구현을 그대로 import (폴더명이 숫자 prefix 라 sys.path 추가 필요).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lat_pid_ff import LatPIDFF
from lateral_pipeline_pid_ff import LateralPipeline
from vehicle_lat_pid import VehicleLat

DT = 0.1
DEGREE = 3
NUM_POINT = 10
X_LOCAL = np.arange(0.0, 10.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * (5.0 / NUM_POINT)  # 0..4.5m, 0.5 간격


def _ref_y(x):
    """직선(첫 20m) → sine(이후) 경로. 차량은 도로 밖(Y0=1)에서 시작 — 첫 step 응답 +
    sine 진입을 함께 학습."""
    L_straight = 40.0
    if np.isscalar(x):
        return 0.0 if x < L_straight else 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0)
    return np.where(x < L_straight, 0.0, 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LatPIDFF 고속(vx=10) 시나리오 실행 → record_high_speed.json 생성")
    parser.add_argument("--plot", action="store_true",
                        help="plotly 정적 그래프(plot_high_speed.html)도 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()
    plot = args.plot
    no_viewer = args.no_viewer

    sim_time = 15.0
    vx = 10.0
    plant = VehicleLat(dt=DT, vx=vx, Y0=1.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    # kff=0 부터 시작해 lateral_error 시계열 본 뒤 FF 도입 여부/강도 직접 결정 권장.
    kp, kd, ki, kff = 0.5, 0.03, 0.0, 0.5
    controller = LatPIDFF(kp=kp, kd=kd, ki=ki, kff=kff, dt=DT)
    pipe = LateralPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, np.size(X_LOCAL)),
        controller=controller,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
    )

    steps = int(sim_time / DT)
    t = np.zeros(steps)
    X = np.zeros(steps)
    Y = np.zeros(steps)
    Yaw = np.zeros(steps)
    delta_arr = np.zeros(steps)
    err_arr = np.zeros(steps)
    fit_curves: list[list[list[float]]] = []  # 매 step 의 fit (global frame) — 떨림 시각 진단용
    lookahead_pts: list[list[float]] = []     # 매 step 의 lookahead point (global frame)
    # PID/FF 항별 contribution (controller 내부와 동일 식 — viz only)
    p_arr = np.zeros(steps); d_arr = np.zeros(steps); i_arr = np.zeros(steps); ff_arr = np.zeros(steps)
    prev_err: float | None = None
    err_sum: float = 0.0
    lookahead_x = vx * controller.lookahead_time
    for i in range(steps):
        t[i] = i * DT
        X[i] = plant.X
        Y[i] = plant.Y
        Yaw[i] = plant.Yaw
        err_arr[i] = plant.Y - float(_ref_y(plant.X))
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=lookahead_x)
        # local → global frame 회전 (viz)
        cos_y, sin_y = np.cos(plant.Yaw), np.sin(plant.Yaw)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        fit_global = (rot @ out.fit_local_points.T).T + np.array([plant.X, plant.Y])
        fit_curves.append(fit_global.tolist())
        lh_global = rot @ np.array(out.lookahead_local) + np.array([plant.X, plant.Y])
        lookahead_pts.append(lh_global.tolist())
        # PID/FF 항별 분리 (controller.step 의 식 재계산 — viz only)
        cur_err = out.lookahead_local[1]
        d_err = 0.0 if prev_err is None else (cur_err - prev_err) / DT
        err_sum += cur_err * DT
        p_arr[i] = kp * cur_err
        d_arr[i] = kd * d_err
        i_arr[i] = ki * err_sum
        ff_arr[i] = kff * (vx**2 * 2.0 * float(out.coeff[-3][0]))
        prev_err = cur_err
        delta_arr[i] = out.delta
        plant.step(out.delta, vx)

    # plotly (opt-in: --plot) --------------------------------------------
    if plot:
        X_ref = np.arange(0.0, 100.0, 0.1)
        Y_ref = _ref_y(X_ref)
        fig = make_subplots(rows=3, cols=1, shared_xaxes=False,
                            subplot_titles=("trajectory (X-Y) — reference vs ego",
                                            "lateral error (m) — FF 도입 여부 판단 단서",
                                            "control δ (steering, rad)"),
                            vertical_spacing=0.10)
        fig.add_trace(go.Scatter(x=X_ref, y=Y_ref, mode="lines", name="reference",
                                 line=dict(color="black", dash="dash")), row=1, col=1)
        fig.add_trace(go.Scatter(x=X, y=Y, mode="lines", name="ego",
                                 line=dict(color="red", width=2)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=err_arr, mode="lines", name="error",
                                 line=dict(color="red"), showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=delta_arr, mode="lines", name="δ",
                                 line=dict(color="blue"), showlegend=False), row=3, col=1)
        fig.update_xaxes(title_text="X (m)", row=1, col=1)
        fig.update_xaxes(title_text="time (s)", row=2, col=1)
        fig.update_xaxes(title_text="time (s)", row=3, col=1)
        fig.update_yaxes(title_text="Y (m)", row=1, col=1)
        fig.update_yaxes(title_text="error (m)", row=2, col=1)
        fig.update_yaxes(title_text="δ (rad)", row=3, col=1)
        fig.update_layout(template="plotly_white",
                          title=f"Lat PID+FF — high speed (vx={vx} m/s)")
        plot_out = Path(__file__).parent / "plot_high_speed.html"
        fig.write_html(plot_out, auto_open=True)
        print(f"[plot] saved → {plot_out}")

    # Rerun record sidecar -----------------------------------------------
    x_lo, x_hi = float(X.min()) - 5.0, float(X.max()) + 10.0
    ref_x = np.arange(x_lo, x_hi, 0.5)
    ref_y = _ref_y(ref_x)
    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/06_lat_pid_ff",
        "scenario": "high_speed",
        "dt": DT,
        "actors": [{
            "name": "ego", "L": 4.0, "W": 2.0, "color": [50, 100, 220, 120],
            "t": t.tolist(), "X": X.tolist(), "Y": Y.tolist(), "Yaw": Yaw.tolist(),
        }],
        "reference_path": {"X": ref_x.tolist(), "Y": ref_y.tolist()},
        "lanes": [
            {"X": ref_x.tolist(), "Y": (ref_y + 1.75).tolist(), "kind": "edge"},
            {"X": ref_x.tolist(), "Y": (ref_y - 1.75).tolist(), "kind": "edge"},
        ],
        "scalars": [
            {"name": "lateral_error", "unit": "m", "t": t.tolist(), "value": err_arr.tolist()},
            {"name": "delta", "unit": "rad", "t": t.tolist(), "value": delta_arr.tolist()},
            {"name": "p_term", "unit": "rad", "t": t.tolist(), "value": p_arr.tolist()},
            {"name": "d_term", "unit": "rad", "t": t.tolist(), "value": d_arr.tolist()},
            {"name": "i_term", "unit": "rad", "t": t.tolist(), "value": i_arr.tolist()},
            {"name": "ff_term", "unit": "rad", "t": t.tolist(), "value": ff_arr.tolist()},
        ],
        "dynamic_paths": [
            {"name": "fit", "color": [255, 150, 0, 200], "radius": 0.08,
             "t": t.tolist(), "points_per_t": fit_curves},
        ],
        "dynamic_points": [
            {"name": "lookahead", "color": [255, 220, 0, 230], "radius": 0.35,
             "t": t.tolist(), "points_per_t": lookahead_pts},
        ],
    }
    out_path = Path(__file__).parent / "record_high_speed.json"
    out_path.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out_path}  |  재생: simulator_vehicle_control.py")

    if not no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_vehicle_control import replay_records
        replay_records([out_path], camera="follow")


if __name__ == "__main__":
    main()
