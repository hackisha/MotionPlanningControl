"""Stanley — cos 경로 추종 (vx=3 m/s, k=1.0).

record.json (Rerun 재생용 sidecar) 을 생성. `--plot` 시 plotly subplot 도.
본 driver 는 본 폴더의 LateralPipeline 을 호출 — perception+fit+control 흐름은
lateral_pipeline_stanley.py 책임 (학생이 controller 와 함께 구현).
재생: 03_vehicle_control/simulator_vehicle_control.py 참조.

실행 전 `stanley.py` 와 `lateral_pipeline_stanley.py` 의 `# TODO` 를 구현해야 동작합니다 —
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

# 05_frame_transform 의 구현을 sys.path 로 직접 import (frame_transform 패턴).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_pipeline_stanley import LateralPipeline  # 본 폴더 (per-problem 사본)
from stanley import Stanley
from vehicle_lat_stanley import VehicleLat

DT = 0.1
DEGREE = 3
NUM_POINT = 5
X_LOCAL = np.arange(0.0, 10.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.0


def _ref_y(x):
    """직선(첫 20m) → sine(이후) 경로. 차량은 도로 밖(Y0=2)에서 시작 — 첫 step 응답 +
    sine 진입을 함께 학습."""
    L_straight = 40.0
    if np.isscalar(x):
        return 0.0 if x < L_straight else 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0)
    return np.where(x < L_straight, 0.0, 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stanley 시나리오 실행 → record.json 생성")
    parser.add_argument("--plot", action="store_true",
                        help="plotly 정적 그래프(plot.html)도 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()
    plot = args.plot
    no_viewer = args.no_viewer

    sim_time = 30.0
    vx = 10.0
    plant = VehicleLat(dt=DT, vx=vx, Y0=2.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    s = Stanley(k=0.0)
    pipe = LateralPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, np.size(X_LOCAL)),
        controller=s,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
    )

    steps = int(sim_time / DT)
    t = np.arange(steps) * DT
    X = np.zeros(steps)
    Y = np.zeros(steps)
    Yaw = np.zeros(steps)
    delta_arr = np.zeros(steps)
    err_arr = np.zeros(steps)
    fit_curves: list[list[list[float]]] = []
    lookahead_pts: list[list[float]] = []  # Stanley 는 cross-track 사용 → lookahead_x=0 (ego 위치)
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄
    for i in range(steps):
        X[i] = plant.X
        Y[i] = plant.Y
        Yaw[i] = plant.Yaw
        err_arr[i] = plant.Y - float(_ref_y(plant.X))
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=0.0)
        # local → global frame 회전 (viz)
        cos_y, sin_y = np.cos(plant.Yaw), np.sin(plant.Yaw)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        fit_global = (rot @ out.fit_local_points.T).T + np.array([plant.X, plant.Y])
        fit_curves.append(fit_global.tolist())
        lh_global = rot @ np.array(out.lookahead_local) + np.array([plant.X, plant.Y])
        lookahead_pts.append(lh_global.tolist())
        delta_arr[i] = out.delta
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )
        plant.step(out.delta, vx)

    # plotly (opt-in: --plot) --------------------------------------------
    if plot:
        X_ref = np.arange(0.0, 100.0, 0.1)
        Y_ref = _ref_y(X_ref)
        fig = make_subplots(rows=2, cols=1, shared_xaxes=False,
                            subplot_titles=("trajectory (X-Y) — reference vs Stanley",
                                            "control δ (steering, rad)"),
                            vertical_spacing=0.12)
        fig.add_trace(go.Scatter(x=X_ref, y=Y_ref, mode="lines", name="reference",
                                 line=dict(color="black", dash="dash")), row=1, col=1)
        fig.add_trace(go.Scatter(x=X, y=Y, mode="lines", name="ego (Stanley)",
                                 line=dict(color="red", width=2)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=delta_arr, mode="lines", name="δ",
                                 line=dict(color="blue"), showlegend=False), row=2, col=1)
        fig.update_xaxes(title_text="X (m)", row=1, col=1)
        fig.update_xaxes(title_text="time (s)", row=2, col=1)
        fig.update_yaxes(title_text="Y (m)", row=1, col=1)
        fig.update_yaxes(title_text="δ (rad)", row=2, col=1)
        fig.update_layout(template="plotly_white",
                          title=f"Stanley — cos path (vx={vx} m/s, k=1.0)")
        plot_out = Path(__file__).parent / "plot.html"
        fig.write_html(plot_out, auto_open=True)
        print(f"[plot] saved → {plot_out}")

    # Rerun record sidecar -----------------------------------------------
    x_lo, x_hi = float(X.min()) - 5.0, float(X.max()) + 10.0
    ref_x = np.arange(x_lo, x_hi, 0.5)
    ref_y = _ref_y(ref_x)
    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/08_stanley",
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
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
        "dynamic_paths": [
            {"name": "fit", "color": [255, 150, 0, 200], "radius": 0.08,
             "t": t.tolist(), "points_per_t": fit_curves},
        ],
        "dynamic_points": [
            {"name": "cross_track", "color": [255, 220, 0, 230], "radius": 0.35,
             "t": t.tolist(), "points_per_t": lookahead_pts},
        ],
    }
    out_path = Path(__file__).parent / "record.json"
    out_path.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out_path}  |  재생: simulator_vehicle_control.py")

    if not no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_vehicle_control import replay_records
        replay_records([out_path], camera="follow")


if __name__ == "__main__":
    main()
