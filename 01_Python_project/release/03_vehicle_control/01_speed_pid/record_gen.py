"""Speed PID — 0 → 30 m/s tracking on VehicleLong.

record.json (Rerun 재생용 sidecar) 을 생성. `--plot` 시 plotly subplot HTML 도 함께.
재생: 같은 폴더의 simulator_vehicle_control.py 참조.

실행 전 `speed_pid.py` 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면 NotImplementedError.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from speed_pid import SpeedPID
from vehicle_long_speed import VehicleLong


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Speed PID 시나리오 실행 → record.json 생성")
    parser.add_argument("--plot", action="store_true",
                        help="plotly 정적 그래프(plot.html)도 생성 (기본: record.json 만)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()
    plot = args.plot
    no_viewer = args.no_viewer

    dt = 0.1
    sim_time = 50.0
    steps = int(sim_time / dt)
    reference = 30.0

    plant = VehicleLong(dt=dt, vx0=0.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = SpeedPID(kp=0.0, kd=0.0, ki=0.0, dt=dt)

    t = np.zeros(steps)
    x = np.zeros(steps)
    vx = np.zeros(steps)
    ax = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        x[i] = plant.x
        vx[i] = plant.vx
        ax[i] = plant.ax
        u = controller.step(reference=reference, measure=plant.vx)
        u_arr[i] = u
        plant.step(u)

    # plotly (opt-in: --plot) --------------------------------------------
    if plot:
        fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                            subplot_titles=("vx (true vs reference)",
                                            "control u_cmd  /  measured ax"),
                            vertical_spacing=0.08)
        fig.add_hline(y=reference, line=dict(color="black", dash="dash"),
                      annotation_text="reference", row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=vx, mode="lines", name="vx",
                                 line=dict(color="red", width=2)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=u_arr, mode="lines", name="u_cmd (PID)",
                                 line=dict(color="blue", width=1)), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=ax, mode="lines", name="ax (after clip + drag)",
                                 line=dict(color="purple", width=1, dash="dot")),
                      row=2, col=1)
        fig.update_xaxes(title_text="time (s)", row=2, col=1)
        fig.update_yaxes(title_text="vx (m/s)", row=1, col=1)
        fig.update_yaxes(title_text="acceleration (m/s²)", row=2, col=1)
        fig.update_layout(template="plotly_white",
                          title="Speed PID — closed-loop velocity tracking")
        plot_out = Path(__file__).parent / "plot.html"
        fig.write_html(plot_out, auto_open=True)
        print(f"[plot] saved → {plot_out}")

    # Rerun record sidecar -----------------------------------------------
    lane_x = [float(x.min()) - 10.0, float(x.max()) + 10.0]
    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/01_speed_pid",
        "dt": dt,
        "actors": [{
            "name": "ego",
            "L": 4.0, "W": 2.0,
            "color": [50, 100, 220, 120],
            "t": t.tolist(),
            "X": x.tolist(),
            "Y": [0.0] * steps,
            "Yaw": [0.0] * steps,
        }],
        "lanes": [
            {"X": lane_x, "Y": [1.75, 1.75], "kind": "edge"},
            {"X": lane_x, "Y": [-1.75, -1.75], "kind": "edge"},
        ],
        "scalars": [
            {"name": "vx", "unit": "m/s", "t": t.tolist(), "value": vx.tolist()},
            {"name": "ax", "unit": "m/s²", "t": t.tolist(), "value": ax.tolist()},
            {"name": "u_cmd", "unit": "m/s²", "t": t.tolist(), "value": u_arr.tolist()},
        ],
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_vehicle_control.py")

    if not no_viewer:
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_vehicle_control import replay_records
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
