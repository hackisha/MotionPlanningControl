"""TimeGap — 정속(constant-velocity) target 추종, 1초 time-gap.

record_stationary.json (Rerun 재생용 sidecar) 을 생성. `--plot` 시 plotly subplot 도.
재생: 03_vehicle_control/simulator_vehicle_control.py 참조.

실행 전 `time_gap_pid.py` 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면 NotImplementedError.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from time_gap_pid import TimeGapPID
from vehicle_long_tg import VehicleLong


def main() -> None:
    parser = argparse.ArgumentParser(
        description="TimeGap (정속 target) 시나리오 실행 → record_stationary.json 생성"
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="plotly 정적 그래프(plot_stationary.html)도 생성",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)",
    )
    args = parser.parse_args()
    plot = args.plot
    no_viewer = args.no_viewer

    dt = 0.1
    sim_time = 50.0
    steps = int(sim_time / dt)
    time_gap = 1.0

    # 초기 조건 = ACC 인계 시점: 이미 정상 time-gap 거리 (ego_vx · time_gap = 10·1 = 10m).
    target = VehicleLong(dt=dt, Ca=0.0, x0=10.0, vx0=10.0)
    ego = VehicleLong(dt=dt, Ca=0.5, x0=0.0, vx0=10.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    pid = TimeGapPID(kp=0.3, kd=0.5, ki=0.0, dt=dt, time_gap=time_gap)

    t = np.zeros(steps)
    x_target = np.zeros(steps)
    x_ego = np.zeros(steps)
    gap = np.zeros(steps)
    tg_actual = np.zeros(steps)
    vx_ego = np.zeros(steps)
    vx_tgt = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        x_target[i] = target.x
        x_ego[i] = ego.x
        gap[i] = target.x - ego.x
        tg_actual[i] = (target.x - ego.x) / max(ego.vx, 1e-3)
        vx_ego[i] = ego.vx
        vx_tgt[i] = target.vx
        u = pid.step(target_x=target.x, ego_x=ego.x, ego_vx=ego.vx)
        u_arr[i] = u
        ego.step(u)
        target.step(0.0)

    # plotly (opt-in: --plot) --------------------------------------------
    if plot:
        fig = make_subplots(
            rows=3,
            cols=1,
            shared_xaxes=True,
            subplot_titles=(
                "inter-vehicle gap (m)",
                "actual time-gap (s)  /  velocities (m/s)",
                "control u (m/s²)",
            ),
            vertical_spacing=0.06,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=gap,
                mode="lines",
                name="gap (m)",
                line=dict(color="red", width=2),
            ),
            row=1,
            col=1,
        )
        fig.add_hline(
            y=time_gap,
            line=dict(color="black", dash="dash"),
            annotation_text=f"target time-gap = {time_gap}s",
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=tg_actual,
                mode="lines",
                name="time-gap (s)",
                line=dict(color="orange", width=2),
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=vx_tgt,
                mode="lines",
                name="vx target",
                line=dict(color="blue", width=1, dash="dot"),
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=vx_ego,
                mode="lines",
                name="vx ego",
                line=dict(color="red", width=1, dash="dot"),
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=u_arr,
                mode="lines",
                name="u_cmd",
                line=dict(color="purple", width=1),
                showlegend=False,
            ),
            row=3,
            col=1,
        )
        fig.update_xaxes(title_text="time (s)", row=3, col=1)
        fig.update_yaxes(title_text="gap (m)", row=1, col=1)
        fig.update_yaxes(title_text="time-gap (s) / vx (m/s)", row=2, col=1)
        fig.update_yaxes(title_text="ax (m/s²)", row=3, col=1)
        fig.update_layout(
            template="plotly_white",
            title="TimeGap following — stationary (constant-velocity) target",
        )
        plot_out = Path(__file__).parent / "plot_stationary.html"
        fig.write_html(plot_out, auto_open=True)
        print(f"[plot] saved → {plot_out}")

    # Rerun record sidecar -----------------------------------------------
    lane_lo = float(min(x_ego.min(), x_target.min())) - 10.0
    lane_hi = float(max(x_ego.max(), x_target.max())) + 10.0
    lane_x = [lane_lo, lane_hi]
    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/03_time_gap",
        "scenario": "stationary",
        "dt": dt,
        "actors": [
            {
                "name": "target",
                "L": 4.0,
                "W": 2.0,
                "color": [150, 150, 150, 120],
                "t": t.tolist(),
                "X": x_target.tolist(),
                "Y": [0.0] * steps,
                "Yaw": [0.0] * steps,
            },
            {
                "name": "ego",
                "L": 4.0,
                "W": 2.0,
                "color": [50, 100, 220, 120],
                "t": t.tolist(),
                "X": x_ego.tolist(),
                "Y": [0.0] * steps,
                "Yaw": [0.0] * steps,
            },
        ],
        "lanes": [
            {"X": lane_x, "Y": [1.75, 1.75], "kind": "edge"},
            {"X": lane_x, "Y": [-1.75, -1.75], "kind": "edge"},
        ],
        "scalars": [
            {"name": "gap", "unit": "m", "t": t.tolist(), "value": gap.tolist()},
            {
                "name": "time_gap",
                "unit": "s",
                "t": t.tolist(),
                "value": tg_actual.tolist(),
            },
            {
                "name": "vx_target",
                "unit": "m/s",
                "t": t.tolist(),
                "value": vx_tgt.tolist(),
            },
            {
                "name": "vx_ego",
                "unit": "m/s",
                "t": t.tolist(),
                "value": vx_ego.tolist(),
            },
            {"name": "u_cmd", "unit": "m/s²", "t": t.tolist(), "value": u_arr.tolist()},
        ],
    }
    out = Path(__file__).parent / "record_stationary.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_vehicle_control.py")

    if not no_viewer:
        import sys

        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_vehicle_control import replay_records

        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
