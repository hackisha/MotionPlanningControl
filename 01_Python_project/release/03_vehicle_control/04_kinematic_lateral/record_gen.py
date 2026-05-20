"""KinematicLateral — 직선 Y_ref=4 추종 (vx=3 m/s).

record.json (Rerun 재생용 sidecar) 을 생성. `--plot` 시 plotly subplot 도.
재생: 03_vehicle_control/simulator_vehicle_control.py 참조.

실행 전 `kinematic_lateral_pid.py` 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면 NotImplementedError.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from kinematic_lateral_pid import KinematicLateralPID
from vehicle_lat_kinematic import VehicleLat


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Kinematic Lateral PID 시나리오 실행 → record.json 생성")
    parser.add_argument("--plot", action="store_true",
                        help="plotly 정적 그래프(plot.html)도 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()
    plot = args.plot
    no_viewer = args.no_viewer

    dt = 0.1
    sim_time = 30.0
    steps = int(sim_time / dt)
    vx = 3.0
    Y_ref = 4.0

    plant = VehicleLat(dt=dt, vx=vx)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    pid = KinematicLateralPID(kp=0.0, kd=0.0, ki=0.0, dt=dt)

    t = np.zeros(steps)
    X = np.zeros(steps)
    Y = np.zeros(steps)
    Yaw = np.zeros(steps)
    delta_arr = np.zeros(steps)
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄
    for i in range(steps):
        t[i] = i * dt
        X[i] = plant.X
        Y[i] = plant.Y
        Yaw[i] = plant.Yaw
        delta = pid.step(reference_Y=Y_ref, ego_Y=plant.Y)
        delta_arr[i] = delta
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )
        plant.step(delta, vx)

    # plotly (opt-in: --plot) --------------------------------------------
    if plot:
        fig = make_subplots(rows=3, cols=1, shared_xaxes=False,
                            subplot_titles=("trajectory (X-Y)",
                                            "Y(t) vs reference",
                                            "control δ (steering, rad)"),
                            vertical_spacing=0.08)
        fig.add_hline(y=Y_ref, line=dict(color="black", dash="dash"),
                      annotation_text="reference Y", row=1, col=1)
        fig.add_trace(go.Scatter(x=X, y=Y, mode="lines", name="trajectory",
                                 line=dict(color="red", width=2)), row=1, col=1)
        fig.add_hline(y=Y_ref, line=dict(color="black", dash="dash"),
                      annotation_text="reference Y", row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=Y, mode="lines", name="Y(t)",
                                 line=dict(color="red", width=2),
                                 showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=delta_arr, mode="lines", name="δ",
                                 line=dict(color="blue", width=1),
                                 showlegend=False), row=3, col=1)
        fig.update_xaxes(title_text="X (m)", row=1, col=1)
        fig.update_xaxes(title_text="time (s)", row=2, col=1)
        fig.update_xaxes(title_text="time (s)", row=3, col=1)
        fig.update_yaxes(title_text="Y (m)", row=1, col=1)
        fig.update_yaxes(title_text="Y (m)", row=2, col=1)
        fig.update_yaxes(title_text="δ (rad)", row=3, col=1)
        fig.update_layout(template="plotly_white",
                          title=f"Kinematic Lateral PID — straight-line Y-tracking (vx={vx} m/s)")
        plot_out = Path(__file__).parent / "plot.html"
        fig.write_html(plot_out, auto_open=True)
        print(f"[plot] saved → {plot_out}")

    # Rerun record sidecar -----------------------------------------------
    ref_x = [float(X.min()) - 5.0, float(X.max()) + 10.0]
    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/04_kinematic_lateral",
        "dt": dt,
        "actors": [{
            "name": "ego", "L": 4.0, "W": 2.0, "color": [50, 100, 220, 120],
            "t": t.tolist(), "X": X.tolist(), "Y": Y.tolist(), "Yaw": Yaw.tolist(),
        }],
        "reference_path": {"X": ref_x, "Y": [Y_ref, Y_ref]},
        "lanes": [
            {"X": ref_x, "Y": [Y_ref + 1.75, Y_ref + 1.75], "kind": "edge"},
            {"X": ref_x, "Y": [Y_ref - 1.75, Y_ref - 1.75], "kind": "edge"},
        ],
        "scalars": [
            {"name": "Y", "unit": "m", "t": t.tolist(), "value": Y.tolist()},
            {"name": "delta", "unit": "rad", "t": t.tolist(), "value": delta_arr.tolist()},
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
