"""PID + LPF — closed-loop with noisy measurement.

3D 시각: ego (true Y) = 파랑 차량. measurement (noisy) = 차선 위 노란 점 시계열
(dynamic_points). LPF estimate 는 차량 옆 옅은 청록 marker 점.
재생: 같은 폴더의 simulator_pid.py.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from closed_loop_lpf import closed_loop_step
from debug_signals import DebugSignals
from low_pass_filter import LowPassFilter
from pid_controller import PIDController
from plant_lpf import Plant

VX_VISUAL = 5.0
LANE_HALF_W = 1.75


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PID + LPF 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    dt = 0.1
    sim_time = 60.0
    steps = int(sim_time / dt)

    plant = Plant(
        dt=dt, y0=1.0,
        disturbance=0.1,
        measurement_noise_std=0.25,
        rng=np.random.default_rng(seed=42),
    )
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    lpf = LowPassFilter(alpha=0.0)
    pid = PIDController(kp=0.0, kd=0.0, ki=0.0, dt=dt)

    t = np.arange(steps) * dt
    y_true = np.zeros(steps)
    y_measure = np.zeros(steps)
    y_estimate = np.zeros(steps)
    u_arr = np.zeros(steps)
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄
    for i in range(steps):
        yt, ym, ye, u = closed_loop_step(plant, lpf, pid, target=0.0)
        y_true[i], y_measure[i], y_estimate[i], u_arr[i] = yt, ym, ye, u
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )

    x_visual = VX_VISUAL * t
    lane_x = [float(x_visual.min()) - 10.0, float(x_visual.max()) + 10.0]

    # measurement 와 estimate 를 매 step 의 단일 marker 로 (dynamic_points)
    measure_points = [[float(xv), float(ym), 0.1] for xv, ym in zip(x_visual, y_measure)]
    estimate_points = [[float(xv), float(ye), 0.15] for xv, ye in zip(x_visual, y_estimate)]

    record = {
        "schema_version": 2,
        "module": "02_pid/05_pid_with_lpf",
        "dt": dt,
        "actors": [{
            "name": "ego",
            "L": 4.0, "W": 2.0,
            "color": [50, 100, 220, 120],
            "t": t.tolist(),
            "X": x_visual.tolist(),
            "Y": y_true.tolist(),
            "Yaw": [0.0] * steps,
        }],
        "lanes": [
            {"X": lane_x, "Y": [LANE_HALF_W, LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [-LANE_HALF_W, -LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [0.0, 0.0], "kind": "center"},
        ],
        "scalars": [
            {"name": "y_true", "unit": "m", "t": t.tolist(), "value": y_true.tolist()},
            {"name": "y_measure", "unit": "m", "t": t.tolist(), "value": y_measure.tolist()},
            {"name": "y_estimate", "unit": "m", "t": t.tolist(), "value": y_estimate.tolist()},
            {"name": "u_cmd", "unit": "N", "t": t.tolist(), "value": u_arr.tolist()},
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
        "dynamic_points": [
            {"name": "measurement", "color": [255, 200, 50, 200], "radius": 0.18,
             "t": t.tolist(), "points_per_t": measure_points},
            {"name": "estimate", "color": [80, 230, 200, 220], "radius": 0.22,
             "t": t.tolist(), "points_per_t": estimate_points},
        ],
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_pid.py")

    if not args.no_viewer:
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_pid import replay_records
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
