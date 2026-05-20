"""PID + 2D KF — closed-loop with noisy measurement, model-based estimation.

3D 시각: ego (true Y) = 파랑 차량. measurement (noisy) = 노란 점 시계열,
KF estimate = 청록 marker. LPF (05) 와 같은 패턴, 다른 추정기.
재생: 같은 폴더의 simulator_pid.py.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from closed_loop_kf import closed_loop_step
from kalman_filter_2d import KalmanFilter2D
from pid_controller import PIDController
from plant_kf import Plant

VX_VISUAL = 5.0
LANE_HALF_W = 1.75


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PID + KF 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    dt = 0.1
    sim_time = 60.0
    steps = int(sim_time / dt)
    damping = 1.0
    m = 1.0
    noise_std = 0.25

    plant = Plant(
        dt=dt, y0=1.0, damping=damping, m=m,
        disturbance=0.1, measurement_noise_std=noise_std,
        rng=np.random.default_rng(seed=42),
    )
    A = np.array([[1.0, dt], [0.0, 1.0 - damping * dt / m]])
    B = np.array([0.0, dt / m])
    C = np.array([1.0, 0.0])
    # [튜닝] Q / R / PID 게인은 0 으로 초기화 — 학생이 직접 값을 골라 채워야 함.
    # test_*.py 의 값은 변경 X (합격 기준).
    Q = np.diag([0.0, 0.0])
    R = 0.0
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R,
                        x0=np.array([1.0, 0.0]), P0=10.0 * np.eye(2))
    pid = PIDController(kp=0.0, kd=0.0, ki=0.0, dt=dt)

    t = np.arange(steps) * dt
    y_true = np.zeros(steps)
    y_measure = np.zeros(steps)
    y_estimate = np.zeros(steps)
    u_arr = np.zeros(steps)
    prev_u = 0.0
    for i in range(steps):
        yt, ym, ye, u = closed_loop_step(plant, kf, pid, target=0.0, prev_u=prev_u)
        y_true[i], y_measure[i], y_estimate[i], u_arr[i] = yt, ym, ye, u
        prev_u = u

    x_visual = VX_VISUAL * t
    lane_x = [float(x_visual.min()) - 10.0, float(x_visual.max()) + 10.0]
    measure_points = [[float(xv), float(ym), 0.1] for xv, ym in zip(x_visual, y_measure)]
    estimate_points = [[float(xv), float(ye), 0.15] for xv, ye in zip(x_visual, y_estimate)]

    record = {
        "schema_version": 2,
        "module": "02_pid/06_pid_with_kf",
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
