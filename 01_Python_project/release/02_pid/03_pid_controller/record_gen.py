"""PID Controller — PD vs PID under constant disturbance.

PD (Ki=0) 와 PID (Ki=0.5) 두 컨트롤러를 같은 plant + 외란 시나리오에서 동시에
시뮬레이션, 두 차량 (PD=회색, PID=파랑) 을 같은 X 축 / 다른 Y 트랙으로 보여줌.
정상상태에서 PD 는 차선 위쪽으로 offset, PID 는 중앙선 (Y=0) 으로 수렴.

재생: 같은 폴더의 simulator_pid.py.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from pid_controller import PIDController
from plant_pid import Plant

VX_VISUAL = 5.0
LANE_HALF_W = 1.75


def _run(kp: float, kd: float, ki: float, dt: float, sim_time: float,
         disturbance: float) -> tuple[np.ndarray, np.ndarray]:
    steps = int(sim_time / dt)
    plant = Plant(dt, y0=1.0, disturbance=disturbance)
    controller = PIDController(kp=kp, kd=kd, ki=ki, dt=dt)
    y = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        y[i] = plant.y
        u = controller.step(reference=0.0, measure=plant.y)
        u_arr[i] = u
        plant.step(u)
    return y, u_arr


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PID Controller (PD vs PID) 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    dt = 0.1
    sim_time = 60.0
    steps = int(sim_time / dt)
    disturbance = 0.5

    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    y_pd, u_pd = _run(kp=0.0, kd=0.0, ki=0.0, dt=dt,
                      sim_time=sim_time, disturbance=disturbance)
    y_pid, u_pid = _run(kp=0.0, kd=0.0, ki=0.0, dt=dt,
                        sim_time=sim_time, disturbance=disturbance)

    t = np.arange(steps) * dt
    x_visual = VX_VISUAL * t
    lane_x = [float(x_visual.min()) - 10.0, float(x_visual.max()) + 10.0]

    record = {
        "schema_version": 2,
        "module": "02_pid/03_pid_controller",
        "scenario": "PD vs PID",
        "dt": dt,
        "actors": [
            {"name": "pd", "L": 4.0, "W": 2.0, "color": [150, 150, 150, 120],
             "t": t.tolist(), "X": x_visual.tolist(),
             "Y": y_pd.tolist(), "Yaw": [0.0] * steps},
            {"name": "ego", "L": 4.0, "W": 2.0, "color": [50, 100, 220, 120],
             "t": t.tolist(), "X": x_visual.tolist(),
             "Y": y_pid.tolist(), "Yaw": [0.0] * steps},
        ],
        "lanes": [
            {"X": lane_x, "Y": [LANE_HALF_W, LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [-LANE_HALF_W, -LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [0.0, 0.0], "kind": "center"},
        ],
        "scalars": [
            {"name": "y_pd", "unit": "m", "t": t.tolist(), "value": y_pd.tolist()},
            {"name": "y_pid", "unit": "m", "t": t.tolist(), "value": y_pid.tolist()},
            {"name": "u_pd", "unit": "N", "t": t.tolist(), "value": u_pd.tolist()},
            {"name": "u_pid", "unit": "N", "t": t.tolist(), "value": u_pid.tolist()},
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
