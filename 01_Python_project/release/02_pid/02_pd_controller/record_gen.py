"""PD Controller — closed-loop step response y0=1.0 → ref=0.0.

1D plant 상태 y 를 차량의 Y (lateral) 로, X 는 시각용 일정 vx 로 전진.
재생: 같은 폴더의 simulator_pid.py.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from pd_controller import PDController
from plant_pd import Plant

VX_VISUAL = 5.0
LANE_HALF_W = 1.75


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PD Controller 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    dt = 0.1
    sim_time = 30.0
    steps = int(sim_time / dt)
    reference = 0.0

    plant = Plant(dt, y0=1.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = PDController(kp=0.0, kd=0.0, dt=dt)

    t = np.arange(steps) * dt
    y = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        y[i] = plant.y
        u = controller.step(reference=reference, measure=plant.y)
        u_arr[i] = u
        plant.step(u)

    x_visual = VX_VISUAL * t
    lane_x = [float(x_visual.min()) - 10.0, float(x_visual.max()) + 10.0]

    record = {
        "schema_version": 2,
        "module": "02_pid/02_pd_controller",
        "dt": dt,
        "actors": [{
            "name": "ego",
            "L": 4.0, "W": 2.0,
            "color": [50, 100, 220, 120],
            "t": t.tolist(),
            "X": x_visual.tolist(),
            "Y": y.tolist(),
            "Yaw": [0.0] * steps,
        }],
        "lanes": [
            {"X": lane_x, "Y": [LANE_HALF_W, LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [-LANE_HALF_W, -LANE_HALF_W], "kind": "edge"},
            {"X": lane_x, "Y": [0.0, 0.0], "kind": "center"},
        ],
        "scalars": [
            {"name": "y", "unit": "m", "t": t.tolist(), "value": y.tolist()},
            {"name": "u_cmd", "unit": "N", "t": t.tolist(), "value": u_arr.tolist()},
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
