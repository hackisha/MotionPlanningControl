"""P Controller — closed-loop step response y0=1.0 → ref=0.0.

1D plant 상태 y 를 차량의 Y (lateral) 좌표로 매핑하고, X 는 시각용 일정 vx 로
전진. 차량이 차로 중앙 (Y=0) 으로 수렴하는 모습을 3D 로 본다.

record.json (Rerun 재생용 sidecar) 을 생성. 재생: 같은 폴더의 simulator_pid.py.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from p_controller import PController
from plant import Plant

VX_VISUAL = 5.0  # 시각용 전진 속도 — 시뮬에는 영향 X
LANE_HALF_W = 1.75  # 차로 반폭 (chapter 3 동일)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="P Controller 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    dt = 0.1
    sim_time = 30.0
    steps = int(sim_time / dt)
    reference = 0.0

    plant = Plant(dt, y0=1.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = PController(kp=0.0)

    t = np.arange(steps) * dt
    y = np.zeros(steps)
    u_arr = np.zeros(steps)
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄
    for i in range(steps):
        y[i] = plant.y
        u = controller.step(reference=reference, measure=plant.y)
        u_arr[i] = u
        plant.step(u)
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )

    x_visual = VX_VISUAL * t
    lane_x = [float(x_visual.min()) - 10.0, float(x_visual.max()) + 10.0]

    record = {
        "schema_version": 2,
        "module": "02_pid/01_p_controller",
        "dt": dt,
        "actors": [{
            "name": "ego",
            "L": 4.0, "W": 2.0,
            "color": [50, 100, 220, 120],  # chapter 3 ego 색 (translucent blue)
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
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
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
