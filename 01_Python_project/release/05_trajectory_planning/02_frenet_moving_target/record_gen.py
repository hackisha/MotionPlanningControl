"""Frenet Moving Target 시나리오 실행 → record.json 생성.

폐루프 트랙 위에서 ego 가 매 스텝 Frenet 최적 궤적 계획을 수행한다. 트랙에는
직선 주행(차선 유지·등속) 중인 타겟 차량 여러 대가 있고, ego 는 느린 타겟을
만나면 차선을 바꿔 추월하며 목표 속도로 주행한다.

3D 시각: ego(파랑) + 타겟들(주황) + 2 차선 폐루프 트랙 + 매 스텝의 후보 궤적
다발(옅은 빨강) + 최적 궤적(파랑). 재생: ../simulator_trajectory_planning.py.

실행 전 frenet_planner.py 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면
NotImplementedError 가 납니다.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from frenet_planner import TARGET_SPEED, frenet_optimal_planning
from target_vehicles import Maneuver, TargetFleet, TargetVehicle
from track_map import LANE_WIDTH, TrackMap

DT = 0.1
SIM_TIME = 120.0
EGO_L = 3.6
EGO_W = 1.8


def build_targets() -> list[TargetVehicle]:
    """주행 타겟 — 타겟별로 매뉴버를 설정할 수 있다.

    현재는 모두 'straight'(차선 유지·등속). 추후 타겟마다 다른 매뉴버
    (lane_change / accel 등) 를 줄 수 있도록 Maneuver 를 개별 지정한다.
    초기 위치·속도·매뉴버는 시나리오 입력 — 자유롭게 바꿔 실험해 보세요.
    """
    return [
        TargetVehicle(s=50.0, d=-LANE_WIDTH / 2, s_d=7.0,
                      maneuver=Maneuver(kind="straight"), name="target_1"),
        TargetVehicle(s=95.0, d=+LANE_WIDTH / 2, s_d=10.0,
                      maneuver=Maneuver(kind="straight"), name="target_2"),
        TargetVehicle(s=150.0, d=-LANE_WIDTH / 2, s_d=8.0,
                      maneuver=Maneuver(kind="straight"), name="target_3"),
        TargetVehicle(s=185.0, d=+LANE_WIDTH / 2, s_d=6.0,
                      maneuver=Maneuver(kind="straight"), name="target_4"),
    ]


def run_sim() -> dict:
    track = TrackMap()
    fleet = TargetFleet(build_targets(), track)
    targets = fleet.targets
    steps = int(SIM_TIME / DT)

    # ego 초기 Frenet 상태 — 오른쪽 차선(d=-LANE_WIDTH/2), 목표 속도로 출발.
    si, si_d, si_dd = 0.0, TARGET_SPEED, 0.0
    di, di_d, di_dd = -LANE_WIDTH / 2, 0.0, 0.0
    opt_d = di

    t_arr: list[float] = []
    ego_x: list[float] = []
    ego_y: list[float] = []
    ego_yaw: list[float] = []
    ego_speed: list[float] = []
    ego_lat: list[float] = []
    tgt_x: dict[str, list[float]] = {tg.name: [] for tg in targets}
    tgt_y: dict[str, list[float]] = {tg.name: [] for tg in targets}
    tgt_yaw: dict[str, list[float]] = {tg.name: [] for tg in targets}
    cand_per_t: list = []
    opt_per_t: list = []
    dbg = DebugSignals()

    for step in range(steps):
        target_states = fleet.states()
        valid, best = frenet_optimal_planning(
            si, si_d, si_dd, TARGET_SPEED, 0.0,
            di, di_d, di_dd, 0.0, 0.0, target_states, track, opt_d)

        if best is None:
            # 양 차선 모두 막힘 (극히 드묾) — 직전 Frenet 위치 유지.
            ex, ey, eyaw = track.to_cartesian(si, di)
            cand_per_t.append([])
            opt_per_t.append([])
        else:
            ex, ey, eyaw = best.x[0], best.y[0], best.yaw[0]
            cand_per_t.append([list(zip(fp.x, fp.y, strict=True)) for fp in valid])
            opt_per_t.append(list(zip(best.x, best.y, strict=True)))

        # 현재 스텝 기록
        t_arr.append(step * DT)
        ego_x.append(ex)
        ego_y.append(ey)
        ego_yaw.append(eyaw)
        ego_speed.append(si_d)
        ego_lat.append(di)
        for tg in targets:
            tgt_x[tg.name].append(tg.x)
            tgt_y[tg.name].append(tg.y)
            tgt_yaw[tg.name].append(tg.yaw)

        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )

        # ego 전진 — planning 만 수행하므로 최적 궤적의 한 스텝 뒤를 다음 초기조건으로.
        if best is not None:
            si, si_d, si_dd = best.s[1], best.s_d[1], best.s_dd[1]
            di, di_d, di_dd = best.d[1], best.d_d[1], best.d_dd[1]
            opt_d = best.d[-1]
        # 타겟 전진 — 근접 시 속도 교환으로 타겟끼리 추돌 회피
        fleet.update_all(DT)

    actors = [
        {"name": "ego", "L": EGO_L, "W": EGO_W, "color": [50, 100, 220, 160],
         "trail": False,
         "t": t_arr, "X": ego_x, "Y": ego_y, "Yaw": ego_yaw},
    ]
    for tg in targets:
        actors.append(
            {"name": tg.name, "L": EGO_L, "W": EGO_W, "color": [225, 130, 40, 160],
             "trail": False,
             "t": t_arr, "X": tgt_x[tg.name], "Y": tgt_y[tg.name],
             "Yaw": tgt_yaw[tg.name]})

    return {
        "schema_version": 2,
        "module": "05_trajectory_planning/02_frenet_moving_target",
        "dt": DT,
        "actors": actors,
        "lanes": track.lanes_for_record(),
        "scalars": [
            {"name": "ego_speed", "unit": "m/s", "t": t_arr, "value": ego_speed},
            {"name": "ego_lateral", "unit": "m", "t": t_arr, "value": ego_lat},
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서
        # /debug/<name> 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t_arr),
        "dynamic_paths": [
            {"name": "candidates", "color": [220, 70, 70, 55], "radius": 0.08,
             "t": t_arr, "points_per_t": cand_per_t},
            {"name": "optimal", "color": [55, 225, 95, 235], "radius": 0.18,
             "t": t_arr, "points_per_t": opt_per_t},
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Frenet Moving Target 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    record = run_sim()
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_trajectory_planning.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_trajectory_planning import replay_records  # type: ignore
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
