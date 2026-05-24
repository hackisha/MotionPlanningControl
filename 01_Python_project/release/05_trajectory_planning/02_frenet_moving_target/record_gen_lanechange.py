"""Frenet Moving Target — 차로변경(lane-change) 시나리오 → record_lanechange.json.

폐루프 트랙 위에서 ego 가 매 스텝 Frenet 최적 궤적 계획을 수행한다. 트랙의 타겟
차량(`TargetVehicle_LC`)들은 `TargetFleet_LC` 의 규칙에 따라 자율적으로 거동한다 —
같은 차선 뒤차가 앞차에 근접하면 옆 차선으로 차로변경해 추월하고(옆 차선이 막혀
불가하면 속도 교환), 자차가 옆 차선에서 뒤따라 다가오면 억제 범위 밖에서 높은
확률로 자차 앞에 끼어들며(약간 악의적), 종방향 주변이 한동안 비어 있으면 주기적
으로 랜덤 차로변경을 한다.

**관찰 포인트 — 예측 모델 업그레이드.** 이 record 의 노란 예측 궤적(`pred_*`)은
`predict_target_lanechange` 로 그린다 — 종방향 등속에 더해 관측된 횡속도 d_d 를
1차 지연 댐퍼로 외삽하고 옆 차선 중심에 도달하면 정지한다. 그 결과 차로변경 중인
타겟의 실제 경로를 꽤 잘 따라간다. 반면 planner 내부의 충돌 검사는 여전히 lanekeep
예측(`predict_target_lanekeep`, d 고정)을 쓴다 — 즉 시각화된 예측과 planner 가
실제로 회피에 쓰는 예측이 다르다. lanechange 예측이 보여 주는 만큼의 정확도를
planner 도 활용하려면 target_states 에 d_d 를 실어 보내는 다음 단계가 필요하다.

각 타겟 위에는 현재 차로변경 매뉴버를 가리키는 화살표를 띄운다 — 차로변경 진행
중이면 청록, 옆 차선이 막혀 보류 중이면 주황. 차선 유지 중에는 화살표가 없다.

3D 시각: ego(파랑) + 타겟들(주황) + 2 차선 폐루프 트랙 + 매 스텝의 후보 궤적
다발(밝은 파랑) + 최적 궤적(초록) + 타겟 lanechange 예측(노랑) + 차로변경 매뉴버
화살표(진행=청록 / 보류=주황). 재생: ../simulator_trajectory_planning.py.

실행 전 frenet_planner.py 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면
NotImplementedError 가 납니다.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals
from frenet_planner import MAX_T, TARGET_SPEED, frenet_optimal_planning
from prediction import predict_target_lanechange
from target_vehicles import TargetFleet_LC, TargetVehicle_LC
from track_map import LANE_WIDTH, TrackMap

DT = 0.1
SIM_TIME = 120.0
EGO_L = 3.6
EGO_W = 1.8

# 차로변경 매뉴버 화살표 — 각 타겟 위 공중에 띄우는 dynamic_paths multi-strip.
ARROW_Z = 2.2                                # 차체(높이 1.5) 위 띄울 높이 [m]
ARROW_LEN = 3.0                              # 화살표 샤프트 길이 [m]
ARROW_HEAD = 1.0                             # 화살촉 길이 [m]
ARROW_HEAD_W = 0.8                           # 화살촉 폭(반) [m]
ARROW_COLOR_ACTIVE = [40, 230, 150, 240]     # 차로변경 진행 중 — 청록
ARROW_COLOR_PENDING = [255, 140, 20, 240]    # 차로변경 보류 중 — 주황


def _maneuver_arrow(x: float, y: float, yaw: float, direction: int) -> list:
    """차로변경 방향 화살표 — 차량 위 공중에 띄운 multi-strip(샤프트 + 화살촉).

    direction: -1=좌 / +1=우. 차량 좌/우(yaw±90°) 방향으로 뻗는다. 반환값은
    dynamic_paths 한 스텝의 points (multi-strip, 3D 점 리스트).
    """
    side = -direction                                      # -1(좌)→좌측, +1(우)→우측
    lx, ly = side * -math.sin(yaw), side * math.cos(yaw)   # 횡(차로변경) 방향 단위벡터
    ax, ay = math.cos(yaw), math.sin(yaw)                  # 진행 방향 (화살촉 폭)
    tx, ty = x + lx * ARROW_LEN, y + ly * ARROW_LEN        # 화살표 끝(촉)
    shaft = [[x, y, ARROW_Z], [tx, ty, ARROW_Z]]
    head = [
        [tx - lx * ARROW_HEAD + ax * ARROW_HEAD_W,
         ty - ly * ARROW_HEAD + ay * ARROW_HEAD_W, ARROW_Z],
        [tx, ty, ARROW_Z],
        [tx - lx * ARROW_HEAD - ax * ARROW_HEAD_W,
         ty - ly * ARROW_HEAD - ay * ARROW_HEAD_W, ARROW_Z],
    ]
    return [shaft, head]


def build_targets() -> list[TargetVehicle_LC]:
    """주행 타겟 — 모두 차로변경 가능한 `TargetVehicle_LC`.

    초기 위치·속도는 시나리오 입력 — 자유롭게 바꿔 실험해 보세요. 차로변경 시점·
    방향은 스크립트가 아니라 `TargetFleet_LC` 의 추월 규칙이 자율적으로 결정한다.
    """
    return [
        TargetVehicle_LC(s=50.0, d=-LANE_WIDTH / 2, s_d=7.0, name="target_1"),
        TargetVehicle_LC(s=95.0, d=+LANE_WIDTH / 2, s_d=10.0, name="target_2"),
        TargetVehicle_LC(s=150.0, d=-LANE_WIDTH / 2, s_d=8.0, name="target_3"),
        TargetVehicle_LC(s=185.0, d=+LANE_WIDTH / 2, s_d=6.0, name="target_4"),
    ]


def run_sim() -> dict:
    # 랜덤 차로변경 재현성 — 데모 record 가 매 실행 동일하도록 시드 고정.
    # 매번 다른 주행을 보려면 이 줄을 지우세요.
    random.seed(2026)

    track = TrackMap()
    fleet = TargetFleet_LC(build_targets(), track)
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
    tgt_pred: dict[str, list] = {tg.name: [] for tg in targets}
    tgt_man: dict[str, list] = {tg.name: [] for tg in targets}
    tgt_dir: dict[str, list] = {tg.name: [] for tg in targets}
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
        for tg, (ts, td, ts_d) in zip(targets, target_states, strict=True):
            tgt_x[tg.name].append(tg.x)
            tgt_y[tg.name].append(tg.y)
            tgt_yaw[tg.name].append(tg.yaw)
            tgt_man[tg.name].append(tg.maneuver)         # 차로변경 매뉴버 상태
            tgt_dir[tg.name].append(tg.lc_direction)     # 차로변경 방향 -1/0/+1
            # 타겟 lanechange 예측 — 횡속도 d_d 도 활용해 옆 차선 중심으로 1차
            # 지연 외삽 (target_states 에 없는 d_d 는 타겟 객체에서 직접 읽는다).
            s_pred, d_pred = predict_target_lanechange(
                ts, td, ts_d, tg.d_d, MAX_T, DT)
            tgt_pred[tg.name].append(
                [list(track.to_cartesian(sp, dp)[:2])
                 for sp, dp in zip(s_pred, d_pred, strict=True)])

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
        # 타겟 전진 — 추월(옆 차선 막히면 속도 교환), 자차가 뒤따라 다가오면
        # 악의적 끼어들기, 주변이 오래 비면 랜덤 차로변경. ego 위치·속도를 넘겨
        # 추월·악의적·랜덤 판정에 자차를 포함.
        fleet.update_all(DT, ego_s=si, ego_d=di, ego_s_d=si_d)

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

    dynamic_paths = [
        {"name": "candidates", "color": [70, 140, 235, 90], "radius": 0.08,
         "t": t_arr, "points_per_t": cand_per_t},
        {"name": "optimal", "color": [55, 225, 95, 235], "radius": 0.18,
         "t": t_arr, "points_per_t": opt_per_t},
    ]
    # 타겟별 lanechange 예측 궤적 — d_d 활용 + 옆 차선 도착 시 정지 (관찰 포인트).
    for name in tgt_pred:
        dynamic_paths.append(
            {"name": f"pred_{name}", "color": [245, 215, 70, 95], "radius": 0.18,
             "t": t_arr, "points_per_t": tgt_pred[name]})
    # 타겟별 차로변경 매뉴버 화살표 — 진행 중(청록)/보류 중(주황)일 때만 표시,
    # 차선 유지 스텝은 빈 strip → viewer 가 화살표를 지운다 (entity Clear).
    for name in tgt_man:
        arrow_pts: list = []
        arrow_cols: list = []
        for x, y, yaw, man, lc_dir in zip(tgt_x[name], tgt_y[name], tgt_yaw[name],
                                          tgt_man[name], tgt_dir[name], strict=True):
            if man == "lane_keep" or lc_dir == 0:
                arrow_pts.append([])                       # 화살표 없음
                arrow_cols.append(ARROW_COLOR_ACTIVE)      # (미사용 placeholder)
            else:
                arrow_pts.append(_maneuver_arrow(x, y, yaw, lc_dir))
                arrow_cols.append(ARROW_COLOR_ACTIVE if man == "lane_change"
                                  else ARROW_COLOR_PENDING)
        dynamic_paths.append(
            {"name": f"maneuver_{name}", "color": ARROW_COLOR_ACTIVE, "radius": 0.14,
             "t": t_arr, "points_per_t": arrow_pts, "colors_per_t": arrow_cols})

    return {
        "schema_version": 2,
        "module": "05_trajectory_planning/02_frenet_moving_target",
        "scenario": "lanechange",
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
        "dynamic_paths": dynamic_paths,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Frenet Moving Target 차로변경 시나리오 → record_lanechange.json")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    record = run_sim()
    out = Path(__file__).parent / "record_lanechange.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  |  재생: simulator_trajectory_planning.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_trajectory_planning import replay_records  # type: ignore
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
