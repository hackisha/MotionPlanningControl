"""Hybrid A* record 생성 — 두 가지 재생 모드.

기본 (옵션 없음) — **search phase 만**:
  Hybrid A* expansion 트리를 2D search record 로 내보낸다. `simulator_search.py`
  (04/05/07 과 공용, `iteration` 타임라인) 로 재생 — expansion 단위로 트리가 자라고
  디버그 신호 곡선이 같은 축에 그려진다. 알고리즘 디버깅·튜닝용.

`--controller` — **search + 추종까지**:
  search 결과 path 를 chapter 3 PurePursuit 으로 추종하는 control phase 를 이어붙여
  3D search_and_control record 로 내보낸다. `simulator_path_planning.py` (연속
  `sim_time` 타임라인) 로 재생. 두 phase 가 한 timeline 에 연결됨:
    1) search phase: t ∈ [0, T_search]. ego 정지, expansion arc 가 트리처럼 자람.
    2) control phase: t ∈ [T_search, ...]. ego 가 등장해 path 를 매끄럽게 추종.
  장애물은 obstacles_3d (Cylinders3D), 탐색 공간 경계는 lanes kind="boundary".
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

# chapter 3 의 PurePursuit 재사용.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent /
                     "03_vehicle_control" / "07_pure_pursuit"))
from hybrid_a_star import hybrid_a_star
from map_hybrid import GOAL, OBSTACLES, SPACE, START
from pure_pursuit import PurePursuit  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals  # noqa: E402

# Search 파라미터
R_SEARCH = 5.0
VX_SEARCH = 2.0
DT_SEARCH = 0.5
WEIGHT = 1.1
DT_SEARCH_VIZ = 0.05   # viewer 에서 search expansion 한 step 당 흘러갈 시간

# Control 파라미터
DT_CTRL = 0.1
VX_CTRL = 2.0
LOOKAHEAD_TIME = 1.0
SIM_TIME_CTRL = 25.0
EGO_L = 2.0            # wheelbase (search R=5 와 호환되는 작은 차)
EGO_W = 1.5            # 차체 폭 (visual)
EGO_BOX_L = 2.86       # 차체 길이 (visual) — _wheel_boxes 가 ±0.35L 에 바퀴를 두므로
                       # 0.7·L_box ≈ wheelbase 2.0 m 가 되도록 맞춤.
MAX_DELTA = 0.5        # 조향 한계 (rad)
GOAL_REACH_DIST = 0.5  # control 종료 조건

# Viz 파라미터
OBSTACLE_HEIGHT = 0.6
OBSTACLE_COLOR = [140, 140, 140, 130]  # 반투명 회색
SEARCH_ARC_COLOR = [200, 220, 255, 160]
SEARCH_ARC_RADIUS = 0.04


def _ego_step(ego_xyz: list[float], delta: float, vx: float, dt: float) -> list[float]:
    """Kinematic bicycle one step. ego_xyz = [X, Y, Yaw]."""
    delta = float(np.clip(delta, -MAX_DELTA, MAX_DELTA))
    x, y, yaw = ego_xyz
    yaw_rate = vx / EGO_L * math.tan(delta)
    new_yaw = yaw + dt * yaw_rate
    new_x = x + vx * dt * math.cos(new_yaw)
    new_y = y + vx * dt * math.sin(new_yaw)
    return [new_x, new_y, new_yaw]


def _follow_step(ego_xyz: list[float],
                 path: list[tuple[float, float, float]],
                 pp: PurePursuit,
                 vx: float) -> float:
    """Pure pursuit 으로 path 의 lookahead waypoint 향한 steering 계산.

    chapter 3 PurePursuit 는 (coeff, vx) 인터페이스 — d_lh = vx·lookahead_time 위치에서
    poly 평가. 여기선 path 의 lookahead waypoint 의 ego-local y 만 알면 되므로
    constant polynomial `[0, 0, 0, y_lh_local]` 로 변환해 그대로 호출.
    """
    d_lookahead = vx * pp.lookahead_time
    ex, ey, eyaw = ego_xyz
    dists = [(p[0] - ex) ** 2 + (p[1] - ey) ** 2 for p in path]
    closest = int(np.argmin(dists))
    cum = 0.0
    lh_idx = len(path) - 1
    for j in range(closest, len(path) - 1):
        seg = math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
        if cum + seg >= d_lookahead:
            lh_idx = j + 1
            break
        cum += seg
    lh_x, lh_y = path[lh_idx][0], path[lh_idx][1]
    cos_t = math.cos(-eyaw)
    sin_t = math.sin(-eyaw)
    dx = lh_x - ex
    dy = lh_y - ey
    y_lh_local = sin_t * dx + cos_t * dy
    coeff = np.array([[0.0], [0.0], [0.0], [float(y_lh_local)]])
    return float(pp.step(coeff, vx))


def _boundary_lane() -> dict:
    """SPACE 의 4 변을 잇는 닫힌 사각형 — 지면에 옅은 회색 선 1줄로."""
    x_min, x_max, y_min, y_max = SPACE
    return {
        "X": [x_min, x_max, x_max, x_min, x_min],
        "Y": [y_min, y_min, y_max, y_max, y_min],
        "kind": "boundary",
    }


def _emit_search_only(raw_steps: list[dict],
                      raw_arcs: list[list[list[float]] | None],
                      dbg: DebugSignals,
                      path: list[tuple[float, float, float]],
                      skip: int, out: Path, no_viewer: bool) -> None:
    """기본 모드 — search phase 만. 2D search record 로 내보내 simulator_search.py
    (`iteration` 타임라인) 로 재생. control phase·ego actor 계산을 모두 건너뛴다.

    expansion arc 를 RRT(07) 와 같은 `new_edges` 트리 구조로 흘려 보낸다 — start
    노드는 parent arc 가 없어 제외하고 `world/start` 마커로만 표시한다.
    """
    # 매 expansion 의 (x, y) 노드 + parent→child arc + 정수 iteration 인덱스.
    steps: list[dict] = []
    for it, (rs, arc) in enumerate(zip(raw_steps, raw_arcs, strict=True)):
        if arc is None:   # start 노드 — parent arc 없음
            continue
        steps.append({
            "node": [rs["current"][0], rs["current"][1]],
            "edge": arc,
            "iteration": it,
        })
    # --skip 단위 batch (07_rrt frame 구조와 동일 — add 별 iteration 보존).
    frames: list[dict] = []
    for i in range(0, len(steps), skip):
        batch = steps[i:i + skip]
        frames.append({
            "current": batch[-1]["node"],
            "expanded": [s["node"] for s in batch],
            "new_edges": [s["edge"] for s in batch],
            "iterations": [s["iteration"] for s in batch],
        })

    record = {
        "schema_version": 1,
        "module": "04_path_planning/06_hybrid_a_star",
        "kind": "search",
        # 연속공간 경계 — simulator_search 가 옅은 회색 사각형으로 렌더 + marker
        # radius 스케일 기준 (격자 60 대비 작은 공간이라 점·선이 비례 축소된다).
        "space": [float(v) for v in SPACE],
        "start": [float(START[0]), float(START[1])],
        "goal": [float(GOAL[0]), float(GOAL[1])],
        # 원형 장애물 (x, y, radius) — simulator_search 가 3-원소면 원형으로 렌더.
        "obstacles": [[float(ox), float(oy), float(orad)]
                      for (ox, oy, orad) in OBSTACLES],
        "frames": frames,
        "path": [[float(p[0]), float(p[1])] for p in path],
        # 디버그 신호 — t = expansion 인덱스 0,1,2,… ('iteration' 타임라인 축).
        "debug_scalars": dbg.to_debug_scalars(),
        "weight": float(WEIGHT),  # simulator_search status 패널 표시용 메타.
    }
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  search={len(raw_steps)} expansions "
          f"→ {len(frames)} frames (skip={skip}), path={len(path)} nodes  |  "
          f"재생: simulator_search.py (iteration 타임라인)")

    if not no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


def _emit_search_and_control(raw_steps: list[dict],
                             raw_arcs: list[list[list[float]] | None],
                             dbg: DebugSignals,
                             path: list[tuple[float, float, float]],
                             skip: int, out: Path, no_viewer: bool) -> None:
    """--controller 모드 — search 결과 path 를 PurePursuit 으로 추종하는 control
    phase 를 이어붙여 3D search_and_control record 로 내보낸다. simulator_path_planning.py
    (연속 `sim_time` 타임라인) 로 재생.
    """
    # ── search timeline: batch 단위로 누적 (text + per-batch search arcs) ──
    scalar_t: list[float] = []
    text_v: list[str] = []
    search_arc_entries: list[dict] = []
    closed_total = 0
    for i in range(0, len(raw_steps), skip):
        end_i = min(i + skip, len(raw_steps))
        batch = raw_steps[i:end_i]
        last = batch[-1]
        t = i * DT_SEARCH_VIZ
        closed_total += len(batch)
        scalar_t.append(t)
        cur = last["current"]
        text_v.append(
            f"[Search]  t = {t:6.2f} s\n"
            f"  closed   : {closed_total}\n"
            f"  open     : {last['open_count']}\n"
            f"  current  : ({cur[0]:6.2f}, {cur[1]:6.2f}, {cur[2]:5.2f} rad)"
        )
        # 해당 batch 의 새 expansion arc 들만 별도 dynamic_paths entry 로 logging.
        # entity 가 t 에 한 번만 log 되고 그 이후 timeline 에서 persist —
        # 누적 데이터를 매 batch 마다 다시 쓰는 비용 없이 트리가 자란다.
        batch_arcs = [a for a in raw_arcs[i:end_i] if a is not None]
        if batch_arcs:
            search_arc_entries.append({
                "name": f"search_arcs/b{i:05d}",
                "color": SEARCH_ARC_COLOR,
                "radius": SEARCH_ARC_RADIUS,
                "t": [t],
                "points_per_t": [batch_arcs],
            })

    t_search_end = max(1, len(raw_steps) - 1) * DT_SEARCH_VIZ

    # ── Control phase (Pure Pursuit path following) ─────────────────
    ego_xyz = [START[0], START[1], START[2]]
    pp = PurePursuit(L=EGO_L, lookahead_time=LOOKAHEAD_TIME)
    ctrl_t: list[float] = []
    ctrl_X: list[float] = []
    ctrl_Y: list[float] = []
    ctrl_Yaw: list[float] = []
    n_ctrl_max = int(SIM_TIME_CTRL / DT_CTRL)
    for k in range(n_ctrl_max):
        delta = _follow_step(ego_xyz, path, pp, VX_CTRL)
        ego_xyz = _ego_step(ego_xyz, delta, VX_CTRL, DT_CTRL)
        t = t_search_end + (k + 1) * DT_CTRL
        ctrl_t.append(t)
        ctrl_X.append(ego_xyz[0])
        ctrl_Y.append(ego_xyz[1])
        ctrl_Yaw.append(ego_xyz[2])
        if math.hypot(ego_xyz[0] - GOAL[0], ego_xyz[1] - GOAL[1]) < GOAL_REACH_DIST:
            break

    # ── Combined record ────────────────────────────────────────────
    # ego 는 search 동안 START 에 정지 (자동차 모양 그대로), search 완료 후 control 로 이어짐.
    # t=0 과 t=t_search_end 두 timestamp 모두 START pose 로 로그 → 그 사이는 viewer 가
    # 보간 없이 동일 pose 유지. control phase 의 첫 timestamp 부터 움직임 시작.
    actor_t = [0.0, t_search_end] + ctrl_t
    actor_X = [float(START[0]), float(START[0])] + ctrl_X
    actor_Y = [float(START[1]), float(START[1])] + ctrl_Y
    actor_Yaw = [float(START[2]), float(START[2])] + ctrl_Yaw

    # Control phase 의 텍스트 (search 완료 시점에 갱신).
    scalar_t.append(t_search_end + DT_CTRL / 2.0)
    text_v.append(
        f"[Control]  search done\n"
        f"  expansions : {len(raw_steps)}\n"
        f"  path nodes : {len(path)}\n"
        f"  goal       : ({GOAL[0]:.2f}, {GOAL[1]:.2f})"
    )

    path_pts = [[float(p[0]), float(p[1])] for p in path]
    obs_3d = [{"x": float(ox), "y": float(oy),
               "radius": float(orad), "height": OBSTACLE_HEIGHT,
               "color": OBSTACLE_COLOR}
              for (ox, oy, orad) in OBSTACLES]

    record = {
        "schema_version": 2,
        "module": "04_path_planning/06_hybrid_a_star",
        "kind": "search_and_control",
        "dt": DT_CTRL,
        "actors": [{
            "name": "ego",
            "L": EGO_BOX_L, "W": EGO_W,
            "color": [50, 100, 220, 120],
            "t": actor_t,
            "X": actor_X,
            "Y": actor_Y,
            "Yaw": actor_Yaw,
        }],
        "obstacles_3d": obs_3d,
        "lanes": [_boundary_lane()],
        # start_marker 는 ego 자동차 모양이 START 에 정지해 있으므로 생략.
        "goal_marker": [float(GOAL[0]), float(GOAL[1])],
        "text_panel": {
            "entity": "info/status",
            "t": scalar_t,
            "text": text_v,
        },
        # 디버그 신호 — expansion 별 시계열 (t = expansion 인덱스 0,1,2,…). 기본
        # blueprint 미포함 — viewer entity 패널에서 /debug/<name> 을 TimeSeriesView
        # 로 추가. control phase 재생축(sim_time)과 디버그 x축은 별개.
        "debug_scalars": dbg.to_debug_scalars(),
        # 최종 path 는 search 종료 시점에 한 번만 — t_search_end 부터 viewer 에 표시.
        # search_arc_entries 가 앞에 — viewer Recordings 패널에서 z-order 영향 없음.
        "dynamic_paths": [
            *search_arc_entries,
            {"name": "final_path",
             "color": [255, 100, 230, 230], "radius": 0.12,
             "t": [t_search_end],
             "points_per_t": [path_pts]},
        ],
    }
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  search={len(raw_steps)} expansions, "
          f"path={len(path)} nodes, control={len(ctrl_X)} steps, "
          f"arc_batches={len(search_arc_entries)}  |  "
          f"재생: simulator_path_planning.py")

    if not no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_path_planning import replay_records  # type: ignore[no-redef]
        replay_records([out], camera="follow")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hybrid A* 탐색 → record.json (Rerun 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움")
    parser.add_argument("--skip", type=int, default=20,
                        help="search expansion 을 N 단위 batch 로 묶음 (기본 20). "
                             "1 = subsample 안 함.")
    parser.add_argument("--controller", action="store_true",
                        help="Pure Pursuit 추종 phase 까지 포함해 3D viewer(sim_time)로 "
                             "재생. 생략 시 search 만 — 2D viewer(iteration 타임라인) "
                             "로 디버깅용 재생.")
    args = parser.parse_args()
    skip = max(1, args.skip)

    # ── Search phase (공통) ─────────────────────────────────────────
    raw_steps: list[dict] = []
    raw_arcs: list[list[list[float]] | None] = []  # (parent_xy, child_xy) per expand
    dbg = DebugSignals()  # 디버그 신호 — on_step 이 매 expansion 마다 채운다

    def on_step(pose: tuple[float, float, float],
                parent_pose: tuple[float, float, float] | None,
                open_set: set[tuple[int, int, int]],
                g_cost: float, f_cost: float) -> None:
        raw_steps.append({
            "current": [float(pose[0]), float(pose[1]), float(pose[2])],
            "open_count": len(open_set),
        })
        # 디버그 신호 — 매 노드 확장(expansion)마다 한 줄. 더 분석할 값은 주석 풀어 추가.
        dbg.add(
            goal_dist=math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1]),
            open_size=float(len(open_set)),
            g_cost=g_cost,            # 시작 → current 누적 비용
            h_cost=f_cost - g_cost,   # heuristic 항 (weight·h)
            f_cost=f_cost,            # 총 비용 f = g + weight·h
            # debug1=<신호 값 또는 식>,
        )
        if parent_pose is None:
            raw_arcs.append(None)
        else:
            raw_arcs.append([
                [float(parent_pose[0]), float(parent_pose[1])],
                [float(pose[0]), float(pose[1])],
            ])

    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R_SEARCH, vx=VX_SEARCH, dt=DT_SEARCH,
                        weight=WEIGHT, on_step=on_step)
    if not path:
        raise SystemExit("[record] ERROR — Hybrid A* 경로 미발견")

    out = Path(__file__).parent / "record.json"
    if args.controller:
        _emit_search_and_control(raw_steps, raw_arcs, dbg, path, skip,
                                 out, args.no_viewer)
    else:
        _emit_search_only(raw_steps, raw_arcs, dbg, path, skip,
                          out, args.no_viewer)


if __name__ == "__main__":
    main()
