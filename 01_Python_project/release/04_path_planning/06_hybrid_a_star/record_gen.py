"""Hybrid A* — kinematic 경로 탐색 + Pure Pursuit 추종 (3D Rerun viz).

두 phase 가 한 timeline 에 연결됨:
  1) search phase: t ∈ [0, T_search]. ego 가 expand 된 노드로 teleport,
     scalars (visited / frontier count) 가 우측 패널에 누적 표시.
  2) control phase: t ∈ [T_search, ...]. Hybrid A* 결과 path 를 chapter 3 의
     PurePursuit 으로 추종 — ego 가 매끄럽게 주행. T_search 시점에 path 가
     dynamic_paths 로 등장.

장애물은 obstacles_3d 필드로 chapter 3 simulator 가 3D box 로 렌더.
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
EGO_BOX_L = 3.0        # 차체 길이 (visual, wheelbase L 의 1.5x)
MAX_DELTA = 0.5        # 조향 한계 (rad)
GOAL_REACH_DIST = 0.5  # control 종료 조건


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
    # 1) 가장 가까운 path waypoint
    ex, ey, eyaw = ego_xyz
    dists = [(p[0] - ex) ** 2 + (p[1] - ey) ** 2 for p in path]
    closest = int(np.argmin(dists))
    # 2) closest 부터 cumulative arc length d_lookahead 까지 전진
    cum = 0.0
    lh_idx = len(path) - 1
    for j in range(closest, len(path) - 1):
        seg = math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
        if cum + seg >= d_lookahead:
            lh_idx = j + 1
            break
        cum += seg
    lh_x, lh_y = path[lh_idx][0], path[lh_idx][1]
    # 3) ego local frame: rotate by -eyaw
    cos_t = math.cos(-eyaw)
    sin_t = math.sin(-eyaw)
    dx = lh_x - ex
    dy = lh_y - ey
    y_lh_local = sin_t * dx + cos_t * dy
    # 4) constant coeff → pure pursuit 공식 그대로
    coeff = np.array([[0.0], [0.0], [0.0], [float(y_lh_local)]])
    return float(pp.step(coeff, vx))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hybrid A* 탐색 + Pure Pursuit 추종 → record.json (Rerun 3D 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움")
    parser.add_argument("--skip", type=int, default=10,
                        help="search expansion 을 N 단위 batch 로 묶음 (기본 10). "
                             "1 = subsample 안 함.")
    args = parser.parse_args()
    skip = max(1, args.skip)

    # ── Search phase ────────────────────────────────────────────────
    raw_steps: list[dict] = []

    def on_step(pose: tuple[float, float, float],
                frontier: set[tuple[int, int, int]]) -> None:
        raw_steps.append({
            "current": [float(pose[0]), float(pose[1]), float(pose[2])],
            "frontier_count": len(frontier),
        })

    path = hybrid_a_star(START, GOAL, SPACE, OBSTACLES,
                        R=R_SEARCH, vx=VX_SEARCH, dt=DT_SEARCH,
                        weight=WEIGHT, on_step=on_step)
    if not path:
        raise SystemExit("[record] ERROR — Hybrid A* 경로 미발견")

    # search 시각화 timeline: scalars (visited/frontier count) 만 batch 단위로 누적.
    # ego 는 search 중 정지 — t=0 에 START pose 로 1회 로그, control phase 가
    # 시작하기 전까지 viewer 가 그 pose 를 유지.
    scalar_t: list[float] = []
    visited_count_v: list[float] = []
    frontier_count_v: list[float] = []
    visited_total = 0
    for i in range(0, len(raw_steps), skip):
        end_i = min(i + skip, len(raw_steps))
        batch = raw_steps[i:end_i]
        last = batch[-1]
        t = i * DT_SEARCH_VIZ
        visited_total += len(batch)
        scalar_t.append(t)
        visited_count_v.append(float(visited_total))
        frontier_count_v.append(float(last["frontier_count"]))

    t_search_end = max(1, len(raw_steps) - 1) * DT_SEARCH_VIZ
    # ego 는 search 동안 START 에 정지. 2 개 timestamp 만 (t=0, t=T_search) 모두 START pose.
    search_actor_t = [0.0, t_search_end]
    search_actor_X = [float(START[0]), float(START[0])]
    search_actor_Y = [float(START[1]), float(START[1])]
    search_actor_Yaw = [float(START[2]), float(START[2])]

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
    actor_t = search_actor_t + ctrl_t
    actor_X = search_actor_X + ctrl_X
    actor_Y = search_actor_Y + ctrl_Y
    actor_Yaw = search_actor_Yaw + ctrl_Yaw

    path_pts = [[float(p[0]), float(p[1])] for p in path]
    obs_3d = [{"x": float(ox), "y": float(oy),
               "radius": float(orad), "height": 2.0}
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
        "start_marker": [float(START[0]), float(START[1])],
        "goal_marker": [float(GOAL[0]), float(GOAL[1])],
        "scalars": [
            {"name": "visited_count", "unit": "-",
             "t": scalar_t, "value": visited_count_v},
            {"name": "frontier_count", "unit": "-",
             "t": scalar_t, "value": frontier_count_v},
        ],
        # 최종 path 는 search 종료 시점에 한 번만 — t_search_end 부터 viewer 에 표시.
        "dynamic_paths": [
            {"name": "final_path",
             "color": [255, 100, 230, 230], "radius": 0.12,
             "t": [t_search_end],
             "points_per_t": [path_pts]},
        ],
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out}  search={len(raw_steps)} expansions, "
          f"path={len(path)} nodes, control={len(ctrl_X)} steps  |  "
          f"재생: simulator_path_planning.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_path_planning import replay_records  # type: ignore[no-redef]
        replay_records([out], camera="follow")


if __name__ == "__main__":
    main()
