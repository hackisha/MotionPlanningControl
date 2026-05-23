"""Informed RRT* + Dubins 시각화 — 슬라럼 맵 위 트리 성장 + informed 타원 수렴을 Rerun 2D 로 재생.

`--seed S` / `--eta E` / `--radius R` / `--max-iter N` / `--kappa K` / `--ds D` 로
실험 가능. `--skip N` 으로 frame subsample (기본 10). 재생: 같은 폴더
../simulator_search.py.

기록 형태 (09 와 동일 구성):
- **트리 엣지**: planner 의 on_step 이 넘겨주는 stored Dubins sample — collision-check
  를 실제로 통과한 곡선이 그대로 옴. `dubins_plan` 재호출 안 함 (fp precision 으로
  Bellman 어긋나 다른 word 가 선택되는 케이스 회피).
- **최종 path**: Dubins fine sample 의 연속 — viewer 의 /world/path 에 부드러운 곡선.
- **informed 타원**: round 시작마다 c_best 로 정의된 Euclidean 타원 — /world/ellipse.
  Dubins 호장 ≥ Euclidean 거리이므로 valid informed superset (Gammell 2014).
- **round 별 채택 경로**: planner 가 c_best 갱신 시 on_improve 로 snapshot 한 경로 —
  /world/round_path 에 밝은 청록 하이라이트. iteration scrub 시 round 마다 좁아짐.
- **debug_scalars**: dbg 가 매 iter 수집한 goal_dist·rejected·tree_size·rewire_count·
  inform_round·best_cost 시계열. viewer entity 패널에서 /debug/<name> 을
  TimeSeriesView 로 추가해 본다. **best_cost** 가 round 마다 계단식으로 줄어드는 게
  informed 수렴이다.

state 는 (x, y, yaw) 지만 record schema 는 [x, y] 만 받으므로 yaw 는 떼고 저장.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from map_rrt_dubins import GOAL, GRID_SIZE, OBSTACLES, START
from rrt_dubins import informed_rrt_star

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals  # noqa: E402


def _ellipse_polygon(focus_a: tuple[float, float],
                     focus_b: tuple[float, float],
                     c_best: float, n: int = 72) -> list[list[float]]:
    """초점 focus_a·focus_b, 장축 길이 c_best 인 타원 경계 polygon (n+1 점, 닫힘)."""
    c_min = math.hypot(focus_b[0] - focus_a[0], focus_b[1] - focus_a[1])
    cx = 0.5 * (focus_a[0] + focus_b[0])
    cy = 0.5 * (focus_a[1] + focus_b[1])
    theta = math.atan2(focus_b[1] - focus_a[1], focus_b[0] - focus_a[0])
    a = 0.5 * c_best
    b = 0.5 * math.sqrt(max(c_best * c_best - c_min * c_min, 0.0))
    pts: list[list[float]] = []
    for k in range(n + 1):
        phi = 2.0 * math.pi * k / n
        ex, ey = a * math.cos(phi), b * math.sin(phi)
        pts.append([cx + ex * math.cos(theta) - ey * math.sin(theta),
                    cy + ex * math.sin(theta) + ey * math.cos(theta)])
    return pts


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Informed RRT* + Dubins 탐색 → record.json (Rerun viewer 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움")
    parser.add_argument("--skip", type=int, default=10,
                        help="N step 마다 frame 1 개로 묶음 (기본 10).")
    # [튜닝] 학생이 viewer 에서 실험 가능 — test 의 값은 변경 X.
    parser.add_argument("--seed", type=int, default=0,
                        help="random seed (기본 0 = 재현 가능).")
    parser.add_argument("--eta", type=float, default=6.0,
                        help="한 steer 가 자라는 Dubins 호장 상한 [m] (기본 6.0).")
    parser.add_argument("--radius", type=float, default=10.0,
                        help="choose-parent·rewire 반경 (기본 10.0).")
    parser.add_argument("--max-iter", type=int, default=1500,
                        help="최대 sampling 반복 (anytime budget, 기본 1500).")
    parser.add_argument("--kappa", type=float, default=1.0 / 3.0,
                        help="최대 곡률 1/R (기본 1/3 → R=3 m).")
    parser.add_argument("--ds", type=float, default=0.2,
                        help="Dubins 호 sampling 호장 step [m] (기본 0.2).")
    args = parser.parse_args()
    skip = max(1, args.skip)

    raw_steps: list[dict] = []
    rounds: list[dict] = []     # on_improve 가 채움: {iteration, c_best, path_xy}
    dbg = DebugSignals()

    def on_step(child, parent, samples, iteration: int) -> None:
        # samples 는 planner 가 collision-check 통과시킨 실제 Dubins fine sample
        # (parent 미포함). schema 의 [x, y] 만 저장하고 parent xy 를 앞에 prepend.
        edge = [[parent[0], parent[1]]] + [[s[0], s[1]] for s in samples]
        raw_steps.append({
            "current": [child[0], child[1]],
            "edge": edge,
            "iteration": iteration,
        })

    def on_improve(iteration: int, c_best: float, snap_path) -> None:
        # snap_path 는 (x, y, yaw) 시퀀스 — xy 만 떼고 마지막에 goal xy 를 덧붙여
        # 그 round 의 채택비용(eff) = path 총 길이 + goal 직선 = c_best 와 일치시킴
        # (09 와 동일한 시각화 규약 — informed 타원 장축이 round_path 와 같음).
        path_xy: list[list[float]] = [[p[0], p[1]] for p in snap_path]
        if path_xy and path_xy[-1] != [GOAL[0], GOAL[1]]:
            path_xy.append([GOAL[0], GOAL[1]])
        rounds.append({"iteration": iteration, "c_best": c_best,
                       "path_xy": path_xy})

    path = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE,
                             kappa=args.kappa, max_iter=args.max_iter,
                             eta=args.eta, search_radius=args.radius,
                             ds=args.ds, seed=args.seed,
                             on_step=on_step, on_improve=on_improve, dbg=dbg)
    if not path:
        print("[record] WARNING: 경로 미발견 — max_iter 늘리거나 seed 바꿔보세요")

    # Batch tree edges: 매 skip 개의 raw step 을 한 frame 으로 묶음.
    frames: list[dict] = []
    for i in range(0, len(raw_steps), skip):
        end = min(i + skip, len(raw_steps))
        batch = raw_steps[i:end]
        last = batch[-1]
        frames.append({
            "current": last["current"],
            "open": [],  # sampling planner 는 priority queue 없음.
            "expanded": [step["current"] for step in batch],
            "new_edges": [step["edge"] for step in batch],
            "iterations": [step["iteration"] for step in batch],
        })

    # informed 타원 + round 별 채택 경로 — round 시작 iteration 에 동시 갱신.
    ellipses: list[dict] = []
    round_paths: list[dict] = []
    for r in rounds:
        ellipses.append({
            "iteration": r["iteration"],
            "points": _ellipse_polygon((START[0], START[1]),
                                       (GOAL[0], GOAL[1]), r["c_best"]),
        })
        round_paths.append({"iteration": r["iteration"],
                            "points": r["path_xy"]})

    record = {
        "schema_version": 1,
        "module": "04_path_planning/10_rrt_dubins",
        "kind": "search",
        "grid_size": GRID_SIZE,
        "start": [START[0], START[1]],
        "goal": [GOAL[0], GOAL[1]],
        "obstacles": [[x, y] for (x, y) in sorted(OBSTACLES)],
        "frames": frames,
        # Dubins fine sample — (x, y, yaw) 중 xy 만 저장.
        "path": [[p[0], p[1]] for p in path],
        # informed 타원 — iteration 별 (round 시작 시점). simulator 가 /world/ellipse.
        "ellipses": ellipses,
        # round 별 채택 경로 — simulator 가 /world/round_path 에 하이라이트.
        "round_paths": round_paths,
        # 디버그 신호 — informed_rrt_star() 가 매 iteration 수집한 시계열.
        "debug_scalars": dbg.to_debug_scalars(),
        "seed": int(args.seed),
        "eta": float(args.eta),
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out} (seed={args.seed}, eta={args.eta}, "
          f"kappa={args.kappa:.4f}, ds={args.ds}, "
          f"{len(raw_steps)} steps → {len(frames)} frames (skip={skip}), "
          f"path={len(path)} nodes, rounds={len(rounds)})  |  "
          f"재생: simulator_search.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


if __name__ == "__main__":
    main()
