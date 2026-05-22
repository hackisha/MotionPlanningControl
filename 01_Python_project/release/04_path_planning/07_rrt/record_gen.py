"""RRT 시각화 — 새 node 추가 + tree edge 누적을 Rerun 2D 로 재생.

`--seed S` / `--eta E` / `--max-iter N` 로 실험 가능. `--skip N` 으로 frame
subsample (기본 10). 재생: 같은 폴더 ../simulator_search.py.

디버그 신호: rrt() 가 매 iteration(reject 포함) `goal_dist`·`rejected`·`tree_size`
를 수집해 record 의 `debug_scalars` 로 저장한다 (x축 = iteration). viewer 기본
화면엔 안 보이며, entity 패널에서 `/debug/<name>` 을 TimeSeriesView 로 추가해 본다.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from map_rrt import GOAL, GRID_SIZE, OBSTACLES, START
from rrt import rrt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RRT 탐색 실행 → record.json 생성 (Rerun viewer 로 단계별 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움")
    parser.add_argument("--skip", type=int, default=10,
                        help="N step 마다 frame 1 개로 묶음 (기본 10).")
    # [튜닝] 학생이 viewer 에서 실험 가능 — test 의 값은 변경 X.
    parser.add_argument("--seed", type=int, default=0,
                        help="random seed (기본 0 = 재현 가능).")
    parser.add_argument("--eta", type=float, default=3.0,
                        help="steer 한 step 최대 거리 (기본 3.0).")
    parser.add_argument("--max-iter", type=int, default=2000,
                        help="최대 sampling 반복 (기본 2000).")
    args = parser.parse_args()
    skip = max(1, args.skip)

    raw_steps: list[dict] = []
    dbg = DebugSignals()  # 디버그 신호 수집기 — rrt() 가 매 iteration 채운다

    def on_step(new_node: tuple[float, float],
                parent_node: tuple[float, float],
                iteration: int) -> None:
        raw_steps.append({
            "current": [new_node[0], new_node[1]],
            "edge": [[parent_node[0], parent_node[1]],
                     [new_node[0], new_node[1]]],
            "iteration": iteration,   # 이 add 가 일어난 iteration
        })

    path = rrt(START, GOAL, OBSTACLES, GRID_SIZE,
               max_iter=args.max_iter, eta=args.eta,
               seed=args.seed, on_step=on_step, dbg=dbg)
    if not path:
        print("[record] WARNING: 경로 미발견 — max_iter 늘리거나 seed 바꿔보세요")

    # Batch: 매 skip 개의 raw step 을 한 frame 으로 묶음.
    frames: list[dict] = []
    for i in range(0, len(raw_steps), skip):
        end = min(i + skip, len(raw_steps))
        batch = raw_steps[i:end]
        last = batch[-1]
        frames.append({
            "current": last["current"],
            "open": [],  # RRT 는 open list (priority queue) 없음.
            "expanded": [step["current"] for step in batch],
            "new_edges": [step["edge"] for step in batch],
            "iterations": [step["iteration"] for step in batch],  # add 별 iteration
        })

    record = {
        "schema_version": 1,
        "module": "04_path_planning/07_rrt",
        "kind": "search",
        "grid_size": GRID_SIZE,
        "start": [START[0], START[1]],
        "goal": [GOAL[0], GOAL[1]],
        "obstacles": [[x, y] for (x, y) in sorted(OBSTACLES)],
        "frames": frames,
        "path": [[p[0], p[1]] for p in path],
        # 디버그 신호 — rrt() 가 매 iteration(reject 포함) 수집한 시계열 (t = iteration).
        # 기본 blueprint 미포함 — viewer entity 패널에서 /debug/<name> 을
        # TimeSeriesView 로 추가해 본다.
        "debug_scalars": dbg.to_debug_scalars(),
        "seed": int(args.seed),  # simulator_search status 패널 메타.
        "eta": float(args.eta),
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out} (seed={args.seed}, eta={args.eta}, "
          f"{len(raw_steps)} samples → {len(frames)} frames (skip={skip}), "
          f"path={len(path)} nodes)  |  재생: simulator_search.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


if __name__ == "__main__":
    main()
