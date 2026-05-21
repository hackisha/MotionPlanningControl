"""A* search 시각화 — 실시간 노드 expansion + 최종 path 를 Rerun 2D 로 재생.

`--weight` 로 heuristic 가중치 조정 (1.0 = 최적, 10.0 = greedy bias).
`--skip` 로 frame subsample (기본 10). 재생: 같은 폴더 ../simulator_search.py.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from a_star import a_star
from map_astar import GOAL, GRID_SIZE, OBSTACLES, START

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="A* 탐색 실행 → record.json 생성 (Rerun viewer 로 단계별 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    parser.add_argument("--skip", type=int, default=10,
                        help="N step 마다 frame 1 개로 묶음 (기본 10). "
                             "1 = subsample 안 함.")
    # [튜닝] weight_heuristic 을 바꿔 응답 변화 비교 — test 의 값은 변경 X (합격 기준)
    parser.add_argument("--weight", type=float, default=1.0,
                        help="heuristic 가중치 (기본 1.0 = admissible 최적). "
                             ">1 일수록 goal 방향 greedy bias — 적은 expand, 최적성 약화.")
    args = parser.parse_args()
    skip = max(1, args.skip)

    raw_steps: list[dict] = []
    dbg = DebugSignals()  # 디버그 신호 — on_step 이 매 expansion 마다 채운다

    def on_step(current: tuple[int, int], open_set: set[tuple[int, int]],
                g_cost: float, f_cost: float) -> None:
        raw_steps.append({
            "current": [current[0], current[1]],
            "open": [[p[0], p[1]] for p in open_set],
        })
        # 디버그 신호 — 매 노드 확장(expansion)마다 한 줄. 더 분석할 값은 주석 풀어 추가.
        dbg.add(
            goal_dist=math.hypot(current[0] - GOAL[0], current[1] - GOAL[1]),
            open_size=float(len(open_set)),
            g_cost=g_cost,            # 시작 → current 누적 비용
            h_cost=f_cost - g_cost,   # heuristic 항 (weight·h)
            f_cost=f_cost,            # 총 비용 f = g + weight·h
            # debug1=<신호 값 또는 식>,
        )

    path = a_star(START, GOAL, OBSTACLES,
                  weight_heuristic=args.weight, on_step=on_step)
    if not path:
        print("[record] WARNING: 경로 미발견")

    frames: list[dict] = []
    for i in range(0, len(raw_steps), skip):
        end = min(i + skip, len(raw_steps))
        batch = raw_steps[i:end]
        last = batch[-1]
        frames.append({
            "current": last["current"],
            "open": last["open"],
            "expanded": [step["current"] for step in batch],
        })

    record = {
        "schema_version": 1,
        "module": "04_path_planning/05_a_star",
        "kind": "search",
        "grid_size": GRID_SIZE,
        "start": [START[0], START[1]],
        "goal": [GOAL[0], GOAL[1]],
        "obstacles": [[x, y] for (x, y) in sorted(OBSTACLES)],
        "frames": frames,
        "path": [[x, y] for (x, y) in path],
        # 디버그 신호 — expansion 별 시계열 (t = expansion 인덱스). 기본 blueprint
        # 미포함 — viewer entity 패널에서 /debug/<name> 을 TimeSeriesView 로 추가.
        "debug_scalars": dbg.to_debug_scalars(),
        "weight": float(args.weight),  # simulator_search status 패널 표시용 메타.
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out} (weight={args.weight}, "
          f"{len(raw_steps)} raw → {len(frames)} frames (skip={skip}), "
          f"path={len(path)} nodes)  |  재생: simulator_search.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


if __name__ == "__main__":
    main()
