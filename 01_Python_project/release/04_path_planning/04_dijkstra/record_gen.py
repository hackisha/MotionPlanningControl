"""Dijkstra search 시각화 — 실시간 노드 expansion + 최종 path 를 Rerun 2D 로 재생.

`on_step` 콜백으로 매 expand 마다 (current, frontier) 를 캡처해 record.json 으로 직렬화.
재생: 같은 폴더 ../simulator_search.py.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from dijkstra import dijkstra
from map_data import GOAL, GRID_SIZE, OBSTACLES, START


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dijkstra 탐색 실행 → record.json 생성 (Rerun viewer 로 단계별 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()

    frames: list[dict] = []

    def on_step(current: tuple[int, int], frontier: set[tuple[int, int]]) -> None:
        frames.append({
            "current": [current[0], current[1]],
            "frontier": [[p[0], p[1]] for p in frontier],
        })

    path = dijkstra(START, GOAL, OBSTACLES, on_step=on_step)
    if not path:
        print("[record] WARNING: 경로 미발견")

    record = {
        "schema_version": 1,
        "module": "04_path_planning/04_dijkstra",
        "kind": "search",
        "grid_size": GRID_SIZE,
        "start": [START[0], START[1]],
        "goal": [GOAL[0], GOAL[1]],
        "obstacles": [[x, y] for (x, y) in sorted(OBSTACLES)],
        "frames": frames,
        "path": [[x, y] for (x, y) in path],
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out} ({len(frames)} steps, path={len(path)} nodes)"
          f"  |  재생: simulator_search.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


if __name__ == "__main__":
    main()
