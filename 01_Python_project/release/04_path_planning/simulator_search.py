"""Rerun replay player for search-algorithm records (Dijkstra / A* / RRT 등).

chapter 4 의 path 찾기 알고리즘들은 2D 격자에서 단계별 expansion 을 보여주는 게
핵심이라 chapter 3 의 3D 차량 시뮬레이터 대신 2D Rerun `Spatial2DView` 를 사용.

JSON schema (search):
    {
      "schema_version": 1,
      "module": "<area/problem>",
      "kind": "search",
      "grid_size": int,
      "start": [x, y],
      "goal":  [x, y],
      "obstacles": [[x, y], ...],
      "frames": [
        {"current": [x, y], "frontier": [[x, y], ...]},
        ...
      ],
      "path": [[x, y], ...]
    }

렌더링:
- /world/obstacles: 검정 Points2D (static)
- /world/start:     파랑 Points2D (static)
- /world/goal:      빨강 Points2D (static)
- /world/visited:   매 step 누적된 expand-된 노드들 (옅은 노랑)
- /world/frontier:  현재 open_list (초록)
- /world/current:   직전 step 에 expand 한 노드 (주황)
- /world/path:      탐색 완료 후 최종 path (마젠타 LineStrips2D)

타임라인: 'step' (sequence 정수). viewer 좌측 timeline scrubber 로 탐색 과정 재생.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

APP_ID = "search_replay"

_OBSTACLE_COLOR = (40, 40, 40)
_START_COLOR = (50, 100, 220)
_GOAL_COLOR = (220, 60, 60)
_VISITED_COLOR = (255, 220, 80, 110)
_FRONTIER_COLOR = (90, 220, 90, 220)
_CURRENT_COLOR = (255, 130, 0)
_PATH_COLOR = (220, 70, 220)


def _log_static(data: dict) -> None:
    obs = np.array(data["obstacles"], dtype=float)
    if obs.size:
        rr.log("world/obstacles",
               rr.Points2D(obs, colors=[_OBSTACLE_COLOR], radii=[0.45]),
               static=True)
    rr.log("world/start",
           rr.Points2D([data["start"]], colors=[_START_COLOR], radii=[0.7]),
           static=True)
    rr.log("world/goal",
           rr.Points2D([data["goal"]], colors=[_GOAL_COLOR], radii=[0.7]),
           static=True)


def _log_search(data: dict) -> None:
    frames: list[dict] = data.get("frames", [])
    visited: list[list[float]] = []
    for i, frame in enumerate(frames):
        rr.set_time("step", sequence=i)
        cur = frame["current"]
        visited.append(cur)
        rr.log("world/visited",
               rr.Points2D(np.array(visited, dtype=float),
                           colors=[_VISITED_COLOR], radii=[0.3]))
        frontier = frame.get("frontier", [])
        if frontier:
            rr.log("world/frontier",
                   rr.Points2D(np.array(frontier, dtype=float),
                               colors=[_FRONTIER_COLOR], radii=[0.32]))
        else:
            rr.log("world/frontier", rr.Clear(recursive=False))
        rr.log("world/current",
               rr.Points2D([cur], colors=[_CURRENT_COLOR], radii=[0.5]))

    # 최종 path — 마지막 step 다음 frame 에 한 번 로그.
    path = data.get("path", [])
    if path:
        rr.set_time("step", sequence=len(frames))
        rr.log("world/path",
               rr.LineStrips2D([np.array(path, dtype=float)],
                               colors=[_PATH_COLOR], radii=[0.18]))


def _build_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Spatial2DView(origin="/world", name="search"),
    )


def _recording_id(record_path: Path) -> str:
    return record_path.parent.name


def replay_search(record_paths: list[Path]) -> None:
    """여러 search record 를 한 viewer 에 별도 recording 으로 로드."""
    plan: list[tuple[rr.RecordingStream, rrb.Blueprint, str]] = []
    for record_path in record_paths:
        data = json.loads(record_path.read_text(encoding="utf-8"))
        rid = _recording_id(record_path)
        rec = rr.RecordingStream(application_id=APP_ID, recording_id=rid)
        rr.set_global_data_recording(rec)
        _log_static(data)
        _log_search(data)
        plan.append((rec, _build_blueprint(), rid))

    plan[0][0].spawn(default_blueprint=plan[0][1])
    for rec, bp, _ in plan[1:]:
        rec.connect_grpc(default_blueprint=bp)

    for i, (_, _, rid) in enumerate(plan):
        print(f"[simulator_search] [{i+1}/{len(plan)}] {rid}")


def _find_records(root: Path) -> list[Path]:
    """search record 만 골라냄 (kind == 'search')."""
    found: list[Path] = []
    for p in sorted(root.rglob("record*.json")):
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            if data.get("kind") == "search":
                found.append(p)
        except (OSError, ValueError):
            continue
    return found


def main() -> None:
    parser = argparse.ArgumentParser(
        description="04_path_planning search record*.json 을 Rerun 2D viewer 로 재생")
    parser.add_argument(
        "path", nargs="?", default=None,
        help="record.json 파일 또는 디렉토리 (생략 시 스크립트 폴더 하위 스캔)")
    args = parser.parse_args()

    arg = Path(args.path) if args.path else Path(__file__).parent
    if not arg.exists():
        print(f"경로 없음: {arg}", file=sys.stderr)
        sys.exit(1)

    records = [arg] if arg.is_file() else _find_records(arg)
    if not records:
        print(f"search record*.json 을 찾지 못함: {arg}\n"
              f"  먼저 각 모듈 record_gen.py 를 실행해 record.json 을 생성하세요.",
              file=sys.stderr)
        sys.exit(1)

    replay_search(records)


if __name__ == "__main__":
    main()
