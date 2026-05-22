"""Rerun replay player for search-algorithm records (Dijkstra / A* / RRT 등).

chapter 4 의 path 찾기 알고리즘들은 2D 격자에서 단계별 expansion 을 보여주는 게
핵심이라 chapter 3 의 3D 차량 시뮬레이터 대신 2D Rerun `Spatial2DView` 를 사용.

JSON schema (search):
    {
      "schema_version": 1,
      "module": "<area/problem>",
      "kind": "search",
      "grid_size": int,                       # optional (격자 record — Dijkstra/A*/RRT)
      "space": [x_min, x_max, y_min, y_max],  # optional (연속공간 record — 경계 렌더)
      "start": [x, y],
      "goal":  [x, y],
      "obstacles": [[x, y], ...],         # 격자 점 (Dijkstra/A*/RRT)
                                          # 또는 [[x, y, radius], ...] 원형 (Hybrid A*)
      "frames": [
        {"current":    [x, y],
         "open":       [[x, y], ...],                          # optional (open list)
         "expanded":   [[x, y], ...],                          # optional (batched)
         "new_edges":  [[[px, py], [cx, cy]], ...],            # optional (RRT-style tree)
         "iterations": [int, ...]},                            # optional (RRT: add 별 iteration)
        ...
      ],
      "path": [[x, y], ...],
      "debug_scalars": [                                        # optional, iteration 별 시계열
        {"name": str, "unit": str, "t": [...], "value": [...]}  # t = iteration
      ],
      "ellipses": [                                             # optional (Informed RRT*)
        {"iteration": int, "points": [[x, y], ...]}             # informed 타원, iteration 별
      ],
      "round_paths": [                                          # optional (Informed RRT*)
        {"iteration": int, "points": [[x, y], ...]}             # round 별 채택 경로, iteration 별
      ],
      "weight": float,   # optional, A* heuristic 가중치 (status meta)
      "seed":   int,     # optional, sampling 알고리즘 seed (status meta)
      "eta":    float    # optional, RRT steer 한 step (status meta)
    }

렌더링:
- /world/space:     탐색 공간 경계 (static, 연속공간 record 만 — 옅은 회색 사각형)
- /world/obstacles: 검정 Points2D (static). [x,y,radius] 형태면 per-point radii 로 원형 렌더
- /world/start:     파랑 Points2D (static)
- /world/goal:      빨강 Points2D (static)
- /world/closed:    매 step 누적된 expand-완료 노드들 = closed list (옅은 노랑)
- /world/open:      현재 open list (초록)
- /world/current:   직전 step 에 expand 한 노드 (주황)
- /world/tree:      RRT 등 sampling-tree 의 edge 누적 (옅은 파랑 LineStrips2D)
- /world/path:      탐색 완료 후 최종 path (마젠타 LineStrips2D)
- /world/ellipse:   Informed RRT* 의 informed 타원 (반투명 보라). round 시작 iteration 에
                    갱신돼 그 round 동안 계속 표시됨.
- /world/round_path: Informed RRT* 의 round 별 채택 경로 (밝은 청록 하이라이트).
                    round 가 바뀌면 그 round 의 경로로 갱신, 다음 round 까지 표시됨.
- /debug/<name>:    debug_scalars (optional) — 'iteration' 타임라인의 디버그 시계열.
                    기본 blueprint 미포함 — entity 패널에서 TimeSeriesView 직접 추가.

타임라인: 'step' (frame 단위 batched 재생) · 'iteration' (expansion 단위).
spatial(closed/open/tree/path)·디버그 신호 모두 두 타임라인에 로그되며,
viewer 기본 타임라인은 'iteration' — expansion 축에서 디버그 신호 곡선이 곧바로
보인다. frame 단위로 보려면 하단 타임라인 드롭다운에서 'step' 선택.
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
_CLOSED_COLOR = (255, 220, 80, 110)
_OPEN_COLOR = (90, 220, 90, 220)
_CURRENT_COLOR = (255, 130, 0)
_PATH_COLOR = (220, 70, 220)
_TREE_COLOR = (110, 150, 220, 90)   # 옅은 파랑 — RRT 의 tree edges (반투명)
_SPACE_COLOR = (150, 156, 166)      # 옅은 회색 — 탐색 공간 경계
_ELLIPSE_COLOR = (150, 90, 220, 95)  # 반투명 보라 — informed 타원 (09)
_ROUND_PATH_COLOR = (40, 215, 205)   # 밝은 청록 — round 별 채택 경로 하이라이트 (09)

_REF_EXTENT = 60.0  # 04/05/07 grid 크기 — 아래 marker radii 가 이 extent 에 맞춰짐


def _radius_scale(data: dict) -> float:
    """맵 크기에 맞춘 marker radius 배율 — grid(60) 기준 1.0, 작은 연속공간은 축소.

    radii 상수들이 60-grid 에 맞춰져 있어, Hybrid A* 처럼 작은 연속공간 (~17 m)
    에서는 그대로 쓰면 start/goal/노드/엣지가 과하게 두껍게 보인다. extent 비례로
    줄여 어느 맵에서나 같은 비율로 보이게 한다.
    """
    space = data.get("space")
    if space:
        x_min, x_max, y_min, y_max = space
        extent = max(x_max - x_min, y_max - y_min)
    else:
        extent = float(data.get("grid_size") or _REF_EXTENT)
    return extent / _REF_EXTENT


def _log_static(data: dict) -> None:
    s = _radius_scale(data)
    # 탐색 공간 경계 — 연속공간 record 만 (격자는 obstacle 분포로 범위가 드러남).
    space = data.get("space")
    if space:
        x_min, x_max, y_min, y_max = space
        rect = [[x_min, y_min], [x_max, y_min], [x_max, y_max],
                [x_min, y_max], [x_min, y_min]]
        rr.log("world/space",
               rr.LineStrips2D([rect], colors=[_SPACE_COLOR],
                               radii=[0.04 * s]),
               static=True)
    obs = np.array(data["obstacles"], dtype=float)
    if obs.size:
        if obs.shape[1] == 3:
            # 원형 장애물 (x, y, radius) — radius 는 실제 크기라 스케일 안 함.
            rr.log("world/obstacles",
                   rr.Points2D(obs[:, :2], colors=[_OBSTACLE_COLOR],
                               radii=obs[:, 2]),
                   static=True)
        else:
            # 격자 점 장애물 [x, y] — Dijkstra / A* / RRT.
            rr.log("world/obstacles",
                   rr.Points2D(obs, colors=[_OBSTACLE_COLOR],
                               radii=[0.45 * s]),
                   static=True)
    rr.log("world/start",
           rr.Points2D([data["start"]], colors=[_START_COLOR],
                       radii=[0.7 * s]),
           static=True)
    rr.log("world/goal",
           rr.Points2D([data["goal"]], colors=[_GOAL_COLOR],
                       radii=[0.7 * s]),
           static=True)


def _log_search(data: dict) -> None:
    s = _radius_scale(data)
    frames: list[dict] = data.get("frames", [])
    closed: list[list[float]] = []
    tree_edges: list[list[list[float]]] = []   # RRT 등 sampling-tree 누적용
    n_frames = len(frames)
    # 알고리즘별 메타 (예: A* weight, RRT seed/eta) — 있으면 status 에 한 줄.
    weight = data.get("weight")
    seed = data.get("seed")
    eta = data.get("eta")
    iter_count = 0  # 누적 expansion 수 — spatial 을 'iteration' 축에도 올린다
    for i, frame in enumerate(frames):
        cur = frame["current"]
        # 'expanded' 가 있으면 batch (skip 시 batch 내 모든 expand 노드) — 한 번에 누적.
        # 없으면 backward-compat: current 1 개만.
        expanded_nodes = frame.get("expanded", [cur])
        iter_count += len(expanded_nodes)
        rr.set_time("step", sequence=i)
        # debug 신호와 같은 'iteration'(expansion) 축에 spatial 도 올린다 — frame 은
        # batch 라 그 batch 의 마지막 expansion 인덱스에 놓는다.
        rr.set_time("iteration", sequence=iter_count - 1)
        closed.extend(expanded_nodes)
        rr.log("world/closed",
               rr.Points2D(np.array(closed, dtype=float),
                           colors=[_CLOSED_COLOR], radii=[0.3 * s]))
        open_nodes = frame.get("open", [])
        if open_nodes:
            rr.log("world/open",
                   rr.Points2D(np.array(open_nodes, dtype=float),
                               colors=[_OPEN_COLOR], radii=[0.32 * s]))
        else:
            rr.log("world/open", rr.Clear(recursive=False))
        # RRT-style tree edges (optional). 각 frame 이 새로 추가한 edge 들을 누적.
        new_edges = frame.get("new_edges", [])
        if new_edges:
            tree_edges.extend(new_edges)
            n_e = len(tree_edges)
            rr.log("world/tree",
                   rr.LineStrips2D(
                       [np.array(e, dtype=float) for e in tree_edges],
                       colors=[_TREE_COLOR] * n_e,
                       radii=[0.08 * s] * n_e))
        rr.log("world/current",
               rr.Points2D([cur], colors=[_CURRENT_COLOR], radii=[0.5 * s]))
        # 실시간 status (TextDocumentView 옆 패널) — scrubber 따라 갱신.
        lines = [f"step {i + 1} / {n_frames}",
                 f"closed: {len(closed)}",
                 f"open: {len(open_nodes)}",
                 f"current: ({cur[0]}, {cur[1]})"]
        it = frame.get("iteration")
        if it is not None:
            lines.append(f"iteration: {it}")
        if tree_edges:
            lines.append(f"tree edges: {len(tree_edges)}")
        if weight is not None:
            lines.append(f"weight: {weight}")
        if seed is not None:
            lines.append(f"seed: {seed}")
        if eta is not None:
            lines.append(f"eta: {eta}")
        rr.log("info/status", rr.TextDocument("\n".join(lines)))

    # 최종 path — 마지막 step 다음 frame 에 한 번 로그.
    path = data.get("path", [])
    if path:
        rr.set_time("step", sequence=len(frames))
        rr.set_time("iteration", sequence=iter_count)
        rr.log("world/path",
               rr.LineStrips2D([np.array(path, dtype=float)],
                               colors=[_PATH_COLOR], radii=[0.18 * s]))


def _log_search_rrt(data: dict) -> None:
    """RRT search record — tree 를 `step`·`iteration` 두 타임라인에 함께 로그.

    프레임 안의 add 들을 per-add 로 펼쳐, add 마다 `step`(frame index)·`iteration`
    을 둘 다 set 하고 누적 tree/closed/current 를 로그한다. `step` 타임라인은
    frame 단위로(같은 step 의 마지막 add 가 표시), `iteration` 타임라인은 add
    단위로 tree 가 자라는 게 보인다. 매 로그마다 양쪽 타임라인을 set 하므로
    교차 오염 없음.
    """
    s = _radius_scale(data)
    # informed round 경계 (09) — status 의 round 표시용. 그 외엔 빈 list.
    round_starts = sorted(int(e["iteration"]) for e in data.get("ellipses", []))
    frames: list[dict] = data.get("frames", [])
    n_frames = len(frames)
    weight = data.get("weight")
    seed = data.get("seed")
    eta = data.get("eta")
    closed: list[list[float]] = []
    tree_edges: list[list[list[float]]] = []
    last_it = 0
    for i, frame in enumerate(frames):
        expanded = frame.get("expanded", [])
        new_edges = frame.get("new_edges", [])
        iters = frame.get("iterations", [])
        for node, edge, it in zip(expanded, new_edges, iters, strict=True):
            rr.set_time("step", sequence=i)
            rr.set_time("iteration", sequence=int(it))
            last_it = int(it)
            closed.append(node)
            tree_edges.append(edge)
            rr.log("world/closed",
                   rr.Points2D(np.array(closed, dtype=float),
                               colors=[_CLOSED_COLOR], radii=[0.3 * s]))
            rr.log("world/tree",
                   rr.LineStrips2D(
                       [np.array(e, dtype=float) for e in tree_edges],
                       colors=[_TREE_COLOR] * len(tree_edges),
                       radii=[0.08 * s] * len(tree_edges)))
            rr.log("world/current",
                   rr.Points2D([node], colors=[_CURRENT_COLOR],
                               radii=[0.5 * s]))
            lines = [f"step {i + 1} / {n_frames}",
                     f"iteration: {it}"]
            if round_starts:
                cur_round = sum(1 for rs in round_starts if rs <= int(it)) - 1
                lines.append(
                    f"inform round: {cur_round + 1} / {len(round_starts)}")
            lines += [f"tree nodes: {len(closed)}",
                      f"tree edges: {len(tree_edges)}"]
            if weight is not None:
                lines.append(f"weight: {weight}")
            if seed is not None:
                lines.append(f"seed: {seed}")
            if eta is not None:
                lines.append(f"eta: {eta}")
            rr.log("info/status", rr.TextDocument("\n".join(lines)))
    # 최종 path — 마지막 step·iteration 다음 위치에 한 번.
    path = data.get("path", [])
    if path:
        rr.set_time("step", sequence=n_frames)
        rr.set_time("iteration", sequence=last_it + 1)
        rr.log("world/path",
               rr.LineStrips2D([np.array(path, dtype=float)],
                               colors=[_PATH_COLOR], radii=[0.18 * s]))


def _log_debug(data: dict) -> None:
    """debug_scalars (optional) — search 알고리즘의 iteration 별 디버그 신호.

    record 의 `debug_scalars` 항목 (`{name, unit, t, value}`, `t` = iteration) 을
    `/debug/<name>` 엔티티로 'iteration' 타임라인에 로그한다. `_build_blueprint`
    는 이를 위한 view 를 만들지 않는다 — 학생이 viewer 좌측 entity 패널에서
    `/debug/...` 를 골라 TimeSeriesView 를 직접 추가하면 본다.
    """
    for sc in data.get("debug_scalars", []):
        name = sc["name"]
        for it, value in zip(sc["t"], sc["value"], strict=True):
            rr.set_time("iteration", sequence=int(it))
            rr.log(f"debug/{name}", rr.Scalars(float(value)))


def _log_ellipses(data: dict) -> None:
    """ellipses (optional, Informed RRT*) — round 별 informed 타원.

    record 의 `ellipses` 항목 (`{iteration, points}`) 을 `/world/ellipse` 엔티티로
    'iteration' 타임라인에 반투명 선으로 로그한다. round 시작 iteration 에 한 번
    로그하면 다음 round 의 타원이 덮어쓸 때까지 persist — 그 round 동안 계속 보인다.
    높은 `draw_order` 로 tree·node 위에 항상 그려진다 (매 step 재로그 불필요).
    points 가 비면 (round 0, 타원 없음) Clear.
    """
    s = _radius_scale(data)
    for e in data.get("ellipses", []):
        rr.set_time("iteration", sequence=int(e["iteration"]))
        pts = e.get("points", [])
        if pts:
            rr.log("world/ellipse",
                   rr.LineStrips2D([np.array(pts, dtype=float)],
                                   colors=[_ELLIPSE_COLOR], radii=[0.07 * s],
                                   draw_order=1000.0))  # tree·node 위에 항상
        else:
            rr.log("world/ellipse", rr.Clear(recursive=False))


def _log_round_paths(data: dict) -> None:
    """round_paths (optional, Informed RRT*) — round 별 채택 경로 하이라이트.

    record 의 `round_paths` (`{iteration, points}`) 를 `/world/round_path`
    엔티티로 'iteration' 타임라인에 밝은 청록 선으로 로그한다. round 시작
    iteration 에 한 번 로그하면 다음 round 의 경로가 덮어쓸 때까지 persist —
    그 round 동안 현재 informed 타원과 같은 경로(타원 장축)가 강조된다. 높은
    `draw_order` 로 tree·node 위에 그려진다. points 가 비면 (round 0) Clear.
    """
    s = _radius_scale(data)
    for rp in data.get("round_paths", []):
        rr.set_time("iteration", sequence=int(rp["iteration"]))
        pts = rp.get("points", [])
        if pts:
            rr.log("world/round_path",
                   rr.LineStrips2D([np.array(pts, dtype=float)],
                                   colors=[_ROUND_PATH_COLOR], radii=[0.16 * s],
                                   draw_order=900.0))  # tree·node 위
        else:
            rr.log("world/round_path", rr.Clear(recursive=False))


def _build_blueprint(data: dict) -> rrb.Blueprint:
    main = rrb.Horizontal(
        rrb.Spatial2DView(origin="/world", name="search"),
        rrb.TextDocumentView(origin="/info/status", name="status"),
        column_shares=[4, 1],
    )
    # search record 는 spatial·debug 모두 'iteration'(expansion) 축에 로그된다 —
    # 기본 타임라인을 iteration 으로 열어 디버그 신호 곡선이 곧바로 보이게 한다.
    # ('step' 은 frame 단위 batched 재생 — 하단 타임라인 드롭다운에서 선택.)
    return rrb.Blueprint(
        main, rrb.TimePanel(timeline="iteration", state="expanded"))


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
        if data.get("frames") and "iterations" in data["frames"][0]:
            _log_search_rrt(data)
        else:
            _log_search(data)
        _log_debug(data)
        _log_ellipses(data)
        _log_round_paths(data)
        plan.append((rec, _build_blueprint(data), rid))

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
