"""Informed RRT* 시각화 — 단일 tree 성장 + informed 타원 수렴을 Rerun 2D 로 재생.

`--seed S` / `--eta E` / `--max-iter N` 로 실험 가능. `--skip N`
으로 frame subsample (기본 10). 재생: 같은 폴더 ../simulator_search.py.

디버그 신호: informed_rrt_star() 가 매 iteration `goal_dist`·`rejected`·`tree_size`·
`rewire_count`·`inform_round`·`best_cost` 를 수집해 record 의 `debug_scalars` 로 저장한다
(x축 = 전역 iteration, round 가 바뀌어도 연속). viewer 기본 화면엔 안 보이며, entity
패널에서 `/debug/<name>` 을 TimeSeriesView 로 추가해 본다 — `best_cost` 가 round 마다
계단식으로 줄어드는 게 informed 수렴이다.

현재 round 의 informed 타원은 record 의 `ellipses` 로, 그 round 에 채택된 경로는
`round_paths` 로 저장돼 viewer 에 `/world/ellipse`(반투명 보라)·`/world/round_path`
(밝은 청록 하이라이트)로 표시된다 — iteration 을 스크럽하면 round 마다 타원과
하이라이트 경로가 함께 좁아진다. round_paths 는 planner 를 건드리지 않고 on_step
엣지 스트림으로 tree 를 복원해 뽑는다 (`_reconstruct_round_path`).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from informed_rrt_star import informed_rrt_star
from map_informed import GOAL, GRID_SIZE, OBSTACLES, START

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from debug_signals import DebugSignals  # noqa: E402

# informed_rrt_star() 의 goal_range 기본값과 일치해야 함 — round 경로 복원 시
# goal 도달 판정 반경. record_gen 은 goal_range 를 따로 넘기지 않으므로 기본값 사용.
_GOAL_RANGE = 8.0


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


def _reconstruct_round_path(
    edges: list[tuple[tuple[float, float], tuple[float, float]]],
    start: tuple[float, float],
    goal: tuple[float, float],
    goal_range: float,
) -> list[list[float]]:
    """on_step 엣지 스트림 `(child, parent)` 을 순서대로 재생해 tree 를 복원하고,
    goal 반경 안 node 중 `(start 까지 cost + goal 직선거리)` 가 최소인 경로를 반환.

    informed_rrt_star() 내부 `_best_goal_path` 와 같은 선택 기준 — planner 를
    건드리지 않고 record_gen 이 round 별 채택 경로를 복원하려는 재구성이다.
    node 추가·rewire 가 모두 on_step 으로 들어오므로, 엣지를 순서대로 적용하면
    (마지막 부모가 유효) 임의 iteration 시점의 tree 가 정확히 복원된다.

    cost 는 `cost(node) = cost(parent) + edge` 로 memo — planner 가 유지하는
    `cost[]` 와 부동소수점까지 동일해야, 동점(node 와 그 자식이 같은 비용으로
    goal 에 닿는 경우) 에서 planner 와 같은 node 를 고른다. dict 삽입 순서가
    node 추가 순서와 같아 `min` 의 동점 처리도 planner 의 index 순서와 일치한다.
    """
    parent: dict[tuple[float, float], tuple[float, float] | None] = {start: None}
    for child, par in edges:
        parent[child] = par

    cost_memo: dict[tuple[float, float], float] = {start: 0.0}

    def cost(pt: tuple[float, float]) -> float:
        # 부모 사슬을 따라 미계산 node 를 쌓고, memo 에 닿으면 위→아래로 채운다.
        stack: list[tuple[float, float]] = []
        node = pt
        while node not in cost_memo:
            up = parent.get(node)
            if up is None or node in stack:    # 끊긴 node / 사이클 방어
                for n in (*stack, node):
                    cost_memo[n] = float("inf")
                return cost_memo[pt]
            stack.append(node)
            node = up
        for n in reversed(stack):
            up = parent[n]
            cost_memo[n] = cost_memo[up] + math.hypot(n[0] - up[0],
                                                      n[1] - up[1])
        return cost_memo[pt]

    reached = [pt for pt in parent
               if math.hypot(pt[0] - goal[0], pt[1] - goal[1]) < goal_range]
    if not reached:
        return []
    best = min(reached, key=lambda pt: cost(pt)
               + math.hypot(pt[0] - goal[0], pt[1] - goal[1]))
    path: list[list[float]] = []
    node = best
    while node is not None:
        path.append([node[0], node[1]])
        node = parent.get(node)
    path.reverse()
    # 마지막 node 가 goal 자체가 아니면 goal 까지의 직선 segment 를 덧붙인다 —
    # planner 의 채택 비용(eff)이 이 segment 를 포함하므로, 그려지는 경로의
    # 전체 길이가 그 round 의 informed 타원 장축과 일치한다.
    if path[-1] != [goal[0], goal[1]]:
        path.append([goal[0], goal[1]])
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Informed RRT* 탐색 실행 → record.json 생성 (Rerun viewer 재생)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움")
    parser.add_argument("--skip", type=int, default=10,
                        help="N step 마다 frame 1 개로 묶음 (기본 10).")
    # [튜닝] 학생이 viewer 에서 실험 가능 — test 의 값은 변경 X.
    parser.add_argument("--seed", type=int, default=0,
                        help="random seed (기본 0 = 재현 가능).")
    parser.add_argument("--eta", type=float, default=6.0,
                        help="첫 경로 전 steer 한 step 거리 (기본 6.0).")
    parser.add_argument("--max-iter", type=int, default=900,
                        help="전체 sampling 반복 수 (anytime budget, 기본 900).")
    args = parser.parse_args()
    skip = max(1, args.skip)

    raw_steps: list[dict] = []
    dbg = DebugSignals()  # 디버그 신호 수집기 — informed_rrt_star() 가 매 iteration 채운다

    def on_step(child: tuple[float, float],
                parent: tuple[float, float],
                iteration: int) -> None:
        # node 추가·rewire 양쪽에서 호출 — 둘 다 (child, parent) edge 로 그린다.
        # iteration 은 round 가 바뀌어도 연속되는 전역 카운터.
        raw_steps.append({
            "current": [child[0], child[1]],
            "edge": [[parent[0], parent[1]], [child[0], child[1]]],
            "iteration": iteration,
        })

    path = informed_rrt_star(START, GOAL, OBSTACLES, GRID_SIZE,
                             max_iter=args.max_iter,
                             eta=args.eta, seed=args.seed, on_step=on_step, dbg=dbg)
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
            "open": [],  # Informed RRT* 도 open list (priority queue) 없음.
            "expanded": [step["current"] for step in batch],
            "new_edges": [step["edge"] for step in batch],
            "iterations": [step["iteration"] for step in batch],
        })

    # informed 타원 + round 별 채택 경로 — round 가 바뀌는 iteration 마다 둘 다 기록.
    # 타원: best_cost 가 장축 (round 0 은 best_cost=0 → 타원 없음, 전체 공간).
    # 경로: 그 iteration 까지의 on_step 엣지로 tree 를 복원해 best 경로를 뽑는다 —
    #       viewer 가 /world/round_path 로 하이라이트, 다음 round 까지 persist.
    scalars = {s["name"]: s["value"] for s in dbg.to_debug_scalars()}
    all_edges = [(tuple(s["edge"][1]), tuple(s["edge"][0]), s["iteration"])
                 for s in raw_steps]
    ellipses: list[dict] = []
    round_paths: list[dict] = []
    prev_round = -1.0
    for it, rnd in enumerate(scalars.get("inform_round", [])):
        if rnd != prev_round:
            prev_round = rnd
            bc = scalars["best_cost"][it]
            ellipses.append({
                "iteration": it,
                "points": _ellipse_polygon(START, GOAL, bc) if bc > 0.0 else [],
            })
            if bc > 0.0:
                edges_upto = [(c, p) for (c, p, e) in all_edges if e <= it]
                rp = _reconstruct_round_path(
                    edges_upto, (START[0], START[1]), (GOAL[0], GOAL[1]),
                    _GOAL_RANGE)
            else:
                rp = []   # round 0 — 아직 경로 없음
            round_paths.append({"iteration": it, "points": rp})

    record = {
        "schema_version": 1,
        "module": "04_path_planning/09_informed_rrt_star",
        "kind": "search",
        "grid_size": GRID_SIZE,
        "start": [START[0], START[1]],
        "goal": [GOAL[0], GOAL[1]],
        "obstacles": [[x, y] for (x, y) in sorted(OBSTACLES)],
        "frames": frames,
        "path": [[p[0], p[1]] for p in path],
        # informed 타원 — iteration 별로 simulator 가 /world/ellipse 에 반투명 렌더.
        "ellipses": ellipses,
        # round 별 채택 경로 — simulator 가 /world/round_path 에 하이라이트 렌더.
        # round 가 바뀌는 iteration 에 갱신, 다음 round 까지 persist.
        "round_paths": round_paths,
        # 디버그 신호 — informed_rrt_star() 가 매 iteration 수집한 시계열
        # (t = 전역 iteration). 기본 blueprint 미포함 — viewer entity 패널에서
        # /debug/<name> 을 TimeSeriesView 로 추가해 본다.
        "debug_scalars": dbg.to_debug_scalars(),
        "seed": int(args.seed),  # simulator_search status 패널 메타.
        "eta": float(args.eta),
    }
    out = Path(__file__).parent / "record.json"
    out.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out} (seed={args.seed}, eta={args.eta}, "
          f"max_iter={args.max_iter}, {len(raw_steps)} steps → "
          f"{len(frames)} frames (skip={skip}), path={len(path)} nodes)  |  "
          f"재생: simulator_search.py")

    if not args.no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_search import replay_search  # type: ignore[no-redef]
        replay_search([out])


if __name__ == "__main__":
    main()
