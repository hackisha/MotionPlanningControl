"""RRT* — Rapidly-exploring Random Tree with optimal rewiring.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

import math
import random
from typing import Callable


def _euclid(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _steer(node_from: tuple[float, float], node_to: tuple[float, float],
           eta: float) -> tuple[float, float]:
    """`node_from` 에서 `node_to` 방향으로 최대 `eta` 만큼 전진한 새 점.

    mag ≤ eta 면 node_to 자체 반환, 아니면 eta 거리로 잘라낸 점.
    """
    dx = node_to[0] - node_from[0]
    dy = node_to[1] - node_from[1]
    mag = math.hypot(dx, dy)
    if mag <= eta:
        return (float(node_to[0]), float(node_to[1]))
    return (node_from[0] + eta * dx / mag,
            node_from[1] + eta * dy / mag)


def _is_collision_free(node_from: tuple[float, float],
                       node_to: tuple[float, float],
                       obstacles: set[tuple[int, int]],
                       step: float = 0.3) -> bool:
    """선분 위를 `step` 간격으로 sample 해서 정수 격자 cell 이 obstacles 에 있으면 충돌."""
    dx = node_to[0] - node_from[0]
    dy = node_to[1] - node_from[1]
    mag = math.hypot(dx, dy)
    if mag < 1e-9:
        return True
    n = max(1, int(math.ceil(mag / step)))
    for k in range(n + 1):
        t = k / n
        x = node_from[0] + t * dx
        y = node_from[1] + t * dy
        cell = (int(round(x)), int(round(y)))
        if cell in obstacles:
            return False
    return True


def rrt_star(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: set[tuple[int, int]],
    grid_size: int,
    max_iter: int = 2000,
    eta: float = 3.0,
    goal_sample_rate: float = 0.05,
    goal_range: float = 1.5,
    search_radius: float = 8.0,
    seed: int | None = 0,
    on_step: Callable[[tuple[float, float], tuple[float, float], int],
                      None] | None = None,
    dbg=None,
) -> list[tuple[float, float]]:
    """start 에서 goal 까지 RRT* 로 경로 탐색.

    Args:
        start, goal: (x, y) 연속 좌표.
        obstacles: 정수 격자 cell set (충돌 검사용).
        grid_size: 샘플링 영역 `[0, grid_size]²`.
        max_iter: 최대 반복 — 도달 못 하면 [] 반환.
        eta: steer 한 step 최대 거리.
        goal_sample_rate: 매 iter 마다 goal 을 직접 sampling 할 확률 (bias).
        goal_range: 새 node 가 goal 에서 이 거리 안이면 도달 판정.
        search_radius: choose-parent·rewire 가 후보를 찾는 반경 (eta 보다 크게).
        seed: random seed (None = OS random). 재현성 목적.
        on_step: node 추가·rewire 직후 호출 — `(child, parent, iteration)`.
        dbg: optional 디버그 신호 수집기 (`DebugSignals`). 주어지면 탐색 루프에서
            매 iteration (reject 포함) `dbg.add(...)` 로 내부 값을 남긴다 —
            record_gen 이 이를 `debug_scalars` 로 저장. None 이면 수집 안 함.

    Returns:
        path: [(x0,y0), ..., (xn,yn)] start → goal-근접. 미발견 시 `[]`.
    """
    # TODO: RRT* 로 start → goal 경로를 구하시오.
    # RRT (07) 의 5 단계 위에 near / choose-parent / rewire 가 얹힌 구조입니다.
    #
    # 자료 구조:
    #   - nodes:    list[(x, y)]                 — tree node. nodes[0] = start.
    #   - parents:  dict[idx, parent_idx_or_None]
    #   - children: dict[idx, list[child_idx]]   — rewire 시 subtree cost 전파용
    #   - cost:     dict[idx, float]             — start 부터 누적 거리. cost[0] = 0.
    #   - rng = random.Random(seed)
    #
    # 매 iter (0 ≤ it < max_iter):
    #   1. Sample:
    #        if rng.random() < goal_sample_rate: sample = goal
    #        else: sample = (rng.uniform(0, grid_size), rng.uniform(0, grid_size))
    #   2. Nearest: nodes 중 sample 과 가장 가까운 node index — `_euclid` 사용.
    #   3. Steer: `_steer(nearest, sample, eta)` → new_pt.
    #      new_pt 가 nearest 와 거의 같거나(`_euclid < 1e-6`),
    #      `_is_collision_free(nearest, new_pt, obstacles)` 가 False 면 이 iter skip.
    #   4. Near: new_pt 기준 search_radius 안의 기존 node index 들.
    #   5. Choose-parent: near 중 `_is_collision_free` 로 이을 수 있고
    #      `cost[i] + _euclid(nodes[i], new_pt)` 가 최소인 i 를 부모로.
    #      후보가 없으면 nearest (reject 통과라 충돌 없음 보장).
    #      → nodes.append(new_pt); parents/children/cost 갱신;
    #        on_step(new_pt, nodes[best_i], it) 호출 (있으면).
    #   6. Rewire: near 의 각 node i 에 대해
    #      `cost[new] + _euclid(new_pt, nodes[i]) < cost[i]` 이고 충돌 없으면
    #      i 의 부모를 new 로 교체 — parents/children 갱신, cost[i] 갱신,
    #      i 의 subtree cost 도 함께 갱신(전파), on_step(nodes[i], new_pt, it) 호출.
    #   7. Goal check: `_euclid(new_pt, goal) < goal_range` 면 path 복원:
    #      parents 체인을 None 까지 따라가며 nodes 누적, 뒤집어 반환.
    #
    # max_iter 다 돌면 [] 반환.
    #
    # 디버그 (선택): 인자 dbg 가 주어지면 매 iter 안에서
    #   if dbg is not None:
    #       dbg.add(goal_dist=..., rejected=..., tree_size=...,
    #               near_count=..., rewire_count=...)
    #   처럼 내부 값을 남길 수 있다 — record_gen 이 debug_scalars 로 저장,
    #   viewer 의 /debug/<name> 에서 iteration 별로 분석.
    raise NotImplementedError
