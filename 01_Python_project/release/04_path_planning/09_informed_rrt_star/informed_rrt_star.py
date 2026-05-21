"""Informed RRT* — 타원 informed sampling 으로 경로를 단계적으로 줄이는 planner.

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
                       step: float = 0.15) -> bool:
    """선분 위를 `step` 간격으로 sample 해서 정수 격자 cell 이 obstacles 에 있으면 충돌.

    step 은 격자 모서리 grazing 까지 잡도록 촘촘하게 (검증 테스트보다 fine).
    """
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


def _path_length(path: list[tuple[float, float]]) -> float:
    """경로의 연속 segment 길이 합."""
    return sum(_euclid(path[i], path[i + 1]) for i in range(len(path) - 1))


def _sample_in_ellipse(rng: random.Random,
                       x_start: tuple[float, float],
                       x_goal: tuple[float, float],
                       c_best: float) -> tuple[float, float]:
    """start·goal 을 초점, `c_best` 를 장축 길이로 하는 타원 안의 uniform random 점.

    비용 c_best 이하의 경로가 지날 수 있는 영역(informed subset)만 샘플한다.
    단위 원판 uniform → 타원 축 스케일 → start→goal 방향 회전 → 중심 평행이동.
    `c_best` 가 초점 간 거리 이하이면 단축이 0 (타원이 선분으로 퇴화).
    """
    c_min = _euclid(x_start, x_goal)
    cx = 0.5 * (x_start[0] + x_goal[0])
    cy = 0.5 * (x_start[1] + x_goal[1])
    theta = math.atan2(x_goal[1] - x_start[1], x_goal[0] - x_start[0])
    a = 0.5 * c_best                                              # 장축 반길이
    b = 0.5 * math.sqrt(max(c_best * c_best - c_min * c_min, 0.0))  # 단축 반길이
    rad = math.sqrt(rng.random())          # 단위 원판 uniform
    phi = 2.0 * math.pi * rng.random()
    ex = a * rad * math.cos(phi)            # 타원 축으로 스케일
    ey = b * rad * math.sin(phi)
    x = cx + ex * math.cos(theta) - ey * math.sin(theta)   # 회전 + 평행이동
    y = cy + ex * math.sin(theta) + ey * math.cos(theta)
    return (x, y)


def informed_rrt_star(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: set[tuple[int, int]],
    grid_size: int,
    max_iter: int = 900,
    eta: float = 6.0,
    goal_range: float = 8.0,
    goal_range_min: float = 1.5,
    eta_decay: float = 0.72,
    eta_min: float = 2.0,
    goal_decay: float = 1.0,
    improve_eps: float = 0.3,
    search_radius: float = 8.0,
    goal_sample_rate: float = 0.05,
    seed: int | None = 0,
    on_step: Callable[[tuple[float, float], tuple[float, float], int],
                      None] | None = None,
    dbg=None,
) -> list[tuple[float, float]]:
    """start 에서 goal 까지 Informed RRT* 로 경로 탐색.

    단일 tree·단일 loop — `max_iter` 회 반복하며 RRT* 를 수행하고, 더 짧은 경로를
    찾을 때마다 `c_best` 와 informed 타원이 즉시 좁아진다. round 는 별도 phase 가
    아니라 "몇 번째 개선인가"를 세는 카운터(`inform_round`)이며, eta·goal_range 의
    coarse→fine schedule 만 그 카운터로 단계 조절한다.

    Args:
        start, goal: (x, y) 연속 좌표.
        obstacles: 정수 격자 cell set (충돌 검사용).
        grid_size: 첫 경로 전 샘플링 영역 `[0, grid_size]²`.
        max_iter: 전체 sampling 반복 수 (anytime budget) — 유일한 종료 조건.
        eta: inform_round 0 (첫 경로 전) 의 steer 한 step 최대 거리
            (round r 은 eta·eta_decay^r, eta_min 으로 하한 clip).
        goal_range: inform_round 0 의 goal 도달 판정 거리
            (round r 은 goal_range·goal_decay^r, goal_range_min 으로 하한 clip).
        goal_range_min: round 별 goal_range 의 하한 clip.
        eta_decay: 개선(round)마다 eta 에 곱해지는 축소율 (<1).
        eta_min: round 별 eta 의 하한 clip — round 가 늘어도 0 으로 안 줄게.
        goal_decay: 개선(round)마다 goal_range 에 곱해지는 비율 — 기본 1.0 (고정),
            <1 로 주면 round 마다 축소.
        improve_eps: round 진행으로 인정하는 최소 개선 폭. 새 경로의 goal 도달
            추정비용이 c_best 보다 이만큼은 짧아야 c_best·타원·round 를 갱신한다.
            floating-point 수준의 미세 개선이 round 를 무수히 늘리는 것을 막는다.
        search_radius: choose-parent·rewire 가 후보를 찾는 반경.
        goal_sample_rate: 매 iter 마다 goal 을 직접 sampling 할 확률 (bias).
        seed: random seed (None = OS random). 재현성 목적.
        on_step: node 추가·rewire 직후 호출 — `(child, parent, iteration)`.
            iteration 은 단일 loop 의 연속 카운터.
        dbg: optional 디버그 신호 수집기 (`DebugSignals`). 주어지면 매 iteration
            `dbg.add(...)` 로 내부 값을 남긴다. None 이면 수집 안 함.

    Returns:
        path: max_iter 까지의 최단 경로. 미발견 시 `[]`.
    """
    # TODO: Informed RRT* 로 start → goal 경로를 구하시오.
    # 08 RRT* 의 inner loop 를 단일 loop `for it in range(max_iter)` 안에서 그대로
    # 수행하되, 더 짧은 경로를 찾을 때마다 informed 타원을 좁히는 구조입니다.
    # tree 는 단 하나 — loop 가 돌아도 유지합니다.
    #
    # 준비 (loop '밖'에서 한 번만):
    #   - rng = random.Random(seed)
    #   - 단일 tree: nodes=[start], parents={0:None}, children={0:[]}, cost={0:0.0}
    #   - best_path = [], c_best = inf
    #   - inform_round = 0  ← 개선을 찾을 때마다 +1 (타원 schedule 단계)
    #
    # for it in range(max_iter):
    #   - eta_r        = max(eta * (eta_decay ** inform_round), eta_min)
    #   - goal_range_r = max(goal_range * (goal_decay ** inform_round), goal_range_min)
    #   - use_ellipse  = (c_best != inf)            # 첫 경로를 찾은 뒤로 True
    #
    #   1. Sample:
    #        rng.random() < goal_sample_rate            → sample = goal
    #        elif use_ellipse → _sample_in_ellipse(rng, start, goal, c_best)
    #        else             → (rng.uniform(0, grid_size), rng.uniform(0, grid_size))
    #   2. Nearest → 3. Steer(eta_r) → 4. Reject(too_short / 충돌)
    #   5. Near (search_radius) → 6. Choose-parent → node 추가 → on_step(.., it)
    #   7. Rewire (+ subtree cost 전파) → 재배선마다 on_step(.., it)
    #   8. 개선 검사: goal_range_r 안 node 중 (cost + goal 까지 직선거리) 가 최소인
    #      것의 경로를 복원. eff = _path_length(경로) + 마지막 node→goal 거리.
    #      eff < c_best - improve_eps (의미 있게 더 짧음) 이면:
    #        c_best, best_path 갱신 + inform_round += 1   ← 타원이 즉시 좁아진다.
    #   dbg 있으면 dbg.add(goal_dist=, rejected=, tree_size=, rewire_count=,
    #                      inform_round=inform_round, best_cost=(0 또는 c_best))
    #
    # max_iter 회 끝나면 best_path 반환 (없으면 []).
    #
    # 디버그 (선택): dbg 가 주어지면 위 dbg.add(...) 한 줄로 매 iteration 내부 값을
    #   남길 수 있다 — record_gen 이 debug_scalars·ellipses 로 저장, viewer 에 표시.
    raise NotImplementedError
