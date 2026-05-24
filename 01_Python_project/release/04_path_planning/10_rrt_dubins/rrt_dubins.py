"""Informed RRT* with Dubins steering — (x, y, yaw) state + 최대 곡률 제약.

과제 명세는 problem.html 참조.

09 Informed RRT* 의 단일 tree·단일 loop·event-driven round 갱신 구조를 그대로
가져오되, 핵심 변경:
- state 가 (x, y) → **(x, y, yaw)** 로 늘어남.
- 두 node 사이의 edge 가 **직선이 아니라 Dubins 곡선** (최대 곡률 |κ| ≤ kappa).
- 충돌 검사는 Dubins 호의 fine sample 들이 obstacle cell 에 닿는지로 수행.
"""
from __future__ import annotations

import math
import random
from collections.abc import Callable

from dubins import dubins_plan, truncate_path

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

State = tuple[float, float, float]   # (x, y, yaw)
ConnectFn = Callable[[State, State], "tuple[list[State], float]"]
# on_step(child_state, parent_state, edge_samples, iteration) — edge_samples 는
# planner 가 collision-check 한 실제 Dubins fine sample (parent → child, parent
# 미포함). 시각화는 항상 이 stored samples 를 써야 tree 의 실제 곡선과 일치.
StepFn = Callable[[State, State, "list[State]", int], None]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _euclid_xy(a: State, b: State) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_collision_free(samples: list[State],
                         obstacles: set[tuple[int, int]]) -> bool:
    """Dubins sample 들의 (round(x), round(y)) cell 이 모두 obstacle 가 아님."""
    for x, y, _ in samples:
        if (int(round(x)), int(round(y))) in obstacles:
            return False
    return True


def _sample_state(rng: random.Random, grid_size: int) -> State:
    """uniform random pose (yaw 는 [−π, π))."""
    return (rng.uniform(0.0, float(grid_size)),
            rng.uniform(0.0, float(grid_size)),
            rng.uniform(-math.pi, math.pi))


def _make_dubins_connect(kappa: float, ds: float) -> ConnectFn:
    """`(s1, s2) -> (samples, length)` Dubins 연결 함수.

    samples 는 s1 다음 점부터 s2 까지 호장 ds 간격 (x, y, yaw) 시퀀스.
    경로가 정의 불가하면 `([], inf)`.
    """
    def _connect(s1: State, s2: State) -> tuple[list[State], float]:
        return dubins_plan(s1, s2, kappa, ds=ds)
    return _connect


def _steer_to(
    s_from: State,
    s_to: State,
    connect_fn: ConnectFn,
    eta: float,
    ds: float,
) -> tuple[list[State], float]:
    """s_from → s_to 를 connect 한 뒤 호장 `eta` 까지 잘라 반환.

    경로가 정의 불가면 ([], 0.0). 잘린 길이는 ds 의 배수에 맞춰 eta 이하.
    """
    full_path, full_len = connect_fn(s_from, s_to)
    if not full_path or full_len == math.inf:
        return [], 0.0
    if full_len <= eta:
        return full_path, full_len
    return truncate_path(full_path, ds, eta)


def _reconstruct_path(
    nodes: list[State],
    parents: dict[int, int | None],
    edge_samples: dict[int, list[State]],
    end_idx: int,
) -> list[State]:
    """parent chain 을 따라 stored edge samples 를 이어 start → end 경로를 만든다.

    edge_samples[i] 는 i 의 부모에서 i 로 가는 collision-check 통과 시점의 실제
    Dubins fine sample (parent 미포함). `dubins_plan` 재호출 대신 이걸 써야
    fp precision 으로 인한 Bellman 어긋남(다른 word 선택)이 없음. path[0] == start.
    """
    chain: list[int] = []
    i: int | None = end_idx
    while i is not None:
        chain.append(i)
        i = parents[i]
    chain.reverse()
    path: list[State] = [nodes[chain[0]]]
    for k in chain[1:]:
        path.extend(edge_samples[k])
    return path


def _sample_in_ellipse(rng: random.Random,
                       x_start: State, x_goal: State,
                       c_best: float) -> tuple[float, float]:
    """start·goal (x,y) 를 초점, c_best 를 장축으로 하는 Euclidean 타원 안 uniform.

    Dubins 비용 ≤ c_best 인 경로의 모든 node 는 이 타원 안에 든다
    (Dubins 호장 ≥ Euclidean 거리 부등식의 따름정리). 즉 valid informed
    superset — 정통 informed RRT* (Gammell 2014) 의 표준 trick.
    """
    c_min = math.hypot(x_goal[0] - x_start[0], x_goal[1] - x_start[1])
    cx = 0.5 * (x_start[0] + x_goal[0])
    cy = 0.5 * (x_start[1] + x_goal[1])
    theta = math.atan2(x_goal[1] - x_start[1], x_goal[0] - x_start[0])
    a = 0.5 * c_best
    b = 0.5 * math.sqrt(max(c_best * c_best - c_min * c_min, 0.0))
    rad = math.sqrt(rng.random())
    phi = 2.0 * math.pi * rng.random()
    ex = a * rad * math.cos(phi)
    ey = b * rad * math.sin(phi)
    return (cx + ex * math.cos(theta) - ey * math.sin(theta),
            cy + ex * math.sin(theta) + ey * math.cos(theta))


# ---------------------------------------------------------------------------
# Informed RRT* + Dubins
# ---------------------------------------------------------------------------

def informed_rrt_star(
    start: State,
    goal: State,
    obstacles: set[tuple[int, int]],
    grid_size: int,
    kappa: float = 1.0 / 3.0,
    max_iter: int = 1500,
    eta: float = 6.0,
    goal_range: float = 4.0,
    goal_range_min: float = 1.5,
    eta_decay: float = 0.8,
    eta_min: float = 2.5,
    improve_eps: float = 0.3,
    search_radius: float = 10.0,
    goal_sample_rate: float = 0.05,
    ds: float = 0.2,
    seed: int | None = 0,
    on_step: StepFn | None = None,
    on_improve: Callable[[int, float, list[State]], None] | None = None,
    dbg=None,
) -> list[State]:
    """Informed RRT* + Dubins steering.

    단일 tree·단일 loop·event-driven round 갱신. edge cost·node connection 이
    Dubins. informed 타원은 Euclidean (xy 만).

    Args:
        start, goal: (x, y, yaw) pose.
        obstacles: 정수 격자 cell set.
        grid_size: 샘플링 영역 `[0, grid_size]²`, yaw ∈ [−π, π).
        kappa: 최대 곡률 (1/R). 기본 1/3 (R=3 m).
        max_iter: 전체 sampling 반복 수 (anytime budget).
        eta: 한 steer 가 자라는 Dubins 호장 상한 [m] (round 마다 eta_decay 로 줄임).
        goal_range: new node 의 (x, y) 가 goal 에서 이 거리 안이면 도달 후보.
        goal_range_min: 수렴 시 goal_range 의 하한.
        eta_decay: round 마다 eta 에 곱하는 감쇠율 (0.8 = 20% 줄임).
        eta_min: eta 의 하한 — 너무 작아지면 진행 안 됨.
        improve_eps: 새 best 경로가 이만큼 짧아져야 round 진행 (잡음 필터).
        search_radius: choose-parent / rewire 의 후보 Euclidean 반경.
        goal_sample_rate: 매 iter goal pose 를 직접 sample 할 확률.
        ds: Dubins 호 sampling 의 호장 step [m] (충돌 검사·path 점 간격).
        seed: random seed.
        on_step: 매 edge 추가/rewire 직후 호출 — `(child, parent, samples, iteration)`.
            samples 는 parent → child 의 collision-check 통과 Dubins fine sample.
        on_improve: c_best 갱신마다 호출 — `(iteration, c_best, snap_path)`.
            snap_path 는 그 round 의 채택 경로(start → best_idx) 의 Dubins fine
            sample 연속 (start 포함). round 별 best path 시각화에 사용.
        dbg: optional 디버그 신호 수집기 (`DebugSignals`). 주어지면 매 iteration
            `dbg.add(...)` 로 내부 값을 남긴다 — best_cost · inform_round 등.

    Returns:
        path: start 포함, Dubins 호 sample 의 연속 (x, y, yaw) 리스트.
        미발견 시 `[]`.
    """
    # TODO: Informed RRT* + Dubins steering 으로 start → goal 경로를 구하시오.
    # 09 Informed RRT* 의 단일 tree·단일 loop·event-driven round 갱신 구조를
    # 그대로 가져오되, edge 가 직선이 아니라 **Dubins 곡선** 입니다.
    #
    # 준비 (loop '밖'에서 한 번만):
    #   - connect = _make_dubins_connect(kappa, ds)  # (s1, s2) → (samples, length)
    #   - rng = random.Random(seed)
    #   - 단일 tree: nodes=[start], parents={0:None}, children={0:[]}, cost={0:0.0}
    #     edge_cost={0:0.0}, edge_samples={0:[]}   ← edge_samples 는 stored Dubins 호
    #   - best_idx=None, c_best=inf, inform_round=0
    #
    # for it in range(max_iter):
    #   eta_r       = max(eta * (eta_decay ** inform_round), eta_min)
    #   use_ellipse = (c_best != inf)
    #
    #   1. Sample pose:
    #        rng.random() < goal_sample_rate → sample = goal
    #        elif use_ellipse → _sample_in_ellipse(rng, start, goal, c_best)
    #          → 거기에 yaw = rng.uniform(-pi, pi) 붙여 (x, y, yaw)
    #        else            → _sample_state(rng, grid_size)
    #   2. Nearest = _euclid_xy 가 최소인 node (Dubins-distance 정확도 포기,
    #      fast heuristic). 실제 Dubins 연결이 실패하면 reject.
    #   3. Steer(eta_r) = _steer_to(nearest, sample, connect, eta_r, ds)
    #      → (steer_samples, steer_len). 빈 경로 / 길이 <1e-9 면 reject.
    #   4. Collision: _path_collision_free(steer_samples, obstacles).
    #
    #   5. Near = _euclid_xy ≤ search_radius 인 node index.
    #   6. Choose-parent — near 각각에 대해 다시 `connect(nodes[i], new_pt)` 호출
    #      (Dubins 가 비대칭이라 nearest→new 와 i→new 가 다름). 충돌 없는 후보
    #      중 cost[i] + Dubins 호장 이 최소인 부모 채택. **그 후보의 samples 를
    #      edge_samples[new_i] 로 저장** — Bellman 정확성의 단일 원천.
    #   7. node 추가 → on_step(new_pt, parent_pt, best_samples, it).
    #   8. Rewire — new → near node 가 더 싸면 부모 교체. 새 부모-자식 Dubins
    #      samples 를 그 자식의 edge_samples 에 저장. subtree cost 전파.
    #      각 rewire 마다 on_step(child, new_pt, rew_samples, it).
    #   9. 개선 검사: goal_range 안 node 중 `cost + _euclid_xy(node, goal)` 최소
    #      → eff. eff < c_best - improve_eps 이면 c_best/best_idx 갱신,
    #      inform_round += 1, on_improve(it, c_best, _reconstruct_path(...)).
    #
    # 반환: best_idx 가 None 이면 [] / 아니면 _reconstruct_path 의 결과
    # (start 포함, Dubins 호 fine sample 의 연속).
    #
    # 디버그 (선택): dbg 가 주어지면 매 iteration dbg.add(goal_dist=, rejected=,
    #   tree_size=, rewire_count=, inform_round=, best_cost=) 한 줄 — record_gen
    #   이 debug_scalars 로 저장, viewer 에 표시 (best_cost 가 round 마다 계단식).
    raise NotImplementedError
