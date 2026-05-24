"""Dubins curve — forward-only car (min turning radius) 의 두 pose 간 최단 경로.

Intent: intents/modules/04_path_planning/10_rrt_dubins.md

두 (x, y, yaw) 사이를 잇는 6 개 word — LSL · RSR · LSR · RSL · RLR · LRL — 중
가장 짧은 것을 골라 일정 호장 간격(`ds`)으로 sampling 한 (x, y, yaw) 시퀀스로
반환한다. 모든 운동은 전진만, 곡률 |κ| ≤ kappa (= 1/R).

참고: Dubins, "On Curves of Minimal Length with a Constraint on Average
Curvature, and with Prescribed Initial and Terminal Positions and Tangents"
(1957).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

_EPS_ZERO = -1e-9   # sqrt 직전 음수 허용 epsilon (수치오차 흡수).
_EPS_KAPPA = 1e-9   # 직선 segment 판정 임계 (|κ| 이하면 직선 적분).


def _twopi(a: float) -> float:
    """[0, 2π) 로 wrap."""
    return a - 2.0 * math.pi * math.floor(a / (2.0 * math.pi))


def _pi(a: float) -> float:
    """(−π, π] 로 wrap."""
    v = math.fmod(a + math.pi, 2.0 * math.pi)
    if v < 0:
        v += 2.0 * math.pi
    return v - math.pi


@dataclass(frozen=True)
class DubinsWord:
    """한 word 의 normalized 길이 (kappa=1 기준) + segment 종류."""
    t: float
    p: float
    q: float
    types: tuple[str, str, str]   # 각 segment 의 'L' / 'S' / 'R'

    def total(self) -> float:
        return self.t + self.p + self.q


def _LSL(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb))
    if tmp < _EPS_ZERO:
        return None
    theta = math.atan2(cb - ca, d + sa - sb)
    return DubinsWord(_twopi(-alpha + theta), math.sqrt(max(tmp, 0.0)),
                      _twopi(beta - theta), ("L", "S", "L"))


def _RSR(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa))
    if tmp < _EPS_ZERO:
        return None
    theta = math.atan2(ca - cb, d - sa + sb)
    return DubinsWord(_twopi(alpha - theta), math.sqrt(max(tmp, 0.0)),
                      _twopi(-beta + theta), ("R", "S", "R"))


def _RSL(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
    if tmp < _EPS_ZERO:
        return None
    p = math.sqrt(max(tmp, 0.0))
    theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return DubinsWord(_twopi(alpha - theta), p,
                      _twopi(beta - theta), ("R", "S", "L"))


def _LSR(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
    if tmp < _EPS_ZERO:
        return None
    p = math.sqrt(max(tmp, 0.0))
    theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return DubinsWord(_twopi(-alpha + theta), p,
                      _twopi(-beta + theta), ("L", "S", "R"))


def _RLR(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb + d * (sa - sb)))
    if abs(tmp) >= 1.0:
        return None
    p = 2.0 * math.pi - math.acos(tmp)
    theta = math.atan2(ca - cb, d - sa + sb)
    t = _twopi(alpha - theta + 0.5 * p)
    q = _twopi(alpha - beta - t + p)
    return DubinsWord(t, p, q, ("R", "L", "R"))


def _LRL(d: float, alpha: float, beta: float) -> DubinsWord | None:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb)))
    if abs(tmp) >= 1.0:
        return None
    p = 2.0 * math.pi - math.acos(tmp)
    theta = math.atan2(-ca + cb, d + sa - sb)
    t = _twopi(-alpha + theta + 0.5 * p)
    q = _twopi(beta - alpha - t + p)
    return DubinsWord(t, p, q, ("L", "R", "L"))


_WORDS = (_LSL, _RSR, _RSL, _LSR, _RLR, _LRL)


def _best_word(d: float, alpha: float, beta: float) -> DubinsWord | None:
    """6 word 중 총 length 최소를 반환. 모두 None 이면 None."""
    best: DubinsWord | None = None
    best_len = math.inf
    for fn in _WORDS:
        w = fn(d, alpha, beta)
        if w is None:
            continue
        L = w.total()
        if L < best_len:
            best, best_len = w, L
    return best


def _integrate(state: tuple[float, float, float], kappa_seg: float,
               length: float, ds: float) -> tuple[
                   list[tuple[float, float, float]], tuple[float, float, float]]:
    """state 에서 곡률 `kappa_seg` 의 호를 호장 `length` 만큼 적분.

    호장 step `ds` 마다 (x, y, yaw) 하나씩 sample 해서 반환 (끝점 포함).
    `kappa_seg` 가 (+) 면 좌회전(반시계), (−) 면 우회전(시계), 0 이면 직선.
    """
    x, y, yaw = state
    pts: list[tuple[float, float, float]] = []
    if length <= 0.0:
        return pts, (x, y, yaw)
    n = max(1, int(math.ceil(length / ds)))
    step = length / n
    for _ in range(n):
        if abs(kappa_seg) > _EPS_KAPPA:
            r = 1.0 / kappa_seg
            dyaw = step * kappa_seg
            x += r * (-math.sin(yaw) + math.sin(yaw + dyaw))
            y += r * (math.cos(yaw) - math.cos(yaw + dyaw))
            yaw = _pi(yaw + dyaw)
        else:
            x += step * math.cos(yaw)
            y += step * math.sin(yaw)
        pts.append((x, y, yaw))
    return pts, (x, y, yaw)


def dubins_plan(s1: tuple[float, float, float],
                s2: tuple[float, float, float],
                kappa: float,
                ds: float = 0.1) -> tuple[list[tuple[float, float, float]], float]:
    """s1 → s2 를 잇는 Dubins 최단 곡선.

    Args:
        s1, s2: (x, y, yaw) 시작·도착 pose.
        kappa: 최대 곡률 (1/R). > 0.
        ds: 출력 sample 의 호장 간격 [m] (작을수록 촘촘).

    Returns:
        (path, length).
        - `path`: s1 다음 점부터 끝점까지 (x, y, yaw) 시퀀스. s1 자체는 미포함.
          dubins 가 정의 불가(6 word 모두 None) 시 `[]`.
        - `length`: 실제 호장 [m]. 불가 시 `math.inf`.
    """
    dx = s2[0] - s1[0]
    dy = s2[1] - s1[1]
    th = math.atan2(dy, dx)
    d = math.hypot(dx, dy) * kappa       # normalized (kappa=1) 좌표
    alpha = _twopi(s1[2] - th)
    beta = _twopi(s2[2] - th)
    word = _best_word(d, alpha, beta)
    if word is None:
        return [], math.inf

    kappa_inv = 1.0 / kappa
    state = (float(s1[0]), float(s1[1]), float(s1[2]))
    path: list[tuple[float, float, float]] = []
    for kind, norm_len in zip(word.types, (word.t, word.p, word.q), strict=True):
        seg_len = kappa_inv * norm_len           # normalized → real arc length
        if seg_len <= 0.0:
            continue
        if kind == "L":
            k = kappa
        elif kind == "R":
            k = -kappa
        else:  # 'S'
            k = 0.0
        pts, state = _integrate(state, k, seg_len, ds)
        path.extend(pts)
    return path, word.total() * kappa_inv


def truncate_path(path: list[tuple[float, float, float]], ds: float,
                  max_arc: float) -> tuple[list[tuple[float, float, float]],
                                            float]:
    """`path` 를 호장 `max_arc` 까지만 잘라 반환.

    `path` 는 호장 ds 간격으로 sampling 됐다고 가정 (dubins_plan 의 출력).
    실제 잘린 길이는 ds 의 배수에 맞춰 max_arc 이하로 내려간다.
    `max_arc` 가 전체 호장 이상이면 path 그대로 반환.

    Returns:
        (truncated_path, truncated_arc_length). path 가 비면 ([], 0.0).
    """
    if not path:
        return [], 0.0
    n_keep = min(len(path), max(1, int(max_arc / ds)))
    return path[:n_keep], n_keep * ds
