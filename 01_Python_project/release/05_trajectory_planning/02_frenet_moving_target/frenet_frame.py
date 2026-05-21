"""Frenet ↔ Cartesian 좌표 변환 — 폐루프 트랙용 infrastructure.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

폐루프(closed-loop) 중심선을 따라 호 길이 s 와 횡 offset d 로 위치를 기술하는
Frenet 좌표계와, 전역 (x, y) Cartesian 좌표계 사이를 오간다. track_map.TrackMap
이 이 함수들을 감싸 .to_cartesian / .to_frenet 메소드로 노출한다.

규약: d > 0 = 진행 방향 기준 왼쪽. s 는 track_length 로 wrap (폐루프).
"""
from __future__ import annotations

import math

import numpy as np


def get_dist(x1: float, y1: float, x2: float, y2: float) -> float:
    """두 점 사이 유클리드 거리."""
    return math.hypot(x1 - x2, y1 - y2)


def build_maps(mapx, mapy) -> np.ndarray:
    """중심선 각 점까지의 누적 호 길이 배열 (Frenet s 좌표축)."""
    maps = np.zeros(len(mapx))
    for i in range(1, len(mapx)):
        maps[i] = maps[i - 1] + get_dist(mapx[i], mapy[i], mapx[i - 1], mapy[i - 1])
    return maps


def get_closest_waypoints(x: float, y: float, mapx, mapy) -> int:
    """(x, y) 에 가장 가까운 중심선 waypoint 인덱스."""
    best, best_d = 0, float("inf")
    for i in range(len(mapx)):
        d = get_dist(x, y, mapx[i], mapy[i])
        if d < best_d:
            best_d, best = d, i
    return best


def next_waypoint(x: float, y: float, mapx, mapy) -> int:
    """진행 방향 기준 (x, y) 앞쪽의 다음 waypoint 인덱스 (폐루프 wrap)."""
    closest = get_closest_waypoints(x, y, mapx, mapy)
    nxt = (closest + 1) % len(mapx)
    mvx, mvy = mapx[nxt] - mapx[closest], mapy[nxt] - mapy[closest]
    evx, evy = x - mapx[closest], y - mapy[closest]
    return nxt if (mvx * evx + mvy * evy) >= 0 else closest


def cartesian_to_frenet(x: float, y: float, mapx, mapy, maps) -> tuple[float, float]:
    """전역 (x, y) → Frenet (s, d)."""
    nxt = next_waypoint(x, y, mapx, mapy)
    prev = (nxt - 1) % len(mapx)
    nx, ny = mapx[nxt] - mapx[prev], mapy[nxt] - mapy[prev]
    xx, xy = x - mapx[prev], y - mapy[prev]
    proj_norm = (xx * nx + xy * ny) / (nx * nx + ny * ny)
    proj_x, proj_y = proj_norm * nx, proj_norm * ny
    d = get_dist(xx, xy, proj_x, proj_y)
    if (xx * ny - xy * nx) > 0:          # ego_vec × map_vec 의 z 성분 > 0 → 오른쪽
        d = -d
    s = float(maps[prev]) + get_dist(0.0, 0.0, proj_x, proj_y)
    return s, d


def frenet_to_cartesian(s: float, d: float, mapx, mapy, maps,
                        track_length: float) -> tuple[float, float, float]:
    """Frenet (s, d) → 전역 (x, y, heading). s 는 track_length 로 wrap."""
    n = len(mapx)
    s = s % track_length
    prev = int(np.searchsorted(maps, s, side="right")) - 1
    prev = min(max(prev, 0), n - 1)
    nxt = (prev + 1) % n
    dx, dy = mapx[nxt] - mapx[prev], mapy[nxt] - mapy[prev]
    heading = math.atan2(dy, dx)
    seg_s = s - float(maps[prev])
    seg_x = mapx[prev] + seg_s * math.cos(heading)
    seg_y = mapy[prev] + seg_s * math.sin(heading)
    perp = heading + math.pi / 2.0
    return seg_x + d * math.cos(perp), seg_y + d * math.sin(perp), heading
