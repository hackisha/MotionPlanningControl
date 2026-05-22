"""열린 곡선 3 차선 도로 — Frenet 좌표 변환 데모용 환경.

이 파일은 데모 환경의 일부입니다. (정식 문제 패키지로 확장 시 '수정 금지'.)

3 차선 도로를 **호 길이로 매개변수화한 중심선**으로 생성한다. 중심선의 heading 이
완만하게 한 주기 진동(S 자)하도록 두어 '적당한 곡률'을 준다. s 원점(s=0)은
자차의 종방향 위치에 맞춰 이동 → 도로는 s ∈ [-50, +100] m 범위를 갖는다.

차선 레이아웃 (LANE_WIDTH = 4 m, 중심선 = 가운데 차로의 중앙):
  d = +4 / 0 / -4   세 차로의 중앙   (d > 0 = 진행 방향 기준 왼쪽)
  d = ±2, ±6        차로 경계선(차선) — 시각화에서 dashed
  d = ±6            도로 양쪽 경계
"""
from __future__ import annotations

import math

import numpy as np

# --- 도로 형상 파라미터 ---
LANE_WIDTH: float = 4.0          # 한 차선 폭 [m]
N_LANES: int = 3                 # 차로 수 (자차 차로 + 좌/우 각 1)
S_START: float = -50.0           # 중심선 시작 호 길이 [m]
S_END: float = 100.0             # 중심선 끝 호 길이 [m]
_DS: float = 0.5                 # 중심선 점 간격 [m]
_THETA_AMP: float = 0.30         # 중심선 heading 진폭 [rad] → 완만한 S 자 곡률

ROAD_HALF_WIDTH: float = 1.5 * LANE_WIDTH                 # 6 m — 도로 반폭
# 차로 중앙 d 값 (왼쪽 → 오른쪽): +4, 0, -4
LANE_CENTERS: tuple[float, ...] = (+LANE_WIDTH, 0.0, -LANE_WIDTH)
# 차선(차로 경계) d 값: ±2, ±6
LANE_LINES: tuple[float, ...] = (-1.5 * LANE_WIDTH, -0.5 * LANE_WIDTH,
                                 +0.5 * LANE_WIDTH, +1.5 * LANE_WIDTH)


class Road:
    """열린 곡선 3 차선 도로 — 중심선 점열 + 차선 폴리라인 제공.

    Frenet ↔ Cartesian 변환 자체는 `frenet_transform.py` 가 담당하고,
    이 클래스는 변환에 필요한 **중심선 데이터**(center_x, center_y, center_s)와
    시각화용 차선 기하만 제공한다.
    """

    def __init__(self) -> None:
        n = int(round((S_END - S_START) / _DS)) + 1
        s_param = S_START + _DS * np.arange(n)
        u = (s_param - S_START) / (S_END - S_START)
        # 중심선 heading 을 호 길이의 함수로 — 한 주기 sin → 완만한 S 자.
        self.heading = _THETA_AMP * np.sin(2.0 * math.pi * u)

        # heading 을 적분해 중심선 점열 생성 (세그먼트 i 의 heading = heading[i]).
        x = np.concatenate([[0.0], np.cumsum(np.cos(self.heading[:-1]) * _DS)])
        y = np.concatenate([[0.0], np.cumsum(np.sin(self.heading[:-1]) * _DS)])

        # s=0 (자차 종방향 위치) 점을 원점으로 이동 — 자차가 x ≈ 0 에 오게.
        i0 = int(round((0.0 - S_START) / _DS))
        self.center_x = x - x[i0]
        self.center_y = y - y[i0]

        # 실제 점 간 거리 기반 누적 호 길이, s=0 점이 원점이 되도록 이동.
        seg = np.hypot(np.diff(self.center_x), np.diff(self.center_y))
        cs_raw = np.concatenate([[0.0], np.cumsum(seg)])
        self.center_s = cs_raw - cs_raw[i0]

        self.s_min = float(self.center_s[0])
        self.s_max = float(self.center_s[-1])

    def lane_line_xy(self, d: float) -> tuple[np.ndarray, np.ndarray]:
        """횡 offset d 인 평행선의 Cartesian 폴리라인 (d > 0 = 왼쪽)."""
        nx = -np.sin(self.heading)        # 왼쪽 법선 = heading + 90°
        ny = np.cos(self.heading)
        return self.center_x + d * nx, self.center_y + d * ny

    def edges_polygon(self) -> tuple[np.ndarray, np.ndarray]:
        """도로 경계(d = ±6)로 닫은 폴리곤 — 도로면 fill 용."""
        lx, ly = self.lane_line_xy(+ROAD_HALF_WIDTH)
        rx, ry = self.lane_line_xy(-ROAD_HALF_WIDTH)
        px = np.concatenate([lx, rx[::-1], lx[:1]])
        py = np.concatenate([ly, ry[::-1], ly[:1]])
        return px, py
