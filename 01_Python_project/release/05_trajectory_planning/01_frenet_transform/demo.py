"""Frenet 좌표 변환 데모 — 곡선 3 차선 도로 + 자차/타겟을 펴서 보이기.

본 모듈은 시간축이 없는 **정적 데모** — Rerun 시간축 재생 가치가 없어
plotly 2-패널로 좌표 변환의 전/후를 직접 보여준다 (§release_pipeline 의
정적-데모 예외, `03_vehicle_control/05_frame_transform/demo.py` 와 같은 패턴).

  ① Cartesian 패널 — 곡선 3 차선 도로 위의 자차(파랑)·타겟 6 대(회색).
  ② Frenet  패널 — 같은 장면을 도로 중심선 기준으로 펴서 직선화한 (s, d) 좌표.

자차/타겟은 Frenet (s, d) 로 시나리오를 구성한 뒤 `frenet_to_cartesian` 으로
전역 포즈를 얻고(① 패널), 그 전역 (x, y) 를 다시 `cartesian_to_frenet` 으로
변환한 결과를 ② 패널에 그린다 — 즉 변환 왕복(round-trip)을 눈으로 확인한다.

실행 전 `frenet_transform.py` 의 `# TODO` 를 구현해야 동작합니다 — 구현 전이면
NotImplementedError 가 납니다.
"""
from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go
from frenet_transform import cartesian_to_frenet, frenet_to_cartesian
from plotly.subplots import make_subplots
from road import LANE_CENTERS, LANE_LINES, ROAD_HALF_WIDTH, Road

# --- 차량 치수 [m] (시인성을 위해 약간 과장 — 실제 승용차 ≈ 4.5 × 1.9) ---
CAR_L: float = 5.0
CAR_W: float = 2.2

# --- 자차 Frenet 포즈 — 종방향 원점(s=0), 차선 중앙이 아닌 횡위치(d≠0) ---
EGO_S: float = 0.0
EGO_D: float = 1.2

EGO_COLOR = "#1f6feb"
EGO_FILL = "rgba(31,111,235,0.55)"
TGT_COLOR = "#5b6675"
TGT_FILL = "rgba(150,156,166,0.60)"
LANE_COLOR = "#37404e"
ROAD_FILL = "rgba(223,228,235,0.95)"


def place_targets(road: Road, seed: int = 7) -> list[tuple[float, float]]:
    """타겟 6 대를 차로별 2 대씩 임의 배치 — 서로/자차와 너무 가깝지 않게.

    제약 (rejection sampling):
      - 같은 차로 두 타겟의 종방향 간격 ≥ MIN_SAME_LANE
      - 임의의 두 타겟 사이 거리 ≥ MIN_PAIR (대각 적체 방지)
      - 자차와의 거리 ≥ MIN_EGO
    """
    MIN_SAME_LANE = 24.0
    MIN_PAIR = 12.0
    MIN_EGO = 16.0
    rng = np.random.default_rng(seed)
    s_lo, s_hi = road.s_min + 12.0, road.s_max - 12.0
    placed: list[tuple[float, float, float]] = []   # (s, d, lane_center)
    for lane_d in LANE_CENTERS:
        count, guard = 0, 0
        while count < 2:
            guard += 1
            if guard > 10000:                       # pragma: no cover
                raise RuntimeError("타겟 배치 실패 — 제약을 완화하세요")
            s = float(rng.uniform(s_lo, s_hi))
            d = lane_d + float(rng.uniform(-0.4, 0.4))
            if math.hypot(s - EGO_S, d - EGO_D) < MIN_EGO:
                continue
            if any(lc == lane_d and abs(s - ps) < MIN_SAME_LANE
                   for ps, _, lc in placed):
                continue
            if any(math.hypot(s - ps, d - pd) < MIN_PAIR
                   for ps, pd, _ in placed):
                continue
            placed.append((s, d, lane_d))
            count += 1
    return [(s, d) for s, d, _ in placed]


def _car_polygon(cx: float, cy: float, yaw: float
                 ) -> tuple[np.ndarray, np.ndarray]:
    """차량 실루엣 5각형(앞쪽이 뾰족) — yaw 회전 후 (cx, cy) 로 평행이동."""
    hl, hw = CAR_L / 2.0, CAR_W / 2.0
    local = np.array([
        (-hl, +hw), (-hl, -hw), (+0.55 * hl, -hw),
        (+hl, 0.0), (+0.55 * hl, +hw), (-hl, +hw),
    ])
    c, s = math.cos(yaw), math.sin(yaw)
    world = local @ np.array([[c, -s], [s, c]]).T
    return world[:, 0] + cx, world[:, 1] + cy


def _add_car(fig, cx, cy, yaw, *, row, line, fill, name, show):
    px, py = _car_polygon(cx, cy, yaw)
    fig.add_trace(go.Scatter(x=px, y=py, mode="lines", fill="toself",
                             line=dict(color=line, width=1.6), fillcolor=fill,
                             name=name, legendgroup=name, showlegend=show,
                             hoverinfo="name"),
                  row=row, col=1)


def main() -> None:
    road = Road()
    targets_sd = place_targets(road)

    # ① 시나리오는 Frenet 으로 authoring → frenet_to_cartesian 으로 전역 포즈.
    ego_x, ego_y, ego_yaw = frenet_to_cartesian(
        EGO_S, EGO_D, road.center_x, road.center_y, road.center_s)
    tgt_cart = [frenet_to_cartesian(s, d, road.center_x, road.center_y,
                                    road.center_s)
                for s, d in targets_sd]

    # ② 전역 (x, y) 를 다시 Frenet 으로 변환 (이 모듈의 핵심) — 왕복 확인.
    ego_s, ego_d = cartesian_to_frenet(
        ego_x, ego_y, road.center_x, road.center_y, road.center_s)
    tgt_sd = [cartesian_to_frenet(x, y, road.center_x, road.center_y,
                                  road.center_s)
              for x, y, _ in tgt_cart]

    rt_err = max(math.hypot(s - rs, d - rd)
                 for (s, d), (rs, rd) in zip(targets_sd, tgt_sd, strict=True))
    print(f"[round-trip] 자차 (s,d) = ({ego_s:+.2f}, {ego_d:+.2f})  "
          f"| 타겟 변환 최대 오차 = {rt_err:.2e} m")

    fig = make_subplots(
        rows=2, cols=1, vertical_spacing=0.12,
        subplot_titles=("① Cartesian 좌표 — 곡선 3 차선 도로",
                        "② Frenet 좌표 — 도로를 펴서 직선화한 (s, d)"))

    # ===== ① Cartesian 패널 =====
    ex, ey = road.edges_polygon()
    fig.add_trace(go.Scatter(x=ex, y=ey, mode="lines", fill="toself",
                             line=dict(width=0), fillcolor=ROAD_FILL,
                             hoverinfo="skip", showlegend=False), row=1, col=1)
    for k, d in enumerate(LANE_LINES):
        lx, ly = road.lane_line_xy(d)
        fig.add_trace(go.Scatter(x=lx, y=ly, mode="lines",
                                 line=dict(color=LANE_COLOR, width=2.5,
                                           dash="dash"),
                                 name="차선 (lane line)", legendgroup="lane",
                                 showlegend=(k == 0), hoverinfo="skip"),
                      row=1, col=1)
    _add_car(fig, ego_x, ego_y, ego_yaw, row=1, line=EGO_COLOR, fill=EGO_FILL,
             name="자차 (ego)", show=True)
    for k, (x, y, yaw) in enumerate(tgt_cart):
        _add_car(fig, x, y, yaw, row=1, line=TGT_COLOR, fill=TGT_FILL,
                 name="타겟 (target)", show=(k == 0))

    # ===== ② Frenet 패널 =====
    fig.add_trace(go.Scatter(
        x=[road.s_min, road.s_max, road.s_max, road.s_min, road.s_min],
        y=[+ROAD_HALF_WIDTH, +ROAD_HALF_WIDTH, -ROAD_HALF_WIDTH,
           -ROAD_HALF_WIDTH, +ROAD_HALF_WIDTH],
        mode="lines", fill="toself", line=dict(width=0), fillcolor=ROAD_FILL,
        hoverinfo="skip", showlegend=False), row=2, col=1)
    for d in LANE_LINES:
        fig.add_trace(go.Scatter(x=[road.s_min, road.s_max], y=[d, d],
                                 mode="lines",
                                 line=dict(color=LANE_COLOR, width=2.5,
                                           dash="dash"),
                                 legendgroup="lane", showlegend=False,
                                 hoverinfo="skip"), row=2, col=1)
    # Frenet 에서 도로와 정렬된 차량 → 방향은 +s 축 (yaw = 0).
    _add_car(fig, ego_s, ego_d, 0.0, row=2, line=EGO_COLOR, fill=EGO_FILL,
             name="자차 (ego)", show=False)
    for s, d in tgt_sd:
        _add_car(fig, s, d, 0.0, row=2, line=TGT_COLOR, fill=TGT_FILL,
                 name="타겟 (target)", show=False)

    fig.update_xaxes(title_text="X (m)", row=1, col=1)
    fig.update_yaxes(title_text="Y (m)", scaleanchor="x", scaleratio=1,
                     row=1, col=1)
    fig.update_xaxes(title_text="s (m) — 중심선 호 길이", row=2, col=1)
    fig.update_yaxes(title_text="d (m) — 횡 offset", scaleanchor="x2",
                     scaleratio=1, row=2, col=1)
    fig.update_layout(
        template="plotly_white", height=820, width=1180,
        title=dict(text="Frenet 좌표 변환 — 곡선 도로를 펴서 (s, d) 로",
                   x=0.5, xanchor="center"),
        legend=dict(yanchor="middle", y=0.5, xanchor="left", x=1.01,
                    bgcolor="rgba(255,255,255,0.85)",
                    bordercolor="#d9dee6", borderwidth=1))
    fig.show()


if __name__ == "__main__":
    main()
