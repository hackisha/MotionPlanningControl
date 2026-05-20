"""IntegratedControl — 통합 종+횡 제어 시나리오.

도로: R=200 좌커브 2차선 (직선 50m → 곡선 100m → 접선 직선).
ego: lane2 (오른쪽) 출발, vx0=10, 솔루션은 PurePursuit + speed PID + constant timegap.
ego 는 lane2 를 쭉 따라간다 (lane change 안 함).

target: lane1 (왼쪽) 50m 앞, vx=8 등속. t=10s 부터 lane1→lane2 (ego 의 lane) 5초 침범.

ego 의 의사결정:
  - 평시: vx=10 speed mode
  - target 의 lane invasion 감지 (target.Y < road center) → 종 mode 가 constant time-gap 으로
    전환되어 더 느린 속도 (target vx=8) 로 추종.

본 driver 는 본 폴더의 ControlPipeline 호출 — 학생이 LongitudinalDecision,
LongitudinalController, LateralController 의 본문을 짜야 동작.

다른 lateral controller (Stanley/LatPIDFF) 로 교체: lateral_controller.py 의 docstring 참조.
재생: 03_vehicle_control/simulator_vehicle_control.py 참조.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# 05_frame_transform 의 구현을 sys.path 로 직접 import (frame_transform 패턴).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from control_pipeline import (
    ControlPipeline,
    EgoState,
    LongitudinalDecision,
    Road,
    TargetState,
)
from debug_signals import DebugSignals
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_controller import LateralController, PurePursuit
from longitudinal_controller import LongitudinalController
from vehicle_combined import VehicleCombined

DT = 0.1
DEGREE = 3
NUM_POINT = 10
X_LOCAL = np.arange(0.0, 13.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.2  # 0..10.8m


def target_state(t: float, road: Road, vx_t: float = 8.0,
                 X0: float = 50.0, t_invasion: float = 10.0,
                 T_invasion: float = 5.0) -> TargetState:
    """target hard-coded 동작: lane1 등속 → t_invasion 부터 5s 동안 lane2 (ego 의 lane) 침범."""
    X = X0 + vx_t * t
    if t < t_invasion:
        Y = road.lane1_center(X)
    elif t < t_invasion + T_invasion:
        phase = (t - t_invasion) / T_invasion
        blend = 0.5 * (1.0 - np.cos(np.pi * phase))
        Y = (1.0 - blend) * road.lane1_center(X) + blend * road.lane2_center(X)
    else:
        Y = road.lane2_center(X)
    return TargetState(X=float(X), Y=float(Y), vx=vx_t)


def _target_yaw_approx(t: float, road: Road, dt_diff: float = 0.05) -> float:
    """target 의 yaw 근사 — finite difference (Rerun viz 용)."""
    s1 = target_state(t - dt_diff, road)
    s2 = target_state(t + dt_diff, road)
    return float(np.arctan2(s2.Y - s1.Y, s2.X - s1.X))


def _radar_arc_paths(t_arr: np.ndarray, X: np.ndarray, Y: np.ndarray,
                     Yaw: np.ndarray, mode_id: np.ndarray,
                     radii: tuple[float, ...] = (2.0, 3.5, 5.0),
                     half_angle_deg: float = 16.0,   # 차선폭 3.5m × 80% = 2.8m (r=5m chord 기준)
                     n_pts: int = 24,
                     color: tuple[int, int, int, int] = (255, 255, 255, 130),
                     ) -> list[dict]:
    """ego 앞 지면에 와이파이 모양 레이더 파형 — concentric arc N개. timegap mode 만 active.

    각 호는 ego 위치 중심, ego.Yaw ± half_angle_deg fan, z=0.05 (지면 위 살짝).
    speed mode step (mode_id<0.5) 은 빈 list → simulator 가 entity Clear.
    """
    half = np.radians(half_angle_deg)
    steps = len(t_arr)
    paths = []
    for r_idx, r in enumerate(radii):
        points_per_t: list[list[list[float]]] = []
        for i in range(steps):
            if mode_id[i] < 0.5:
                points_per_t.append([])
                continue
            yaw_i = float(Yaw[i])
            angles = np.linspace(yaw_i - half, yaw_i + half, n_pts)
            xs = float(X[i]) + r * np.cos(angles)
            ys = float(Y[i]) + r * np.sin(angles)
            zs = np.full(n_pts, 0.05)
            points_per_t.append(np.column_stack([xs, ys, zs]).tolist())
        paths.append({
            "name": f"radar_arc_{r_idx + 1}",
            "color": list(color),
            "radius": 0.06,
            "t": t_arr.tolist(),
            "points_per_t": points_per_t,
        })
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(
        description="IntegratedControl 시나리오 실행 → record.json 생성")
    parser.add_argument("--no-viewer", action="store_true",
                        help="record JSON 만 생성하고 Rerun viewer 안 띄움 (CI/batch 용)")
    args = parser.parse_args()
    no_viewer = args.no_viewer

    sim_time = 45.0
    road = Road(R=200.0)
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    lat_ctrl = LateralController(pp, lookahead_x_fn=lambda vx: vx * pp.lookahead_time)
    # [튜닝] 게인 값 변경 가능 — test_*.py 의 값은 변경 X (합격 기준)
    long_ctrl = LongitudinalController(
        dt=DT, kp_v=0.5, kd_v=0.0, kp_g=2.0, kd_g=3.0, tau_gap=1.5,
    )
    decision = LongitudinalDecision(road)
    pipe = ControlPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, int(X_LOCAL.size)),
        lat_ctrl=lat_ctrl,
        long_ctrl=long_ctrl,
        decision=decision,
        ref_y_fn=road.lane2_center,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
        v_des=10.0,
    )

    plant = VehicleCombined(
        dt=DT, vx0=10.0,
        X0=0.0, Y0=float(road.lane2_center(0.0)), Yaw0=0.0,
    )

    steps = int(sim_time / DT)
    t = np.arange(steps) * DT
    X_ego = np.zeros(steps); Y_ego = np.zeros(steps); Yaw_ego = np.zeros(steps)
    vx_ego = np.zeros(steps); delta_arr = np.zeros(steps); ax_arr = np.zeros(steps)
    X_tgt = np.zeros(steps); Y_tgt = np.zeros(steps); Yaw_tgt = np.zeros(steps)
    gap_arr = np.zeros(steps); mode_id_arr = np.zeros(steps)  # 0=speed, 1=timegap
    fit_curves: list[list[list[float]]] = []
    dbg = DebugSignals()  # 디버그 신호 수집기 — 신호 추가/삭제는 아래 dbg.add() 한 줄
    lookahead_pts: list[list[float]] = []

    for i in range(steps):
        tt = i * DT
        ego = EgoState(X=plant.X, Y=plant.Y, Yaw=plant.Yaw, vx=plant.vx)
        tgt = target_state(tt, road)
        out = pipe.step(tt, ego, tgt)
        X_ego[i] = plant.X; Y_ego[i] = plant.Y; Yaw_ego[i] = plant.Yaw
        vx_ego[i] = plant.vx; delta_arr[i] = out.delta; ax_arr[i] = out.ax
        X_tgt[i] = tgt.X; Y_tgt[i] = tgt.Y
        Yaw_tgt[i] = _target_yaw_approx(tt, road)
        gap_arr[i] = (np.cos(ego.Yaw) * (tgt.X - ego.X)
                      + np.sin(ego.Yaw) * (tgt.Y - ego.Y))
        mode_id_arr[i] = 1.0 if out.long_mode == "timegap" else 0.0
        # 디버그 신호 — 주석을 풀고 원하는 값/식을 넣으세요.
        # 추가·삭제·수정은 이 dbg.add() 의 kwarg 한 줄로 끝납니다.
        # (예: debug1=ego.Y - road.lane2_center(ego.X))
        dbg.add(
            # debug1=<신호 값 또는 식>,
            # debug2=<신호 값 또는 식>,
            # debug3=<신호 값 또는 식>,
        )
        # viz: fit + lookahead → global frame
        cos_y, sin_y = np.cos(plant.Yaw), np.sin(plant.Yaw)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        fit_global = (rot @ out.fit_local_points.T).T + np.array([plant.X, plant.Y])
        fit_curves.append(fit_global.tolist())
        lh_global = rot @ np.array(out.lookahead_local) + np.array([plant.X, plant.Y])
        lookahead_pts.append(lh_global.tolist())
        plant.step(out.delta, out.ax)

    # 도로 line — ego 진행 범위 cover
    x_lo = float(min(X_ego.min(), X_tgt.min())) - 5.0
    x_hi = float(max(X_ego.max(), X_tgt.max())) + 10.0
    ref_x = np.arange(x_lo, x_hi, 0.5)
    center_y = road.y_center(ref_x)
    lane1_y = center_y + road.lane_offset
    lane2_y = center_y - road.lane_offset
    edge_top = center_y + 2.0 * road.lane_offset
    edge_bot = center_y - 2.0 * road.lane_offset

    record = {
        "schema_version": 2,
        "module": "03_vehicle_control/09_integrated_control",
        "dt": DT,
        "actors": [
            {"name": "ego", "L": 4.0, "W": 2.0, "color": [50, 100, 220, 200],
             "t": t.tolist(), "X": X_ego.tolist(), "Y": Y_ego.tolist(),
             "Yaw": Yaw_ego.tolist()},
            {"name": "target", "L": 4.0, "W": 2.0, "color": [220, 80, 50, 200],
             "t": t.tolist(), "X": X_tgt.tolist(), "Y": Y_tgt.tolist(),
             "Yaw": Yaw_tgt.tolist()},
        ],
        "reference_path": {"X": ref_x.tolist(), "Y": center_y.tolist()},
        "lanes": [
            {"X": ref_x.tolist(), "Y": edge_top.tolist(), "kind": "edge"},
            {"X": ref_x.tolist(), "Y": edge_bot.tolist(), "kind": "edge"},
            {"X": ref_x.tolist(), "Y": center_y.tolist(), "kind": "center"},
            {"X": ref_x.tolist(), "Y": lane1_y.tolist(), "kind": "lane"},
            {"X": ref_x.tolist(), "Y": lane2_y.tolist(), "kind": "lane"},
        ],
        "scalars": [
            {"name": "vx_ego", "unit": "m/s", "t": t.tolist(), "value": vx_ego.tolist()},
            {"name": "ax_ego", "unit": "m/s²", "t": t.tolist(), "value": ax_arr.tolist()},
            {"name": "delta", "unit": "rad", "t": t.tolist(), "value": delta_arr.tolist()},
            {"name": "gap_long", "unit": "m", "t": t.tolist(), "value": gap_arr.tolist()},
            {"name": "long_mode", "unit": "-",
             "t": t.tolist(), "value": mode_id_arr.tolist()},
        ],
        # 디버그 신호 — 기본 blueprint 미포함. viewer 의 entity 패널에서 /debug/<name>
        # 을 골라 TimeSeriesView 를 직접 추가하면 심화 분석 가능.
        "debug_scalars": dbg.to_debug_scalars(t),
        "dynamic_paths": [
            {"name": "fit", "color": [255, 150, 0, 200], "radius": 0.08,
             "t": t.tolist(), "points_per_t": fit_curves},
            # 종 mode 의 3D 시각화 — timegap mode 일 때 ego 앞 지면에 와이파이 모양
            # 레이더 파형 (concentric arc 3개, 반경 2/3.5/5m, fan ±16° = 차선폭 80%). 흰색 투명.
            # speed mode 시 빈 list → simulator 가 entity Clear → 안 보임.
            *_radar_arc_paths(t, X_ego, Y_ego, Yaw_ego, mode_id_arr),
        ],
        "dynamic_points": [
            {"name": "lookahead", "color": [255, 220, 0, 230], "radius": 0.35,
             "t": t.tolist(), "points_per_t": lookahead_pts},
        ],
    }
    out_path = Path(__file__).parent / "record.json"
    out_path.write_text(json.dumps(record), encoding="utf-8")
    print(f"[record] saved → {out_path}  |  invaded={decision.invaded}  |  "
          f"final ego.vx={float(vx_ego[-1]):.2f}  |  재생: simulator_vehicle_control.py")

    if not no_viewer:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from simulator_vehicle_control import replay_records
        replay_records([out_path], camera="follow")


if __name__ == "__main__":
    main()
