"""Rerun replay player for vehicle-control demo records (JSON sidecars).

각 record_gen.py 가 생성한 `record*.json` 을 읽어 Rerun viewer 에 재생.
03_vehicle_control 부터는 실제 차량 주행이라, z=0 평면에 채워진 차체 + 바퀴를
시간축(`sim_time`, 초)을 따라 그린다. viewer 1x 재생 = 실제 dt.

**멀티-recording 동작**: 인자 없이 / 디렉토리 인자로 실행하면 그 폴더 하위의
모든 `record*.json` 을 **각각 별도 Rerun recording** 으로 같은 viewer 에 로드.
viewer 좌측 패널의 Recordings 목록에서 클릭하면 시나리오 전환 — 터미널 번호
선택 없이 viewer 안에서만 비교 가능.

Usage (git root cwd 기준):
    # 1) 인자 없이 - 스크립트 폴더 하위 모든 record*.json 멀티 로드 (기본)
    uv run python \\
        01_Python_project/solutions/03_vehicle_control/simulator_vehicle_control.py

    # 2) 파일 지정 - 그 파일 하나만 로드
    uv run python \\
        01_Python_project/solutions/03_vehicle_control/simulator_vehicle_control.py \\
        01_Python_project/solutions/03_vehicle_control/06_lat_pid_ff/record_high_speed.json

    # 3) 디렉토리 지정 - 그 폴더 하위 모든 record 멀티 로드
    uv run python \\
        01_Python_project/solutions/03_vehicle_control/simulator_vehicle_control.py \\
        01_Python_project/solutions/03_vehicle_control/06_lat_pid_ff

    # 카메라 모드 (기본 follow=ego 추종) - ego 시작 위치 고정으로 보려면
    uv run python \\
        01_Python_project/solutions/03_vehicle_control/simulator_vehicle_control.py \\
        --camera fixed

JSON schema (v2; v1 도 계속 재생 가능 - lanes 없으면 생략):
    {
      "schema_version": 2,
      "module": "<area/problem>",
      "scenario": "<optional name>",
      "dt": float,
      "actors": [
        {
          "name": "ego" | "target",
          "L": float,  "W": float,             # vehicle box dims (optional)
          "color": [r, g, b] or [r, g, b, a],  # optional
          "trail": bool,                       # optional, default true (false=궤적선 끔)
          "t":   [t0, t1, ...],
          "X":   [...], "Y": [...], "Yaw": [...],   # Yaw in rad
        },
        ...
      ],
      "reference_path": {"X": [...], "Y": [...]},        # optional
      "lanes": [                                         # optional (v2+)
        {"X": [...], "Y": [...], "kind": "edge" | "lane" | "center",
         "color": [r, g, b(, a)]},                       # optional, kind 기본색 덮어씀
        ...
      ],
      "scalars": [
        {"name": "delta", "unit": "rad", "t": [...], "value": [...]},
        ...
      ],
      "debug_scalars": [                                  # optional — scalars 와 동일 구조
        {"name": "lat_error", "unit": "m", "t": [...], "value": [...]},
        ...
      ]
    }

`debug_scalars` 는 `/debug/<name>` 엔티티로 recording 에 저장되지만 기본 blueprint 에는
포함되지 않는다 — 학생이 viewer 의 entity 패널에서 `/debug/...` 를 골라 TimeSeriesView 를
직접 추가하면 볼 수 있다.

렌더링 규약 (Rerun 0.32):
    - 차량은 z=0 평면 위 3D 로 그린다. Boxes2D 는 회전 미지원이라 채워진 +
      방향 있는 차체가 2D 로 불가능 -> Boxes3D (fill=Solid, yaw->quaternion)
      로 차체 + 바퀴 4개.
    - 카메라: Spatial3DView blueprint + 바닥 그리드 + 비스듬한 3/4 top-down
      orbital eye (차량 진행방향이 화면 1시쯤). --camera follow(ego 추종, 기본)
      / fixed(ego 시작 위치 고정) 토글이고, 뷰어 안에서도 eye controls 로 전환 가능.
    - trail: 각 actor 가 지나온 경로를 매 프레임 누적 LineStrips3D 로.
    - 시간축 sim_time(초) -> viewer 1x = 실제 dt. demo 는 타임스탬프만 로그하고
      sleep 하지 않는다 (재생 속도는 viewer 가 제어).
    - **멀티-recording**: 첫 record 가 viewer 를 spawn, 나머지는 gRPC connect.
      각 record 는 별도 `recording_id` (예: "03_time_gap/maneuvering") — viewer
      좌측 Recordings 패널에서 클릭 전환.

Intent: intents/release_pipeline.md "Rerun replay" 섹션
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

DEFAULT_L = 4.0
DEFAULT_W = 2.0
DEFAULT_H = 1.5
DEFAULT_COLOR = (50, 100, 220, 120)  # ego 투명 파랑 (RGBA, a≈47%)
WHEEL_COLOR = (30, 30, 30)

APP_ID = "vehicle_control_replay"

# 카메라 초기 시점
CAMERA_TILT_DEG = 40.0              # vertical 기준 기울기 (0=top-down, 클수록 측면)
CAMERA_HEADING_OFFSET_DEG = 30.0    # 차량 진행방향을 화면상 1시(시계 30°)로

# lane kind -> style dict.
# render: "line" 는 연속 LineStrips3D, "dashed" 는 dash_m/gap_m 으로 segment split,
# "dots" 는 stride_m 마다 Points3D marker.
_LANE_STYLE: dict[str, dict] = {
    "edge":   {"color": (230, 230, 230), "radius": 0.12, "render": "line"},
    "lane":   {"color": (120, 120, 120), "radius": 0.06, "render": "line"},
    "center": {"color": (255, 255, 255), "radius": 0.06, "render": "dashed",
               "dash_m": 2.0, "gap_m": 2.0},
    "dotted": {"color": (255, 255, 255), "radius": 0.12, "render": "dots",
               "stride_m": 1.0},
    # boundary: search space 경계선 — 지면에 옅은 회색 선 (3D 벽 대신).
    "boundary": {"color": (140, 140, 140), "radius": 0.08, "render": "line"},
}


def _yaw_quat(yaw: float) -> rr.Quaternion:
    """z 축 회전 yaw(rad) -> Rerun quaternion (xyzw)."""
    return rr.Quaternion(xyzw=[0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])


def _wheel_boxes(x: float, y: float, yaw: float, L: float, W: float
                 ) -> tuple[np.ndarray, np.ndarray]:
    """차량 pose 기준 4 바퀴 (centers (4,3), half_sizes (4,3)).

    앞축/뒤축 ±0.35L, 트랙 ±0.40W 에 배치. 바퀴는 차량 yaw 로 정렬, 지면 접지.
    """
    wl, ww, wh = 0.18 * L, 0.10 * W, 0.40   # 바퀴 길이 / 폭 / 높이
    local = np.array([
        (0.35 * L, 0.40 * W), (0.35 * L, -0.40 * W),
        (-0.35 * L, 0.40 * W), (-0.35 * L, -0.40 * W),
    ])
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
    world = (rot @ local.T).T + np.array([x, y])
    centers = np.column_stack([world, np.full(4, wh / 2.0)])
    half = np.tile([wl / 2.0, ww / 2.0, wh / 2.0], (4, 1))
    return centers, half


def _scene_bounds(data: dict) -> tuple[float, float, float]:
    """모든 actor / reference_path / lane 좌표에서 (center_x, center_y, span)."""
    xs: list[float] = []
    ys: list[float] = []
    for actor in data.get("actors", []):
        xs += list(actor["X"])
        ys += list(actor["Y"])
    if "reference_path" in data:
        xs += list(data["reference_path"]["X"])
        ys += list(data["reference_path"]["Y"])
    for lane in data.get("lanes", []):
        xs += list(lane["X"])
        ys += list(lane["Y"])
    if not xs:
        return 0.0, 0.0, 50.0
    x_lo, x_hi, y_lo, y_hi = min(xs), max(xs), min(ys), max(ys)
    cx, cy = (x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0
    span = max(x_hi - x_lo, y_hi - y_lo, 10.0)
    return cx, cy, span


def _ego_actor(data: dict) -> dict | None:
    """'ego' 라는 이름의 actor, 없으면 첫 actor, actor 가 없으면 None."""
    actors = data.get("actors", [])
    for actor in actors:
        if actor.get("name") == "ego":
            return actor
    return actors[0] if actors else None


def _lateral_extent(data: dict) -> float:
    """모든 좌표의 Y 폭 - fixed 카메라 로컬 줌 산정용."""
    ys: list[float] = []
    for actor in data.get("actors", []):
        ys += list(actor["Y"])
    if "reference_path" in data:
        ys += list(data["reference_path"]["Y"])
    for lane in data.get("lanes", []):
        ys += list(lane["Y"])
    return (max(ys) - min(ys)) if ys else 0.0


def _view_distance(data: dict) -> float:
    """카메라-타겟 거리(zoom) - 차량 주변이 자연스럽게 보이는 로컬 스케일.

    전체 궤적이 아니라 t=0 시점 actor 들의 퍼짐 + 차선 폭 기준 (예: ego-target 간격).
    """
    actors = data.get("actors", [])
    xs0 = [float(a["X"][0]) for a in actors if a.get("X")]
    ys0 = [float(a["Y"][0]) for a in actors if a.get("Y")]
    x_spread = (max(xs0) - min(xs0)) if xs0 else 0.0
    y_spread = (max(ys0) - min(ys0)) if ys0 else 0.0
    return max((x_spread + y_spread) * 1.6, _lateral_extent(data) * 4.0, 40.0)


def _ego_heading(ego: dict | None) -> float:
    """ego 진행 방향(rad) - 순 변위로 추정, 거의 안 움직이면 Yaw[0]."""
    if ego is None:
        return 0.0
    xs, ys = ego.get("X", []), ego.get("Y", [])
    if len(xs) >= 2:
        dx, dy = float(xs[-1]) - float(xs[0]), float(ys[-1]) - float(ys[0])
        if dx * dx + dy * dy > 1e-6:
            return math.atan2(dy, dx)
    yaws = ego.get("Yaw", [])
    return float(yaws[0]) if yaws else 0.0


def _eye_controls(data: dict, camera: str) -> rrb.EyeControls3D:
    """ego(없으면 씬 중심) 를 비스듬한 3/4 top-down 으로 내려다본다. 차량 진행
    방향이 화면상 1시쯤을 향하도록 배치. camera='follow' 면 그 오프셋을 유지하며
    ego 추종, 'fixed' 면 시작 위치 고정.
    """
    ego = _ego_actor(data)
    if ego is not None:
        cx, cy = float(ego["X"][0]), float(ego["Y"][0])
    else:
        cx, cy, _ = _scene_bounds(data)

    # 카메라가 바라보는 ground 방향 v: 진행방향을 +30° 돌려 화면 1시로 보냄
    cam_yaw = _ego_heading(ego) + math.radians(CAMERA_HEADING_OFFSET_DEG)
    vx, vy = math.cos(cam_yaw), math.sin(cam_yaw)

    tilt = math.radians(CAMERA_TILT_DEG)
    r = _view_distance(data)
    position = [
        cx - vx * r * math.sin(tilt),   # v 반대쪽(뒤)으로 물러나고
        cy - vy * r * math.sin(tilt),
        r * math.cos(tilt),             # 위로 올라감
    ]
    kwargs: dict = {
        "kind": rrb.Eye3DKind.Orbital,
        "position": position,
        "look_target": [cx, cy, 0.0],   # 지면의 ego 지점
        "eye_up": [0.0, 0.0, 1.0],      # 월드 Z-up → 입체감
    }
    if camera == "follow" and ego is not None:
        kwargs["tracking_entity"] = f"/world/actors/{ego['name']}/body"
    return rrb.EyeControls3D(**kwargs)


def _build_blueprint(data: dict, camera: str) -> rrb.Blueprint:
    """좌: top-down 3D 뷰(+바닥 그리드), 우: 신호별 시계열 패널 또는 text 패널."""
    world = rrb.Spatial3DView(
        origin="/world",
        name="Top-down",
        line_grid=rrb.LineGrid3D(visible=True),
        eye_controls=_eye_controls(data, camera),
    )
    # text_panel 이 있으면 우측은 TextDocumentView (scalars 보다 우선).
    tp = data.get("text_panel")
    if tp:
        entity = tp.get("entity", "info/status")
        side = rrb.TextDocumentView(origin=f"/{entity}", name="status")
        return rrb.Blueprint(rrb.Horizontal(world, side, column_shares=[3, 1]))
    scalar_names = [sc["name"] for sc in data.get("scalars", [])]
    if not scalar_names:
        return rrb.Blueprint(world)
    scalars = rrb.Vertical(
        *[rrb.TimeSeriesView(origin=f"/scalars/{n}", name=n) for n in scalar_names]
    )
    return rrb.Blueprint(rrb.Horizontal(world, scalars, column_shares=[3, 1]))


def _log_static_environment(data: dict) -> None:
    """기준 경로 + 차선 + 3D 장애물 + start/goal marker (시간 무관, 1회 로그)."""
    if "reference_path" in data:
        ref = data["reference_path"]
        pts = np.column_stack([ref["X"], ref["Y"], np.zeros(len(ref["X"]))])
        rr.log("world/reference_path",
               rr.LineStrips3D([pts], colors=[(80, 80, 80)], radii=[0.05]),
               static=True)
    # 3D 장애물 — (x, y, radius, height) 원기둥. collision 이 원형이라 시각도 원기둥.
    for i, obs in enumerate(data.get("obstacles_3d", [])):
        ox = float(obs["x"])
        oy = float(obs["y"])
        radius = float(obs["radius"])
        height = float(obs.get("height", 0.6))
        color = tuple(obs.get("color", (140, 140, 140, 130)))  # 반투명 회색 기본
        rr.log(f"world/obstacles_3d/obs_{i}",
               rr.Cylinders3D(centers=[[ox, oy, height / 2.0]],
                              lengths=[height], radii=[radius],
                              fill_mode=rr.components.FillMode.Solid,
                              colors=[color]),
               static=True)
    # boundary_walls — 탐색 공간 밖 경계벽 (axis-aligned box 4개).
    for i, wall in enumerate(data.get("boundary_walls", [])):
        x_min = float(wall["x_min"])
        x_max = float(wall["x_max"])
        y_min = float(wall["y_min"])
        y_max = float(wall["y_max"])
        height = float(wall.get("height", 1.0))
        color = tuple(wall.get("color", (140, 140, 140, 130)))  # 반투명 회색 기본
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        half_x = (x_max - x_min) / 2.0
        half_y = (y_max - y_min) / 2.0
        rr.log(f"world/boundary_walls/wall_{i}",
               rr.Boxes3D(centers=[[cx, cy, height / 2.0]],
                          half_sizes=[[half_x, half_y, height / 2.0]],
                          fill_mode=rr.components.FillMode.Solid,
                          colors=[color]),
               static=True)
    # start / goal marker — 2D 좌표 ([x, y]) 만 받음, 지면 위 작은 sphere 로.
    if "start_marker" in data:
        sx, sy = float(data["start_marker"][0]), float(data["start_marker"][1])
        rr.log("world/start_marker",
               rr.Points3D([[sx, sy, 0.3]], colors=[(50, 100, 220)], radii=[0.4]),
               static=True)
    if "goal_marker" in data:
        gx, gy = float(data["goal_marker"][0]), float(data["goal_marker"][1])
        rr.log("world/goal_marker",
               rr.Points3D([[gx, gy, 0.3]], colors=[(220, 60, 60)], radii=[0.4]),
               static=True)
    for i, lane in enumerate(data.get("lanes", [])):
        kind = lane.get("kind", "lane")
        style = _LANE_STYLE.get(kind, _LANE_STYLE["lane"])
        color = tuple(lane.get("color", style["color"]))   # record 가 color 덮어쓰기 가능
        radius = style["radius"]
        render = style.get("render", "line")
        xs = np.asarray(lane["X"], dtype=float)
        ys = np.asarray(lane["Y"], dtype=float)
        zs = np.zeros(len(xs))
        entity = f"world/lanes/{kind}_{i}"
        if render == "dashed" and len(xs) >= 2:
            # dashed: lane["X"] 의 등간격 가정 — 점 개수로 split.
            step = float(xs[1] - xs[0])
            dash_pts = max(2, int(round(style["dash_m"] / max(step, 1e-6))))
            gap_pts = max(1, int(round(style["gap_m"] / max(step, 1e-6))))
            segments: list[np.ndarray] = []
            j = 0
            while j < len(xs):
                end = min(j + dash_pts, len(xs))
                if end - j >= 2:
                    segments.append(np.column_stack([xs[j:end], ys[j:end], zs[j:end]]))
                j += dash_pts + gap_pts
            if segments:
                n_seg = len(segments)
                rr.log(entity,
                       rr.LineStrips3D(segments, colors=[color] * n_seg,
                                       radii=[radius] * n_seg),
                       static=True)
        elif render == "dots" and len(xs) >= 1:
            # dots: stride_m 마다 marker 1 개 (Points3D).
            step = float(xs[1] - xs[0]) if len(xs) >= 2 else 1.0
            stride = max(1, int(round(style["stride_m"] / max(step, 1e-6))))
            pts = np.column_stack([xs[::stride], ys[::stride], zs[::stride]])
            rr.log(entity,
                   rr.Points3D(pts, colors=[color], radii=[radius]),
                   static=True)
        else:
            pts = np.column_stack([xs, ys, zs])
            rr.log(entity,
                   rr.LineStrips3D([pts], colors=[color], radii=[radius]),
                   static=True)


def _log_actor(actor: dict) -> None:
    """한 actor 의 매 프레임: 채워진 차체 + 바퀴 4개 + heading + 누적 trail.

    `actor["trail"]` 가 False 면 누적 trail(지나온 궤적선)을 그리지 않는다 (기본 True).
    """
    name = actor["name"]
    L = float(actor.get("L", DEFAULT_L))
    W = float(actor.get("W", DEFAULT_W))
    color = tuple(actor.get("color", DEFAULT_COLOR))
    draw_trail = actor.get("trail", True)
    body_half = [L / 2.0, W / 2.0, DEFAULT_H / 2.0]
    trail: list[list[float]] = []
    for t, x, y, yaw in zip(actor["t"], actor["X"], actor["Y"], actor["Yaw"],
                            strict=True):
        t, x, y, yaw = float(t), float(x), float(y), float(yaw)
        rr.set_time("sim_time", duration=t)
        quat = _yaw_quat(yaw)
        # 차체: 지면 위에 놓이도록 center z = H/2
        rr.log(f"world/actors/{name}/body",
               rr.Boxes3D(centers=[[x, y, DEFAULT_H / 2.0]], half_sizes=[body_half],
                          quaternions=[quat],
                          fill_mode=rr.components.FillMode.Solid, colors=[color]))
        # 바퀴 4개 (차량 yaw 로 정렬)
        w_centers, w_half = _wheel_boxes(x, y, yaw, L, W)
        rr.log(f"world/actors/{name}/wheels",
               rr.Boxes3D(centers=w_centers, half_sizes=w_half,
                          quaternions=[quat] * 4,
                          fill_mode=rr.components.FillMode.Solid,
                          colors=[WHEEL_COLOR]))
        # heading 화살표 (차체 위 - 앞/뒤 구분용)
        rr.log(f"world/actors/{name}/heading",
               rr.Arrows3D(origins=[[x, y, DEFAULT_H + 0.1]],
                           vectors=[[0.6 * L * np.cos(yaw),
                                     0.6 * L * np.sin(yaw), 0.0]],
                           colors=[color], radii=[0.08]))
        # 누적 trail (actor["trail"] == False 면 생략)
        if draw_trail:
            trail.append([x, y, 0.02])
            if len(trail) >= 2:
                rr.log(f"world/actors/{name}/trail",
                       rr.LineStrips3D([np.array(trail)], colors=[color],
                                       radii=[0.04]))


def _log_scalars(data: dict) -> None:
    """per-frame 스칼라 시계열."""
    for sc in data.get("scalars", []):
        name = sc["name"]
        for t, v in zip(sc["t"], sc["value"], strict=True):
            rr.set_time("sim_time", duration=float(t))
            rr.log(f"scalars/{name}", rr.Scalars(float(v)))


def _log_debug_scalars(data: dict) -> None:
    """디버그 신호 — recording 에는 저장되지만 기본 blueprint 에는 포함 안 됨.

    record JSON 의 `debug_scalars` 항목 (optional) — `scalars` 와 동일 구조.
    `/debug/<name>` 엔티티로 로그되며 `_build_blueprint` 는 이를 위한 view 를 만들지
    않는다. 학생이 viewer 좌측 entity 패널에서 `/debug/...` 를 골라 TimeSeriesView 를
    직접 추가하면 분석 가능. 기본 화면은 깔끔하게 유지하면서 필요 시 심화 분석.
    """
    for sc in data.get("debug_scalars", []):
        name = sc["name"]
        for t, v in zip(sc["t"], sc["value"], strict=True):
            rr.set_time("sim_time", duration=float(t))
            rr.log(f"debug/{name}", rr.Scalars(float(v)))


def _log_dynamic_paths(data: dict) -> None:
    """시간축에 따라 변하는 line (예: 매 step 의 lookahead polynomial fit, mode-gated radar).

    record JSON 의 `dynamic_paths` 항목 (optional, v2+):
        [{"name": ..., "color": [r,g,b(,a)], "radius": float,
          "t": [...],
          "points_per_t": [<single-strip 또는 multi-strip>, ...],
          "colors_per_t": [[r,g,b(,a)], ...]}              # optional, per-step color
        ]
    각 `points_per_t[i]` 형태:
      - single-strip: `[[x,y], [x,y], ...]` 또는 `[[x,y,z], ...]` — 한 step 에 한 라인.
      - multi-strip:  `[[[x,y], ...], [[x,y], ...]]` — 한 step 에 여러 라인.
    빈 list 또는 점 1개 이하인 strip 은 무시되고, 모든 strip 이 무효하면 entity Clear.
    `colors_per_t` 가 있으면 매 step color 달라짐.
    """
    for dp in data.get("dynamic_paths", []):
        name = dp["name"]
        color_static = tuple(dp.get("color", (200, 100, 0)))
        colors_per_t = dp.get("colors_per_t")
        radius = float(dp.get("radius", 0.05))
        for i, (t, raw_pts) in enumerate(zip(dp["t"], dp["points_per_t"], strict=True)):
            rr.set_time("sim_time", duration=float(t))
            # multi-strip 인지 단일-strip 인지 detect (내부 element 가 list-of-list 이면 multi).
            if (raw_pts and isinstance(raw_pts[0], (list, tuple))
                    and raw_pts[0] and isinstance(raw_pts[0][0], (list, tuple))):
                strips_raw = raw_pts
            else:
                strips_raw = [raw_pts] if raw_pts else []
            strips_3d: list[np.ndarray] = []
            for s in strips_raw:
                arr = np.asarray(s, dtype=float)
                if arr.ndim != 2 or arr.shape[0] < 2:
                    continue
                if arr.shape[1] == 2:
                    arr = np.column_stack([arr, np.full(arr.shape[0], 0.05)])
                elif arr.shape[1] != 3:
                    continue
                strips_3d.append(arr)
            if not strips_3d:
                rr.log(f"world/dynamic_paths/{name}", rr.Clear(recursive=False))
                continue
            color = tuple(colors_per_t[i]) if colors_per_t is not None else color_static
            n = len(strips_3d)
            rr.log(f"world/dynamic_paths/{name}",
                   rr.LineStrips3D(strips_3d, colors=[color] * n,
                                   radii=[radius] * n))


def _log_text_panel(data: dict) -> None:
    """우측 텍스트 패널 — 시간축에 따라 갱신되는 TextDocument.

    record JSON 의 `text_panel` 항목 (optional, v2+):
        {"entity": "info/status",        # default
         "t": [t0, t1, ...],
         "text": ["...", "...", ...]}
    각 t 에서 entity 에 TextDocument 로그. 블루프린트는 `_build_blueprint` 가
    text_panel 존재 시 TextDocumentView 로 자동 전환.
    """
    tp = data.get("text_panel")
    if not tp:
        return
    entity = tp.get("entity", "info/status")
    for t, txt in zip(tp["t"], tp["text"], strict=True):
        rr.set_time("sim_time", duration=float(t))
        rr.log(entity, rr.TextDocument(str(txt)))


def _log_dynamic_points(data: dict) -> None:
    """시간축에 따라 변하는 단일 marker (예: lookahead point, mode indicator).

    record JSON 의 `dynamic_points` 항목 (optional, v2+):
        [{"name": ..., "color": [r,g,b(,a)], "radius": float,
          "t": [...],
          "points_per_t": [[x,y] or [x,y,z], ...],   # 2D 또는 3D position
          "colors_per_t": [[r,g,b(,a)], ...]}        # optional, per-step color
        ]
    `colors_per_t` 가 있으면 매 step color 달라짐 (예: mode indicator). 없으면 `color` static.
    point 가 3-tuple 이면 z 그대로 사용, 2-tuple 이면 z=0.1.
    """
    for dp in data.get("dynamic_points", []):
        name = dp["name"]
        color_static = tuple(dp.get("color", (255, 220, 0)))
        colors_per_t = dp.get("colors_per_t")
        radius = float(dp.get("radius", 0.3))
        for i, (t, pt) in enumerate(zip(dp["t"], dp["points_per_t"], strict=True)):
            rr.set_time("sim_time", duration=float(t))
            arr = np.asarray(pt, dtype=float).reshape(-1)
            if arr.size < 2:
                continue
            z = float(arr[2]) if arr.size >= 3 else 0.1
            pt3d = np.array([[float(arr[0]), float(arr[1]), z]])
            color = tuple(colors_per_t[i]) if colors_per_t is not None else color_static
            rr.log(f"world/dynamic_points/{name}",
                   rr.Points3D(pt3d, colors=[color], radii=[radius]))


def _recording_id(record_path: Path) -> str:
    """viewer Recordings 패널에 표시될 이름.

    예) <module>/record.json → "01_speed_pid"
        <module>/record_maneuvering.json → "03_time_gap/maneuvering"
    """
    parent = record_path.parent.name
    stem = record_path.stem
    if stem == "record":
        return parent
    if stem.startswith("record_"):
        return f"{parent}/{stem[len('record_'):]}"
    return f"{parent}/{stem}"


def replay_records(record_paths: list[Path], camera: str) -> None:
    """여러 record 를 한 viewer 에 별도 recording 으로 로드.

    **전송 순서 (viewer 자동 timeline 점프 최소화):**
    1) 모든 RecordingStream 만들고 데이터를 log (이 시점엔 sink 없음 → 메모리 batch).
    2) 모든 log 완료 후, 첫 stream spawn (viewer 띄움) + 나머지 connect_grpc.
       각 sink 연결 시점에 batch 가 flush 되어 viewer 가 데이터를 한 번에 받음 —
       매 record 별 streaming 진행을 viewer 가 따라가며 "쭉 play" 하는 현상 완화.
    """
    plan: list[tuple[rr.RecordingStream, rrb.Blueprint, str, int, int, int]] = []
    for record_path in record_paths:
        data = json.loads(record_path.read_text(encoding="utf-8"))
        schema = int(data.get("schema_version", 1))
        if schema > 2:
            print(f"[simulator] 경고: {record_path.name} schema_version={schema} "
                  f"(이 플레이어는 v2까지). 신규 필드는 무시될 수 있음.",
                  file=sys.stderr)

        rid = _recording_id(record_path)
        rec = rr.RecordingStream(application_id=APP_ID, recording_id=rid)
        rr.set_global_data_recording(rec)

        _log_static_environment(data)
        for actor in data.get("actors", []):
            _log_actor(actor)
        _log_scalars(data)
        _log_debug_scalars(data)
        _log_dynamic_paths(data)
        _log_dynamic_points(data)
        _log_text_panel(data)

        plan.append((rec, _build_blueprint(data, camera), rid,
                     len(data.get("actors", [])),
                     len(data.get("scalars", [])), schema))

    # Sink 연결 — 모든 buffered 데이터가 이 시점에 flush.
    plan[0][0].spawn(default_blueprint=plan[0][1])
    for rec, bp, *_ in plan[1:]:
        rec.connect_grpc(default_blueprint=bp)

    for i, (_, _, rid, n_actors, n_scalars, schema) in enumerate(plan):
        print(f"[simulator] [{i+1}/{len(plan)}] {rid}: "
              f"{n_actors} actor / {n_scalars} scalar series (v{schema})")

    if len(plan) > 1:
        print(f"\n[simulator] {len(plan)} recording 로드 완료. "
              f"viewer 좌측 Recordings 패널에서 클릭으로 시나리오 전환.")


def _find_records(root: Path) -> list[Path]:
    """root 하위(재귀)의 record*.json 을 경로순 정렬해 반환."""
    return sorted(root.rglob("record*.json"))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="vehicle-control record*.json 을 Rerun viewer 로 재생 "
                    "(여러 record 를 한 viewer 에 멀티-recording 으로 로드)")
    parser.add_argument(
        "path", nargs="?", default=None,
        help="record.json 파일 또는 스캔할 디렉토리 (생략 시 스크립트 폴더 하위 스캔). "
             "파일이면 그 한 record 만, 디렉토리면 하위 모든 record*.json 을 "
             "별도 recording 으로 viewer 에 로드.")
    parser.add_argument(
        "--camera", choices=("follow", "fixed"), default="follow",
        help="초기 카메라 - follow: ego 추종(기본), fixed: ego 시작 위치 고정 "
             "(뷰어 안에서도 eye controls 로 전환 가능)")
    args = parser.parse_args()

    arg = Path(args.path) if args.path else Path(__file__).parent
    if not arg.exists():
        print(f"경로 없음: {arg}", file=sys.stderr)
        sys.exit(1)

    if arg.is_file():
        records = [arg]
    else:
        records = _find_records(arg)
        if not records:
            print(f"record*.json 을 찾지 못함: {arg}\n"
                  f"  먼저 각 모듈 record_gen.py 를 실행해 record.json 을 생성하세요.",
                  file=sys.stderr)
            sys.exit(1)

    replay_records(records, camera=args.camera)


if __name__ == "__main__":
    main()
