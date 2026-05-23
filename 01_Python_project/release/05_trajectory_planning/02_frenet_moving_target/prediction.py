"""타겟 차량 예측 모델 — planner 가 타겟의 미래 위치를 추정하는 방식.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

planner 는 실제 타겟 거동(`target_vehicles`)을 모른 채, 관측된 Frenet 상태만으로
타겟의 미래를 추정한다. 두 가지 모델을 제공한다:

  - `predict_target_lanekeep`   — 종방향 등속 외삽, 횡방향 d 고정. "타겟이 현재
                                  속도로 차선을 유지한다"는 naive baseline.
  - `predict_target_lanechange` — 종방향 등속 외삽, 횡방향은 관측된 d_d 도 등속으로
                                  외삽하되 (1) 1차 지연 댐퍼로 접근 속도가 부드럽게
                                  감쇠하고 (2) 옆 차선 중심에 도달하면 거기서 멈춘다.

`predict_target_lanekeep` 은 차선 유지 타겟에는 잘 맞지만, 차로변경 의도를 가진
타겟에는 'd 고정' 가정이 깨져 예측이 어긋난다. `predict_target_lanechange` 는
관측된 횡방향 속도 d_d 로 차로변경 방향을 추정하므로 그 괴리를 크게 줄인다 —
의도 인식·multi-modal 같은 더 고도화된 예측은 다음 예제 영역이다.
"""
from __future__ import annotations

import numpy as np
from track_map import LANE_WIDTH

# 1차 지연 시정수 — 횡방향 접근 속도 감쇠 [s]. 클수록 댐퍼가 약해 d 가 멀리 외삽된다.
LC_DAMP_TAU = 1.5


def predict_target_lanekeep(s, d, s_d, t_horizon, dt):
    """차선 유지 타겟 예측 — 종방향 등속, 횡방향 d 고정.

    가장 단순한 baseline. 차선 유지 타겟에는 잘 맞고, 차로변경 의도를 가진 타겟
    에는 d 고정 가정이 깨져 예측이 어긋난다 (그 경우 `predict_target_lanechange`).

    Args:
        s, d, s_d: 타겟의 현재 Frenet 종방향 위치·횡방향 위치·종방향 속도.
        t_horizon: 예측 구간 길이 [s].
        dt: 예측 시간 분해능 [s].

    Returns:
        (s_pred, d_pred) — 길이 t_horizon/dt 의 예측 Frenet 시계열.
    """
    ts = np.arange(0.0, t_horizon, dt)
    s_pred = [s + s_d * t for t in ts]
    d_pred = [d for _ in ts]
    return s_pred, d_pred


def predict_target_lanechange(s, d, s_d, d_d, t_horizon, dt,
                              tau: float = LC_DAMP_TAU,
                              lane_width: float = LANE_WIDTH):
    """차로변경 의도 반영 예측 — 종·횡 모두 CV 외삽 + 옆 차선 도착 시 정지.

    종방향은 등속 외삽. 횡방향은 관측된 d_d 의 방향으로 옆 차선 중심을 향해 이동
    하되, 두 가지 사실적 효과를 더한다:

    (1) **1차 지연 댐퍼.** d_d 가 시정수 τ 의 1차 지연으로 0 으로 감쇠한다:
        d_d(t) = d_d(0) · exp(-t/τ).  적분하면
        d(t)   = d(0)   + d_d(0) · τ · (1 - exp(-t/τ)).
        실제 차로변경은 양 끝에서 횡속도가 0 인 S 자 곡선이라, 단순 CV(등속) 외삽
        보다 '접근하면서 속도가 줄어드는' 1차 응답이 매뉴버 후반부와 더 잘 맞는다.

    (2) **목표 차선에서 정지.** d_d 의 방향으로 가장 가까운 옆 차선 중심
        (±lane_width/2) 에 도달하면 거기서 멈춘다 (방향 의식 clip). 도달 후
        횡속도는 0 으로 고정된 것과 같다.

    d_d 가 0 이면 `predict_target_lanekeep` 과 동일한 결과 (degenerate case).

    Args:
        s, d, s_d, d_d: 타겟의 현재 Frenet 종·횡 위치, 종·횡 속도.
        t_horizon: 예측 구간 길이 [s].
        dt: 예측 시간 분해능 [s].
        tau: 1차 지연 시정수 [s]. 클수록 댐퍼가 약해 d 가 멀리 외삽된다.
        lane_width: 차선 폭 [m]. 목표 차선 중심 = ±lane_width/2.

    Returns:
        (s_pred, d_pred) — 길이 t_horizon/dt 의 예측 Frenet 시계열.
    """
    ts = np.arange(0.0, t_horizon, dt)
    s_pred = s + s_d * ts

    if d_d == 0.0:
        d_pred = np.full_like(ts, d)
    else:
        # 옆 차선 중심 — d_d 방향으로 가장 가까운 lane center.
        target_d = (lane_width / 2.0) if d_d > 0.0 else (-lane_width / 2.0)
        # 1차 지연 적분: d_d 가 exp(-t/τ) 로 감쇠 → 위치는 (1-exp) 로 수렴.
        d_unclipped = d + d_d * tau * (1.0 - np.exp(-ts / tau))
        # target_d 에 도달하면 거기서 정지 — 방향에 따른 clip.
        if d_d > 0.0:
            d_pred = np.minimum(d_unclipped, target_d)
        else:
            d_pred = np.maximum(d_unclipped, target_d)

    return s_pred.tolist(), d_pred.tolist()
