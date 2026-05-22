"""Frenet 좌표계 최적 궤적 계획기 — 이동 타겟 회피.

Intent: intents/modules/05_trajectory_planning/02_frenet_moving_target.md

매 스텝 ego 의 현재 Frenet 상태에서 여러 후보 궤적(목표 차선 × 종방향 속도 ×
종료 시간 조합)을 생성하고, 비용(jerk + 종료시간 + 차선 일관성 + 목표 속도)이
최소이면서 동역학 한계·충돌 검사를 통과하는 궤적을 고른다.

타겟 예측: CV(등속) 모델 — predict_target_cv. 종방향은 현재 s_d 로 등속 외삽,
횡방향 d 는 고정. 의도 인식·multi-modal 같은 고도화된 예측은 다음 예제 영역이다.

구현 과제 (# TODO):
  - QuinticPolynomial.__init__ / QuarticPolynomial.__init__  (다항식 계수 풀이)
  - calc_frenet_paths                                        (후보 궤적 생성 + 비용)
그 외(좌표 변환·동역학/충돌 검사·예측·최적 선택)는 주어진 환경이다.
"""
from __future__ import annotations

from copy import deepcopy  # noqa: F401  (calc_frenet_paths 구현 시 사용)

import numpy as np

# --- 차량·트랙 한계 (차량 스케일) ---
V_MAX = 18.0      # 최대 속도 [m/s]
V_MIN = 1.0       # 최소 속도 [m/s]
ACC_MAX = 8.0     # 최대 가속도 [m/s^2]
K_MAX = 0.5       # 최대 곡률 [1/m]

TARGET_SPEED = 10.0   # 목표 주행 속도 [m/s]
COL_CHECK = 3.5       # 충돌 판정 거리 [m] (차선 간격 LANE_WIDTH 보다 작아야 함)

# --- 후보 궤적 생성 파라미터 ---
MIN_T = 2.0       # 최소 종료 시간 [s]
MAX_T = 5.0       # 최대 종료 시간 [s]
DT_T = 1.0        # 종료 시간 후보 간격 [s]
DT = 0.1          # 궤적 시간 분해능 [s]

# --- 비용 가중치 ---
K_J_lat = 0.05    # 횡방향 jerk
K_J_lon = 0.1     # 종방향 jerk
K_T = 0.5         # 종료 시간
K_D = 5.0         # 차선 일관성 (consistency)
K_V = 80.0        # 목표 속도 도달
K_LAT = 1.0       # 횡방향 비용 비중
K_LON = 1.0       # 종방향 비용 비중

# 횡방향 종료 위치 후보 (양 차선 중앙) — LANE_WIDTH=4.0 기준
DF_SET = np.array([2.0, -2.0])
# 종방향 종료 속도 후보 (목표 속도 대비 가감속)
SF_D_SET = np.array([-2.0, 0.0, 2.0])


class QuinticPolynomial:
    """5 차 다항식 — 양 끝의 위치·속도·가속도 6 개 경계조건을 만족 (횡방향 궤적용)."""

    def __init__(self, xi, vi, ai, xf, vf, af, T):
        """t=0 에서 (xi, vi, ai), t=T 에서 (xf, vf, af) 를 만족하는 6 개 계수 산출.

        a0~a2 는 t=0 조건에서 바로 결정되고, a3~a5 는 t=T 의 위치·속도·가속도
        3 개 조건이 만드는 3×3 선형계를 풀어 얻는다.
        """
        # TODO: 경계조건으로부터 self.a0 ~ self.a5 를 계산하세요.
        #   - a0, a1, a2 : t=0 의 (위치, 속도, 가속도) 조건에서 직접 결정.
        #   - a3, a4, a5 : t=T 의 (위치, 속도, 가속도) 3 개 조건이 만드는
        #                  3×3 선형계 (intent 의 Math 섹션 참조) 를 풀어 구한다.
        raise NotImplementedError("QuinticPolynomial.__init__ 를 구현하세요")

    def calc_pos(self, t):
        return (self.a0 + self.a1 * t + self.a2 * t**2
                + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5)

    def calc_vel(self, t):
        return (self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2
                + 4 * self.a4 * t**3 + 5 * self.a5 * t**4)

    def calc_acc(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

    def calc_jerk(self, t):
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2


class QuarticPolynomial:
    """4 차 다항식 — 시작의 위치·속도·가속도 + 종료의 속도·가속도 (종방향 궤적용).

    종방향은 종료 '위치' 를 구속하지 않는다 (velocity-keeping) — 그래서 5 차가 아닌
    4 차로 충분하다.
    """

    def __init__(self, xi, vi, ai, vf, af, T):
        """t=0 에서 (xi, vi, ai), t=T 에서 (vf, af) 를 만족하는 5 개 계수 산출.

        a0~a2 는 t=0 조건에서 바로 결정되고, a3~a4 는 t=T 의 속도·가속도 2 개
        조건이 만드는 2×2 선형계를 풀어 얻는다.
        """
        # TODO: 경계조건으로부터 self.a0 ~ self.a4 를 계산하세요.
        #   - a0, a1, a2 : t=0 의 (위치, 속도, 가속도) 조건에서 직접 결정.
        #   - a3, a4     : t=T 의 (속도, 가속도) 2 개 조건이 만드는 2×2 선형계
        #                  (intent 의 Math 섹션 참조) 를 풀어 구한다.
        raise NotImplementedError("QuarticPolynomial.__init__ 를 구현하세요")

    def calc_pos(self, t):
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

    def calc_vel(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

    def calc_acc(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

    def calc_jerk(self, t):
        return 6 * self.a3 + 24 * self.a4 * t


class FrenetPath:
    """한 후보 궤적 — Frenet 시계열 + 전역 시계열 + 비용."""

    def __init__(self):
        self.t: list[float] = []
        # 횡방향 (Frenet)
        self.d: list[float] = []
        self.d_d: list[float] = []
        self.d_dd: list[float] = []
        self.d_ddd: list[float] = []
        # 종방향 (Frenet)
        self.s: list[float] = []
        self.s_d: list[float] = []
        self.s_dd: list[float] = []
        self.s_ddd: list[float] = []
        # 비용
        self.c_lat = 0.0
        self.c_lon = 0.0
        self.c_tot = 0.0
        # 전역 좌표
        self.x: list[float] = []
        self.y: list[float] = []
        self.yaw: list[float] = []
        self.ds: list[float] = []
        self.kappa: list[float] = []


def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd,
                      di, di_d, di_dd, df_d, df_dd, opt_d):
    """ego 의 현재 Frenet 상태에서 후보 궤적들을 생성하고 각 비용을 매긴다.

    (종방향 속도 후보 SF_D_SET) × (횡방향 종료 위치 후보 DF_SET) ×
    (종료 시간 후보 MIN_T..MAX_T) 의 모든 조합에 대해:
      1. 횡방향: QuinticPolynomial 로 (di,di_d,di_dd) → (df, df_d, df_dd) 궤적.
      2. 종방향: QuarticPolynomial 로 (si,si_d,si_dd) → (sf_d+Δ, sf_dd) 궤적.
      3. T < MAX_T 면 마지막 상태를 등속 외삽해 모든 궤적 길이를 MAX_T 로 통일.
      4. 비용 = jerk + 종료시간 + 차선 일관성(opt_d 대비) + 목표 속도 오차.

    Args:
        si, si_d, si_dd: 종방향 시작 위치·속도·가속도.
        sf_d, sf_dd: 종방향 종료 기준 속도·가속도 (SF_D_SET 이 sf_d 에 더해짐).
        di, di_d, di_dd: 횡방향 시작 위치·속도·가속도.
        df_d, df_dd: 횡방향 종료 속도·가속도 (보통 0 — 차선 중앙 정렬).
        opt_d: 직전 스텝 최적 궤적의 종료 횡위치 — 차선 일관성 비용 기준.

    Returns:
        list[FrenetPath] — Frenet 시계열(t,d,s,...)과 비용(c_lat,c_lon,c_tot)이
        채워진 후보들. 전역 좌표(x,y,...)는 calc_global_paths 에서 채운다.
    """
    # TODO: 후보 궤적들을 생성하고 각 비용을 매겨 list[FrenetPath] 로 반환하세요.
    #   (SF_D_SET × DF_SET × np.arange(MIN_T, MAX_T+DT_T, DT_T)) 의 모든 조합:
    #     1. 횡방향 QuinticPolynomial, 종방향 QuarticPolynomial 로 0..T 를 DT 간격
    #        샘플링해 FrenetPath 의 d/d_d/.../s/s_d/... 시계열을 채운다.
    #     2. T < MAX_T 인 궤적은 마지막 상태를 등속 외삽해 길이를 MAX_T 로 통일.
    #     3. 비용 c_lat / c_lon / c_tot 산정 (intent 의 Math 섹션 참조).
    #   deepcopy 는 횡방향만 채운 FrenetPath 를 종방향용으로 복제할 때 유용하다.
    raise NotImplementedError("calc_frenet_paths 를 구현하세요")


def calc_global_paths(fplist, track):
    """각 후보 궤적의 Frenet (s, d) 시계열을 전역 (x, y, yaw, ds, kappa) 로 변환."""
    for fp in fplist:
        for _s, _d in zip(fp.s, fp.d, strict=True):
            x, y, _ = track.to_cartesian(_s, _d)
            fp.x.append(x)
            fp.y.append(y)
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(np.arctan2(dy, dx))
            fp.ds.append(np.hypot(dx, dy))
        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])
        for i in range(len(fp.yaw) - 1):
            yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            fp.kappa.append(yaw_diff / fp.ds[i] if fp.ds[i] > 1e-9 else 0.0)
    return fplist


def predict_target_cv(s, d, s_d, t_horizon, dt):
    """타겟의 CV(등속) 예측 — 종방향 등속 외삽, 횡방향 d 고정.

    이 예제의 타겟 예측 수준이다. 실제 타겟 거동(target_vehicles)과 무관하게
    planner 는 "타겟이 현재 속도로 차선을 유지한다"고만 가정한다. 의도 인식·
    multi-modal 같은 고도화된 예측은 다음 예제에서 다룬다.

    Returns:
        (s_pred, d_pred) — 길이 t_horizon/dt 의 예측 Frenet 시계열.
    """
    ts = np.arange(0.0, t_horizon, dt)
    s_pred = [s + s_d * t for t in ts]
    d_pred = [d for _ in ts]
    return s_pred, d_pred


def collision_check(fp, target_states, track):
    """fp 가 어떤 타겟의 CV 예측 궤적과 COL_CHECK 이내로 접근하면 True."""
    for s, d, s_d in target_states:
        s_pred, d_pred = predict_target_cv(s, d, s_d, MAX_T, DT)
        for i in range(len(fp.t)):
            tx, ty, _ = track.to_cartesian(s_pred[i], d_pred[i])
            if (tx - fp.x[i]) ** 2 + (ty - fp.y[i]) ** 2 <= COL_CHECK ** 2:
                return True
    return False


def check_path(fplist, target_states, track):
    """동역학 한계(속도·가속도·곡률)와 충돌 검사를 통과하는 후보만 반환."""
    ok: list[FrenetPath] = []
    for fp in fplist:
        acc_sq = [a_s**2 + a_d**2 for a_s, a_d in zip(fp.s_dd, fp.d_dd, strict=True)]
        if any(v > V_MAX for v in fp.s_d):
            continue
        if any(a > ACC_MAX**2 for a in acc_sq):
            continue
        if any(abs(k) > K_MAX for k in fp.kappa):
            continue
        if collision_check(fp, target_states, track):
            continue
        if any(v < V_MIN for v in fp.s_d):
            continue
        ok.append(fp)
    return ok


def frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd,
                            di, di_d, di_dd, df_d, df_dd,
                            target_states, track, opt_d):
    """후보 궤적을 생성·검증하고 비용 최소 궤적을 고른다.

    Returns:
        (valid, best) — valid: 검사를 통과한 후보 목록(시각화용),
        best: 비용 최소 FrenetPath (검사 통과 후보가 없으면 None).
    """
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd,
                               di, di_d, di_dd, df_d, df_dd, opt_d)
    fplist = calc_global_paths(fplist, track)
    valid = check_path(fplist, target_states, track)

    best, best_cost = None, float("inf")
    for fp in valid:
        if fp.c_tot <= best_cost:
            best_cost, best = fp.c_tot, fp
    return valid, best
