"""Frenet optimal trajectory planner for moving-target avoidance."""
from __future__ import annotations

import numpy as np
from prediction import predict_target_lanekeep

V_MAX = 18.0
V_MIN = 1.0
ACC_MAX = 8.0
K_MAX = 0.5

TARGET_SPEED = 10.0
COL_CHECK = 3.5

MIN_T = 2.0
MAX_T = 5.0
DT_T = 1.0
DT = 0.1

K_J_lat = 0.05
K_J_lon = 0.1
K_T = 0.5
K_D = 5.0
K_V = 80.0
K_LAT = 1.0
K_LON = 1.0

DF_SET = np.array([2.0, -2.0])
SF_D_SET = np.array([-6.0, -4.0, -2.0, 0.0, 2.0])


class QuinticPolynomial:
    """Fifth-order polynomial satisfying position/velocity/acceleration ends."""

    def __init__(self, xi, vi, ai, xf, vf, af, T):
        self.a0 = float(xi)
        self.a1 = float(vi)
        self.a2 = float(ai) / 2.0

        A = np.array([
            [T ** 3, T ** 4, T ** 5],
            [3.0 * T ** 2, 4.0 * T ** 3, 5.0 * T ** 4],
            [6.0 * T, 12.0 * T ** 2, 20.0 * T ** 3],
        ], dtype=float)
        b = np.array([
            xf - (self.a0 + self.a1 * T + self.a2 * T ** 2),
            vf - (self.a1 + 2.0 * self.a2 * T),
            af - (2.0 * self.a2),
        ], dtype=float)
        self.a3, self.a4, self.a5 = np.linalg.solve(A, b)

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
    """Fourth-order polynomial for longitudinal speed keeping."""

    def __init__(self, xi, vi, ai, vf, af, T):
        self.a0 = float(xi)
        self.a1 = float(vi)
        self.a2 = float(ai) / 2.0

        A = np.array([
            [3.0 * T ** 2, 4.0 * T ** 3],
            [6.0 * T, 12.0 * T ** 2],
        ], dtype=float)
        b = np.array([
            vf - (self.a1 + 2.0 * self.a2 * T),
            af - (2.0 * self.a2),
        ], dtype=float)
        self.a3, self.a4 = np.linalg.solve(A, b)

    def calc_pos(self, t):
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

    def calc_vel(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

    def calc_acc(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

    def calc_jerk(self, t):
        return 6 * self.a3 + 24 * self.a4 * t


class FrenetPath:
    """One candidate trajectory in Frenet and global coordinates."""

    def __init__(self):
        self.t: list[float] = []
        self.d: list[float] = []
        self.d_d: list[float] = []
        self.d_dd: list[float] = []
        self.d_ddd: list[float] = []
        self.s: list[float] = []
        self.s_d: list[float] = []
        self.s_dd: list[float] = []
        self.s_ddd: list[float] = []
        self.c_lat = 0.0
        self.c_lon = 0.0
        self.c_tot = 0.0
        self.x: list[float] = []
        self.y: list[float] = []
        self.yaw: list[float] = []
        self.ds: list[float] = []
        self.kappa: list[float] = []


def _append_lateral(fp: FrenetPath, lat: QuinticPolynomial,
                    t: float, T: float, df: float,
                    df_d: float, df_dd: float) -> None:
    if t <= T:
        fp.d.append(float(lat.calc_pos(t)))
        fp.d_d.append(float(lat.calc_vel(t)))
        fp.d_dd.append(float(lat.calc_acc(t)))
        fp.d_ddd.append(float(lat.calc_jerk(t)))
    else:
        fp.d.append(float(df))
        fp.d_d.append(float(df_d))
        fp.d_dd.append(float(df_dd))
        fp.d_ddd.append(0.0)


def _append_longitudinal(fp: FrenetPath, lon: QuarticPolynomial,
                         t: float, T: float) -> None:
    if t <= T:
        fp.s.append(float(lon.calc_pos(t)))
        fp.s_d.append(float(lon.calc_vel(t)))
        fp.s_dd.append(float(lon.calc_acc(t)))
        fp.s_ddd.append(float(lon.calc_jerk(t)))
    else:
        dt_tail = t - T
        s_T = float(lon.calc_pos(T))
        v_T = float(lon.calc_vel(T))
        a_T = float(lon.calc_acc(T))
        fp.s.append(s_T + v_T * dt_tail + 0.5 * a_T * dt_tail ** 2)
        fp.s_d.append(v_T + a_T * dt_tail)
        fp.s_dd.append(a_T)
        fp.s_ddd.append(0.0)


def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd,
                      di, di_d, di_dd, df_d, df_dd, opt_d):
    """Generate Frenet candidate trajectories and assign scalar costs."""
    paths: list[FrenetPath] = []
    time_candidates = np.arange(MIN_T, MAX_T + 0.5 * DT_T, DT_T)
    horizon_ts = np.arange(0.0, MAX_T, DT)

    for target_speed_delta in SF_D_SET:
        target_speed = float(sf_d + target_speed_delta)
        for df in DF_SET:
            for T in time_candidates:
                lat = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, float(T))
                lon = QuarticPolynomial(si, si_d, si_dd, target_speed, sf_dd, float(T))

                fp = FrenetPath()
                for t in horizon_ts:
                    fp.t.append(float(t))
                    _append_lateral(fp, lat, float(t), float(T), float(df), df_d, df_dd)
                    _append_longitudinal(fp, lon, float(t), float(T))

                j_lat = float(np.sum(np.square(fp.d_ddd)))
                j_lon = float(np.sum(np.square(fp.s_ddd)))
                terminal_d = fp.d[-1]
                terminal_speed = fp.s_d[-1]

                fp.c_lat = K_J_lat * j_lat + K_T * float(T) + K_D * (terminal_d - opt_d) ** 2
                fp.c_lon = K_J_lon * j_lon + K_T * float(T) + K_V * (TARGET_SPEED - terminal_speed) ** 2
                fp.c_tot = K_LAT * fp.c_lat + K_LON * fp.c_lon
                paths.append(fp)

    return paths


def calc_global_paths(fplist, track):
    """Convert each Frenet candidate to global x, y, yaw, ds, and curvature."""
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


def collision_check(fp, target_states, track):
    """Return True when a candidate comes too close to predicted targets."""
    for s, d, s_d in target_states:
        s_pred, d_pred = predict_target_lanekeep(s, d, s_d, MAX_T, DT)
        for i in range(len(fp.t)):
            tx, ty, _ = track.to_cartesian(s_pred[i], d_pred[i])
            if (tx - fp.x[i]) ** 2 + (ty - fp.y[i]) ** 2 <= COL_CHECK ** 2:
                return True
    return False


def check_path(fplist, target_states, track):
    """Filter candidates by dynamics limits and collision checks."""
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
    """Generate, convert, validate, and select the minimum-cost trajectory."""
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd,
                               di, di_d, di_dd, df_d, df_dd, opt_d)
    fplist = calc_global_paths(fplist, track)
    valid = check_path(fplist, target_states, track)

    best, best_cost = None, float("inf")
    for fp in valid:
        if fp.c_tot <= best_cost:
            best_cost, best = fp.c_tot, fp
    return valid, best
