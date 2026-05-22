"""Frenet Moving Target — behavioral spec (requirements level).

다항식 계수 풀이 방식·후보 생성 방식은 자유. 다항식 경계조건 만족 + 후보 궤적
구조 + 폐루프 주행(충돌 없음·전진·목표 속도)으로 합격 판정.
"""
import math

from frenet_planner import (
    DF_SET,
    SF_D_SET,
    TARGET_SPEED,
    QuarticPolynomial,
    QuinticPolynomial,
    calc_frenet_paths,
    frenet_optimal_planning,
)
from target_vehicles import Maneuver, TargetFleet, TargetVehicle
from track_map import LANE_WIDTH, TrackMap

DT = 0.1
SIM_TIME = 20.0


# ---------------------------------------------------------------- 다항식 경계조건


def test_quintic_polynomial_boundary():
    """5 차 다항식이 t=0 / t=T 의 위치·속도·가속도 6 조건을 모두 만족."""
    xi, vi, ai, xf, vf, af, T = 1.0, 2.0, 0.5, 7.0, -1.0, 0.3, 4.0
    p = QuinticPolynomial(xi, vi, ai, xf, vf, af, T)
    assert math.isclose(p.calc_pos(0.0), xi, abs_tol=1e-6)
    assert math.isclose(p.calc_vel(0.0), vi, abs_tol=1e-6)
    assert math.isclose(p.calc_acc(0.0), ai, abs_tol=1e-6)
    assert math.isclose(p.calc_pos(T), xf, abs_tol=1e-6)
    assert math.isclose(p.calc_vel(T), vf, abs_tol=1e-6)
    assert math.isclose(p.calc_acc(T), af, abs_tol=1e-6)


def test_quartic_polynomial_boundary():
    """4 차 다항식이 t=0 의 위치·속도·가속도 + t=T 의 속도·가속도를 만족."""
    xi, vi, ai, vf, af, T = 0.0, 10.0, 0.0, 12.0, 0.0, 3.0
    p = QuarticPolynomial(xi, vi, ai, vf, af, T)
    assert math.isclose(p.calc_pos(0.0), xi, abs_tol=1e-6)
    assert math.isclose(p.calc_vel(0.0), vi, abs_tol=1e-6)
    assert math.isclose(p.calc_acc(0.0), ai, abs_tol=1e-6)
    assert math.isclose(p.calc_vel(T), vf, abs_tol=1e-6)
    assert math.isclose(p.calc_acc(T), af, abs_tol=1e-6)


# ---------------------------------------------------------------- 후보 궤적 구조


def test_calc_frenet_paths_structure():
    """후보 궤적 = (SF_D_SET × DF_SET × 종료시간) 조합 수, 각 시계열 길이 일치."""
    paths = calc_frenet_paths(0.0, TARGET_SPEED, 0.0, TARGET_SPEED, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, opt_d=0.0)
    n_T = 4  # MIN_T..MAX_T, DT_T 간격
    assert len(paths) == len(SF_D_SET) * len(DF_SET) * n_T
    for fp in paths:
        n = len(fp.t)
        assert n > 0
        assert len(fp.s) == n and len(fp.d) == n
        assert len(fp.s_d) == n and len(fp.d_dd) == n
        assert math.isfinite(fp.c_tot)


def test_calc_frenet_paths_reach_lane_targets():
    """후보의 종료 횡위치가 DF_SET(차선 중앙 후보)을 모두 커버."""
    paths = calc_frenet_paths(0.0, TARGET_SPEED, 0.0, TARGET_SPEED, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, opt_d=0.0)
    finals = [round(fp.d[-1], 3) for fp in paths]
    for df in DF_SET:
        assert any(math.isclose(f, df, abs_tol=0.2) for f in finals)


# ---------------------------------------------------------------- 폐루프 주행


def _run_closed_loop():
    """ego 가 폐루프 트랙에서 직선 주행 타겟들을 피하며 Frenet 최적 계획 주행.

    Returns:
        (min_gap, ego_final_s, tail_speed, n_no_solution)
    """
    track = TrackMap()
    fleet = TargetFleet([
        TargetVehicle(s=50.0, d=-LANE_WIDTH / 2, s_d=7.0,
                      maneuver=Maneuver(kind="straight"), name="t1"),
        TargetVehicle(s=95.0, d=+LANE_WIDTH / 2, s_d=10.0,
                      maneuver=Maneuver(kind="straight"), name="t2"),
        TargetVehicle(s=150.0, d=-LANE_WIDTH / 2, s_d=8.0,
                      maneuver=Maneuver(kind="straight"), name="t3"),
    ], track)
    targets = fleet.targets

    si, si_d, si_dd = 0.0, TARGET_SPEED, 0.0
    di, di_d, di_dd = -LANE_WIDTH / 2, 0.0, 0.0
    opt_d = di

    min_gap = float("inf")
    speeds: list[float] = []
    n_no_solution = 0
    steps = int(SIM_TIME / DT)

    for _ in range(steps):
        states = fleet.states()
        _, best = frenet_optimal_planning(si, si_d, si_dd, TARGET_SPEED, 0.0,
                                          di, di_d, di_dd, 0.0, 0.0,
                                          states, track, opt_d)
        if best is None:
            n_no_solution += 1
            ex, ey, _ = track.to_cartesian(si, di)
        else:
            ex, ey = best.x[0], best.y[0]

        for tg in targets:
            gap = math.hypot(ex - tg.x, ey - tg.y)
            min_gap = min(min_gap, gap)

        speeds.append(si_d)
        if best is not None:
            si, si_d, si_dd = best.s[1], best.s_d[1], best.s_dd[1]
            di, di_d, di_dd = best.d[1], best.d_d[1], best.d_dd[1]
            opt_d = best.d[-1]
        fleet.update_all(DT)

    tail = speeds[len(speeds) // 2:]
    return min_gap, si, sum(tail) / len(tail), n_no_solution


def test_closed_loop_no_collision_and_progress():
    """20 s 폐루프 주행:
       - ego 가 타겟과 충돌하지 않음 (min gap 충분).
       - ego 가 전진해 트랙을 따라 주행 (final s 충분).
       - tail 평균 속도가 목표 속도 근방.
    """
    min_gap, final_s, tail_speed, n_no_sol = _run_closed_loop()
    assert min_gap > 2.0, f"타겟과 최소 간격 {min_gap:.2f} m — 충돌 위험"
    assert final_s > 120.0, f"ego 전진 부족 (final s = {final_s:.1f} m)"
    assert abs(tail_speed - TARGET_SPEED) < 4.0, \
        f"tail 평균 속도 {tail_speed:.2f} m/s 가 목표 {TARGET_SPEED} 에서 벗어남"
    assert n_no_sol == 0, f"해 없음 스텝 {n_no_sol} 회 발생"
