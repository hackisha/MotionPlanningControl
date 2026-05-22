"""Frenet 좌표 변환 — behavioral spec (requirements level).

투영/탐색 구현 방식은 자유. 중심선 위 점 복원, 횡 offset 의 크기·부호,
변환 왕복(round-trip) 일관성으로 합격 판정한다.
"""
import math

from frenet_transform import cartesian_to_frenet, frenet_to_cartesian
from road import Road

# 양 끝 세그먼트 클램프 영향을 피해 중심선 내부 구간만 샘플.
_ROAD = Road()
_CX, _CY, _CS = _ROAD.center_x, _ROAD.center_y, _ROAD.center_s
_SAMPLE_I = list(range(20, len(_CX) - 20, 23))


def _seg_heading(i: int) -> float:
    """세그먼트 i 의 chord heading [rad]."""
    return math.atan2(_CY[i + 1] - _CY[i], _CX[i + 1] - _CX[i])


def _seg_len(i: int) -> float:
    """세그먼트 i 의 길이 [m]."""
    return math.hypot(_CX[i + 1] - _CX[i], _CY[i + 1] - _CY[i])


# ----------------------------------------------------------- frenet → cartesian


def test_frenet_to_cartesian_on_centerline():
    """d=0 은 중심선 점 위 — frenet_to_cartesian(cs_i, 0) == (cx_i, cy_i)."""
    for i in _SAMPLE_I:
        x, y, _ = frenet_to_cartesian(_CS[i], 0.0, _CX, _CY, _CS)
        assert math.isclose(x, _CX[i], abs_tol=1e-6)
        assert math.isclose(y, _CY[i], abs_tol=1e-6)


def test_frenet_to_cartesian_lateral_offset():
    """횡 offset d 는 중심선 왼쪽 법선 방향으로 거리 |d| 만큼 이동."""
    for i in _SAMPLE_I:
        th = _seg_heading(i)
        nx, ny = -math.sin(th), math.cos(th)          # 왼쪽 법선
        for d in (+3.0, -3.0, +5.5):
            x, y, h = frenet_to_cartesian(_CS[i], d, _CX, _CY, _CS)
            assert math.isclose(x, _CX[i] + d * nx, abs_tol=1e-6)
            assert math.isclose(y, _CY[i] + d * ny, abs_tol=1e-6)
            assert math.isclose(h, th, abs_tol=1e-6)


# ----------------------------------------------------------- cartesian → frenet


def test_cartesian_to_frenet_on_centerline():
    """중심선 점은 (s ≈ cs_i, d ≈ 0) 으로 변환된다."""
    for i in _SAMPLE_I:
        s, d = cartesian_to_frenet(_CX[i], _CY[i], _CX, _CY, _CS)
        assert math.isclose(s, _CS[i], abs_tol=0.05)
        assert math.isclose(d, 0.0, abs_tol=0.05)


def test_lateral_sign_convention():
    """진행 방향 기준 왼쪽 = d>0, 오른쪽 = d<0 — 부호 규약."""
    for i in _SAMPLE_I:
        th = _seg_heading(i)
        nx, ny = -math.sin(th), math.cos(th)
        _, d_left = cartesian_to_frenet(_CX[i] + 2.0 * nx, _CY[i] + 2.0 * ny,
                                        _CX, _CY, _CS)
        _, d_right = cartesian_to_frenet(_CX[i] - 2.0 * nx, _CY[i] - 2.0 * ny,
                                         _CX, _CY, _CS)
        assert d_left > 0.0, f"왼쪽 offset 은 d>0 이어야 함 (got {d_left:.3f})"
        assert d_right < 0.0, f"오른쪽 offset 은 d<0 이어야 함 (got {d_right:.3f})"
        assert math.isclose(d_left, 2.0, abs_tol=0.05)
        assert math.isclose(d_right, -2.0, abs_tol=0.05)


# ----------------------------------------------------------------- round-trip


def test_round_trip_consistency():
    """cartesian → frenet → cartesian 왕복이 원래 점을 복원 — 두 변환이 역함수."""
    max_err = 0.0
    for i in _SAMPLE_I:
        th = _seg_heading(i)
        tx, ty = math.cos(th), math.sin(th)
        nx, ny = -math.sin(th), math.cos(th)
        seg_len = _seg_len(i)
        for frac in (0.3, 0.7):
            for d in (-4.0, -1.5, 0.0, 2.5, 4.0):
                px = _CX[i] + frac * seg_len * tx + d * nx
                py = _CY[i] + frac * seg_len * ty + d * ny
                s, dd = cartesian_to_frenet(px, py, _CX, _CY, _CS)
                rx, ry, _ = frenet_to_cartesian(s, dd, _CX, _CY, _CS)
                max_err = max(max_err, math.hypot(rx - px, ry - py))
    assert max_err < 1e-3, f"왕복 오차 {max_err:.2e} m — 변환이 역함수가 아님"
