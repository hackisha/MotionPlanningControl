"""Frenet ↔ Cartesian 좌표 변환 — 열린 곡선 도로용.

Chapter 5 / 01_frenet_transform 의 구현 대상 모듈.

도로 중심선(centerline)을 따라간 호 길이 `s` 와, 중심선에서 옆으로 벗어난
부호 있는 거리 `d` 로 위치를 기술하는 **Frenet 좌표계**와, 전역 (x, y)
**Cartesian 좌표계** 사이를 오간다.

규약
  - `s` : 중심선 시작점부터의 누적 호 길이 [m]. 이 도로는 s 원점(s=0)이 자차의
          종방향 위치 → s ∈ [-50, +100].
  - `d` : 중심선 기준 횡 offset [m]. **d > 0 = 진행 방향 기준 왼쪽**.
  - 열린(open) 도로 — s 는 양 끝에서 clamp 한다.

중심선 데이터(`cx`, `cy`, `cs`)는 `road.Road` 가 제공한다 — 각각 중심선 점들의
x 좌표 / y 좌표 / 누적 호 길이 배열.

구현 과제 (# TODO):
  - cartesian_to_frenet  (전역 → Frenet)
  - frenet_to_cartesian  (Frenet → 전역)
"""
from __future__ import annotations

import math  # noqa: F401  (변환 함수 구현 시 사용)

import numpy as np  # noqa: F401  (변환 함수 구현 시 사용)


def cartesian_to_frenet(x: float, y: float,
                        cx: np.ndarray, cy: np.ndarray, cs: np.ndarray
                        ) -> tuple[float, float]:
    """전역 (x, y) → Frenet (s, d).

    중심선의 모든 세그먼트에 점을 정사영해 가장 가까운 발(foot)을 찾고,
    그 발까지의 호 길이를 s, 발에서 점까지의 부호 있는 수직 거리를 d 로 한다.
    """
    # TODO: 전역 (x, y) 를 Frenet (s, d) 로 변환해 반환하세요.
    #   - 중심선의 각 세그먼트에 점을 정사영 → 보간 파라미터 t 를 [0, 1] 로 clamp.
    #   - 가장 가까운 발(foot)을 갖는 세그먼트를 선택.
    #   - s = 그 발까지의 누적 호 길이, d = 발→점 의 부호 있는 수직 거리.
    #   - 부호: 진행 방향 기준 왼쪽이 +, 오른쪽이 - (외적 z 성분 부호).
    raise NotImplementedError("cartesian_to_frenet 를 구현하세요")


def frenet_to_cartesian(s: float, d: float,
                        cx: np.ndarray, cy: np.ndarray, cs: np.ndarray
                        ) -> tuple[float, float, float]:
    """Frenet (s, d) → 전역 (x, y, heading). s 는 도로 양 끝에서 clamp."""
    # TODO: Frenet (s, d) 를 전역 (x, y, heading) 으로 변환해 반환하세요.
    #   - s 를 도로 범위 [cs[0], cs[-1]] 로 clamp.
    #   - s 가 속한 중심선 세그먼트를 찾는다 (numpy.searchsorted 활용).
    #   - 세그먼트 시작점에서 호 길이만큼 전진한 중심선 위 기준점을 구하고,
    #     왼쪽 법선(heading + 90°) 방향으로 d 만큼 이동.
    raise NotImplementedError("frenet_to_cartesian 를 구현하세요")
