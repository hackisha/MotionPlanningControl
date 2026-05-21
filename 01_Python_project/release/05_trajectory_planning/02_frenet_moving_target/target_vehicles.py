"""주행 타겟 차량 — 타겟별 매뉴버를 설정할 수 있는 plant.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

각 타겟은 Maneuver 명세를 들고 Frenet 좌표(s, d)에서 거동한다. 현재는 'straight'
(차선 유지) 매뉴버만 구현되어 있으며, 타겟마다 다른 매뉴버를 줄 수 있도록 구조를
잡아 두었다 (추후 'lane_change' / 'accel' 등으로 확장 예정).

**TargetFleet — 타겟끼리의 간단한 지능 (속도 교환).** 같은 차선에서 뒤차가 앞차에
근접하면 두 차의 종방향 속도 s_d 를 맞바꾼다. 이는 1D 탄성충돌(같은 질량)에서 두
속도가 교환되는 것과 동일 — 닿기 전에 교환하므로 타겟끼리는 구조적으로 추돌하지
않는다. 가속도 모델·파라미터가 없어 매우 가볍다.

ego 의 planner 는 이 타겟들을 CV(등속) 모델로 예측한다
(frenet_planner.predict_target_cv) — 타겟은 교환 순간을 빼면 등속이라 CV 와 잘 맞는다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Maneuver:
    """타겟 차량의 주행 매뉴버 명세.

    kind 별로 TargetVehicle.update 의 거동이 갈린다:
      - "straight": 차선 유지(d 고정). 종방향 속도는 그대로 두되, TargetFleet 가
        근접 시 다른 타겟과 교환할 수 있다.

    확장 슬롯 (다음 단계) — "lane_change"(목표 차선 + 전환 시간),
    "accel"(가감속 프로파일) 등을 같은 dataclass 에 필드로 추가한다.
    """

    kind: str = "straight"


@dataclass
class TargetVehicle:
    """Frenet 좌표에서 거동하는 타겟 차량 한 대."""

    s: float                                  # Frenet 종방향 위치 [m]
    d: float                                  # Frenet 횡방향 위치 [m] (차선)
    s_d: float                                # 종방향 속도 [m/s]
    maneuver: Maneuver = field(default_factory=Maneuver)
    name: str = "target"
    d_d: float = 0.0                          # 횡방향 속도 [m/s]
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def update(self, dt: float, track) -> None:
        """매뉴버에 따라 한 스텝 Frenet 상태를 갱신하고 Cartesian 으로 동기화."""
        if self.maneuver.kind == "straight":
            self.d_d = 0.0                    # 차선 유지
        else:
            raise ValueError(f"미지원 매뉴버: {self.maneuver.kind!r}")
        self.s = (self.s + self.s_d * dt) % track.length
        self.d = self.d + self.d_d * dt
        self.sync_cartesian(track)

    def sync_cartesian(self, track) -> None:
        """현재 Frenet (s, d) 를 전역 (x, y, yaw) 로 동기화."""
        self.x, self.y, self.yaw = track.to_cartesian(self.s, self.d)

    def state(self) -> tuple[float, float, float]:
        """planner 예측 입력용 상태 — (s, d, s_d)."""
        return (self.s, self.d, self.s_d)


class TargetFleet:
    """타겟 차량 집합 — 매 스텝 '속도 교환'으로 타겟끼리 추돌을 막는다.

    같은 차선에서 뒤차가 앞차에 EXCHANGE_GAP 이내로 접근하면서 더 빠르면(접근 중),
    두 차의 종방향 속도 s_d 를 맞바꾼다. 교환 직후 앞차가 더 빨라져 둘은 다시 벌어지므로
    같은 쌍이 한 스텝에 반복 교환되지 않고, 타겟끼리는 절대 닿지 않는다.
    """

    EXCHANGE_GAP: float = 10.0     # 이 거리 이내 + 접근 중이면 속도 교환 [m]

    def __init__(self, targets: list[TargetVehicle], track) -> None:
        self.targets = targets
        self.track = track
        for tg in targets:
            tg.sync_cartesian(track)

    def update_all(self, dt: float) -> None:
        """속도 교환을 먼저 해소한 뒤 각 타겟을 한 스텝 전진."""
        self._resolve_exchanges()
        for tg in self.targets:
            tg.update(dt, self.track)

    def states(self) -> list[tuple[float, float, float]]:
        """planner 예측 입력용 — 모든 타겟의 (s, d, s_d)."""
        return [tg.state() for tg in self.targets]

    def _resolve_exchanges(self) -> None:
        """근접한 같은 차선 앞·뒤차 쌍의 s_d 를 교환 (차당 한 스텝 최대 1회)."""
        length = self.track.length
        used: set[int] = set()
        for tg in self.targets:
            if id(tg) in used:
                continue
            # 같은 차선에서 cyclic 으로 바로 앞차 찾기 (최소 양의 gap)
            leader, best_gap = None, float("inf")
            for other in self.targets:
                if other is tg or not _same_lane(tg, other):
                    continue
                gap = (other.s - tg.s) % length
                if 0.0 < gap < best_gap:
                    best_gap, leader = gap, other
            # 근접 + 접근 중(뒤차가 더 빠름) 이면 속도 교환
            if (leader is not None and id(leader) not in used
                    and best_gap < self.EXCHANGE_GAP and tg.s_d > leader.s_d):
                tg.s_d, leader.s_d = leader.s_d, tg.s_d
                used.add(id(tg))
                used.add(id(leader))


def _same_lane(a: TargetVehicle, b: TargetVehicle) -> bool:
    """두 타겟이 같은 차선(횡위치 d 의 같은 쪽)에 있는지."""
    return (a.d >= 0.0) == (b.d >= 0.0)
