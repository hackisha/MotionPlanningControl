"""주행 타겟 차량 — 타겟별 거동을 설정할 수 있는 plant.

이 파일은 검증 환경의 일부입니다. 수정하지 마세요.

각 타겟은 Frenet 좌표(s, d)에서 거동한다. 거동 종류에 따라 두 클래스를 둔다:
  - `TargetVehicle`     — Maneuver 명세 기반. 현재 'straight'(차선 유지)만 구현.
  - `TargetVehicle_LC`  — 차로변경 의도(-1=좌, 0=없음, +1=우)를 받아 3 차 곡선으로
                          옆 차선으로 횡 이동.

타겟 집합(fleet)은 타겟끼리의 간단한 지능을 담는다 — 거동 종류별로 두 클래스:
  - `TargetFleet`     — **속도 교환.** 같은 차선 뒤차가 앞차에 근접하면 두 차의
                        종방향 속도 s_d 를 맞바꾼다 (1D 탄성충돌과 동일 — 닿기 전
                        교환하므로 구조적으로 추돌 불가). `TargetVehicle` 용.
  - `TargetFleet_LC`  — **차로변경 추월.** 뒤차가 앞차에 근접하면 옆 차선으로
                        차로변경해 추월하고, 옆 차선이 막혀 변경 불가일 때만 앞차와
                        속도 교환으로 fallback. `TargetVehicle_LC` 용.

ego 의 planner 는 이 타겟들을 lanekeep(등속) 모델로 예측한다
(`prediction.predict_target_lanekeep`) — `TargetVehicle` 은 교환 순간을 빼면 등속이라
CV 와 잘 맞지만, `TargetVehicle_LC` 는 차로변경 중 'd 고정' 가정이 깨져 예측이
어긋난다. `predict_target_lanechange` 는 관측된 d_d 까지 활용해 그 괴리를 줄인다.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field

from track_map import LANE_WIDTH


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


@dataclass
class TargetVehicle_LC:
    """차로변경 매뉴버를 수행하는 타겟 차량 — 매뉴버 상태 기계를 가진다.

    `maneuver` 필드가 현재 거동 상태를 나타낸다 (각 자동차 위 화살표로 시각화):
      - "lane_keep"            차선 유지 — 횡 이동 없음.
      - "lane_change_pending"  차로변경을 원하지만 옆 차선이 막혀 *보류 중* — 횡
                               이동은 아직 없고, 옆 차선이 안전해지면 자동 시작.
      - "lane_change"          차로변경 진행 중 — LANE_WIDTH 만큼 `lc_duration` 동안
                               3 차 곡선(smoothstep, 양 끝 횡속도 0)으로 횡 이동.

    `TargetFleet_LC` 가 거동을 결정한다:
      `request_lane_change(dir)` → 매뉴버 "lane_change_pending" (방향 `lc_direction`).
      `start_lane_change()`      → 옆 차선이 안전해지면 fleet 가 호출 →
                                   "lane_change_pending" → "lane_change" 전환.
    `lc_direction` = 차로변경 방향: -1(좌) / 0(없음) / +1(우).

    planner 의 lanekeep 예측(`prediction.predict_target_lanekeep`)은 'd 고정'을
    가정하므로 차로변경 중인 이 타겟에는 예측이 어긋난다 — `record_gen_lanechange`
    가 그 예측-실제 괴리와, 횡속도 d_d 까지 활용하는 `predict_target_lanechange`
    의 개선을 시각화로 비교한다.
    """

    s: float                                  # Frenet 종방향 위치 [m]
    d: float                                  # Frenet 횡방향 위치 [m] (차선)
    s_d: float                                # 종방향 속도 [m/s]
    name: str = "target"
    d_d: float = 0.0                          # 횡방향 속도 [m/s]
    lc_duration: float = 3.0                  # 차로변경 1 회 소요 시간 [s]
    maneuver: str = "lane_keep"               # 현재 매뉴버 상태 (위 docstring 참조)
    lc_direction: int = 0                     # 차로변경 방향 -1/0/+1 (pending·진행 중)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self) -> None:
        self._lc_t = 0.0             # 차로변경 경과 시간 [s]
        self._lc_d0 = self.d         # 차로변경 시작 횡위치
        self._lc_d1 = self.d         # 차로변경 목표 횡위치

    @property
    def is_changing_lane(self) -> bool:
        """차로변경 매뉴버 진행 중이면 True."""
        return self.maneuver == "lane_change"

    @property
    def is_pending(self) -> bool:
        """차로변경 보류(대기) 중이면 True."""
        return self.maneuver == "lane_change_pending"

    def request_lane_change(self, direction: int) -> None:
        """차로변경 요청 — 매뉴버를 "lane_change_pending" 으로 (방향 저장).

        차선 유지 중일 때만 받아들인다 (이미 보류·진행 중이면 무시). 실제 시작은
        옆 차선이 안전해질 때 `TargetFleet_LC` 가 `start_lane_change` 로 한다.
        """
        if self.maneuver == "lane_keep" and direction != 0:
            self.maneuver = "lane_change_pending"
            self.lc_direction = direction

    def start_lane_change(self) -> None:
        """보류 중인 차로변경을 시작 — "lane_change_pending" → "lane_change"."""
        if self.maneuver != "lane_change_pending":
            return
        self.maneuver = "lane_change"
        self._lc_t = 0.0
        self._lc_d0 = self.d
        # -1(좌) → d 증가, +1(우) → d 감소
        self._lc_d1 = self.d - self.lc_direction * LANE_WIDTH

    def update(self, dt: float, track) -> None:
        """한 스텝 Frenet 상태 갱신 — "lane_change" 일 때만 3 차 곡선 횡 이동."""
        if self.maneuver == "lane_change":
            self._lc_t += dt
            tau = min(self._lc_t / self.lc_duration, 1.0)
            # 3 차 곡선 smoothstep — tau=0,1 에서 기울기 0 (양 끝 횡속도 0)
            shape = 3.0 * tau**2 - 2.0 * tau**3
            d_new = self._lc_d0 + (self._lc_d1 - self._lc_d0) * shape
            self.d_d = (d_new - self.d) / dt
            self.d = d_new
            if tau >= 1.0:                       # 차로변경 완료
                self.maneuver = "lane_keep"
                self.lc_direction = 0
                self.d_d = 0.0
        else:                                    # lane_keep / lane_change_pending
            self.d_d = 0.0                       # 횡 이동 없음

        self.s = (self.s + self.s_d * dt) % track.length
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


class TargetFleet_LC:
    """차로변경 타겟(`TargetVehicle_LC`) 집합 — 추월·랜덤·악의적 거동을 결정한다.

    매 스텝 다음 거동 규칙을 적용한다:

    **(1) 추월 — 같은 차선 뒤차가 앞차에 `OVERTAKE_GAP` 이내로 접근 + 더 빠르면:**
    뒤차에 차로변경을 요청한다(`request_lane_change` → 매뉴버 "lane_change_pending").
    추가로 옆 차선이 막혀 있으면 추돌 회피로 앞차와 종방향 속도 `s_d` 를 맞바꾼다.

    **(2) 악의적 끼어들기 — 자차(ego)가 옆 차선에서 뒤따라 다가오면, 차로변경이
    억제되는 범위(`SAFE_GAP`) *밖* 이면서 `MALICIOUS_RANGE` 이내일 때 `MALICIOUS_PROB`
    의 높은 확률로 자차 차선으로 끼어든다** (자차 앞을 가로막는 약간 악의적인 거동).
    자차 접근 한 번당 한 번만 roll 한다.

    **(3) 랜덤 주행 — 종방향 주변(`CLEAR_GAP` 이내)에 다른 차(타겟·ego 모두)가
    없는 상태가 `CLEAR_HOLD`(3 s) 이상 지속되면, 이후 `ROAM_PERIOD`(4 s) 마다 한 번
    랜덤하게 차로변경을 요청한다** (텅 빈 도로에서의 자유 주행).

    **(4) 보류 해소 — 차로변경 보류("lane_change_pending") 중인 타겟은 매 스텝 옆
    차선 안전을 재검사**해, 안전해지면 `start_lane_change` 로 차로변경을 시작한다.
    '막힘' 조건은 OR 두 가지: 옆 차선 `SAFE_GAP` 이내에 (a) 다른 타겟이 있음, 또는
    (b) 자차(ego)가 종거리상 있음. 둘 다 해소되면 보류가 풀려 차로변경이 재개된다.

    `TargetFleet`(항상 속도 교환)과 달리 추돌 불가가 구조적으로 보장되지는 않지만,
    추월(차로변경) 우선·교착 시 속도 교환으로 뒤차가 앞차를 들이받지 않는다.
    """

    OVERTAKE_GAP: float = 15.0     # 앞차와 이 거리 이내 + 접근 중이면 추월 시도 [m]
    SAFE_GAP: float = 10.0         # 옆 차선 이 거리 이내에 차 있으면 차로변경 불가 [m]
    CLEAR_GAP: float = 20.0        # 종방향 이 거리 이내에 차 없으면 '주변 빔' [m]
    CLEAR_HOLD: float = 3.0        # 이만큼 계속 비어 있어야 랜덤 주행 시작 [s]
    ROAM_PERIOD: float = 4.0       # 랜덤 주행 중 차로변경 의도 roll 주기 [s]
    MALICIOUS_RANGE: float = 28.0  # 뒤따르는 자차를 악의적 끼어들기로 감지하는 거리 [m]
    MALICIOUS_PROB: float = 0.8    # 감지 시 자차 차선으로 끼어들 확률

    def __init__(self, targets: list[TargetVehicle_LC], track) -> None:
        self.targets = targets
        self.track = track
        # 타겟별 상태 타이머·플래그 (id → 값) — 거동 결정은 fleet 책임이므로
        # 타겟 클래스가 아닌 fleet 가 보관.
        self._clear_time: dict[int, float] = {id(tg): 0.0 for tg in targets}
        self._roam_timer: dict[int, float] = {id(tg): 0.0 for tg in targets}
        # 악의적 끼어들기 — 자차 접근 한 번당 한 번만 roll 하기 위한 재무장 플래그.
        self._malicious_armed: dict[int, bool] = {id(tg): True for tg in targets}
        for tg in targets:
            tg.sync_cartesian(track)

    def update_all(self, dt: float, ego_s: float | None = None,
                   ego_d: float | None = None,
                   ego_s_d: float | None = None) -> None:
        """거동(추월·악의적·랜덤)을 먼저 결정한 뒤 각 타겟을 한 스텝 전진.

        Args:
            dt: 시간 스텝 [s].
            ego_s, ego_d: 자차의 Frenet 종·횡 위치. 주면 추월 fallback 의 'ego 가
                옆 차선에 있음' 조건, 랜덤 주행의 '주변 빔' 판정, 악의적 끼어들기
                판정에 자차도 포함된다.
            ego_s_d: 자차의 Frenet 종방향 속도. 악의적 끼어들기의 '자차가 접근
                중'(타겟보다 빠름) 판정에 쓰인다.
        """
        self._resolve_overtakes(ego_s, ego_d)
        self._resolve_malicious(ego_s, ego_d, ego_s_d)
        self._resolve_random_roam(dt, ego_s)
        self._resolve_pending(ego_s, ego_d)
        for tg in self.targets:
            tg.update(dt, self.track)

    def states(self) -> list[tuple[float, float, float]]:
        """planner 예측 입력용 — 모든 타겟의 (s, d, s_d)."""
        return [tg.state() for tg in self.targets]

    def _resolve_overtakes(self, ego_s: float | None = None,
                           ego_d: float | None = None) -> None:
        """근접한 뒤차마다 차로변경을 요청하고, 옆 차선이 막혔으면 속도 교환."""
        length = self.track.length
        used: set[int] = set()
        for tg in self.targets:
            # 차로변경 진행 중이거나 이미 속도교환한 타겟은 건너뜀
            if tg.is_changing_lane or id(tg) in used:
                continue
            # 같은 차선에서 cyclic 으로 바로 앞차 찾기 (최소 양의 gap)
            leader, best_gap = None, float("inf")
            for other in self.targets:
                if other is tg or not _same_lane(tg, other):
                    continue
                gap = (other.s - tg.s) % length
                if 0.0 < gap < best_gap:
                    best_gap, leader = gap, other
            # 근접 + 접근 중(뒤차가 더 빠름) 이 아니면 거동 변화 없음
            if leader is None or best_gap >= self.OVERTAKE_GAP or tg.s_d <= leader.s_d:
                continue
            # 추월 욕구 — 차로변경 요청 (매뉴버 보류 상태로). 옆 차선이 비면 곧
            # _resolve_pending 이 시작하고, 막혔으면 보류 유지된다.
            tg.request_lane_change(-1 if tg.d < 0.0 else +1)
            # 옆 차선이 막혀 차로변경이 곧장 불가하면, 추돌 회피로 앞차와 속도 교환
            if (not self._other_lane_clear(tg, ego_s, ego_d)
                    and id(leader) not in used):
                tg.s_d, leader.s_d = leader.s_d, tg.s_d
                used.add(id(tg))
                used.add(id(leader))

    def _resolve_malicious(self, ego_s: float | None, ego_d: float | None,
                           ego_s_d: float | None) -> None:
        """자차가 뒤에서 다가오면 (억제 범위 밖에서) 높은 확률로 자차 차선에 끼어든다.

        조건 — 자차가 (a) 타겟의 옆 차선(= 타겟이 끼어들 차선)에 있고, (b) 타겟 뒤쪽
        `SAFE_GAP`(억제 범위) 밖 ~ `MALICIOUS_RANGE` 이내, (c) 타겟보다 빨라 접근 중.
        세 조건이 처음 성립한 순간 한 번만 `MALICIOUS_PROB` 확률로 차로변경 요청.
        """
        if ego_s is None or ego_d is None or ego_s_d is None:
            return
        length = self.track.length
        for tg in self.targets:
            ego_other_lane = (ego_d >= 0.0) != (tg.d >= 0.0)
            gap_behind = (tg.s - ego_s) % length        # 자차→타겟 종거리 (자차가 뒤)
            in_zone = self.SAFE_GAP < gap_behind < self.MALICIOUS_RANGE
            approaching = ego_s_d > tg.s_d
            if not (ego_other_lane and in_zone and approaching):
                self._malicious_armed[id(tg)] = True    # 접근 종료 → 다음 접근에 재무장
            elif self._malicious_armed[id(tg)] and tg.maneuver == "lane_keep":
                self._malicious_armed[id(tg)] = False   # 한 접근당 한 번만 roll
                if random.random() < self.MALICIOUS_PROB:
                    tg.request_lane_change(-1 if tg.d < 0.0 else +1)  # 자차 차선으로

    def _resolve_random_roam(self, dt: float, ego_s: float | None = None) -> None:
        """주변이 오래 비어 있는 타겟에 주기적으로 랜덤 차로변경을 요청한다."""
        length = self.track.length
        for tg in self.targets:
            # 종방향 주변(타겟·ego)에 다른 차가 CLEAR_GAP 이내인지
            others_s = [o.s for o in self.targets if o is not tg]
            if ego_s is not None:
                others_s.append(ego_s)
            clear = all(
                min((s - tg.s) % length, (tg.s - s) % length) >= self.CLEAR_GAP
                for s in others_s
            )
            self._clear_time[id(tg)] = self._clear_time[id(tg)] + dt if clear else 0.0

            # CLEAR_HOLD 이상 비어 있으면 ROAM_PERIOD 마다 랜덤 차로변경 roll
            if self._clear_time[id(tg)] >= self.CLEAR_HOLD:
                self._roam_timer[id(tg)] += dt
                if self._roam_timer[id(tg)] >= self.ROAM_PERIOD:
                    self._roam_timer[id(tg)] = 0.0
                    if tg.maneuver == "lane_keep":
                        direction = random.choice((-1, 0, 1))
                        # 트랙(2 차선) 밖으로 나가는 방향이면 무시
                        if direction != 0 and abs(tg.d - direction * LANE_WIDTH) < LANE_WIDTH:
                            tg.request_lane_change(direction)
            else:
                self._roam_timer[id(tg)] = 0.0

    def _resolve_pending(self, ego_s: float | None = None,
                         ego_d: float | None = None) -> None:
        """보류("lane_change_pending") 중인 타겟 — 옆 차선이 안전하면 차로변경 시작."""
        for tg in self.targets:
            if tg.is_pending and self._other_lane_clear(tg, ego_s, ego_d):
                tg.start_lane_change()

    def _other_lane_clear(self, tg: TargetVehicle_LC, ego_s: float | None = None,
                          ego_d: float | None = None) -> bool:
        """tg 가 옆 차선으로 변경 시 `SAFE_GAP` 이내에 다른 차가 없으면 True.

        다른 타겟뿐 아니라 자차(ego)도 옆 차선 종거리 `SAFE_GAP` 이내면 막힘으로 본다.
        """
        length = self.track.length
        for other in self.targets:
            if other is tg or _same_lane(tg, other):
                continue                          # 같은 차선 타겟은 차로변경과 무관
            gap = (other.s - tg.s) % length
            gap = min(gap, length - gap)          # 폐루프 최단 종방향 거리
            if gap < self.SAFE_GAP:
                return False
        # 자차가 tg 의 옆 차선에 있고 종거리상 가까우면 차로변경 불가
        if ego_s is not None and ego_d is not None:
            if (ego_d >= 0.0) != (tg.d >= 0.0):   # ego 가 tg 의 옆 차선에 위치
                gap = (ego_s - tg.s) % length
                gap = min(gap, length - gap)
                if gap < self.SAFE_GAP:
                    return False
        return True


def _same_lane(a, b) -> bool:
    """두 타겟이 같은 차선(횡위치 d 의 같은 쪽)에 있는지."""
    return (a.d >= 0.0) == (b.d >= 0.0)
