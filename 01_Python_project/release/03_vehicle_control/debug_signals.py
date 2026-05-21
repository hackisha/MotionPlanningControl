"""DebugSignals — record_gen 에서 디버그 신호를 손쉽게 모으는 수집기.

디버그 신호는 record JSON 의 `debug_scalars` 필드로 저장된다. recording 에는
들어가지만 기본 blueprint 에는 표시되지 않는다 — Rerun viewer 에서 좌측 entity
패널의 `/debug/<name>` 을 골라 TimeSeriesView 를 직접 추가하면 분석할 수 있다.

이 파일은 여러 record_gen 이 공유하는 헬퍼다. **신호를 넣고 빼고 고치는 작업은
record_gen.py 의 `dbg.add(...)` 호출 한 줄만** 수정하면 된다 — 배열 선언이나
record dict 손댈 필요 없음. 이 파일 자체는 보통 수정 불필요.

사용 예 (record_gen.py):

    from debug_signals import DebugSignals

    dbg = DebugSignals()
    for i in range(steps):
        ...
        # ↓ 디버그 신호 추가 / 삭제 / 수정은 이 kwarg 들만 고치면 됨
        dbg.add(
            cross_track=ego.Y - road.lane2_center(ego.X),
            rel_vx=target.vx - ego.vx,
        )
    record["debug_scalars"] = dbg.to_debug_scalars(t)
"""
from __future__ import annotations


class DebugSignals:
    """이름별 디버그 시계열 수집기.

    매 스텝 `add(name=value, ...)` 로 신호를 넘기면 이름별 리스트에 누적한다.
    신호 한 종류를 넣고 빼려면 `add(...)` 호출의 kwarg 한 줄만 고치면 된다.
    """

    def __init__(self) -> None:
        self._series: dict[str, list[float]] = {}

    def add(self, **signals: float) -> None:
        """한 스텝의 디버그 신호들을 kwarg 로 받아 이름별로 누적.

        매 스텝 같은 신호 이름 집합을 넘겨야 시계열 길이가 일치한다.
        예) `dbg.add(cross_track=0.3, rel_vx=-1.2)`
        """
        for name, value in signals.items():
            self._series.setdefault(name, []).append(float(value))

    def names(self) -> list[str]:
        """현재까지 수집된 신호 이름 목록."""
        return list(self._series)

    def to_debug_scalars(self, t=None,
                         units: dict[str, str] | None = None) -> list[dict]:
        """record JSON 의 `debug_scalars` 필드 형태로 변환.

        Args:
            t: 시간축 시퀀스 (각 신호 value 와 길이가 같아야 함). None 이면
                0,1,2,… 정수 인덱스를 시간축으로 쓴다 (search-record 의 iteration 등).
            units: optional `{신호이름: 단위}` — 생략하거나 빠진 이름은 "-".

        Returns:
            `[{"name", "unit", "t", "value"}, ...]` — `debug_scalars` 에 그대로 대입.
        """
        units = units or {}
        out: list[dict] = []
        for name, values in self._series.items():
            t_list = (list(range(len(values))) if t is None
                      else [float(x) for x in t])
            out.append({"name": name, "unit": units.get(name, "-"),
                        "t": t_list, "value": values})
        return out
