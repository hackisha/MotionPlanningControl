"""Moving Average Filter demo — N(5, 1) 노이즈, window=15 (plotly, 브라우저 렌더).

실행 전 `moving_average_filter.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 `step(x)` 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from moving_average_filter import MovingAverageFilter


def main() -> None:
    rng = np.random.default_rng(0)
    truth = 5.0
    samples = rng.normal(loc=truth, scale=1.0, size=200)

    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준).
    # window 은 0 으로 초기화 — 학생이 직접 값을 골라 채워야 함.
    maf = MovingAverageFilter(window=0)
    estimate = np.array([maf.step(s) for s in samples])

    fig = go.Figure()
    fig.add_hline(y=truth, line=dict(color="black", dash="dash"), annotation_text="true mean")
    fig.add_trace(
        go.Scatter(y=samples, mode="markers", name="samples",
                   marker=dict(color="red", opacity=0.4))
    )
    fig.add_trace(
        go.Scatter(y=estimate, mode="lines", name="estimate (window=15)",
                   line=dict(color="blue", width=2))
    )
    fig.update_layout(
        title="Moving Average Filter — sliding window mean",
        xaxis_title="sample index",
        yaxis_title="value",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()
