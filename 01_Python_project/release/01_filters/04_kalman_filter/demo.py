"""Kalman Filter demo — 상수 truth 5.0 + N(0, 1) 노이즈 측정 (plotly, 브라우저 렌더).

실행 전 `kalman_filter.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 `step(...)` 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from kalman_filter import KalmanFilter


def main() -> None:
    rng = np.random.default_rng(0)
    n = 500
    truth = 5.0
    samples = truth + rng.normal(loc=0.0, scale=1.0, size=n)

    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.1, r=0.9, p0=10.0)
    estimate = np.array([kf.step(z, control_input=0.0) for z in samples])

    fig = go.Figure()
    fig.add_hline(y=truth, line=dict(color="black", dash="dash"), annotation_text="true value")
    fig.add_trace(
        go.Scatter(y=samples, mode="markers", name="measurements",
                   marker=dict(color="red", opacity=0.4, size=4))
    )
    fig.add_trace(
        go.Scatter(y=estimate, mode="lines", name="Kalman estimate",
                   line=dict(color="blue", width=2))
    )
    fig.update_layout(
        title="Kalman Filter — scalar (q=0.01, r=1.0)",
        xaxis_title="step",
        yaxis_title="value",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()
