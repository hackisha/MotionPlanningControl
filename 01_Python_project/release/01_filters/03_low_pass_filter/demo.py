"""Low-Pass Filter demo — sin(t) + N(0, 0.5) 노이즈, α=0.9 (plotly, 브라우저 렌더).

실행 전 `low_pass_filter.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 `step(x)` 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from low_pass_filter import LowPassFilter


def main() -> None:
    rng = np.random.default_rng(0)
    t = np.linspace(0, 4 * np.pi, 400)
    truth = np.sin(t)
    noise = rng.normal(loc=0.0, scale=0.5, size=len(t))
    samples = truth + noise

    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준).
    # alpha=0 은 pass-through (필터 효과 0) — 작동 가능한 seed 값으로 시작.
    lpf = LowPassFilter(alpha=0.9)
    estimate = np.array([lpf.step(s) for s in samples])

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(x=t, y=truth, mode="lines", name="ground truth (sin)",
                   line=dict(color="black", dash="dash"))
    )
    fig.add_trace(
        go.Scatter(x=t, y=samples, mode="markers", name="samples",
                   marker=dict(color="red", opacity=0.4, size=4))
    )
    fig.add_trace(
        go.Scatter(x=t, y=estimate, mode="lines", name="estimate (α=0.9)",
                   line=dict(color="blue", width=2))
    )
    fig.update_layout(
        title="Low-Pass Filter — α = 0.9",
        xaxis_title="t (rad)",
        yaxis_title="value",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()
