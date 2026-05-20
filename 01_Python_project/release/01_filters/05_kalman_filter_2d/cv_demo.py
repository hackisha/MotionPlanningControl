"""Kalman 2D demo — Constant Velocity (등속 운동).

위치 truth = v_truth · t (등속 직선), 측정은 위치만 + N(0, 0.5).
2D 칼만으로 위치와 속도를 동시에 추정.

실행 전 `kalman_filter_2d.py` 의 `# TODO` 를 구현해야 동작합니다.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from kalman_filter_2d import KalmanFilter2D
from plotly.subplots import make_subplots


def main() -> None:
    rng = np.random.default_rng(0)
    dt = 0.1
    n = 200
    v_truth = 2.0

    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    C = np.array([1.0, 0.0])
    # [튜닝] Q / R 은 0 으로 초기화 — 학생이 직접 값을 골라 채워야 함.
    # test_*.py 의 값은 변경 X (합격 기준).
    Q = np.diag([0.0, 0.0])
    R = 0.0

    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R)

    t = np.arange(n) * dt
    truth_pos = v_truth * t
    samples = truth_pos + rng.normal(0, 0.5, size=n)

    pos_est = np.zeros(n)
    vel_est = np.zeros(n)
    for k in range(n):
        state = kf.step(measurement=samples[k], control_input=0.0)
        pos_est[k] = state[0]
        vel_est[k] = state[1]

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("Position", "Velocity"))
    fig.add_trace(go.Scatter(x=t, y=truth_pos, mode="lines", name="truth pos",
                             line=dict(color="black", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=samples, mode="markers", name="measurements",
                             marker=dict(color="gray", opacity=0.4, size=4)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=pos_est, mode="lines", name="estimated pos",
                             line=dict(color="red", width=2)), row=1, col=1)
    fig.add_hline(y=v_truth, line=dict(color="black", dash="dash"),
                  annotation_text="truth vel", row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=vel_est, mode="lines", name="estimated vel",
                             line=dict(color="blue", width=2)), row=2, col=1)
    fig.update_xaxes(title_text="t (s)", row=2, col=1)
    fig.update_yaxes(title_text="position", row=1, col=1)
    fig.update_yaxes(title_text="velocity", row=2, col=1)
    fig.update_layout(
        title="Kalman 2D — Constant Velocity (truth v = 2.0 m/s)",
        template="plotly_white",
        height=600,
    )
    fig.show()


if __name__ == "__main__":
    main()
