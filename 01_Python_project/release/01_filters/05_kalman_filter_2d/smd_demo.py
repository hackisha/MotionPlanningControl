"""Kalman 2D demo — Spring-Mass-Damper (자유 진동).

진짜 시스템: m·ẍ + b·ẋ + k·x = 0 (자유 진동, x0 = 1.0)
이산 시간:  x_{k+1} = A·x_k with A = [[1, dt], [-dt·k/m, 1 - dt·b/m]]

위치만 노이즈 측정. 칼만으로 위치 + 속도 추정.

실행 전 `kalman_filter_2d.py` 의 `# TODO` 를 구현해야 동작합니다.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from kalman_filter_2d import KalmanFilter2D
from plotly.subplots import make_subplots


def simulate_truth(dt: float, n: int, m: float, k: float, b: float,
                   x0_pos: float, x0_vel: float) -> np.ndarray:
    """SMD 시스템의 결정론적 시뮬 (노이즈 없음). 자유 진동 (u=0)."""
    A = np.array([[1.0, dt], [-dt * k / m, 1.0 - dt * b / m]])
    truth = np.zeros((n, 2))
    state = np.array([x0_pos, x0_vel])
    for i in range(n):
        truth[i] = state
        state = A @ state
    return truth


def main() -> None:
    rng = np.random.default_rng(0)
    dt = 0.1
    n = 300
    m, k, b = 10.0, 100.0, 2.0

    truth = simulate_truth(dt, n, m, k, b, x0_pos=1.0, x0_vel=0.0)
    truth_pos = truth[:, 0]
    truth_vel = truth[:, 1]
    samples = truth_pos + rng.normal(0, 0.1, size=n)

    A = np.array([[1.0, dt], [-dt * k / m, 1.0 - dt * b / m]])
    B = np.array([0.0, dt / m])
    C = np.array([1.0, 0.0])
    # [튜닝] Q / R 은 0 으로 초기화 — 학생이 직접 값을 골라 채워야 함.
    # test_*.py 의 값은 변경 X (합격 기준).
    Q = np.diag([0.0, 0.0])
    R = 0.0
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R, x0=np.array([1.0, 0.0]))

    pos_est = np.zeros(n)
    vel_est = np.zeros(n)
    for i in range(n):
        state = kf.step(measurement=samples[i], control_input=0.0)
        pos_est[i] = state[0]
        vel_est[i] = state[1]

    t = np.arange(n) * dt
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("Position", "Velocity"))
    fig.add_trace(go.Scatter(x=t, y=truth_pos, mode="lines", name="truth pos",
                             line=dict(color="black", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=samples, mode="markers", name="measurements",
                             marker=dict(color="gray", opacity=0.4, size=4)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=pos_est, mode="lines", name="estimated pos",
                             line=dict(color="red", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=truth_vel, mode="lines", name="truth vel",
                             line=dict(color="black", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=vel_est, mode="lines", name="estimated vel",
                             line=dict(color="blue", width=2)), row=2, col=1)
    fig.update_xaxes(title_text="t (s)", row=2, col=1)
    fig.update_yaxes(title_text="position", row=1, col=1)
    fig.update_yaxes(title_text="velocity", row=2, col=1)
    fig.update_layout(
        title=f"Kalman 2D — Spring-Mass-Damper (m={m}, k={k}, b={b})",
        template="plotly_white",
        height=600,
    )
    fig.show()


if __name__ == "__main__":
    main()
