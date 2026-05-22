"""Closed-loop integration glue — measure → filter → control → actuate.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


def closed_loop_step(
    plant, estimator, controller, target: float,
) -> tuple[float, float, float, float]:
    """One step of closed-loop simulation with measurement filter.

    Order matters: estimator must filter the noisy measurement BEFORE the
    controller sees it (controller takes the estimate, not the raw measurement).

    Returns:
        (y_true, y_measure, y_estimate, u)
    """
    # TODO: 다음 4단계를 순서대로 구현하시오.
    # 1) y_measure = plant.measure()        # 노이즈 포함 관측
    # 2) y_estimate = estimator.step(y_measure)  # LPF 평활
    # 3) u = controller.step(target, y_estimate) # PID 는 추정값 사용 (raw 노이즈 ❌)
    # 4) plant.step(u)                      # actuator → 동역학 한 스텝
    # 반환 4-tuple: (plant.y, y_measure, y_estimate, u)  ← 모두 float
    raise NotImplementedError
