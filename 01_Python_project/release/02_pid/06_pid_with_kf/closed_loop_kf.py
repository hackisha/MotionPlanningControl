"""Closed-loop integration glue — measure → KF(measure, prev_u) → control → actuate.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations


def closed_loop_step(
    plant, estimator, controller, target: float, prev_u: float,
) -> tuple[float, float, float, float]:
    """One step of closed-loop simulation with KF state estimator.

    KF needs the previous control input (prev_u) for its prediction step.
    Caller (driver loop) is responsible for tracking prev_u across iterations.

    Returns:
        (y_true, y_measure, y_estimate_position, u)
    """
    # TODO: 다음 5단계를 순서대로 구현하시오.
    # 1) y_measure = plant.measure()
    # 2) state = estimator.step(y_measure, prev_u)   # KF 는 control input 도 받음
    # 3) y_estimate = float(state[0])                # state[0]=위치, state[1]=속도
    # 4) u = controller.step(target, y_estimate)
    # 5) plant.step(u)
    # 반환 4-tuple: (plant.y, y_measure, y_estimate, u)
    raise NotImplementedError
