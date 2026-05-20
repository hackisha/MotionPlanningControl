"""Student-supplied PID gains for the tuning challenge.

과제 명세는 problem.html 참조.
"""
from __future__ import annotations

# TODO: 아래 plant 에서 폐루프가 수렴하도록 KP, KD, KI 값을 결정하시오.
# - plant: actuation_gain=0.5 (제어 권한 약함) + disturbance=0.3 (상수 외란)
# - PID 알고리즘은 pid_controller.py 에 이미 구현되어 있음 (수정 X)
# - demo.py 를 실행해 응답 모양을 보고 직관을 잡은 뒤 값을 조절하는 것을 권장
# - 시작점 힌트: 모두 작은 양수부터 (예: 0.5) 시작해 응답 보고 키워가기
KP: float = 0.0
KD: float = 0.0
KI: float = 0.0
