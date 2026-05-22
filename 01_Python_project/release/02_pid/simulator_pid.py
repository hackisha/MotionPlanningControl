"""Rerun replay player for PID demo records.

chapter 2 의 1D plant 상태 (y) 를 차량의 Y 좌표로 매핑하고, X 는 시각용 일정 vx
로 전진. 결과는 chapter 3 와 동일 3D 차량 렌더링 — y 가 차로 중앙(Y=0)으로
수렴하는 모습을 차량이 횡 방향으로 라인 위에 정렬하는 식으로 본다.

JSON schema 는 chapter 3 (simulator_vehicle_control) 와 동일 v2 — actors / lanes
/ scalars / dynamic_points / dynamic_paths. 이 파일은 chapter 3 simulator 의 thin
wrapper (재사용 + 기본 스캔 경로만 chapter 2 로).

Usage (git root cwd 기준):
    # 1) 인자 없이 - 스크립트 폴더 하위 모든 record*.json 멀티 로드
    uv run python 01_Python_project/solutions/02_pid/simulator_pid.py

    # 2) 파일 또는 디렉토리 지정
    uv run python 01_Python_project/solutions/02_pid/simulator_pid.py \\
        01_Python_project/solutions/02_pid/01_p_controller/

    # 카메라 모드 (기본 follow)
    uv run python 01_Python_project/solutions/02_pid/simulator_pid.py \\
        --camera fixed
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# chapter 3 simulator 재사용 — 동일 JSON schema, 동일 3D 렌더링.
sys.path.insert(0, str(Path(__file__).parent.parent / "03_vehicle_control"))
from simulator_vehicle_control import _find_records, replay_records  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="02_pid record*.json 을 Rerun viewer 로 재생 (chapter 3 simulator 재사용)")
    parser.add_argument(
        "path", nargs="?", default=None,
        help="record.json 파일 또는 디렉토리 (생략 시 스크립트 폴더 하위 스캔)")
    parser.add_argument(
        "--camera", choices=("follow", "fixed"), default="follow",
        help="초기 카메라 (기본 follow=ego 추종)")
    args = parser.parse_args()

    arg = Path(args.path) if args.path else Path(__file__).parent
    if not arg.exists():
        print(f"경로 없음: {arg}", file=sys.stderr)
        sys.exit(1)

    records = [arg] if arg.is_file() else _find_records(arg)
    if not records:
        print(f"record*.json 을 찾지 못함: {arg}\n"
              f"  먼저 각 모듈 record_gen.py 를 실행해 record.json 을 생성하세요.",
              file=sys.stderr)
        sys.exit(1)

    replay_records(records, camera=args.camera)


if __name__ == "__main__":
    main()
