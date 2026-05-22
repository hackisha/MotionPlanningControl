"""Rerun replay player for path-planning demo records.

chapter 4 도 chapter 3 와 동일 JSON schema (v2: actors / lanes / scalars /
dynamic_paths / dynamic_points). 이 파일은 chapter 3 simulator 의 thin wrapper —
재사용 + 기본 스캔 경로만 chapter 4 로.

Usage (git root cwd 기준):
    # 1) 인자 없이 - 스크립트 폴더 하위 모든 record*.json 멀티 로드
    uv run python 01_Python_project/solutions/04_path_planning/simulator_path_planning.py

    # 2) 파일 또는 디렉토리 지정
    uv run python 01_Python_project/solutions/04_path_planning/simulator_path_planning.py \\
        01_Python_project/solutions/04_path_planning/01_both_lane/

    # 카메라 모드 (기본 follow)
    uv run python 01_Python_project/solutions/04_path_planning/simulator_path_planning.py \\
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
        description="04_path_planning record*.json 을 Rerun viewer 로 재생 "
                    "(chapter 3 simulator 재사용)")
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
