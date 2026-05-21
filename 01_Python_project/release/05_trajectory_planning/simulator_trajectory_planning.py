"""Rerun replay player for trajectory-planning demo records.

chapter 5 도 chapter 3 와 동일 JSON schema (v2: actors / lanes / scalars /
dynamic_paths / dynamic_points). 이 파일은 chapter 3 simulator 의 thin wrapper —
재사용 + 기본 스캔 경로만 chapter 5 로.

Usage (git root cwd 기준 — 아래 예시에서 이 파일 경로를 SIM 으로 표기):
  SIM =
    01_Python_project/release/05_trajectory_planning/
    simulator_trajectory_planning.py

    uv run python SIM                  # 스크립트 폴더 하위 모든 record*.json 멀티 로드
    uv run python SIM <파일|디렉토리>   # 특정 record.json 또는 디렉토리만
    uv run python SIM --camera fixed   # 초기 카메라 (기본 follow=ego 추종)
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
        description="05_trajectory_planning record*.json 을 Rerun viewer 로 재생 "
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
