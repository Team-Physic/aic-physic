#!/usr/bin/env python3
"""PortOffset dataset sample과 trial MCAP의 일치 여부를 offline 검증한다."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    """offline 검증에 필요한 dataset, rosbag, sample 선택 인자를 정의한다."""
    parser = argparse.ArgumentParser(
        description="Validate PortOffset dataset samples against their trial MCAP."
    )
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--rosbag", type=Path, required=True)
    parser.add_argument(
        "--sample-id",
        action="append",
        default=[],
        help="Validate only this sample ID; repeat for multiple samples.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="Output JSON path; defaults to <rosbag>/portoffset_validation.json.",
    )
    parser.add_argument(
        "--max-record-delay-ms",
        type=float,
        help=(
            "Fail when source image stamp to MCAP record time exceeds this value. "
            "Available only for bags recorded with ROS simulation time."
        ),
    )
    return parser


def main() -> int:
    """검증을 실행하고 JSON 보고서 및 process exit status를 반환한다."""
    args = build_parser().parse_args()
    if args.max_record_delay_ms is not None and (
        not math.isfinite(args.max_record_delay_ms)
        or args.max_record_delay_ms < 0
    ):
        print(
            "[validation] ERROR: --max-record-delay-ms must be finite "
            "and non-negative"
        )
        return 2
    try:
        from phy_data_collection.portoffset_validation import validate_trial
    except ModuleNotFoundError as exc:
        print(f"[validation] ERROR: missing runtime dependency: {exc.name}")
        return 2

    rosbag_dir = args.rosbag.expanduser().resolve()
    if rosbag_dir.is_file():
        rosbag_dir = rosbag_dir.parent
    report_path = (
        args.report.expanduser().resolve()
        if args.report
        else rosbag_dir / "portoffset_validation.json"
    )
    try:
        report = validate_trial(
            args.dataset_dir,
            rosbag_dir,
            set(args.sample_id) or None,
            args.max_record_delay_ms,
        )
    except Exception as exc:
        print(f"[validation] ERROR: {exc}")
        return 2
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    color = "\033[1;32m" if report["status"] == "PASS" else "\033[1;31m"
    reset = "\033[0m"
    print(
        f"{color}[validation] {report['status']}{reset}: "
        f"passed={report['passed_sample_count']}, "
        f"failed={report['failed_sample_count']}, "
        f"report={report_path}"
    )
    events = report["camera_event_validation"]
    delay = events["record_delay"]
    delay_text = (
        f"mean={delay['mean_ms']:.3f}ms, p95={delay['p95_ms']:.3f}ms, "
        f"max={delay['max_ms']:.3f}ms"
        if delay["available"]
        else "unavailable"
    )
    print(
        "[validation] camera events: "
        f"passed={events['passed_event_count']}, "
        f"failed={events['failed_event_count']}, "
        f"record_delay={delay_text}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
