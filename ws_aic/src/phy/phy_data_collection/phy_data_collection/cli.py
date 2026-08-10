"""PortOffset randomization runner의 역할별 CLI 정의."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .constants import (
    BOOLEAN_METAVAR,
    CLI_DEFAULTS,
    COLLECTION_POLICY_CHOICES,
    ENGINE_SETUP,
    PORT_ORDER_CHOICES,
    TRIAL_TIMEOUT_GRACE_S,
)


def _parse_bool(value: str) -> bool:
    """CLI의 true/false 문자열을 boolean으로 변환한다."""
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def _add_argument(
    parser: argparse.ArgumentParser,
    name: str,
    **kwargs: Any,
) -> None:
    """상수로 관리되는 기본값을 적용해 CLI 인자를 추가한다."""
    option = f"--{name.replace('_', '-')}"
    parser.add_argument(option, default=CLI_DEFAULTS[name], **kwargs)


def _add_trial_args(parser: argparse.ArgumentParser) -> None:
    """trial 개수, 순서, simulator 시작과 관련된 기본 인자를 추가한다."""
    _add_argument(parser, "trials", type=int)
    _add_argument(parser, "workers", type=int)
    _add_argument(parser, "seed", type=int)
    _add_argument(parser, "port_types")
    _add_argument(parser, "port_order", choices=PORT_ORDER_CHOICES)
    _add_argument(parser, "color_log", type=_parse_bool, metavar=BOOLEAN_METAVAR)
    _add_argument(parser, "samples_per_trial", type=int)
    _add_argument(parser, "time_limit_s", type=int)
    _add_argument(
        parser,
        "trial_timeout_s",
        type=float,
        help=f"Defaults to time-limit-s + {TRIAL_TIMEOUT_GRACE_S:g}s.",
    )
    _add_argument(parser, "distrobox")
    _add_argument(parser, "headless", action=argparse.BooleanOptionalAction)
    _add_argument(parser, "launch_rviz", type=_parse_bool, metavar=BOOLEAN_METAVAR)
    _add_argument(parser, "ros_domain_id_base", type=int)
    _add_argument(parser, "zenoh_port_base", type=int)
    _add_argument(parser, "worker_start_delay_s", type=float)
    parser.add_argument("--engine-setup", default=ENGINE_SETUP)
    _add_argument(parser, "collection_policy", choices=COLLECTION_POLICY_CHOICES)
    parser.add_argument(
        "--policy",
        default="",
        help="Expert override for the ROS policy module selected by --collection-policy.",
    )
    _add_argument(parser, "policy_start_wait_s", type=float)
    _add_argument(parser, "robot_joint_noise_deg", type=float)
    _add_argument(parser, "cable_rpy_noise_deg", type=float)


def _add_dataset_args(parser: argparse.ArgumentParser) -> None:
    """dataset 경로와 Hugging Face 업로드 관련 인자를 추가한다."""
    _add_argument(parser, "dataset_version")
    _add_argument(parser, "resume", action="store_true")
    _add_argument(parser, "val_ratio", type=float)
    _add_argument(parser, "test_ratio", type=float)
    _add_argument(parser, "push_to_hub", type=_parse_bool, metavar=BOOLEAN_METAVAR)
    _add_argument(parser, "hf_repo_id")
    _add_argument(parser, "hf_revision")
    _add_argument(parser, "hf_private", action="store_true")


def _add_rosbag_args(parser: argparse.ArgumentParser) -> None:
    """trial별 MCAP rosbag 기록 옵션을 추가한다."""
    _add_argument(
        parser,
        "record_rosbag",
        type=_parse_bool,
        metavar=BOOLEAN_METAVAR,
    )
    _add_argument(parser, "rosbag_output_dir", type=Path)
    _add_argument(parser, "rosbag_topics", nargs="+")
    _add_argument(parser, "rosbag_start_timeout_s", type=float)
    _add_argument(parser, "rosbag_stop_grace_s", type=float)


def _add_pose_args(parser: argparse.ArgumentParser) -> None:
    """port-local XYZ/RPY sampling 범위 인자를 추가한다."""
    for name in (
        "dx_min_mm",
        "dx_max_mm",
        "dy_min_mm",
        "dy_max_mm",
        "dz_min_mm",
        "dz_max_mm",
        "port_roll_limit_deg",
        "port_pitch_limit_deg",
        "port_yaw_limit_deg",
        "roll_min_deg",
        "roll_max_deg",
        "pitch_min_deg",
        "pitch_max_deg",
        "yaw_min_deg",
        "yaw_max_deg",
        "rpy_norm_max_rad",
        "base_z_offset_mm",
        "board_distance_min_mm",
        "board_distance_max_mm",
        "board_lateral_limit_mm",
        "board_angle_limit_deg",
        "descent_start_distance_mm",
        "descent_lateral_limit_mm",
        "descent_angle_limit_deg",
        "visibility_margin_px",
        "board_visibility_margin_px",
    ):
        _add_argument(parser, name, type=float)
    _add_argument(parser, "min_visible_cameras", type=int)
    _add_argument(parser, "board_min_visible_cameras", type=int)
    _add_argument(parser, "sampling_tiers_mm")
    _add_argument(parser, "sampling_tier_weights")


def _add_sync_args(parser: argparse.ArgumentParser) -> None:
    """sample timestamp 허용 오차와 늦은 source 대기시간을 추가한다."""
    _add_argument(parser, "sync_tolerance_ms", type=float)
    _add_argument(parser, "sync_wait_timeout_s", type=float)
    _add_argument(parser, "settle_timeout_s", type=float)
    _add_argument(parser, "settle_position_tolerance_mm", type=float)
    _add_argument(parser, "settle_orientation_tolerance_deg", type=float)
    _add_argument(parser, "settle_stable_observations", type=int)
    _add_argument(parser, "settle_poll_s", type=float)
    _add_argument(parser, "capture_attempt_multiplier", type=float)


def _add_world_args(parser: argparse.ArgumentParser) -> None:
    """Gazebo 조명과 배경 Gaussian randomization 인자를 추가한다."""
    _add_argument(
        parser,
        "randomize_lighting",
        type=_parse_bool,
        metavar=BOOLEAN_METAVAR,
    )
    for name in (
        "light_intensity_scale_min",
        "light_intensity_scale_max",
        "light_color_jitter",
        "light_pose_xy_jitter_m",
        "light_pose_z_jitter_m",
        "ambient_min",
        "ambient_max",
        "background_min",
        "background_max",
    ):
        _add_argument(parser, name, type=float)


def _add_lifecycle_args(parser: argparse.ArgumentParser) -> None:
    """policy와 simulator PGID 종료 단계 및 cleanup 인자를 추가한다."""
    for name in (
        "policy_stop_grace_s",
        "post_summary_wait_s",
        "sim_sigint_grace_s",
        "sim_cleanup_grace_s",
        "sim_sigkill_grace_s",
        "between_trial_wait_s",
    ):
        _add_argument(parser, name, type=float)
    for name in ("cleanup", "cleanup_only", "dry_run"):
        _add_argument(parser, name, action="store_true")


def build_parser() -> argparse.ArgumentParser:
    """수집 runner와 보조 도구가 공유할 CLI parser를 구성한다."""
    parser = argparse.ArgumentParser(
        description="Collect PortOffsetCollect samples from randomized trials."
    )
    _add_trial_args(parser)
    _add_dataset_args(parser)
    _add_rosbag_args(parser)
    _add_pose_args(parser)
    _add_sync_args(parser)
    _add_world_args(parser)
    _add_lifecycle_args(parser)
    return parser


def parse_args() -> argparse.Namespace:
    """명령행 인자를 파싱한다."""
    return build_parser().parse_args()
