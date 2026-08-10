"""PortOffset 무작위 수집 모듈이 공유하는 경로와 물리 상수."""

from __future__ import annotations

import math
from pathlib import Path

from phy_data_collection.paths import PROJECT_ROOT

ROOT = PROJECT_ROOT
WS_SRC = ROOT / "ws_aic" / "src"
PIXI_WS = WS_SRC
POLICY_PACKAGE_ROOT = WS_SRC / "phy" / "phy_policy"
DATASET_ROOT = ROOT / "ws_aic" / "data" / "img2pos"
ROSBAG_ROOT = ROOT / "rosbags" / "portoffset"
CONFIG_DIR = Path("/tmp/phy_portoffset_randomization")
WORLD_TEMPLATE_PATH = (
    ROOT / "ws_aic" / "src" / "aic" / "aic_description" / "world" / "aic.sdf"
)
EPISODE_TRACKING_DIR = Path("/tmp/aic_episodes")

COLLECTION_POLICY_MODULES = {
    "board-view": "phy_policy.ros.BoardViewCollect",
    "descent": "phy_policy.ros.DescentCollect",
    "near-port": "phy_policy.ros.NearPortCollect",
}
COLLECTION_POLICY_CHOICES = tuple(COLLECTION_POLICY_MODULES)
POLICY_MODULE = COLLECTION_POLICY_MODULES["near-port"]
ENGINE_SETUP = "/ws_aic/install/setup.bash"
RUN_MARKER_ENV = "AIC_PORTOFFSET_RUN_ID"
REGISTRY_FILENAME = "owned_process_groups.json"
TRIAL_TIMEOUT_GRACE_S = 180.0
MIN_CLEARANCE_MM = 20.0

BOOLEAN_METAVAR = "{true,false}"
ROSBAG_TOPICS = (
    "/clock",
    "/joint_states",
    "/tf",
    "/tf_static",
    "/scoring/tf",
    "/aic_controller/controller_state",
    "/aic_controller/pose_commands",
    "/left_camera/image",
    "/center_camera/image",
    "/right_camera/image",
)

CLI_DEFAULTS = {
    # Trial and simulator.
    "sfp_trials": 31,
    "sc_trials": 3,
    "workers": 1,
    "seed": 30,
    "color_log": True,
    "samples_per_trial": 40,
    "time_limit_s": 600,
    "trial_timeout_s": None,
    "distrobox": "aic_eval_physic",
    "headless": True,
    "launch_rviz": False,
    "ros_domain_id_base": 40,
    "zenoh_port_base": 7600,
    "worker_start_delay_s": 2.0,
    "policy_start_wait_s": 5.0,
    "robot_joint_noise_deg": 4.0,
    "cable_rpy_noise_deg": 20.0,
    "collection_policy": "near-port",
    # Per-trial rosbag2 recording.
    "record_rosbag": False,
    "rosbag_output_dir": ROSBAG_ROOT,
    "rosbag_topics": ROSBAG_TOPICS,
    "rosbag_start_timeout_s": 20.0,
    "rosbag_stop_grace_s": 30.0,
    # Dataset and Hugging Face.
    "dataset_version": "",
    "resume": False,
    "val_ratio": 0.15,
    "test_ratio": 0.15,
    "push_to_hub": False,
    "hf_repo_id": "team-physic/aic-align",
    "hf_revision": "",
    "hf_private": False,
    # Port-local pose sampling.
    "dx_min_mm": -50.0,
    "dx_max_mm": 50.0,
    "dy_min_mm": -50.0,
    "dy_max_mm": 50.0,
    "dz_min_mm": 0.0,
    "dz_max_mm": 50.0,
    "sampling_tiers_mm": "50,10,5,2",
    "sampling_tier_weights": "1,1,1,1",
    "port_roll_limit_deg": 25.0,
    "port_pitch_limit_deg": 25.0,
    "port_yaw_limit_deg": 35.0,
    "roll_min_deg": None,
    "roll_max_deg": None,
    "pitch_min_deg": None,
    "pitch_max_deg": None,
    "yaw_min_deg": None,
    "yaw_max_deg": None,
    "rpy_norm_max_rad": None,
    "base_z_offset_mm": MIN_CLEARANCE_MM,
    "board_distance_min_mm": 750.0,
    "board_distance_max_mm": 850.0,
    "board_lateral_limit_mm": 30.0,
    "board_angle_limit_deg": 15.0,
    "descent_start_distance_mm": 550.0,
    "descent_lateral_limit_mm": 40.0,
    "descent_angle_limit_deg": 20.0,
    "min_visible_cameras": 2,
    "visibility_margin_px": 64.0,
    # Capture timestamp synchronization.
    "sync_tolerance_ms": 30.0,
    "sync_wait_timeout_s": 1.0,
    "settle_timeout_s": 8.0,
    "settle_position_tolerance_mm": 1.0,
    "settle_orientation_tolerance_deg": 1.0,
    "settle_stable_observations": 3,
    "settle_poll_s": 0.02,
    "capture_attempt_multiplier": 2.0,
    # World and lighting.
    "randomize_lighting": True,
    "light_intensity_scale_min": 0.65,
    "light_intensity_scale_max": 1.35,
    "light_color_jitter": 0.12,
    "light_pose_xy_jitter_m": 0.25,
    "light_pose_z_jitter_m": 0.20,
    "ambient_min": 0.0,
    "ambient_max": 0.08,
    "background_min": 0.08,
    "background_max": 0.20,
    # Process lifecycle.
    "policy_stop_grace_s": 10.0,
    "post_summary_wait_s": 3.0,
    "sim_sigint_grace_s": 5.0,
    "sim_cleanup_grace_s": 2.0,
    "sim_sigkill_grace_s": 1.0,
    "between_trial_wait_s": 3.0,
    "cleanup": False,
    "cleanup_only": False,
    "dry_run": False,
}

ANSI_COLORS = {
    "reset": "\033[0m",
    "bold": "\033[1m",
    "cyan": "\033[36m",
    "yellow": "\033[33m",
    "magenta": "\033[35m",
    "green": "\033[32m",
    "blue": "\033[34m",
}

SFP_NIC_RAIL_COUNT = 5
SFP_PORT_COUNT = 2
SC_RAIL_COUNT = 2

BASE_ROBOT_HOME = {
    "shoulder_pan_joint": -0.1597,
    "shoulder_lift_joint": -1.3542,
    "elbow_joint": -1.6648,
    "wrist_1_joint": -1.6933,
    "wrist_2_joint": 1.5710,
    "wrist_3_joint": 1.4110,
}

LIMITS = {
    "nic_translation": (-0.0215, 0.0234),
    "nic_yaw": (-math.radians(10.0), math.radians(10.0)),
    "sc_translation": (-0.06, 0.055),
    "sfp_board_x": (0.13, 0.17),
    "sfp_board_y": (-0.25, -0.20),
    "sfp_board_yaw": (3.10, 3.1415),
    "sc_board_x": (0.15, 0.19),
    "sc_board_y": (-0.05, 0.05),
    "sc_board_yaw": (3.10, 3.1415),
    "gripper_offset_noise": (-0.002, 0.002),
    "sfp_gripper_offset_x": 0.0,
    "sfp_gripper_offset_y": 0.015385,
    "sfp_gripper_offset_z": 0.04245,
    "sc_gripper_offset_x": 0.0,
    "sc_gripper_offset_y": 0.015385,
    "sc_gripper_offset_z": 0.04045,
    "cable_roll": 0.4432,
    "cable_pitch": -0.4838,
    "cable_yaw": 1.3303,
}
