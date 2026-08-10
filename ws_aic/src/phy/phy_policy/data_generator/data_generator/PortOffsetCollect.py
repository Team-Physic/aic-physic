from __future__ import annotations

"""Port-relative offset collection policy entrypoint."""

import os
import numpy as np
from pathlib import Path
from aic_model.policy import Policy
from data_generator import port_offset_base as runtime
from data_generator import port_offset_dataset as dataset
from data_generator import port_offset_manifest as manifest
from data_generator import port_offset_sampling as sampling
from data_generator import port_offset_stages as stages
from data_generator.port_offset_helpers import (
    _default_dataset_dir,
    _env_bool,
    _env_deg,
    _env_deg_range,
    _env_mm,
    _env_mm_range,
)

class PortOffsetCollect(Policy):
    """Ground Truth TF로 포트 주변 XYZ/RPY offset image를 수집한다."""

    # Methods implemented in port_offset_base.py
    _watch_stop_file = runtime._watch_stop_file
    _on_sigterm = runtime._on_sigterm
    _wait_for_tf = runtime._wait_for_tf
    _lookup_transform = runtime._lookup_transform
    _lookup_latest_transform_stamped = runtime._lookup_latest_transform_stamped
    _lookup_transform_at = runtime._lookup_transform_at
    _collect_log_text = runtime._collect_log_text
    _select_port_frame = runtime._select_port_frame
    _select_cable_tip_frame = runtime._select_cable_tip_frame
    set_pose_target = runtime.set_pose_target
    _transform_translation_array = runtime._transform_translation_array
    _transform_rotation_matrix = runtime._transform_rotation_matrix
    _shift_transform_origin = runtime._shift_transform_origin
    _plug_location_label_in_base_frame = runtime._plug_location_label_in_base_frame
    _plug_reference_offset_local = runtime._plug_reference_offset_local
    _plug_reference_metadata = runtime._plug_reference_metadata
    _json_safe = runtime._json_safe
    _image_msg_to_bgr = runtime._image_msg_to_bgr
    _image_msg_for_camera = runtime._image_msg_for_camera
    _camera_info_for_camera = runtime._camera_info_for_camera
    _camera_intrinsic_matrix = runtime._camera_intrinsic_matrix
    _base_to_camera_optical_matrix = runtime._base_to_camera_optical_matrix
    _port_local_xy_axes = runtime._port_local_xy_axes
    _upload_vision_offset_dataset_to_hub = runtime._upload_vision_offset_dataset_to_hub
    _finish_data_collection_episode = runtime._finish_data_collection_episode
    _write_episode_summary = runtime._write_episode_summary

    # Methods implemented in port_offset_stages.py
    _configure_port_collect_control = stages._configure_port_collect_control
    _stage_lift_up = stages._stage_lift_up
    _stage_approach = stages._stage_approach
    _stage_collect = stages._stage_collect
    insert_cable = stages.insert_cable

    # Methods implemented in port_offset_manifest.py
    _write_img2pos_data_yaml = manifest._write_img2pos_data_yaml

    # Methods implemented in port_offset_dataset.py
    _split_for_trial = dataset._split_for_trial
    _trial_id = dataset._trial_id
    _port_projection_for_camera = dataset._port_projection_for_camera
    _observation_sync_metadata = dataset._observation_sync_metadata
    _wait_for_synchronized_observation = dataset._wait_for_synchronized_observation
    _tf_sync_metadata = dataset._tf_sync_metadata
    _save_img2pos_sample = dataset._save_img2pos_sample
    _save_vision_offset_sample = dataset._save_vision_offset_sample

    # Methods implemented in port_offset_sampling.py
    _stratified_axis = sampling._stratified_axis
    _build_port_collect_samples = sampling._build_port_collect_samples
    _sample_port_collect_offset = sampling._sample_port_collect_offset
    _apply_collect_offset = sampling._apply_collect_offset

    def __init__(self, parent_node):
        """PortOffset 데이터 수집 정책의 환경과 sampling 범위를 초기화한다."""
        os.environ.setdefault("AIC_COLLECT_STEPS", "1000")
        dataset_dir = Path(
            os.environ.setdefault(
                "AIC_VISION_OFFSET_DATASET_DIR", str(_default_dataset_dir())
            )
        ).expanduser()
        Policy.__init__(self, parent_node)
        runtime.init_runtime(self, parent_node)
        self._rpy_dataset_dir = dataset_dir
        self._rpy_dataset_version = os.environ.get("AIC_RPY_DATASET_VERSION", "").strip()
        self._img2pos_samples_path = self._rpy_dataset_dir / "samples.jsonl"
        self._collection_metadata = {
            "run_id": os.environ.get("AIC_PORTOFFSET_RUN_ID", "").strip(),
            "trial_index": os.environ.get("AIC_PORTOFFSET_TRIAL_INDEX", "").strip(),
        }
        self._rpy_push_to_hub = _env_bool("AIC_VISION_OFFSET_PUSH_TO_HUB", True)
        self._rpy_hf_repo_id = os.environ.get("AIC_VISION_OFFSET_REPO_ID", "").strip()
        self._rpy_hf_revision = (
            os.environ.get("AIC_VISION_OFFSET_HF_REVISION", "").strip()
            or self._rpy_dataset_version
            or "main"
        )
        self._rpy_hf_private = _env_bool("AIC_VISION_OFFSET_HF_PRIVATE", False)
        self._rpy_hf_path_in_repo = os.environ.get("AIC_VISION_OFFSET_HF_PATH_IN_REPO", "").strip()
        self._rpy_hf_upload_on_port_type = os.environ.get(
            "AIC_VISION_OFFSET_UPLOAD_ON_PORT_TYPE",
            "sc",
        ).strip().lower()
        self._rpy_sample_count = 0
        self._rpy_val_ratio = float(
            os.environ.get("AIC_RPY_RANDOMIZATION_VAL_RATIO", "0.3")
        )
        self._rpy_visibility_margin_px = float(
            os.environ.get("AIC_RPY_VISIBILITY_MARGIN_PX", "64.0")
        )
        self._rpy_min_visible_cameras = max(
            1, int(os.environ.get("AIC_RPY_MIN_VISIBLE_CAMERAS", "2"))
        )
        self.collect_pattern = "port_offset_xyz_rpy"
        xy_limit_m = _env_mm("AIC_PORT_COLLECT_XY_LIMIT_MM", 50.0)
        z_limit_m = _env_mm("AIC_PORT_COLLECT_Z_LIMIT_MM", 100.0)
        self.port_collect_x_min_m, self.port_collect_x_max_m = _env_mm_range(
            "AIC_PORT_COLLECT_DX_MIN_MM",
            "AIC_PORT_COLLECT_DX_MAX_MM",
            -xy_limit_m,
            xy_limit_m,
        )
        self.port_collect_y_min_m, self.port_collect_y_max_m = _env_mm_range(
            "AIC_PORT_COLLECT_DY_MIN_MM",
            "AIC_PORT_COLLECT_DY_MAX_MM",
            -xy_limit_m,
            xy_limit_m,
        )
        self.port_collect_z_min_m, self.port_collect_z_max_m = _env_mm_range(
            "AIC_PORT_COLLECT_DZ_MIN_MM",
            "AIC_PORT_COLLECT_DZ_MAX_MM",
            0.0,
            z_limit_m,
        )
        roll_limit_rad = _env_deg(
            "AIC_PORT_COLLECT_ROLL_LIMIT_DEG", 25.0
        )
        pitch_limit_rad = _env_deg(
            "AIC_PORT_COLLECT_PITCH_LIMIT_DEG", 25.0
        )
        yaw_limit_rad = _env_deg(
            "AIC_PORT_COLLECT_YAW_LIMIT_DEG", 35.0
        )
        self.port_collect_roll_min_rad, self.port_collect_roll_max_rad = _env_deg_range(
            "AIC_PORT_COLLECT_ROLL_MIN_DEG",
            "AIC_PORT_COLLECT_ROLL_MAX_DEG",
            -roll_limit_rad,
            roll_limit_rad,
        )
        self.port_collect_pitch_min_rad, self.port_collect_pitch_max_rad = _env_deg_range(
            "AIC_PORT_COLLECT_PITCH_MIN_DEG",
            "AIC_PORT_COLLECT_PITCH_MAX_DEG",
            -pitch_limit_rad,
            pitch_limit_rad,
        )
        self.port_collect_yaw_min_rad, self.port_collect_yaw_max_rad = _env_deg_range(
            "AIC_PORT_COLLECT_YAW_MIN_DEG",
            "AIC_PORT_COLLECT_YAW_MAX_DEG",
            -yaw_limit_rad,
            yaw_limit_rad,
        )
        self.port_collect_rpy_norm_max_rad = max(
            0.0, float(os.environ.get("AIC_PORT_COLLECT_RPY_NORM_MAX_RAD", "0.0"))
        )
        self._write_img2pos_data_yaml()
        self._port_collect_samples = self._build_port_collect_samples(
            max(1, self.collect_steps)
        )
        self.get_logger().info(
            "[PortOffsetCollect] Ready: "
            f"dx=[{self.port_collect_x_min_m * 1000.0:.1f}, "
            f"{self.port_collect_x_max_m * 1000.0:.1f}]mm, "
            f"dy=[{self.port_collect_y_min_m * 1000.0:.1f}, "
            f"{self.port_collect_y_max_m * 1000.0:.1f}]mm, "
            f"dz=[{self.port_collect_z_min_m * 1000.0:.1f}, "
            f"{self.port_collect_z_max_m * 1000.0:.1f}]mm, "
            f"roll=[{np.rad2deg(self.port_collect_roll_min_rad):.1f}, "
            f"{np.rad2deg(self.port_collect_roll_max_rad):.1f}]deg, "
            f"pitch=[{np.rad2deg(self.port_collect_pitch_min_rad):.1f}, "
            f"{np.rad2deg(self.port_collect_pitch_max_rad):.1f}]deg, "
            f"yaw=[{np.rad2deg(self.port_collect_yaw_min_rad):.1f}, "
            f"{np.rad2deg(self.port_collect_yaw_max_rad):.1f}]deg, "
            f"rpy_norm_max={np.rad2deg(self.port_collect_rpy_norm_max_rad):.2f}deg, "
            f"dataset={self._rpy_dataset_dir}, "
            f"version={self._rpy_dataset_version or 'default'}, "
            f"push_to_hub={self._rpy_push_to_hub}, "
            f"upload_on_port_type={self._rpy_hf_upload_on_port_type or 'any'}, "
            f"min_visible_cameras={self._rpy_min_visible_cameras}, "
            "first COLLECT sample uses zero offset/rpy"
        )

# 기존 로더가 파일 안의 DataCollect 심볼을 찾는 경우도 같이 지원한다.
DataCollect = PortOffsetCollect
