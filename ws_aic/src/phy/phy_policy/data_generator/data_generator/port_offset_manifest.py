from __future__ import annotations

"""Dataset manifest writer for PortOffsetCollect."""

import numpy as np


def _write_rpy_data_yaml(self) -> None:
    """XYZ/RPY 데이터셋의 경로·범위·timestamp 조건을 data.yaml에 기록한다."""
    self._rpy_dataset_dir.mkdir(parents=True, exist_ok=True)
    (self._rpy_dataset_dir / "data.yaml").write_text(
        "\n".join(
            [
                f"path: {self._rpy_dataset_dir.resolve()}",
                "train: images/train",
                "val: images/val",
                "image_layout: images/<split>/<connector>/<camera>/*.jpg",
                "metadata: metadata/<split>/<connector>/<camera>/*.json",
                "label_format: per_image_json",
                "task: phy_rpy_randomization",
                f"version: {self._rpy_dataset_version or 'default'}",
                "collect_range_mm:",
                f"  dx: [{self.port_collect_x_min_m * 1000.0:.6f}, {self.port_collect_x_max_m * 1000.0:.6f}]",
                f"  dy: [{self.port_collect_y_min_m * 1000.0:.6f}, {self.port_collect_y_max_m * 1000.0:.6f}]",
                f"  dz: [{self.port_collect_z_min_m * 1000.0:.6f}, {self.port_collect_z_max_m * 1000.0:.6f}]",
                f"base_z_offset_mm: {self.collect_base_z_offset_m * 1000.0:.6f}",
                "timestamp_clock: ros",
                "timestamp_unit: nanoseconds",
                f"sync_tolerance_ms: {self.collect_sync_tolerance_ns / 1_000_000.0:.6f}",
                f"sync_wait_timeout_s: {self.collect_sync_wait_timeout_sec:.6f}",
                "collect_rpy_range_deg:",
                f"  roll: [{np.rad2deg(self.port_collect_roll_min_rad):.6f}, {np.rad2deg(self.port_collect_roll_max_rad):.6f}]",
                f"  pitch: [{np.rad2deg(self.port_collect_pitch_min_rad):.6f}, {np.rad2deg(self.port_collect_pitch_max_rad):.6f}]",
                f"  yaw: [{np.rad2deg(self.port_collect_yaw_min_rad):.6f}, {np.rad2deg(self.port_collect_yaw_max_rad):.6f}]",
                f"  norm_max_rad: {self.port_collect_rpy_norm_max_rad:.9f}",
                "fields:",
                "  command: base_link target pose xyz + quaternion xyzw",
                "  location: measured plug reference offset from port entrance in base_link",
                "  label: base_link correction from plug reference to port entrance alignment",
                "  collect: commanded port-local dx/dy/dz + roll/pitch/yaw sample",
                "  timestamps: source ROS 시각과 source 간 최대 시각 차이 및 허용 오차",
                "  image: 수집 시각 일치 조건을 통과한 camera image",
                "",
            ]
        ),
        encoding="utf-8",
    )
