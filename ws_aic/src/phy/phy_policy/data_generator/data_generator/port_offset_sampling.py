from __future__ import annotations
from typing import Any, Optional
from geometry_msgs.msg import Pose, Transform
from data_generator.port_offset_geometry import _quat_from_axis_angle_xyzw, _quat_multiply_xyzw

"""Port-local XYZ/RPY sampling for PortOffsetCollect."""

import numpy as np

def _stratified_axis(self, low: float, high: float, steps: int) -> np.ndarray:
    """한 축 범위를 동일 구간으로 나눠 각 구간에서 uniform sample을 뽑는다."""
    if steps <= 1:
        return np.array([(low + high) * 0.5], dtype=float)
    edges = np.linspace(low, high, steps + 1, dtype=float)
    values = self._collect_rng.uniform(edges[:-1], edges[1:])
    self._collect_rng.shuffle(values)
    return values

def _build_port_collect_samples(self, steps: int) -> list[dict[str, float]]:
    """첫 영점과 독립 stratified XYZ/RPY sample 목록을 생성한다."""
    if steps <= 1:
        return [
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        ]

    random_steps = steps - 1
    x_values = self._stratified_axis(
        self.port_collect_x_min_m,
        self.port_collect_x_max_m,
        random_steps,
    )
    y_values = self._stratified_axis(
        self.port_collect_y_min_m,
        self.port_collect_y_max_m,
        random_steps,
    )
    z_values = self._stratified_axis(
        self.port_collect_z_min_m,
        self.port_collect_z_max_m,
        random_steps,
    )
    roll_values = self._stratified_axis(
        self.port_collect_roll_min_rad,
        self.port_collect_roll_max_rad,
        random_steps,
    )
    pitch_values = self._stratified_axis(
        self.port_collect_pitch_min_rad,
        self.port_collect_pitch_max_rad,
        random_steps,
    )
    yaw_values = self._stratified_axis(
        self.port_collect_yaw_min_rad,
        self.port_collect_yaw_max_rad,
        random_steps,
    )

    samples = [
        {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
    ]
    for idx in range(random_steps):
        rpy = np.array(
            [roll_values[idx], pitch_values[idx], yaw_values[idx]],
            dtype=float,
        )
        rpy_norm = float(np.linalg.norm(rpy))
        if (
            self.port_collect_rpy_norm_max_rad > 0.0
            and rpy_norm > self.port_collect_rpy_norm_max_rad
            and rpy_norm > 1e-12
        ):
            rpy *= self.port_collect_rpy_norm_max_rad / rpy_norm
        samples.append(
            {
                "x": float(x_values[idx]),
                "y": float(y_values[idx]),
                "z": float(z_values[idx]),
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            }
        )
    samples[1:] = sorted(
        samples[1:],
        key=lambda sample: float(
            np.linalg.norm([sample["x"], sample["y"], sample["z"]])
        ),
    )
    return samples

def _sample_port_collect_offset(self, step_idx: int) -> dict[str, float]:
    """현재 collect step에 대응하는 port-local offset sample을 반환한다."""
    if len(self._port_collect_samples) != max(1, self.collect_steps):
        self._port_collect_samples = self._build_port_collect_samples(
            max(1, self.collect_steps)
        )
    return self._port_collect_samples[step_idx % len(self._port_collect_samples)]

def _apply_collect_offset(
    self,
    pose: Pose,
    port_transform: Transform,
    port_axis: Optional[dict[str, float]],
    step_idx: int,
) -> tuple[Pose, dict[str, Any]]:
    """하나의 stratified port-local XYZ와 RPY offset을 목표 pose에 적용한다."""
    denom = float(max(1, self.collect_steps - 1))
    progress = float(np.clip(step_idx / denom, 0.0, 1.0))
    sample = self._sample_port_collect_offset(step_idx)
    x_axis, y_axis, z_axis = self._port_local_xy_axes(port_transform, port_axis)
    offset = sample["x"] * x_axis + sample["y"] * y_axis + sample["z"] * z_axis

    pose.position.x += float(offset[0])
    pose.position.y += float(offset[1])
    pose.position.z += float(offset[2])

    roll_quat = _quat_from_axis_angle_xyzw(x_axis, sample["roll"])
    pitch_quat = _quat_from_axis_angle_xyzw(y_axis, sample["pitch"])
    yaw_quat = _quat_from_axis_angle_xyzw(z_axis, sample["yaw"])
    delta_quat = _quat_multiply_xyzw(
        yaw_quat, _quat_multiply_xyzw(pitch_quat, roll_quat)
    )
    base_quat = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    qx, qy, qz, qw = _quat_multiply_xyzw(delta_quat, base_quat)
    q_norm = float(np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw))
    if q_norm > 1e-9:
        pose.orientation.x = float(qx / q_norm)
        pose.orientation.y = float(qy / q_norm)
        pose.orientation.z = float(qz / q_norm)
        pose.orientation.w = float(qw / q_norm)

    distance = float(np.linalg.norm([sample["x"], sample["y"], sample["z"]]))
    return pose, {
        "collect_pattern": self.collect_pattern,
        "collect_progress": progress,
        "collect_theta": float(np.arctan2(sample["y"], sample["x"])),
        "collect_radius": float(np.hypot(sample["x"], sample["y"])),
        "collect_distance": distance,
        "collect_local_x": sample["x"],
        "collect_local_y": sample["y"],
        "collect_local_z": sample["z"],
        "collect_local_roll": sample["roll"],
        "collect_local_pitch": sample["pitch"],
        "collect_local_yaw": sample["yaw"],
        "collect_local_roll_deg": float(np.rad2deg(sample["roll"])),
        "collect_local_pitch_deg": float(np.rad2deg(sample["pitch"])),
        "collect_local_yaw_deg": float(np.rad2deg(sample["yaw"])),
        "collect_offset_x": float(offset[0]),
        "collect_offset_y": float(offset[1]),
        "collect_offset_z": float(offset[2]),
        "collect_spin_angle": sample["yaw"],
    }
