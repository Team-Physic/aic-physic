from __future__ import annotations
"""PortOffsetCollect의 plug 기준점과 실제 offset label 계산."""

import json
import os
import signal
import sys
import threading
import time
import cv2
import numpy as np

from datetime import datetime
from pathlib import Path
from typing import Optional, Any
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose, Transform, Vector3, Wrench
from data_generator.lib.cheatcode import CheatCodePlanner
from data_generator.port_offset_config import (
    DAMPING_DEFAULT,
    SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME,
    STIFFNESS_DEFAULT,
    TOOL0_TO_OPTICAL,
    TOOL0_TO_TCP_Z,
)
from data_generator.port_offset_geometry import (
    _matrix_to_rpy_xyz,
    _matrix_from_pose,
    _matrix_from_translation_quat,
    _quat_to_matrix_xyzw,
)
from tf2_ros import TransformException

def _transform_translation_array(self, transform: Transform) -> np.ndarray:
    """ROS Transform의 translation을 3차원 numpy 배열로 변환한다."""
    return np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=float,
    )

def _transform_rotation_matrix(self, transform: Transform) -> np.ndarray:
    """ROS Transform의 quaternion을 3×3 회전 행렬로 변환한다."""
    return _quat_to_matrix_xyzw(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )

def _shift_transform_origin(self, transform: Transform, local_offset_xyz: np.ndarray) -> Transform:
    """Transform 원점을 자체 좌표계의 local offset만큼 이동한다."""
    local_offset = np.asarray(local_offset_xyz, dtype=float)
    shifted_xyz = self._transform_translation_array(transform) + self._transform_rotation_matrix(transform) @ local_offset
    shifted = Transform()
    shifted.translation.x = float(shifted_xyz[0])
    shifted.translation.y = float(shifted_xyz[1])
    shifted.translation.z = float(shifted_xyz[2])
    shifted.rotation.x = float(transform.rotation.x)
    shifted.rotation.y = float(transform.rotation.y)
    shifted.rotation.z = float(transform.rotation.z)
    shifted.rotation.w = float(transform.rotation.w)
    return shifted

def _plug_location_label_in_base_frame(
    self,
    port_tf: Transform,
    plug_tf: Transform,
) -> dict[str, dict[str, float]]:
    """settle 후 실제 plug reference 위치와 정렬 correction을 base_link 기준으로 계산한다."""
    port_position = self._transform_translation_array(port_tf)
    port_rotation = self._transform_rotation_matrix(port_tf)
    plug_position = self._transform_translation_array(plug_tf)
    plug_rotation = self._transform_rotation_matrix(plug_tf)

    location_position = plug_position - port_position
    location_rotation = plug_rotation @ port_rotation.T
    location_roll, location_pitch, location_yaw = _matrix_to_rpy_xyz(location_rotation)

    label_position = port_position - plug_position
    label_rotation = port_rotation @ plug_rotation.T
    label_roll, label_pitch, label_yaw = _matrix_to_rpy_xyz(label_rotation)

    location = {
        "x_m": float(location_position[0]),
        "y_m": float(location_position[1]),
        "z_m": float(location_position[2]),
        "roll_rad": float(location_roll),
        "pitch_rad": float(location_pitch),
        "yaw_rad": float(location_yaw),
    }
    label = {
        "x_m": float(label_position[0]),
        "y_m": float(label_position[1]),
        "z_m": float(label_position[2]),
        "roll_rad": float(label_roll),
        "pitch_rad": float(label_pitch),
        "yaw_rad": float(label_yaw),
    }
    return {"location": location, "label": label}

def _plug_reference_offset_local(self, task: Task, cable_tip_frame: str) -> np.ndarray:
    """plug TF 원점에서 label과 제어에 쓸 물리 기준점까지의 offset을 반환한다."""
    is_sfp = (
        "sfp" in str(task.plug_type).lower()
        or "sfp" in str(task.plug_name).lower()
        or "sfp" in cable_tip_frame.lower()
    )
    if not is_sfp:
        return np.zeros(3, dtype=float)
    return np.array(
        [
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_X", str(SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME[0]))),
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_Y", str(SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME[1]))),
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_Z", str(SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME[2]))),
        ],
        dtype=float,
    )

def _plug_reference_metadata(
    self,
    task: Task,
    cable_tip_frame: str,
    offset_local_xyz: np.ndarray,
) -> dict[str, Any]:
    """선택한 plug 기준점의 프레임·offset·설명을 metadata로 구성한다."""
    is_sfp = (
        "sfp" in str(task.plug_type).lower()
        or "sfp" in str(task.plug_name).lower()
        or "sfp" in cable_tip_frame.lower()
    )
    return {
        "plug_frame": cable_tip_frame,
        "point_name": "sfp_tip_top_center" if is_sfp else "plug_frame_origin",
        "local_offset_xyz_m": [float(value) for value in np.asarray(offset_local_xyz, dtype=float)],
        "description": (
            "SFP contact-collision top center; zero label means this point is on the port entrance frame."
            if is_sfp
            else "Selected plug frame origin."
        ),
    }

def _json_safe(self, value):
    """numpy scalar/array가 섞인 metadata를 jsonl로 쓸 수 있는 기본 타입으로 바꾼다."""
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, np.ndarray):
        return [self._json_safe(v) for v in value.tolist()]
    if isinstance(value, dict):
        return {str(k): self._json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [self._json_safe(v) for v in value]
    if value is None or isinstance(value, str):
        return value
    return str(value)
