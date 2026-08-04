from __future__ import annotations
"""PortOffsetCollect의 camera와 port-local frame 변환."""

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

def _image_msg_to_bgr(self, img_msg, camera_name: str = "image") -> Optional[np.ndarray]:
    """ROS Image 메시지를 OpenCV에서 쓰는 BGR numpy 이미지로 변환한다."""
    if img_msg is None or img_msg.width == 0 or img_msg.height == 0:
        return None
    try:
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
    except ValueError:
        self.get_logger().warn(f"[PortOffsetCollect] Invalid {camera_name} buffer size")
        return None
    if img_msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()

def _image_msg_for_camera(self, obs, camera_name: str):
    """Observation에서 camera_name에 해당하는 Image 메시지를 꺼낸다."""
    if camera_name == "left":
        return obs.left_image
    if camera_name == "right":
        return obs.right_image
    return obs.center_image

def _camera_info_for_camera(self, obs, camera_name: str):
    """Observation에서 camera_name에 해당하는 CameraInfo 메시지를 꺼낸다."""
    if camera_name == "left":
        return obs.left_camera_info
    if camera_name == "right":
        return obs.right_camera_info
    return obs.center_camera_info

def _camera_intrinsic_matrix(self, camera_info) -> Optional[np.ndarray]:
    """CameraInfo.k 배열을 3x3 intrinsic 행렬 K로 변환한다."""
    if camera_info is None or len(camera_info.k) < 9:
        return None
    k = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
    if abs(k[0, 0]) < 1e-9 or abs(k[1, 1]) < 1e-9:
        return None
    return k

def _base_to_camera_optical_matrix(self, obs, camera_name: str) -> np.ndarray:
    """tcp_pose와 고정 extrinsic으로 base_link 좌표를 camera optical 좌표로 보내는 행렬을 만든다."""
    t_base_tcp = _matrix_from_pose(obs.controller_state.tcp_pose)
    t_base_tool0 = t_base_tcp @ np.linalg.inv(self._t_tool0_tcp)
    t_base_optical = t_base_tool0 @ self._t_tool0_to_optical[camera_name]
    return np.linalg.inv(t_base_optical)

def _port_local_xy_axes(self, port_transform: Transform, port_axis: Optional[dict[str, float]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """포트 로컬 X/Y 축과 approach 축을 base_link 방향 벡터로 변환한다."""
    port_rotation = _quat_to_matrix_xyzw(
        port_transform.rotation.x,
        port_transform.rotation.y,
        port_transform.rotation.z,
        port_transform.rotation.w,
    )
    x_axis = port_rotation[:, 0].copy()
    y_axis = port_rotation[:, 1].copy()
    if port_axis is None:
        z_axis = port_rotation[:, 2].copy()
    else:
        z_axis = np.array(
            [
                float(port_axis.get("x", 0.0)),
                float(port_axis.get("y", 0.0)),
                float(port_axis.get("z", 1.0)),
            ],
            dtype=float,
        )
    z_norm = float(np.linalg.norm(z_axis))
    z_axis = z_axis / z_norm if z_norm > 1e-9 else np.array([0.0, 0.0, 1.0], dtype=float)

    for basis in (x_axis, y_axis):
        basis -= z_axis * float(np.dot(basis, z_axis))
        basis_norm = float(np.linalg.norm(basis))
        if basis_norm > 1e-9:
            basis /= basis_norm
    return x_axis, y_axis, z_axis
