"""Policy에서 공유하는 최소 좌표·quaternion 계산."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose


def quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """ROS xyzw quaternion을 3x3 회전 행렬로 변환한다."""
    norm = float(np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw))
    if norm < 1e-12:
        return np.eye(3, dtype=float)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def transform_matrix(translation, quaternion_xyzw) -> np.ndarray:
    """translation과 xyzw quaternion을 4x4 변환 행렬로 묶는다."""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_matrix(*quaternion_xyzw)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def pose_matrix(pose: Pose) -> np.ndarray:
    """ROS Pose를 4x4 변환 행렬로 바꾼다."""
    return transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    )


def multiply_quaternions(a, b) -> tuple[float, float, float, float]:
    """xyzw quaternion 두 개를 곱한다."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def axis_angle_quaternion(axis_xyz, angle_rad: float):
    """base_link 기준 회전축과 각도를 xyzw quaternion으로 변환한다."""
    axis = np.asarray(axis_xyz, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis /= norm
    half = 0.5 * float(angle_rad)
    sin_half = float(np.sin(half))
    return (
        float(axis[0] * sin_half),
        float(axis[1] * sin_half),
        float(axis[2] * sin_half),
        float(np.cos(half)),
    )
