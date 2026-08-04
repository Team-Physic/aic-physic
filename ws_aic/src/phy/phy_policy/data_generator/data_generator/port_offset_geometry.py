from __future__ import annotations

"""Geometry helpers for PortOffsetCollect."""

import numpy as np
from geometry_msgs.msg import Pose

def s_curve_quintic(t: float) -> float:
    """0~1 진행률을 부드러운 S-curve 값으로 바꿔 로봇 이동 시작/끝을 완만하게 만든다."""
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5

def interp_profile(t: float, quintic: bool = True) -> float:
    """입력 진행률 t를 로봇 pose 보간에 사용할 진행률로 변환한다."""
    return s_curve_quintic(t) if quintic else (3.0 * t**2 - 2.0 * t**3)

def _quat_to_matrix_xyzw(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """ROS 형식 xyzw quaternion을 3x3 회전 행렬로 변환한다."""
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

def _matrix_to_rpy_xyz(matrix: np.ndarray) -> tuple[float, float, float]:
    """3x3 회전 행렬을 roll/pitch/yaw(rad)로 변환한다."""
    rot = np.asarray(matrix, dtype=float)
    sy = float(np.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0]))
    if sy > 1e-9:
        roll = float(np.arctan2(rot[2, 1], rot[2, 2]))
        pitch = float(np.arctan2(-rot[2, 0], sy))
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    else:
        roll = float(np.arctan2(-rot[1, 2], rot[1, 1]))
        pitch = float(np.arctan2(-rot[2, 0], sy))
        yaw = 0.0
    return roll, pitch, yaw

def _matrix_from_translation_quat(translation, quat_xyzw) -> np.ndarray:
    """translation과 xyzw quaternion을 하나의 4x4 좌표 변환 행렬로 묶는다."""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = _quat_to_matrix_xyzw(*quat_xyzw)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix

def _matrix_from_pose(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose를 행렬 곱에 사용할 수 있는 4x4 좌표 변환 행렬로 바꾼다."""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = _quat_to_matrix_xyzw(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix

def _quat_multiply_xyzw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """xyzw quaternion 두 개를 곱해 회전을 합성한다."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )

def _quat_from_axis_angle_xyzw(
    axis_xyz: np.ndarray,
    angle_rad: float,
) -> tuple[float, float, float, float]:
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
