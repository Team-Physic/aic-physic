"""Minimal transform helpers used by PHY policies."""

from __future__ import annotations

import numpy as np


def quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a ROS xyzw quaternion to a 3x3 rotation matrix."""
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
    """Build a 4x4 transform from translation and an xyzw quaternion."""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_matrix(*quaternion_xyzw)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix
