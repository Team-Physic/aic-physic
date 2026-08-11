"""SE(3) pose serialization and logarithmic residual helpers."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


SE3_LOG_ORDER = ("vx_m", "vy_m", "vz_m", "wx_rad", "wy_rad", "wz_rad")


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def quaternion_matrix(quaternion_xyzw) -> np.ndarray:
    """정규화한 xyzw quaternion을 3x3 회전행렬로 변환한다."""
    quaternion = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    """3x3 회전행렬을 w>=0인 정규화 xyzw quaternion으로 변환한다."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ],
            dtype=float,
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ],
                dtype=float,
            )
        elif index == 1:
            scale = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ],
                dtype=float,
            )
        else:
            scale = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ],
                dtype=float,
            )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("rotation matrix produced a zero-norm quaternion")
    quaternion /= norm
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def transform_matrix(translation, quaternion_xyzw) -> np.ndarray:
    """translation과 xyzw quaternion으로 4x4 homogeneous transform을 만든다."""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_matrix(quaternion_xyzw)
    matrix[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return matrix


def se3_exp(residual_vw) -> np.ndarray:
    """[v(m), w(rad)] 순서의 se(3) residual을 4x4 transform으로 복원한다."""
    residual = np.asarray(residual_vw, dtype=float).reshape(6)
    linear = residual[:3]
    angular = residual[3:]
    theta = float(np.linalg.norm(angular))
    omega = _skew(angular)
    omega_squared = omega @ omega
    if theta < 1e-8:
        theta_squared = theta * theta
        a = 1.0 - theta_squared / 6.0 + theta_squared * theta_squared / 120.0
        b = 0.5 - theta_squared / 24.0 + theta_squared * theta_squared / 720.0
        c = 1.0 / 6.0 - theta_squared / 120.0 + theta_squared * theta_squared / 5040.0
    else:
        theta_squared = theta * theta
        a = math.sin(theta) / theta
        b = (1.0 - math.cos(theta)) / theta_squared
        c = (theta - math.sin(theta)) / (theta_squared * theta)
    rotation = np.eye(3) + a * omega + b * omega_squared
    jacobian = np.eye(3) + b * omega + c * omega_squared
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = jacobian @ linear
    return matrix


def se3_log(matrix: np.ndarray) -> np.ndarray:
    """4x4 transform을 [v(m), w(rad)] 순서의 principal se(3) residual로 바꾼다."""
    transform = np.asarray(matrix, dtype=float).reshape(4, 4)
    quaternion = matrix_quaternion(transform[:3, :3])
    vector_norm = float(np.linalg.norm(quaternion[:3]))
    if vector_norm < 1e-12:
        angular = 2.0 * quaternion[:3]
    else:
        angle = 2.0 * math.atan2(vector_norm, float(quaternion[3]))
        angular = quaternion[:3] * (angle / vector_norm)
    theta = float(np.linalg.norm(angular))
    omega = _skew(angular)
    omega_squared = omega @ omega
    if theta < 1e-8:
        theta_squared = theta * theta
        b = 0.5 - theta_squared / 24.0 + theta_squared * theta_squared / 720.0
        c = 1.0 / 6.0 - theta_squared / 120.0 + theta_squared * theta_squared / 5040.0
    else:
        theta_squared = theta * theta
        b = (1.0 - math.cos(theta)) / theta_squared
        c = (theta - math.sin(theta)) / (theta_squared * theta)
    jacobian = np.eye(3) + b * omega + c * omega_squared
    linear = np.linalg.solve(jacobian, transform[:3, 3])
    return np.concatenate((linear, angular))


def pose_record(matrix: np.ndarray) -> dict[str, list[float]]:
    """4x4 transform을 JSON 직렬화용 translation/quaternion으로 변환한다."""
    transform = np.asarray(matrix, dtype=float).reshape(4, 4)
    return {
        "translation_m": [float(value) for value in transform[:3, 3]],
        "quaternion_xyzw": [
            float(value) for value in matrix_quaternion(transform[:3, :3])
        ],
    }


def matrix_from_pose_record(record: dict[str, Any]) -> np.ndarray:
    """pose_record()가 만든 JSON 구조를 4x4 transform으로 역직렬화한다."""
    return transform_matrix(record["translation_m"], record["quaternion_xyzw"])


def residual_record(matrix: np.ndarray) -> dict[str, Any]:
    """상대 transform과 동일 정보를 갖는 SE(3) logarithm을 함께 반환한다."""
    record: dict[str, Any] = pose_record(matrix)
    record["se3_log_vw"] = [float(value) for value in se3_log(matrix)]
    return record


def matrix_from_residual_record(record: dict[str, Any]) -> np.ndarray:
    """residual_record()의 logarithm만 사용해 상대 transform을 복원한다."""
    return se3_exp(record["se3_log_vw"])


def pose_residual_labels(
    *,
    base_tcp: np.ndarray,
    base_cameras: dict[str, np.ndarray],
    base_plug_tip: np.ndarray,
    base_plug_reference: np.ndarray,
    base_port_entrance: np.ndarray,
) -> tuple[dict[str, Any], dict[str, Any], float]:
    """base pose와 상대 SE(3) residual을 만들고 역산 최대오차를 반환한다."""
    base_tcp = np.asarray(base_tcp, dtype=float).reshape(4, 4)
    base_cameras = {
        camera: np.asarray(matrix, dtype=float).reshape(4, 4)
        for camera, matrix in base_cameras.items()
    }
    base_plug_tip = np.asarray(base_plug_tip, dtype=float).reshape(4, 4)
    base_plug_reference = np.asarray(base_plug_reference, dtype=float).reshape(4, 4)
    base_port_entrance = np.asarray(base_port_entrance, dtype=float).reshape(4, 4)

    matrices = {
        "base_T_tcp": base_tcp,
        "base_T_plug_tip": base_plug_tip,
        "base_T_plug_reference": base_plug_reference,
        "base_T_port_entrance": base_port_entrance,
    }
    pose_records: dict[str, Any] = {
        name: pose_record(matrix) for name, matrix in matrices.items()
    }
    pose_records["base_T_cameras"] = {
        camera: pose_record(matrix) for camera, matrix in base_cameras.items()
    }

    tcp_plug = np.linalg.inv(base_tcp) @ base_plug_reference
    tip_reference = np.linalg.inv(base_plug_tip) @ base_plug_reference
    plug_port = np.linalg.inv(base_plug_reference) @ base_port_entrance
    camera_plug = {
        camera: np.linalg.inv(base_camera) @ base_plug_reference
        for camera, base_camera in base_cameras.items()
    }
    camera_port = {
        camera: np.linalg.inv(base_camera) @ base_port_entrance
        for camera, base_camera in base_cameras.items()
    }
    residual_records: dict[str, Any] = {
        "encoding": "se3_log_identity_reference",
        "order": list(SE3_LOG_ORDER),
        "tcp_T_plug_reference": residual_record(tcp_plug),
        "plug_tip_T_plug_reference": residual_record(tip_reference),
        "plug_reference_T_port_entrance": residual_record(plug_port),
        "camera_T_plug_reference": {
            camera: residual_record(matrix) for camera, matrix in camera_plug.items()
        },
        "camera_T_port_entrance": {
            camera: residual_record(matrix) for camera, matrix in camera_port.items()
        },
    }

    errors: list[float] = []

    def compare(actual: np.ndarray, reconstructed: np.ndarray) -> None:
        errors.append(
            float(
                np.max(
                    np.abs(
                        np.asarray(actual, dtype=float)
                        - np.asarray(reconstructed, dtype=float)
                    )
                )
            )
        )

    for name, matrix in matrices.items():
        compare(matrix, matrix_from_pose_record(pose_records[name]))
    for camera, matrix in base_cameras.items():
        compare(
            matrix,
            matrix_from_pose_record(pose_records["base_T_cameras"][camera]),
        )
    compare(
        tcp_plug,
        matrix_from_residual_record(residual_records["tcp_T_plug_reference"]),
    )
    compare(
        tip_reference,
        matrix_from_residual_record(
            residual_records["plug_tip_T_plug_reference"]
        ),
    )
    compare(
        plug_port,
        matrix_from_residual_record(
            residual_records["plug_reference_T_port_entrance"]
        ),
    )
    compare(
        base_plug_reference,
        base_tcp
        @ matrix_from_residual_record(residual_records["tcp_T_plug_reference"]),
    )
    compare(
        base_port_entrance,
        base_plug_reference
        @ matrix_from_residual_record(
            residual_records["plug_reference_T_port_entrance"]
        ),
    )
    for camera, base_camera in base_cameras.items():
        reconstructed_plug = matrix_from_residual_record(
            residual_records["camera_T_plug_reference"][camera]
        )
        reconstructed_port = matrix_from_residual_record(
            residual_records["camera_T_port_entrance"][camera]
        )
        compare(camera_plug[camera], reconstructed_plug)
        compare(camera_port[camera], reconstructed_port)
        compare(base_plug_reference, base_camera @ reconstructed_plug)
        compare(base_port_entrance, base_camera @ reconstructed_port)
    return pose_records, residual_records, max(errors, default=0.0)
