"""동기화된 camera image와 img2pos XYZ label 저장."""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import fcntl

import cv2
import numpy as np
from geometry_msgs.msg import Transform

from . import se3
from .geometry import pose_matrix, quaternion_matrix


SFP_REFERENCE_OFFSET = np.array([0.0, 0.0021125, 0.0], dtype=float)
PORT_VISIBILITY_FAILURE_PREFIX = "포트 가시성 부족:"
ROBOT_ARM_OCCLUSION_REASON = "robot_arm_occlusion"
ROBOT_ARM_DARK_THRESHOLD = 40
ROBOT_ARM_MIN_AREA_RATIO = 0.01
ROBOT_ARM_MASK_DILATION_PX = 8
ANNOTATION_DEPTH_MARGIN_M = 0.002
ANNOTATION_DEPTH_PATCH_RADIUS_PX = 2
ANNOTATION_DEPTH_NEAR_QUANTILE = 0.2
ANNOTATION_MIN_VISIBLE_KEYPOINTS = 2
ANNOTATION_PRESERVE_OCCLUDED_POLICIES = frozenset({"near-port", "reacquisition"})
ANNOTATION_DEPTH_IMAGE_SIZE_PX = (576, 512)
ANNOTATION_DEPTH_UPDATE_RATE_HZ = 5.0
SFP_RAIL_COUNT = 5
SFP_PORT_COUNT = 2
SC_CLASS_ID = SFP_RAIL_COUNT * SFP_PORT_COUNT
PORT_CLASS_NAMES = {
    **{
        rail * SFP_PORT_COUNT + port: f"SFP_{rail}{port}"
        for rail in range(SFP_RAIL_COUNT)
        for port in range(SFP_PORT_COUNT)
    },
    SC_CLASS_ID: "sc_port",
}
PORT_OUTER_SIZE_M = {
    "sfp": (0.016224, 0.013698),
    "sc": (0.025781, 0.009300),
}
POSE_RECONSTRUCTION_ATOL = 1e-9
DATASET_SCHEMA_VERSION = 12
TRIAL_CONSTANT_TRANSLATION_ATOL_M = 1e-5
TRIAL_CONSTANT_ROTATION_ATOL_RAD = 1e-4
CONNECTOR_CONSTANT_TRANSLATION_ATOL_M = 1e-9
CONNECTOR_CONSTANT_ROTATION_ATOL_RAD = 1e-9
CAMERA_CALIBRATION_PIXEL_ATOL = 1e-3
CAMERA_CALIBRATION_MATRIX_ATOL = 1e-9


def _transform_message_matrix(transform: Transform) -> np.ndarray:
    """geometry_msgs/Transform을 4x4 homogeneous transform으로 변환한다."""
    return se3.transform_matrix(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    )


def pose_residual_labels(
    *,
    base_tcp: np.ndarray,
    base_cameras: dict[str, np.ndarray],
    base_plug_tip: np.ndarray,
    base_plug_reference: np.ndarray,
    base_port_entrance: np.ndarray,
) -> tuple[dict[str, Any], dict[str, Any], float]:
    """pure SE(3) helper를 dataset API로 노출한다."""
    return se3.pose_residual_labels(
        base_tcp=base_tcp,
        base_cameras=base_cameras,
        base_plug_tip=base_plug_tip,
        base_plug_reference=base_plug_reference,
        base_port_entrance=base_port_entrance,
    )


def capture_pose_residual_labels(
    policy,
    observation,
    plug_tip_tf: Transform,
    plug_reference_tf: Transform,
    port_tf: Transform,
) -> tuple[dict[str, Any], dict[str, Any], float]:
    """동기화 observation과 TF에서 sample pose/residual label을 생성한다."""
    base_tcp = pose_matrix(observation.controller_state.tcp_pose)
    base_tool0 = base_tcp @ np.linalg.inv(policy._tool0_tcp)
    base_cameras = {
        camera: base_tool0 @ policy._tool0_optical[camera]
        for camera in ("left", "center", "right")
    }
    return pose_residual_labels(
        base_tcp=base_tcp,
        base_cameras=base_cameras,
        base_plug_tip=_transform_message_matrix(plug_tip_tf),
        base_plug_reference=_transform_message_matrix(plug_reference_tf),
        base_port_entrance=_transform_message_matrix(port_tf),
    )


def port_annotation_class(port_type: str, rail: int, port: int = 0) -> tuple[int, str]:
    """connector 위치를 YOLO class ID와 사람이 읽는 rail/port label로 변환한다."""
    if port_type == "sfp":
        if rail not in range(SFP_RAIL_COUNT) or port not in range(SFP_PORT_COUNT):
            raise ValueError(f"invalid SFP rail/port: rail={rail}, port={port}")
        class_id = rail * SFP_PORT_COUNT + port
        return class_id, PORT_CLASS_NAMES[class_id]
    if port_type == "sc":
        return SC_CLASS_ID, PORT_CLASS_NAMES[SC_CLASS_ID]
    raise ValueError(f"unsupported port type: {port_type}")


def benchmark_port_annotation(
    port_type: str,
    rail: int,
    port: int = 0,
    *,
    collapse_sfp_class: bool,
) -> tuple[int, str, str]:
    """학습 class는 보존하고 ReID benchmark에서만 SFP identity를 분리한다."""
    class_id, label = port_annotation_class(port_type, rail, port)
    if port_type == "sfp" and collapse_sfp_class:
        return 0, "sfp_port", f"{rail}{port}"
    instance_id = (
        f"nic_card_mount_{rail}/sfp_port_{port}"
        if port_type == "sfp"
        else f"sc_port_{rail}/sc_port_base"
    )
    return class_id, label, instance_id


def is_port_visibility_failure(detail: str) -> bool:
    """sample 저장 실패가 port 가시성 조건 때문인지 반환한다."""
    return detail.startswith(PORT_VISIBILITY_FAILURE_PREFIX)


def image_for_camera(observation, camera: str):
    """Observation에서 지정한 camera Image를 반환한다."""
    return getattr(observation, f"{camera}_image")


def camera_info_for(observation, camera: str):
    """Observation에서 지정한 camera의 CameraInfo를 반환한다."""
    return getattr(observation, f"{camera}_camera_info")


def image_to_bgr(policy, message, camera: str) -> np.ndarray | None:
    """ROS Image를 JPEG 저장에 사용할 BGR 배열로 변환한다."""
    if message is None or message.width == 0 or message.height == 0:
        return None
    try:
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
    except ValueError:
        policy.get_logger().warn(f"[PortOffsetCollect] Invalid {camera} buffer size")
        return None
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else image.copy()


def _camera_matrix(camera_info, image=None) -> np.ndarray | None:
    """CameraInfo intrinsic을 검증하고 필요하면 image 해상도로 변환한다."""
    if camera_info is None or len(camera_info.k) < 9:
        return None
    matrix = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
    if abs(matrix[0, 0]) < 1e-9 or abs(matrix[1, 1]) < 1e-9:
        return None
    if image is None:
        return matrix

    source_width = int(getattr(camera_info, "width", 0))
    source_height = int(getattr(camera_info, "height", 0))
    target_width = int(getattr(image, "width", 0))
    target_height = int(getattr(image, "height", 0))
    if min(source_width, source_height, target_width, target_height) <= 0:
        return None
    if source_width * target_height != source_height * target_width:
        return None

    # Gazebo depth/RGB sensor의 CameraInfo가 같은 topic에 교차로 발행될 수
    # 있다. 두 센서는 pose/FOV/aspect ratio가 같으므로 depth 보정값이
    # 들어와도 RGB image 해상도로 intrinsic을 확대하면 같은 ray가 된다.
    scale = np.diag(
        [target_width / source_width, target_height / source_height, 1.0]
    )
    return scale @ matrix


def _base_to_camera(policy, observation, camera: str) -> np.ndarray:
    """base_link 좌표를 camera optical 좌표로 변환하는 행렬을 반환한다."""
    base_tcp = pose_matrix(observation.controller_state.tcp_pose)
    base_tool0 = base_tcp @ np.linalg.inv(policy._tool0_tcp)
    return np.linalg.inv(base_tool0 @ policy._tool0_optical[camera])


def _points_projection(
    policy,
    observation,
    camera: str,
    points_base: list[list[float]],
) -> dict[str, Any]:
    """모든 base_link 점이 지정 camera의 유효 영상 영역에 보이는지 검사한다."""
    message = image_for_camera(observation, camera)
    camera_info = camera_info_for(observation, camera)
    intrinsic = _camera_matrix(camera_info, message)
    if message is None or message.width == 0 or message.height == 0 or intrinsic is None:
        return {"visible": False, "reason": "missing_image_or_intrinsics"}
    try:
        base_to_camera = _base_to_camera(policy, observation, camera)
        points_camera = [
            base_to_camera @ np.array([*point, 1.0], dtype=float)
            for point in points_base
        ]
    except Exception as exc:
        return {"visible": False, "reason": f"camera_transform_error: {exc}"}
    projections = []
    for point_camera in points_camera:
        depth = float(point_camera[2])
        if depth <= 1e-6:
            return {"visible": False, "reason": "behind_camera", "depth_m": depth}
        u = float(intrinsic[0, 0] * point_camera[0] / depth + intrinsic[0, 2])
        v = float(intrinsic[1, 1] * point_camera[1] / depth + intrinsic[1, 2])
        projections.append({"u_px": u, "v_px": v, "depth_m": depth})
    visible = all(
        0.0 <= projection["u_px"] < float(message.width)
        and 0.0 <= projection["v_px"] < float(message.height)
        for projection in projections
    )
    return {
        "visible": visible,
        "reason": "visible" if visible else "outside_image_bounds",
        "points": projections,
        "image_size_px": [int(message.width), int(message.height)],
    }


def _robot_arm_mask(image: np.ndarray) -> np.ndarray:
    """영상 아래쪽에 연결된 큰 검은 영역을 robot arm mask로 반환한다."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dark = (gray <= ROBOT_ARM_DARK_THRESHOLD).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    height, width = gray.shape
    minimum_area = int(height * width * ROBOT_ARM_MIN_AREA_RATIO)
    mask = np.zeros_like(dark)
    for label in range(1, count):
        _, y, _, component_height, area = stats[label]
        if y + component_height < height or area < minimum_area:
            continue
        component = np.where(labels == label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(mask, contours, -1, 1, thickness=cv2.FILLED)
    if np.any(mask):
        size = ROBOT_ARM_MASK_DILATION_PX * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.dilate(mask, kernel)
    return mask.astype(bool)


def _port_projection(policy, observation, camera: str, port_tf: Transform) -> dict[str, Any]:
    """포트 투영점이 유효 영상 안에 있고 검은 robot arm에 가리지 않는지 검사한다."""
    result = _points_projection(
        policy,
        observation,
        camera,
        [[port_tf.translation.x, port_tf.translation.y, port_tf.translation.z]],
    )
    if not result.get("visible", False):
        return result
    image = image_to_bgr(policy, image_for_camera(observation, camera), camera)
    if image is None:
        return {**result, "visible": False, "reason": "image_conversion_failed"}
    point = result["points"][0]
    u = int(np.clip(round(point["u_px"]), 0, image.shape[1] - 1))
    v = int(np.clip(round(point["v_px"]), 0, image.shape[0] - 1))
    mask = _robot_arm_mask(image)
    if mask[v, u]:
        return {**result, "visible": False, "reason": ROBOT_ARM_OCCLUSION_REASON}
    return result


def _port_outer_corners_base(port_type: str, port_tf: Transform) -> list[list[float]]:
    """port entrance 로컬 외곽 4점을 회전을 포함한 base_link 좌표로 변환한다."""
    width, height = PORT_OUTER_SIZE_M[port_type]
    rotation = quaternion_matrix(
        port_tf.rotation.x,
        port_tf.rotation.y,
        port_tf.rotation.z,
        port_tf.rotation.w,
    )
    translation = np.array(
        [port_tf.translation.x, port_tf.translation.y, port_tf.translation.z],
        dtype=float,
    )
    local_corners = (
        (-width / 2.0, height / 2.0, 0.0),
        (width / 2.0, height / 2.0, 0.0),
        (width / 2.0, -height / 2.0, 0.0),
        (-width / 2.0, -height / 2.0, 0.0),
    )
    return [
        [float(value) for value in translation + rotation @ np.asarray(corner)]
        for corner in local_corners
    ]


def _port_outer_projection(
    policy, observation, camera: str, annotation_port: dict[str, Any]
) -> dict[str, Any]:
    """port 외곽 4점을 camera image로 투영한다."""
    return _points_projection(
        policy,
        observation,
        camera,
        _port_outer_corners_base(
            annotation_port["port_type"], annotation_port["transform"]
        ),
    )


def _depth_image_meters(message) -> np.ndarray:
    """ROS depth Image의 32FC1/16UC1 buffer를 meter 단위 2D 배열로 변환한다."""
    encoding = str(getattr(message, "encoding", "")).upper()
    byte_order = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    if encoding == "32FC1":
        dtype = np.dtype(f"{byte_order}f4")
        scale = 1.0
    elif encoding in {"16UC1", "MONO16"}:
        dtype = np.dtype(f"{byte_order}u2")
        scale = 0.001
    else:
        raise ValueError(f"unsupported depth encoding: {encoding or 'empty'}")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    row_bytes = width * dtype.itemsize
    if height <= 0 or width <= 0 or step < row_bytes:
        raise ValueError(
            f"invalid depth dimensions: width={width}, height={height}, step={step}"
        )
    buffer = memoryview(message.data)
    if buffer.nbytes < step * height:
        raise ValueError(
            f"invalid depth buffer size: expected={step * height}, got={buffer.nbytes}"
        )
    image = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=buffer,
        strides=(step, dtype.itemsize),
    )
    return image.astype(np.float32, copy=True) * scale


def _depth_for_capture(policy, observation, camera: str) -> np.ndarray:
    """RGB 촬영 stamp와 허용 오차 안에서 가장 가까운 depth frame을 반환한다."""
    capture_stamp = _stamp_ns(image_for_camera(observation, camera).header.stamp)
    provider = getattr(policy, "depth_image_at", None)
    if capture_stamp is None or not callable(provider):
        raise ValueError(f"{camera}: synchronized depth provider unavailable")
    message = provider(camera, capture_stamp)
    depth_stamp = _stamp_ns(getattr(getattr(message, "header", None), "stamp", None))
    max_skew_ns = int(getattr(policy, "annotation_depth_max_skew_ns", 0))
    if (
        message is None
        or depth_stamp is None
        or abs(depth_stamp - capture_stamp) > max_skew_ns
    ):
        raise ValueError(
            f"{camera}: depth frame missing within {max_skew_ns}ns of "
            f"RGB stamp {capture_stamp}"
        )
    depth = _depth_image_meters(message)
    rgb = image_for_camera(observation, camera)
    if depth.shape[0] * int(rgb.width) != depth.shape[1] * int(rgb.height):
        raise ValueError(
            f"{camera}: depth/RGB aspect ratio mismatch: depth={depth.shape}, "
            f"rgb={(int(rgb.height), int(rgb.width))}"
        )
    return depth


def _keypoint_visibilities(
    projection: dict[str, Any],
    depth: np.ndarray,
    occlusion_mask: np.ndarray | None = None,
) -> tuple[int, ...]:
    """depth 또는 RGB robot mask가 앞을 가린 keypoint를 가림(1)으로 판정한다."""
    height, width = depth.shape
    rgb_width, rgb_height = projection["image_size_px"]
    scale_x = width / float(rgb_width)
    scale_y = height / float(rgb_height)
    visibilities = []
    radius = max(
        1,
        round(ANNOTATION_DEPTH_PATCH_RADIUS_PX * min(scale_x, scale_y)),
    )
    for point in projection["points"]:
        rgb_u = int(
            np.clip(round(float(point["u_px"])), 0, int(rgb_width) - 1)
        )
        rgb_v = int(
            np.clip(round(float(point["v_px"])), 0, int(rgb_height) - 1)
        )
        mask_occluded = bool(
            occlusion_mask is not None and occlusion_mask[rgb_v, rgb_u]
        )
        u = int(
            np.clip(
                round((float(point["u_px"]) + 0.5) * scale_x - 0.5),
                0,
                width - 1,
            )
        )
        v = int(
            np.clip(
                round((float(point["v_px"]) + 0.5) * scale_y - 0.5),
                0,
                height - 1,
            )
        )
        patch = depth[
            max(0, v - radius) : min(height, v + radius + 1),
            max(0, u - radius) : min(width, u + radius + 1),
        ]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            visibilities.append(1 if mask_occluded else 2)
            continue
        observed_depth = float(
            np.quantile(valid, ANNOTATION_DEPTH_NEAR_QUANTILE)
        )
        expected_depth = float(point["depth_m"])
        occluded = (
            mask_occluded
            or observed_depth + ANNOTATION_DEPTH_MARGIN_M < expected_depth
        )
        visibilities.append(1 if occluded else 2)
    return tuple(visibilities)


def _yolo_pose_row(
    class_id: int,
    projection: dict[str, Any],
    visibilities: tuple[int, ...] | None = None,
) -> str:
    """외곽 투영점에서 정규화 bbox와 4-keypoint YOLO pose row를 만든다."""
    width, height = projection["image_size_px"]
    points = projection["points"]
    xs = [float(point["u_px"]) / width for point in points]
    ys = [float(point["v_px"]) / height for point in points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    bbox = [
        (x_min + x_max) / 2.0,
        (y_min + y_max) / 2.0,
        x_max - x_min,
        y_max - y_min,
    ]
    values = [f"{value:.9f}" for value in bbox]
    if visibilities is None:
        visibilities = (2,) * len(points)
    if len(visibilities) != len(points):
        raise ValueError(
            f"visibility count mismatch: points={len(points)}, "
            f"visibilities={len(visibilities)}"
        )
    for x, y, visibility in zip(xs, ys, visibilities):
        values.extend((f"{x:.9f}", f"{y:.9f}", str(int(visibility))))
    return " ".join([str(int(class_id)), *values])


def _annotation_relative_path(image_relative_path: Path) -> Path:
    """images 상대 경로에 대응하는 annotations TXT 상대 경로를 반환한다."""
    return Path("annotations", *image_relative_path.parts[1:]).with_suffix(".txt")


def _annotation_rows_by_camera(
    policy,
    observation,
    annotation_ports: list[dict[str, Any]] | tuple[dict[str, Any], ...],
) -> tuple[dict[str, list[str]], dict[str, list[str]], dict[str, list[str]]]:
    """camera별로 화면 안에 완전히 투영된 모든 port의 YOLO pose row를 만든다."""
    rows = {camera: [] for camera in ("left", "center", "right")}
    labels = {camera: [] for camera in rows}
    instances = {camera: [] for camera in rows}
    preserve_occluded = (
        str(getattr(policy, "collection_policy", ""))
        in ANNOTATION_PRESERVE_OCCLUDED_POLICIES
    )
    for camera in rows:
        depth_visibility = bool(
            getattr(policy, "auto_annotation_visibility", False)
        )
        depth = (
            _depth_for_capture(policy, observation, camera)
            if depth_visibility
            else None
        )
        occlusion_mask = None
        if depth_visibility and preserve_occluded:
            image = image_to_bgr(policy, image_for_camera(observation, camera), camera)
            if image is None:
                raise ValueError(f"{camera}: RGB robot-arm mask image unavailable")
            occlusion_mask = _robot_arm_mask(image)
        for port in annotation_ports:
            projection = _port_outer_projection(policy, observation, camera, port)
            if projection.get("visible", False):
                visibilities = (
                    _keypoint_visibilities(
                        projection,
                        depth,
                        occlusion_mask=occlusion_mask,
                    )
                    if depth is not None
                    else (2,) * len(projection["points"])
                )
                if (
                    not preserve_occluded
                    and visibilities.count(2) < ANNOTATION_MIN_VISIBLE_KEYPOINTS
                ):
                    continue
                rows[camera].append(
                    _yolo_pose_row(port["class_id"], projection, visibilities)
                )
                labels[camera].append(port["label"])
                instances[camera].append(str(port["instance_id"]))
    return rows, labels, instances


def _stamp_ns(stamp) -> int | None:
    """ROS Time 메시지를 nanosecond 정수로 변환한다."""
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def observation_sync(policy, observation) -> tuple[bool, dict[str, Any]]:
    """세 camera와 controller source timestamp가 허용 오차 안인지 검사한다."""
    timestamps: dict[str, Any] = {
        "clock": "ros",
        "unit": "nanoseconds",
        "sync_tolerance_ns": policy.sync_tolerance_ns,
        "sync_valid": False,
    }
    if observation is None:
        timestamps["rejection_reason"] = "missing_observation"
        return False, timestamps
    image_stamps = {}
    for camera in ("left", "center", "right"):
        message = image_for_camera(observation, camera)
        image_stamps[camera] = _stamp_ns(
            getattr(getattr(message, "header", None), "stamp", None)
        )
    controller_stamp = _stamp_ns(
        getattr(
            getattr(getattr(observation, "controller_state", None), "header", None),
            "stamp",
            None,
        )
    )
    timestamps.update(images=image_stamps, controller_stamp_ns=controller_stamp)
    missing = [f"image:{name}" for name, stamp in image_stamps.items() if not stamp]
    if not controller_stamp:
        missing.append("controller")
    if missing:
        timestamps.update(rejection_reason="missing_or_zero_timestamp", missing_sources=missing)
        return False, timestamps
    capture_stamp = int(image_stamps["center"])
    camera_skew = max(image_stamps.values()) - min(image_stamps.values())
    controller_skew = abs(int(controller_stamp) - capture_stamp)
    timestamps.update(
        capture_stamp_ns=capture_stamp,
        skew_ns={"camera": int(camera_skew), "controller": int(controller_skew)},
    )
    if camera_skew != 0:
        timestamps["rejection_reason"] = "camera_timestamp_mismatch"
        return False, timestamps
    if controller_skew > policy.sync_tolerance_ns:
        timestamps["rejection_reason"] = "controller_time_difference_exceeded"
        return False, timestamps
    timestamps["sync_valid"] = True
    return True, timestamps


def wait_for_observation(policy, get_observation, command_stamp_ns: int):
    """명령보다 새롭고 source 시각이 일치하는 Observation을 제한 시간 동안 기다린다."""
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(policy.sync_wait_timeout_s * 1_000_000_000)
    logged = False
    timestamps: dict[str, Any] = {"sync_valid": False, "rejection_reason": "missing_observation"}
    while True:
        observation = get_observation()
        valid, timestamps = observation_sync(policy, observation)
        capture_stamp = timestamps.get("capture_stamp_ns")
        if valid and capture_stamp is not None and int(capture_stamp) <= command_stamp_ns:
            valid = False
            timestamps.update(sync_valid=False, rejection_reason="capture_not_after_command")
        now_ns = time.monotonic_ns()
        timestamps.setdefault("wait_ns", {})["observation"] = now_ns - start_ns
        if valid:
            return observation, timestamps
        if now_ns >= deadline_ns:
            timestamps["rejection_reason"] = (
                "observation_sync_timeout:"
                f"{timestamps.get('rejection_reason', 'unknown')}"
            )
            return None, timestamps
        if not logged:
            policy.get_logger().info(
                policy.log_text(
                    "[PortOffsetCollect] Waiting for synchronized Observation: "
                    f"timeout={policy.sync_wait_timeout_s:.3f}s",
                    "cyan",
                )
            )
            logged = True
        policy.sleep_for(min(policy.sync_poll_s, (deadline_ns - now_ns) / 1e9))


def tf_sync(
    policy,
    timestamps: dict[str, Any],
    transforms: dict[str, Any],
    *,
    static_sources: set[str] | None = None,
) -> tuple[bool, dict[str, Any]]:
    """촬영 시각으로 조회한 TF의 timestamp가 허용 오차 안인지 검사한다."""
    capture_stamp = int(timestamps.get("capture_stamp_ns", 0))
    static_sources = static_sources or set()
    records: dict[str, Any] = {}
    skews: list[int] = []
    for name, stamped in transforms.items():
        header = getattr(stamped, "header", None)
        stamp = _stamp_ns(getattr(header, "stamp", None))
        if header is None or stamp is None:
            timestamps.update(sync_valid=False, rejection_reason=f"missing_tf_timestamp:{name}")
            return False, timestamps
        static_snapshot = name in static_sources
        static = stamp == 0 or static_snapshot
        skew = 0 if static else abs(stamp - capture_stamp)
        records[name] = {
            "stamp_ns": stamp,
            "parent_frame_id": str(getattr(header, "frame_id", "")),
            "child_frame_id": str(getattr(stamped, "child_frame_id", "")),
            "is_static": static,
            "is_static_snapshot": static_snapshot,
            "skew_ns": skew,
        }
        skews.append(skew)
    timestamps["tf"] = records
    timestamps.setdefault("skew_ns", {})["tf"] = max(skews, default=0)
    if max(skews, default=0) > policy.sync_tolerance_ns:
        timestamps.update(sync_valid=False, rejection_reason="tf_time_difference_exceeded")
        return False, timestamps
    timestamps["sync_valid"] = True
    timestamps.pop("rejection_reason", None)
    return True, timestamps


def shift_origin(transform: Transform, local_offset) -> Transform:
    """Transform 원점을 자체 좌표계의 local offset만큼 이동한다."""
    rotation = quaternion_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=float,
    ) + rotation @ np.asarray(local_offset, dtype=float)
    shifted = Transform()
    shifted.translation.x, shifted.translation.y, shifted.translation.z = map(float, translation)
    shifted.rotation = transform.rotation
    return shifted


def target_xyz(port_tf: Transform, plug_tf: Transform) -> list[float]:
    """촬영 시점의 port-entrance minus plug-reference XYZ label을 반환한다."""
    return [
        float(port_tf.translation.x - plug_tf.translation.x),
        float(port_tf.translation.y - plug_tf.translation.y),
        float(port_tf.translation.z - plug_tf.translation.z),
    ]


def plug_reference_offset(task, cable_tip_frame: str) -> np.ndarray:
    """plug TF 원점에서 물리 기준점까지의 local offset을 반환한다."""
    text = f"{task.plug_type} {task.plug_name} {cable_tip_frame}".lower()
    if "sfp" not in text:
        return np.zeros(3, dtype=float)
    return np.array(
        [
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_X", SFP_REFERENCE_OFFSET[0])),
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_Y", SFP_REFERENCE_OFFSET[1])),
            float(os.environ.get("AIC_SFP_PLUG_REFERENCE_Z", SFP_REFERENCE_OFFSET[2])),
        ],
        dtype=float,
    )


def _trial_id(policy, episode_name: str) -> str:
    """run metadata로 trial ID를 만들고 없으면 episode 이름을 반환한다."""
    run_id = policy.run_id
    trial_index = policy.trial_index
    return (
        f"{run_id}:{trial_index}"
        if run_id and trial_index not in (None, "")
        else episode_name
    )


def split_for_trial(policy, trial_id: str) -> str:
    """같은 trial을 안정적으로 동일한 train/val/test split에 배정한다."""
    if policy.trial_split in {"train", "val", "test"}:
        return policy.trial_split
    val_ratio = min(max(float(policy.val_ratio), 0.0), 1.0)
    test_ratio = min(max(float(policy.test_ratio), 0.0), 1.0 - val_ratio)
    fraction = int.from_bytes(hashlib.sha256(trial_id.encode()).digest()[:8], "big") / float(1 << 64)
    if fraction < test_ratio:
        return "test"
    return "val" if fraction < test_ratio + val_ratio else "train"


def _connector(task) -> str:
    """학습 sample에 기록할 connector 타입을 반환한다."""
    text = " ".join(
        str(getattr(task, name, ""))
        for name in ("port_type", "port_name", "plug_type", "plug_name")
    ).lower()
    if "sfp" in text:
        return "SFP"
    return "SC" if "sc" in text else "UNKNOWN"


def _last_number(value: Any, default: str = "x") -> str:
    """문자열의 마지막 숫자 묶음을 반환한다."""
    matches = re.findall(r"\d+", str(value))
    return matches[-1] if matches else default


def _trial_index(policy, task) -> int:
    """policy 또는 task ID에서 숫자 trial index를 반환한다."""
    trial_index = getattr(policy, "trial_index", None)
    if trial_index in (None, ""):
        trial_match = re.search(
            r"portoffset_(?:sfp|sc)_(\d+)", str(getattr(task, "id", ""))
        )
        trial_index = int(trial_match.group(1)) if trial_match else 0
    return int(trial_index)


def _card_mask(task) -> str:
    """task ID bitmask를 rail 0부터 읽는 순서로 뒤집어 반환한다."""
    match = re.search(r"(?:^|_)cards(\d+)(?:_|$)", str(getattr(task, "id", "")))
    return match.group(1)[::-1] if match else "unknown"


def _compact_capture_id(policy, episode_name: str, task, step_idx: int) -> str:
    """시간, trial, 목표 port와 sample만 남긴 짧은 capture ID를 만든다."""
    timestamp_match = re.match(r"(\d{8})_(\d{6})", episode_name)
    timestamp = (
        f"{timestamp_match.group(1)}-{timestamp_match.group(2)}"
        if timestamp_match
        else str(int(time.time()))
    )
    trial_index = _trial_index(policy, task)
    connector = _connector(task).lower()
    rail = _last_number(getattr(task, "target_module_name", ""))
    target = f"{connector}-r{rail}"
    if connector == "sfp":
        port = _last_number(getattr(task, "port_name", ""))
        target += f"-p{port}"
    return f"{timestamp}_t{trial_index:04d}_{target}_s{step_idx:03d}"


def _image_relative_path(policy, task, step_idx: int, split: str, camera: str) -> Path:
    """사람이 읽기 쉬운 camera별 trial image 상대 경로를 만든다."""
    connector = _connector(task).lower()
    rail = _last_number(getattr(task, "target_module_name", ""))
    target = f"{connector}_card_{_card_mask(task)}_rail{rail}"
    if connector == "sfp":
        port = _last_number(getattr(task, "port_name", ""))
        target += f"_port{port}"
    trial_dir = f"trial_{_trial_index(policy, task):03d}"
    filename = f"{target}_num{step_idx + 1:03d}_{camera}.jpg"
    return Path("images") / split / camera / trial_dir / filename


def _visibility_detail(visibility: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """모든 camera의 가시성 결과를 동일한 log 구조로 정리한다."""
    detail = {}
    for camera, result in visibility.items():
        camera_detail: dict[str, Any] = {
            "visible": bool(result.get("visible", False)),
            "reason": result.get("reason")
            or ("visible" if result.get("visible", False) else "unknown"),
        }
        points = result.get("points", ())
        if points:
            camera_detail["projection_px"] = [
                [round(point["u_px"], 1), round(point["v_px"], 1)]
                for point in points
            ]
        if result.get("image_size_px"):
            camera_detail["image_size_px"] = result["image_size_px"]
        detail[camera] = camera_detail
    return detail


def _max_skew(timestamps: dict[str, Any]) -> int:
    """승인 source들의 최대 시각 차이를 반환한다."""
    try:
        return max((int(value) for value in timestamps.get("skew_ns", {}).values()), default=0)
    except (TypeError, ValueError):
        return 0


@contextmanager
def _dataset_write_lock(policy):
    """병렬 policy가 manifest와 JSONL을 동시에 수정하지 못하게 막는다."""
    policy.dataset_dir.mkdir(parents=True, exist_ok=True)
    lock_path = policy.dataset_dir / ".write.lock"
    with lock_path.open("a", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    """JSON object를 같은 directory의 임시 파일을 거쳐 원자적으로 교체한다."""
    temporary = path.parent / f".{path.name}.{os.getpid()}.tmp"
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _camera_calibration_record(observation, camera: str) -> dict[str, Any]:
    """RGB image 해상도에 맞춘 ROS CameraInfo calibration을 직렬화한다."""
    image = image_for_camera(observation, camera)
    camera_info = camera_info_for(observation, camera)
    intrinsic = _camera_matrix(camera_info, image)
    if image is None or camera_info is None or intrinsic is None:
        raise ValueError(f"{camera}: camera calibration을 만들 수 없음")

    source_width = int(getattr(camera_info, "width", 0))
    source_height = int(getattr(camera_info, "height", 0))
    image_width = int(getattr(image, "width", 0))
    image_height = int(getattr(image, "height", 0))
    if min(source_width, source_height, image_width, image_height) <= 0:
        raise ValueError(f"{camera}: invalid camera/image size")
    scale = np.diag(
        [image_width / source_width, image_height / source_height, 1.0]
    )
    projection_values = list(getattr(camera_info, "p", ()))
    projection = (
        scale @ np.asarray(projection_values, dtype=float).reshape(3, 4)
        if len(projection_values) == 12
        else np.column_stack((intrinsic, np.zeros(3, dtype=float)))
    )
    rectification_values = list(getattr(camera_info, "r", ()))
    rectification = (
        np.asarray(rectification_values, dtype=float).reshape(3, 3)
        if len(rectification_values) == 9
        else np.eye(3, dtype=float)
    )
    header = getattr(camera_info, "header", None)
    roi = getattr(camera_info, "roi", None)
    return {
        "frame_id": str(getattr(header, "frame_id", "")),
        "image_size_px": [image_width, image_height],
        "distortion_model": str(
            getattr(camera_info, "distortion_model", "")
        ),
        "D": [float(value) for value in getattr(camera_info, "d", ())],
        "K": [float(value) for value in intrinsic.reshape(-1)],
        "R": [float(value) for value in rectification.reshape(-1)],
        "P": [float(value) for value in projection.reshape(-1)],
        "binning": [
            int(getattr(camera_info, "binning_x", 0)),
            int(getattr(camera_info, "binning_y", 0)),
        ],
        "roi": {
            "x_offset": int(getattr(roi, "x_offset", 0)),
            "y_offset": int(getattr(roi, "y_offset", 0)),
            "height": int(getattr(roi, "height", 0)),
            "width": int(getattr(roi, "width", 0)),
            "do_rectify": bool(getattr(roi, "do_rectify", False)),
        },
    }


def _dataset_constants_candidate(policy, observation) -> dict[str, Any]:
    """dataset 전체에서 공유하는 camera와 좌표계 상수를 만든다."""
    return {
        "schema_version": DATASET_SCHEMA_VERSION,
        "pose_convention": "parent_T_child",
        "pose_representation": "translation_m + quaternion_xyzw",
        "se3": {
            "encoding": "se3_log_identity_reference",
            "order": list(se3.SE3_LOG_ORDER),
            "reconstruction_tolerance": POSE_RECONSTRUCTION_ATOL,
        },
        "constant_validation": {
            "connector_translation_tolerance_m": (
                CONNECTOR_CONSTANT_TRANSLATION_ATOL_M
            ),
            "connector_rotation_tolerance_rad": (
                CONNECTOR_CONSTANT_ROTATION_ATOL_RAD
            ),
            "trial_translation_tolerance_m": TRIAL_CONSTANT_TRANSLATION_ATOL_M,
            "trial_rotation_tolerance_rad": TRIAL_CONSTANT_ROTATION_ATOL_RAD,
            "trial_port_pose_only": True,
            "tcp_plug_mode": "nominal_plus_residual",
            "camera_calibration_pixel_atol": CAMERA_CALIBRATION_PIXEL_ATOL,
            "camera_calibration_matrix_atol": CAMERA_CALIBRATION_MATRIX_ATOL,
        },
        "static_transforms": {
            "tool0_T_tcp": se3.pose_record(policy._tool0_tcp),
            "tool0_T_camera_optical": {
                camera: se3.pose_record(policy._tool0_optical[camera])
                for camera in ("left", "center", "right")
            },
        },
        "camera_calibration": {
            camera: _camera_calibration_record(observation, camera)
            for camera in ("left", "center", "right")
        },
        "connector_geometry": {},
    }


def _transform_error(
    reference: np.ndarray,
    actual: np.ndarray,
) -> tuple[float, float, np.ndarray]:
    """reference^{-1} actual의 translation/rotation 크기와 SE(3) log를 반환한다."""
    delta = np.linalg.inv(reference) @ actual
    residual = se3.se3_log(delta)
    return (
        float(np.linalg.norm(delta[:3, 3])),
        float(np.linalg.norm(residual[3:])),
        residual,
    )


def _verify_camera_calibration(
    existing: dict[str, Any],
    candidate: dict[str, Any],
) -> None:
    """RGB/depth CameraInfo의 미세 차이를 허용하며 calibration을 검증한다."""
    if set(existing) != set(candidate):
        raise ValueError("dataset constant mismatch: camera_calibration cameras")

    exact_keys = {
        "frame_id",
        "image_size_px",
        "distortion_model",
        "binning",
        "roi",
    }
    matrix_tolerances = {
        "D": CAMERA_CALIBRATION_MATRIX_ATOL,
        "K": CAMERA_CALIBRATION_PIXEL_ATOL,
        "R": CAMERA_CALIBRATION_MATRIX_ATOL,
        "P": CAMERA_CALIBRATION_PIXEL_ATOL,
    }
    for camera in sorted(existing):
        reference = existing[camera]
        actual = candidate[camera]
        for key in exact_keys:
            if reference.get(key) != actual.get(key):
                raise ValueError(
                    "dataset constant mismatch: camera_calibration "
                    f"{camera}.{key}"
                )
        for key, tolerance in matrix_tolerances.items():
            reference_values = np.asarray(reference.get(key, ()), dtype=float)
            actual_values = np.asarray(actual.get(key, ()), dtype=float)
            if reference_values.shape != actual_values.shape:
                raise ValueError(
                    "dataset constant mismatch: camera_calibration "
                    f"{camera}.{key} shape"
                )
            if not np.allclose(
                reference_values,
                actual_values,
                rtol=0.0,
                atol=tolerance,
            ):
                max_error = float(np.max(np.abs(reference_values - actual_values)))
                raise ValueError(
                    "dataset constant mismatch: camera_calibration "
                    f"{camera}.{key} max_abs_error={max_error:.3e} "
                    f"tolerance={tolerance:.3e}"
                )


def _verify_same_constants(
    existing: dict[str, Any],
    candidate: dict[str, Any],
) -> None:
    """worker마다 관측한 dataset 공통 상수를 허용 오차 내에서 검사한다."""
    for key in (
        "schema_version",
        "pose_convention",
        "pose_representation",
        "se3",
        "constant_validation",
        "static_transforms",
    ):
        if existing.get(key) != candidate.get(key):
            raise ValueError(f"dataset constant mismatch: {key}")
    _verify_camera_calibration(
        existing.get("camera_calibration", {}),
        candidate.get("camera_calibration", {}),
    )


def _ensure_dataset_constants(
    policy,
    observation,
    connector: str,
    plug_tip_reference: np.ndarray,
) -> np.ndarray:
    """constants.json을 생성/검증하고 connector 기준 변환을 반환한다."""
    candidate = _dataset_constants_candidate(policy, observation)
    path = policy.constants_path
    with _dataset_write_lock(policy):
        if path.is_file():
            existing = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(existing, dict):
                raise ValueError("constants.json must contain an object")
            _verify_same_constants(existing, candidate)
        else:
            existing = candidate

        geometries = existing.setdefault("connector_geometry", {})
        geometry = geometries.get(connector)
        if geometry is None:
            geometries[connector] = {
                "plug_tip_T_plug_reference": se3.pose_record(
                    plug_tip_reference
                )
            }
            _atomic_write_json(path, existing)
            return plug_tip_reference

        nominal = se3.matrix_from_pose_record(
            geometry["plug_tip_T_plug_reference"]
        )
        translation_error, rotation_error, _ = _transform_error(
            nominal, plug_tip_reference
        )
        if (
            translation_error > CONNECTOR_CONSTANT_TRANSLATION_ATOL_M
            or rotation_error > CONNECTOR_CONSTANT_ROTATION_ATOL_RAD
        ):
            raise ValueError(
                "connector constant mismatch: "
                f"{connector} plug_tip_T_plug_reference "
                f"translation={translation_error:.3e}m, "
                f"rotation={rotation_error:.3e}rad"
            )
        if not path.is_file():
            _atomic_write_json(path, existing)
        return nominal


def _read_trial_record(path: Path, trial_id: str) -> dict[str, Any] | None:
    """trials.jsonl에서 지정 trial row를 찾아 반환한다."""
    if not path.is_file():
        return None
    match = None
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError(f"{path}:{line_number}: expected object")
            if str(row.get("trial_id")) != trial_id:
                continue
            if match is not None:
                raise ValueError(f"duplicate trial row: {trial_id}")
            match = row
    return match


def _ensure_trial_record(
    policy,
    *,
    trial_id: str,
    split: str,
    task,
    connector: str,
    base_port: np.ndarray,
    tcp_plug_reference: np.ndarray,
    port_stamp_ns: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """trial 상수를 한 번 기록하고 매 capture의 상수 drift residual을 반환한다."""
    cached = getattr(policy, "_normalized_trial_record", None)
    if cached is None:
        candidate = {
            "trial_id": trial_id,
            "run_id": policy.run_id,
            "trial_index": policy.trial_index,
            "task_id": str(getattr(task, "id", "")),
            "split": split,
            "connector": connector,
            "collection_policy": policy.collection_policy,
            "scenario": policy.trial_metadata,
            "pose_source_stamps_ns": {"port_tf": int(port_stamp_ns)},
            "poses": {
                "base_T_port_entrance": se3.pose_record(base_port),
            },
            "transforms": {
                "tcp_T_plug_reference_nominal": se3.pose_record(
                    tcp_plug_reference
                ),
            },
        }
        with _dataset_write_lock(policy):
            existing = _read_trial_record(policy.trials_path, trial_id)
            if existing is None:
                with policy.trials_path.open("a", encoding="utf-8") as stream:
                    stream.write(
                        json.dumps(candidate, ensure_ascii=False) + "\n"
                    )
                existing = candidate
            elif existing != candidate:
                raise ValueError(f"trial constant mismatch: {trial_id}")
        policy._normalized_trial_record = existing
        cached = existing

    nominal_port = se3.matrix_from_pose_record(
        cached["poses"]["base_T_port_entrance"]
    )
    nominal_tcp_plug = se3.matrix_from_pose_record(
        cached["transforms"]["tcp_T_plug_reference_nominal"]
    )
    port_translation_error, port_rotation_error, port_delta = _transform_error(
        nominal_port, base_port
    )
    tcp_translation_error, tcp_rotation_error, tcp_delta = _transform_error(
        nominal_tcp_plug, tcp_plug_reference
    )
    if port_translation_error > TRIAL_CONSTANT_TRANSLATION_ATOL_M:
        raise ValueError(
            "trial translation constant drift: "
            f"port={port_translation_error:.3e}m, "
            f"tcp_plug={tcp_translation_error:.3e}m"
        )
    if port_rotation_error > TRIAL_CONSTANT_ROTATION_ATOL_RAD:
        raise ValueError(
            "trial rotation constant drift: "
            f"port={port_rotation_error:.3e}rad, "
            f"tcp_plug={tcp_rotation_error:.3e}rad"
        )
    return nominal_port, nominal_tcp_plug, port_delta, tcp_delta


def _log_only(record: dict[str, Any]) -> dict[str, list[float]]:
    """상대 transform record에서 학습·역산에 필요한 SE(3) log만 남긴다."""
    return {
        "se3_log_vw": [float(value) for value in record["se3_log_vw"]]
    }


def _normalized_sample_residuals(
    residuals: dict[str, Any],
    *,
    connector_delta: np.ndarray,
    port_delta: np.ndarray,
    tcp_plug_delta: np.ndarray,
) -> dict[str, Any]:
    """dataset/trial 상수를 제외한 capture별 residual만 직렬화한다."""
    return {
        "constant_deltas": {
            "plug_tip_T_plug_reference": {
                "se3_log_vw": [float(value) for value in connector_delta]
            },
        },
        "trial_deltas": {
            "base_T_port_entrance": {
                "se3_log_vw": [float(value) for value in port_delta]
            },
            "tcp_T_plug_reference": {
                "se3_log_vw": [float(value) for value in tcp_plug_delta]
            },
        },
        "plug_reference_T_port_entrance": _log_only(
            residuals["plug_reference_T_port_entrance"]
        ),
        "camera_T_plug_reference": {
            camera: _log_only(record)
            for camera, record in residuals["camera_T_plug_reference"].items()
        },
        "camera_T_port_entrance": {
            camera: _log_only(record)
            for camera, record in residuals["camera_T_port_entrance"].items()
        },
    }


def _normalized_reconstruction_error(
    *,
    poses: dict[str, Any],
    residuals: dict[str, Any],
    nominal_connector: np.ndarray,
    nominal_port: np.ndarray,
    nominal_tcp_plug: np.ndarray,
) -> float:
    """constants+trial+sample 세 파일을 합쳤을 때의 최대 pose 역산 오차를 계산한다."""
    base_tcp = se3.matrix_from_pose_record(poses["base_T_tcp"])
    base_tip = se3.matrix_from_pose_record(poses["base_T_plug_tip"])
    base_plug = se3.matrix_from_pose_record(poses["base_T_plug_reference"])
    base_cameras = {
        camera: se3.matrix_from_pose_record(record)
        for camera, record in poses["base_T_cameras"].items()
    }
    connector_delta = se3.se3_exp(
        residuals["constant_deltas"]["plug_tip_T_plug_reference"][
            "se3_log_vw"
        ]
    )
    port_delta = se3.se3_exp(
        residuals["trial_deltas"]["base_T_port_entrance"]["se3_log_vw"]
    )
    tcp_delta = se3.se3_exp(
        residuals["trial_deltas"]["tcp_T_plug_reference"]["se3_log_vw"]
    )
    base_port = nominal_port @ port_delta
    errors = [
        float(np.max(np.abs(base_tip @ nominal_connector @ connector_delta - base_plug))),
        float(np.max(np.abs(base_tcp @ nominal_tcp_plug @ tcp_delta - base_plug))),
    ]

    plug_port = se3.se3_exp(
        residuals["plug_reference_T_port_entrance"]["se3_log_vw"]
    )
    errors.append(float(np.max(np.abs(base_plug @ plug_port - base_port))))
    for camera, base_camera in base_cameras.items():
        camera_plug = se3.se3_exp(
            residuals["camera_T_plug_reference"][camera]["se3_log_vw"]
        )
        camera_port = se3.se3_exp(
            residuals["camera_T_port_entrance"][camera]["se3_log_vw"]
        )
        errors.append(float(np.max(np.abs(base_camera @ camera_plug - base_plug))))
        errors.append(float(np.max(np.abs(base_camera @ camera_port - base_port))))
    return max(errors, default=0.0)


def _write_yolo_pose_config(policy) -> None:
    """Ultralytics가 annotations를 labels로 찾을 수 있는 symlink와 YAML을 만든다."""
    labels_path = policy.dataset_dir / "labels"
    if labels_path.is_symlink():
        if os.readlink(labels_path) != "annotations":
            raise OSError(f"unexpected labels symlink: {labels_path}")
    elif labels_path.exists():
        raise OSError(f"labels path already exists and is not a symlink: {labels_path}")
    else:
        try:
            labels_path.symlink_to("annotations", target_is_directory=True)
        except FileExistsError:
            if not labels_path.is_symlink() or os.readlink(labels_path) != "annotations":
                raise
    class_names = (
        {0: "sfp_port"}
        if bool(getattr(policy, "reid_benchmark_labels", False))
        else PORT_CLASS_NAMES
    )
    content = "\n".join(
        [
            "train: images/train",
            "val: images/val",
            "test: images/test",
            "names:",
            *(f"  {class_id}: {name}" for class_id, name in class_names.items()),
            "kpt_shape: [4, 3]",
            "flip_idx: [0, 1, 2, 3]",
            "",
        ]
    )
    temporary = policy.dataset_dir / f".yolo_pose.yaml.{os.getpid()}.tmp"
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(policy.dataset_dir / "yolo_pose.yaml")


def write_manifest(policy) -> None:
    """compact img2pos 데이터셋 schema를 data.yaml에 기록한다."""
    policy.dataset_dir.mkdir(parents=True, exist_ok=True)
    sampling = [
        "sampling:",
        f"  collection_policy: {policy.collection_policy}",
        f"  minimum_clearance_mm: {policy.base_z_offset * 1000.0:g}",
    ]
    if policy.collection_policy == "board-view":
        sampling.extend(
            [
                "  optical_distance_mm: "
                f"[{policy.board_distance_range[0] * 1000.0:g}, "
                f"{policy.board_distance_range[1] * 1000.0:g}]",
                f"  lateral_limit_mm: {policy.board_lateral_limit * 1000.0:g}",
                f"  angle_limit_rad: {policy.board_angle_limit:g}",
            ]
        )
    elif policy.collection_policy == "descent":
        sampling.extend(
            [
                "  approach_distance_mm: "
                f"[{policy.descent_start_distance * 1000.0:g}, "
                f"{policy.base_z_offset * 1000.0:g}]",
                f"  lateral_limit_mm: {policy.descent_lateral_limit * 1000.0:g}",
                f"  angle_limit_rad: {policy.descent_angle_limit:g}",
            ]
        )
    elif policy.collection_policy == "near-port":
        sampling.extend(
            [
                "  position_tiers_mm: ["
                + ", ".join(
                    f"{value * 1000.0:g}" for value in policy.sampling_tiers_m
                )
                + "]",
                "  minimum_actual_capture_clearance_mm: "
                f"{policy.min_capture_clearance * 1000.0:g}",
            ]
        )
    else:
        sampling.extend(
            [
                f"  events_per_trial: {policy.collect_steps}",
                f"  visible_distance_mm: {policy.reid_baseline_distance_m * 1000.0:g}",
                f"  occlusion_distance_mm: {policy.reid_occlusion_distance_m * 1000.0:g}",
                f"  phase_hold_s: {policy.reid_phase_hold_s:g}",
                "  phase_topic: /reid_benchmark/phase",
            ]
        )
    if policy.collection_policy in {"descent", "near-port"}:
        sampling.extend(
            [
                f"  haptic_guard: {str(policy.haptic_guard_enabled).lower()}",
                f"  haptic_force_threshold_n: {policy.haptic_force_threshold_n:g}",
                f"  haptic_contact_duration_s: {policy.haptic_contact_duration_s:g}",
                f"  haptic_baseline_samples: {policy.haptic_baseline_samples}",
            ]
        )
    auto_annotate_ports = bool(getattr(policy, "auto_annotate_ports", False))
    depth_visibility = bool(
        getattr(policy, "auto_annotation_visibility", False)
    )
    annotations = ["annotations:", f"  enabled: {str(auto_annotate_ports).lower()}"]
    if auto_annotate_ports:
        annotations.extend(
            [
                "  format: yolo_pose",
                "  layout: annotations/<split>/<camera>/trial_<index>/*.txt",
                "  class_names: yolo_pose.yaml#names",
                "  identity_field: samples.jsonl#annotation_instance_ids",
                "  keypoint_order: [x_min_y_max, x_max_y_max, x_max_y_min, x_min_y_min]",
                "  keypoint_shape: [4, 3]",
                "  port_outer_size_mm: {sfp: [16.224, 13.698], sc: [25.781, 9.300]}",
            ]
        )
        if depth_visibility:
            preserve_occluded = (
                policy.collection_policy in ANNOTATION_PRESERVE_OCCLUDED_POLICIES
            )
            annotations.extend(
                [
                    "  visibility_source: synchronized_depth_with_rgb_robot_mask",
                    "  depth_image_size_px: "
                    f"[{ANNOTATION_DEPTH_IMAGE_SIZE_PX[0]}, {ANNOTATION_DEPTH_IMAGE_SIZE_PX[1]}]",
                    f"  depth_update_rate_hz: {ANNOTATION_DEPTH_UPDATE_RATE_HZ:g}",
                    "  max_depth_rgb_skew_ms: "
                    f"{policy.annotation_depth_max_skew_ns / 1e6:g}",
                    "  minimum_visible_keypoints: "
                    f"{0 if preserve_occluded else ANNOTATION_MIN_VISIBLE_KEYPOINTS}",
                    "  preserve_fully_occluded: "
                    f"{str(preserve_occluded).lower()}",
                    "  rgb_robot_mask_fallback: "
                    f"{str(preserve_occluded).lower()}",
                    "  depth_patch_near_quantile: "
                    f"{ANNOTATION_DEPTH_NEAR_QUANTILE:g}",
                    f"  occlusion_margin_m: {ANNOTATION_DEPTH_MARGIN_M:g}",
                ]
            )
    content = "\n".join(
        [
            f"schema_version: {DATASET_SCHEMA_VERSION}",
            "task: img2pos",
            f"version: {policy.dataset_version or 'default'}",
            "sample_unit: synchronized_capture",
            "input: synchronized_rgb_images",
            "constants: constants.json",
            "trials: trials.jsonl",
            "samples: samples.jsonl",
            "metadata: metadata.jsonl",
            "image_layout: images/<split>/<camera>/trial_<index>/*.jpg",
            "cameras: [left, center, right]",
            "target:",
            "  name: correction_xyz",
            "  definition: port_entrance - plug_reference",
            "  frame: base_link",
            "  unit: meter",
            "  timestamp: capture_stamp",
            "poses:",
            "  convention: parent_T_child",
            "  representation: translation_m + quaternion_xyzw",
            "  dataset_constants: constants.json#static_transforms",
            "  trial_constants: trials.jsonl#poses+transforms",
            "  sample_dynamic: samples.jsonl#poses",
            "se3_residuals:",
            "  field: se3_residuals",
            "  definition: Log(parent_T_child)",
            "  reference: identity",
            "  order: [vx_m, vy_m, vz_m, wx_rad, wy_rad, wz_rad]",
            f"  reconstruction_tolerance: {POSE_RECONSTRUCTION_ATOL:g}",
            "normalization:",
            "  dataset: camera calibration, static tool transforms, connector geometry",
            "  trial: scenario, port pose, nominal TCP-to-plug grasp",
            "  sample: images, dynamic poses, learning residuals, constant deltas",
            "split:",
            "  group_by: trial_id",
            f"  validation_ratio: {policy.val_ratio:.6f}",
            f"  test_ratio: {policy.test_ratio:.6f}",
            *annotations,
            *sampling,
            "",
        ]
    )
    with _dataset_write_lock(policy):
        temporary = policy.dataset_dir / f".data.yaml.{os.getpid()}.tmp"
        temporary.write_text(content, encoding="utf-8")
        temporary.replace(policy.dataset_dir / "data.yaml")
        if auto_annotate_ports:
            _write_yolo_pose_config(policy)


def save_sample(
    policy,
    *,
    episode_name: str,
    task,
    step_idx: int,
    observation,
    port_tf: Transform,
    plug_tip_tf: Transform,
    plug_reference_tf: Transform,
    timestamps: dict[str, Any],
    label_xyz: list[float],
    sample: dict[str, Any],
    settle: dict[str, float],
    annotation_ports: list[dict[str, Any]] | tuple[dict[str, Any], ...] = (),
) -> tuple[bool, str]:
    """port 가시성 승인 후 동일 촬영시각의 세 camera와 공통 label을 저장한다."""
    if observation is None:
        return False, "Observation을 받지 못함"
    if not timestamps.get("sync_valid", False):
        return False, f"수집 시각 일치 검사 실패: {timestamps.get('rejection_reason', 'unknown reason')}"
    capture_stamp = int(timestamps.get("capture_stamp_ns", 0))
    if capture_stamp <= 0:
        return False, "유효한 camera capture timestamp가 없음"
    if len(label_xyz) != 3 or not all(np.isfinite(float(value)) for value in label_xyz):
        return False, "유효한 XYZ correction label이 없음"
    try:
        poses, residuals, reconstruction_error = capture_pose_residual_labels(
            policy,
            observation,
            plug_tip_tf,
            plug_reference_tf,
            port_tf,
        )
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        return False, f"pose/residual label 생성 실패: {exc}"
    if not np.isfinite(reconstruction_error) or reconstruction_error > POSE_RECONSTRUCTION_ATOL:
        return False, (
            "pose/residual 역산 검증 실패: "
            f"max_abs_error={reconstruction_error:.3e}, "
            f"tolerance={POSE_RECONSTRUCTION_ATOL:.3e}"
        )
    visibility = {
        camera: _port_projection(policy, observation, camera, port_tf)
        for camera in ("left", "center", "right")
    }
    arm_occluded = [
        camera
        for camera, result in visibility.items()
        if result.get("reason") == ROBOT_ARM_OCCLUSION_REASON
    ]
    visible = [
        camera for camera, result in visibility.items() if result.get("visible", False)
    ]
    required_cameras = policy.min_visible_cameras
    if len(visible) < required_cameras:
        detail = _visibility_detail(visibility)
        occlusion = (
            f", robot_arm_occlusion={arm_occluded}" if arm_occluded else ""
        )
        return False, (
            f"{PORT_VISIBILITY_FAILURE_PREFIX} visible={visible}{occlusion}, "
            f"required={required_cameras}, cameras={detail}"
        )

    auto_annotate_ports = bool(getattr(policy, "auto_annotate_ports", False))
    if auto_annotate_ports and not annotation_ports:
        return False, "자동 annotation 대상 port가 없음"
    annotation_rows: dict[str, list[str]] = {}
    annotation_labels: dict[str, list[str]] = {}
    annotation_instance_ids: dict[str, list[str]] = {}
    annotation_counts: dict[str, int] = {}
    if auto_annotate_ports:
        try:
            (
                annotation_rows,
                annotation_labels,
                annotation_instance_ids,
            ) = _annotation_rows_by_camera(policy, observation, annotation_ports)
            annotation_counts = {
                camera: len(labels) for camera, labels in annotation_labels.items()
            }
        except (KeyError, TypeError, ValueError) as exc:
            return False, f"port annotation 생성 실패: {exc}"

    trial_id = _trial_id(policy, episode_name)
    split = split_for_trial(policy, trial_id)
    capture_id = _compact_capture_id(policy, episode_name, task, step_idx)
    images: dict[str, str] = {}
    annotations: dict[str, str] = {}
    written: list[Path] = []
    failures: list[str] = []
    for camera in visibility:
        image = image_to_bgr(policy, image_for_camera(observation, camera), camera)
        if image is None:
            failures.append(f"{camera}: image 변환 실패")
            continue
        relative_path = _image_relative_path(policy, task, step_idx, split, camera)
        path = policy.dataset_dir / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            path.unlink(missing_ok=True)
            failures.append(f"{camera}: JPEG 저장 실패 ({path})")
            continue
        written.append(path)
        if auto_annotate_ports:
            relative_annotation = _annotation_relative_path(relative_path)
            annotation_path = policy.dataset_dir / relative_annotation
            annotation_path.parent.mkdir(parents=True, exist_ok=True)
            rows = annotation_rows[camera]
            try:
                annotation_path.write_text(
                    "\n".join(rows) + ("\n" if rows else ""), encoding="utf-8"
                )
            except OSError as exc:
                annotation_path.unlink(missing_ok=True)
                failures.append(f"{camera}: annotation 저장 실패 ({exc})")
                continue
            written.append(annotation_path)
            annotations[camera] = str(relative_annotation)
        images[camera] = str(relative_path)
    if len(images) != len(visibility):
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"세 camera 저장 불완전: details={failures}"

    connector = _connector(task)
    port_stamp_ns = int(
        timestamps.get("tf", {}).get("port", {}).get("stamp_ns", 0)
    )
    try:
        actual_connector = se3.matrix_from_residual_record(
            residuals["plug_tip_T_plug_reference"]
        )
        actual_tcp_plug = se3.matrix_from_residual_record(
            residuals["tcp_T_plug_reference"]
        )
        base_port = se3.matrix_from_pose_record(
            poses["base_T_port_entrance"]
        )
        nominal_connector = _ensure_dataset_constants(
            policy,
            observation,
            connector,
            actual_connector,
        )
        (
            nominal_port,
            nominal_tcp_plug,
            port_delta,
            tcp_plug_delta,
        ) = _ensure_trial_record(
            policy,
            trial_id=trial_id,
            split=split,
            task=task,
            connector=connector,
            base_port=base_port,
            tcp_plug_reference=actual_tcp_plug,
            port_stamp_ns=port_stamp_ns,
        )
        _, _, connector_delta = _transform_error(
            nominal_connector, actual_connector
        )
        normalized_residuals = _normalized_sample_residuals(
            residuals,
            connector_delta=connector_delta,
            port_delta=port_delta,
            tcp_plug_delta=tcp_plug_delta,
        )
        normalized_error = _normalized_reconstruction_error(
            poses=poses,
            residuals=normalized_residuals,
            nominal_connector=nominal_connector,
            nominal_port=nominal_port,
            nominal_tcp_plug=nominal_tcp_plug,
        )
    except (KeyError, TypeError, ValueError, OSError, np.linalg.LinAlgError) as exc:
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"normalized constants/trial 저장 검증 실패: {exc}"
    reconstruction_error = max(reconstruction_error, normalized_error)
    if not np.isfinite(reconstruction_error) or reconstruction_error > POSE_RECONSTRUCTION_ATOL:
        for path in written:
            path.unlink(missing_ok=True)
        return False, (
            "normalized pose/residual 역산 검증 실패: "
            f"max_abs_error={reconstruction_error:.3e}, "
            f"tolerance={POSE_RECONSTRUCTION_ATOL:.3e}"
        )

    dynamic_poses = dict(poses)
    dynamic_poses.pop("base_T_port_entrance", None)
    planned_tier_mm = (
        float(sample["tier_m"] * 1000.0)
        if sample["tier_m"] is not None
        else None
    )
    actual_tier_mm = (
        float(sample["actual_tier_m"] * 1000.0)
        if sample.get("actual_tier_m") is not None
        else None
    )
    record = {
        "id": capture_id,
        "trial_id": trial_id,
        "images": images,
        "target_xyz_m": [float(value) for value in label_xyz],
        "sampling_offset_xyz_m": sample["actual_xyz_m"],
        "sampling_tier_mm": planned_tier_mm,
        "planned_sampling_tier_mm": planned_tier_mm,
        "actual_sampling_tier_mm": actual_tier_mm,
        "view_distance_m": sample["actual_view_distance_m"],
        "capture_stamp_ns": capture_stamp,
        "max_sync_skew_ns": _max_skew(timestamps),
        "pose_source_stamps_ns": {
            "camera_capture": capture_stamp,
            "controller": int(timestamps.get("controller_stamp_ns", 0)),
            "plug_tf": int(timestamps.get("tf", {}).get("plug", {}).get("stamp_ns", 0)),
        },
        "poses": dynamic_poses,
        "se3_residuals": normalized_residuals,
        "pose_reconstruction_max_abs_error": float(reconstruction_error),
        "settle_position_error_mm": float(settle["position_error_m"] * 1000.0),
        "settle_orientation_error_rad": float(settle["orientation_error_rad"]),
        "settle_wait_ms": float(settle["wait_ns"] / 1e6),
    }
    haptic = sample.get("haptic")
    if haptic is not None:
        record["haptic_baseline_force_n"] = float(haptic["baseline_force_n"])
        record["haptic_peak_delta_force_n"] = float(
            haptic["peak_delta_force_n"]
        )
    if auto_annotate_ports:
        record["annotations"] = annotations
        record["annotation_format"] = "yolo_pose"
        record["annotation_object_counts"] = annotation_counts
        record["annotation_labels"] = annotation_labels
        record["annotation_instance_ids"] = annotation_instance_ids
    try:
        with _dataset_write_lock(policy):
            with policy.samples_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    except OSError as exc:
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"samples.jsonl 저장 실패: {exc}"
    policy.capture_count += 1
    occlusion = (
        f", robot_arm_occlusion={arm_occluded}, cameras={_visibility_detail(visibility)}"
        if arm_occluded
        else ""
    )
    return True, (
        f"capture_id={capture_id}, cameras={sorted(images)}, visible={visible}"
        f"{occlusion}, annotations={annotation_counts or 'disabled'}, "
        f"saved_count={policy.capture_count}"
    )
