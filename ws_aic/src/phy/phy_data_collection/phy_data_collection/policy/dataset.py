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

from .geometry import pose_matrix, quaternion_matrix


SFP_REFERENCE_OFFSET = np.array([0.0, 0.0021125, 0.0], dtype=float)
PORT_VISIBILITY_FAILURE_PREFIX = "포트 가시성 부족:"
ROBOT_ARM_OCCLUSION_REASON = "robot_arm_occlusion"
ROBOT_ARM_DARK_THRESHOLD = 40
ROBOT_ARM_MIN_AREA_RATIO = 0.01
ROBOT_ARM_MASK_DILATION_PX = 8
ANNOTATION_DEPTH_MARGIN_M = 0.002
ANNOTATION_DEPTH_PATCH_RADIUS_PX = 2
ANNOTATION_MIN_VISIBLE_KEYPOINTS = 2
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


def _camera_matrix(camera_info) -> np.ndarray | None:
    """CameraInfo의 intrinsic 행렬을 검증해 반환한다."""
    if camera_info is None or len(camera_info.k) < 9:
        return None
    matrix = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
    return matrix if abs(matrix[0, 0]) >= 1e-9 and abs(matrix[1, 1]) >= 1e-9 else None


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
    intrinsic = _camera_matrix(camera_info_for(observation, camera))
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
) -> tuple[int, ...]:
    """투영 깊이보다 앞선 surface가 있는 keypoint를 가림(1)으로 판정한다."""
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
            visibilities.append(2)
            continue
        observed_depth = float(np.median(valid))
        expected_depth = float(point["depth_m"])
        occluded = observed_depth + ANNOTATION_DEPTH_MARGIN_M < expected_depth
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
) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
    """camera별로 화면 안에 완전히 투영된 모든 port의 YOLO pose row를 만든다."""
    rows = {camera: [] for camera in ("left", "center", "right")}
    labels = {camera: [] for camera in rows}
    for camera in rows:
        depth_visibility = bool(
            getattr(policy, "auto_annotation_visibility", False)
        )
        depth = (
            _depth_for_capture(policy, observation, camera)
            if depth_visibility
            else None
        )
        for port in annotation_ports:
            projection = _port_outer_projection(policy, observation, camera, port)
            if projection.get("visible", False):
                visibilities = (
                    _keypoint_visibilities(projection, depth)
                    if depth is not None
                    else (2,) * len(projection["points"])
                )
                if visibilities.count(2) < ANNOTATION_MIN_VISIBLE_KEYPOINTS:
                    continue
                rows[camera].append(
                    _yolo_pose_row(port["class_id"], projection, visibilities)
                )
                labels[camera].append(port["label"])
    return rows, labels


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
    content = "\n".join(
        [
            "train: images/train",
            "val: images/val",
            "test: images/test",
            "names:",
            *(f"  {class_id}: {name}" for class_id, name in PORT_CLASS_NAMES.items()),
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
    else:
        sampling.append(
            "  position_tiers_mm: ["
            + ", ".join(f"{value * 1000.0:g}" for value in policy.sampling_tiers_m)
            + "]"
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
                "  keypoint_order: [x_min_y_max, x_max_y_max, x_max_y_min, x_min_y_min]",
                "  keypoint_shape: [4, 3]",
                "  port_outer_size_mm: {sfp: [16.224, 13.698], sc: [25.781, 9.300]}",
            ]
        )
        if depth_visibility:
            annotations.extend(
                [
                    "  visibility_source: synchronized_depth",
                    "  depth_image_size_px: "
                    f"[{ANNOTATION_DEPTH_IMAGE_SIZE_PX[0]}, {ANNOTATION_DEPTH_IMAGE_SIZE_PX[1]}]",
                    f"  depth_update_rate_hz: {ANNOTATION_DEPTH_UPDATE_RATE_HZ:g}",
                    "  max_depth_rgb_skew_ms: "
                    f"{policy.annotation_depth_max_skew_ns / 1e6:g}",
                    f"  minimum_visible_keypoints: {ANNOTATION_MIN_VISIBLE_KEYPOINTS}",
                    f"  occlusion_margin_m: {ANNOTATION_DEPTH_MARGIN_M:g}",
                ]
            )
    content = "\n".join(
        [
            f"schema_version: {8 if depth_visibility else 6}",
            "task: img2pos",
            f"version: {policy.dataset_version or 'default'}",
            "sample_unit: synchronized_capture",
            "input: synchronized_rgb_images",
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
    annotation_counts: dict[str, int] = {}
    if auto_annotate_ports:
        try:
            annotation_rows, annotation_labels = _annotation_rows_by_camera(
                policy, observation, annotation_ports
            )
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
    record = {
        "id": capture_id,
        "trial_id": trial_id,
        "split": split,
        "images": images,
        "connector": _connector(task),
        "collection_policy": policy.collection_policy,
        "target_xyz_m": [float(value) for value in label_xyz],
        "sampling_offset_xyz_m": sample["actual_xyz_m"],
        "sampling_tier_mm": (
            float(sample["tier_m"] * 1000.0)
            if sample["tier_m"] is not None
            else None
        ),
        "view_distance_m": sample["actual_view_distance_m"],
        "capture_stamp_ns": capture_stamp,
        "max_sync_skew_ns": _max_skew(timestamps),
        "settle_position_error_mm": float(settle["position_error_m"] * 1000.0),
        "settle_orientation_error_rad": float(settle["orientation_error_rad"]),
        "settle_wait_ms": float(settle["wait_ns"] / 1e6),
    }
    if auto_annotate_ports:
        record["annotations"] = annotations
        record["annotation_format"] = "yolo_pose"
        record["annotation_object_counts"] = annotation_counts
        record["annotation_labels"] = annotation_labels
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
