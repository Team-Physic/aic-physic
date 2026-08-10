"""PortOffset dataset sample과 같은 trial의 MCAP source를 대조한다."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rosbag2_py
import yaml
from data_generator.port_offset_config import (
    SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME,
    TF_RECONSTRUCTION_ANGLE_TOLERANCE_RAD,
    TF_RECONSTRUCTION_POSITION_TOLERANCE_M,
)
from data_generator.port_offset_geometry import _matrix_to_rpy_xyz, _quat_to_matrix_xyzw
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException

from phy_data_collection.camera_event_validation import (
    SIMULATION_CLOCK,
    SourceImageEvent,
    summarize_record_delays,
    validate_image_event,
)


CAMERA_TOPICS = {
    "left": "/left_camera/image",
    "center": "/center_camera/image",
    "right": "/right_camera/image",
}
CONTROLLER_TOPIC = "/aic_controller/controller_state"
TF_TOPICS = {"/tf", "/tf_static", "/scoring/tf"}
POSITION_TOLERANCE_M = TF_RECONSTRUCTION_POSITION_TOLERANCE_M
ANGLE_TOLERANCE_RAD = TF_RECONSTRUCTION_ANGLE_TOLERANCE_RAD
ROSBAG_EVENT_CLOCK_KEY = "phy_event_timestamp_clock"


@dataclass
class SampleMetadata:
    """동일 sample_id를 공유하는 카메라별 metadata JSON을 묶는다."""

    sample_id: str
    records: dict[str, dict[str, Any]]

    @property
    def common(self) -> dict[str, Any]:
        return next(iter(self.records.values()))


def load_samples(dataset_dir: Path) -> list[SampleMetadata]:
    """카메라별 sample metadata JSON을 sample_id 기준으로 묶어 읽는다."""
    grouped: dict[str, dict[str, dict[str, Any]]] = {}
    for path in sorted((dataset_dir / "metadata").glob("**/*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        sample_id = str(record.get("sample_id", "")).strip()
        camera = str(record.get("camera", "")).strip()
        if sample_id and camera in CAMERA_TOPICS:
            record["_metadata_path"] = str(path)
            grouped.setdefault(sample_id, {})[camera] = record
    return [SampleMetadata(sample_id, records) for sample_id, records in grouped.items()]


def select_trial_samples(
    samples: list[SampleMetadata],
    rosbag_dir: Path,
    sample_ids: set[str] | None = None,
) -> tuple[list[SampleMetadata], list[str]]:
    """명시된 ID 또는 metadata의 rosbag 경로로 검증할 sample을 선택한다."""
    if sample_ids:
        selected = [sample for sample in samples if sample.sample_id in sample_ids]
        missing = sorted(sample_ids - {sample.sample_id for sample in selected})
        if missing:
            raise ValueError(f"sample_id not found: {', '.join(missing)}")
        return selected, []

    target = rosbag_dir.resolve()
    selected = []
    for sample in samples:
        recorded = sample.common.get("collection", {}).get("rosbag_path", "")
        if recorded and Path(recorded).expanduser().resolve() == target:
            selected.append(sample)
    if selected:
        return selected, []

    task_matches = []
    for sample in samples:
        task_id = str(sample.common.get("task", {}).get("id", "")).strip()
        if task_id and task_id in rosbag_dir.name:
            task_matches.append(sample)
    if task_matches:
        return task_matches, [
            "legacy metadata has no rosbag_path; samples were selected by task.id"
        ]
    raise ValueError(
        "no sample metadata is linked to this rosbag; use --sample-id for legacy data"
    )


def image_message_to_bgr(message: Any) -> np.ndarray:
    """수집기와 동일하게 sensor_msgs/Image의 rgb8/bgr8 payload를 BGR로 변환한다."""
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(
        int(message.height), int(message.width), 3
    )
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding != "bgr8":
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    return image.copy()


def encoded_jpeg(message: Any) -> bytes:
    """ROS Image를 dataset 저장과 동일한 OpenCV 기본 JPEG 설정으로 encoding한다."""
    success, data = cv2.imencode(".jpg", image_message_to_bgr(message))
    if not success:
        raise ValueError("OpenCV JPEG encoding failed")
    return data.tobytes()


def read_rosbag_record_clock(rosbag_dir: Path) -> str:
    """rosbag metadata에 명시된 record timestamp clock을 읽는다."""
    metadata_path = rosbag_dir / "metadata.yaml"
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        info = metadata.get("rosbag2_bagfile_information", {})
        custom_data = info.get("custom_data", {}) or {}
        value = str(custom_data.get(ROSBAG_EVENT_CLOCK_KEY, "")).strip()
    except (OSError, AttributeError, TypeError, yaml.YAMLError):
        return "unknown"
    return value or "unknown"


def _transform_arrays(stamped: Any) -> tuple[np.ndarray, np.ndarray]:
    transform = stamped.transform
    position = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=float,
    )
    quaternion = np.array(
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
        dtype=float,
    )
    return position, quaternion


def _quaternion_angle_rad(left: np.ndarray, right: np.ndarray) -> float:
    left = left / np.linalg.norm(left)
    right = right / np.linalg.norm(right)
    return float(2.0 * math.acos(min(1.0, abs(float(np.dot(left, right))))))


def _transform_difference(left: Any, right: Any) -> tuple[float, float]:
    left_position, left_quaternion = _transform_arrays(left)
    right_position, right_quaternion = _transform_arrays(right)
    return (
        float(np.linalg.norm(left_position - right_position)),
        _quaternion_angle_rad(left_quaternion, right_quaternion),
    )


def _stored_transform_difference(
    stamped: Any,
    stored: dict[str, Any],
) -> tuple[float, float]:
    position, quaternion = _transform_arrays(stamped)
    translation = stored["translation_m"]
    rotation = stored["rotation_xyzw"]
    stored_position = np.array([translation[key] for key in ("x", "y", "z")])
    stored_quaternion = np.array([rotation[key] for key in ("x", "y", "z", "w")])
    return (
        float(np.linalg.norm(position - stored_position)),
        _quaternion_angle_rad(quaternion, stored_quaternion),
    )


def _candidate_frames(record: dict[str, Any]) -> tuple[list[str], list[str]]:
    timestamps = record.get("timestamps", {})
    tf_metadata = timestamps.get("tf", {})
    task = record.get("task", {})
    port = tf_metadata.get("port", {}).get("child_frame_id")
    plug = tf_metadata.get("plug", {}).get("child_frame_id")
    port_base = (
        f"task_board/{task.get('target_module_name', '')}/"
        f"{task.get('port_name', '')}_link"
    )
    cable = str(task.get("cable_name", ""))
    plug_name = str(task.get("plug_name", ""))
    plug_reference = record.get("plug_reference", {}).get("plug_frame")
    port_candidates = [
        value for value in (port, f"{port_base}_entrance", port_base) if value
    ]
    plug_candidates = [
        value
        for value in (
            plug,
            plug_reference,
            f"{cable}/{plug_name}_link",
            f"{cable}/{plug_name}_tip_link",
        )
        if value
    ]
    return list(dict.fromkeys(port_candidates)), list(dict.fromkeys(plug_candidates))


def _lookup_first(buffer: Buffer, frames: list[str], stamp_ns: int) -> tuple[Any, str]:
    errors = []
    for frame in frames:
        try:
            transform = buffer.lookup_transform(
                "base_link",
                frame,
                Time(nanoseconds=stamp_ns),
            )
            return transform, frame
        except TransformException as exc:
            errors.append(f"{frame}: {exc}")
    raise TransformException("; ".join(errors) or "no candidate frame")


def _recomputed_offsets(
    port_stamped: Any,
    plug_stamped: Any,
    local_offset_xyz_m: list[float],
) -> dict[str, dict[str, float]]:
    port_position, port_quaternion = _transform_arrays(port_stamped)
    plug_position, plug_quaternion = _transform_arrays(plug_stamped)
    port_rotation = _quat_to_matrix_xyzw(*port_quaternion)
    plug_rotation = _quat_to_matrix_xyzw(*plug_quaternion)
    plug_reference_position = plug_position + plug_rotation @ np.asarray(
        local_offset_xyz_m, dtype=float
    )
    location_rotation = plug_rotation @ port_rotation.T
    label_rotation = port_rotation @ plug_rotation.T
    location_rpy = _matrix_to_rpy_xyz(location_rotation)
    label_rpy = _matrix_to_rpy_xyz(label_rotation)
    location_position = plug_reference_position - port_position
    label_position = port_position - plug_reference_position

    def values(position: np.ndarray, rpy: tuple[float, float, float]) -> dict[str, float]:
        return dict(
            zip(
                ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"),
                [*map(float, position), *map(float, rpy)],
            )
        )

    return {
        "location": values(location_position, location_rpy),
        "label": values(label_position, label_rpy),
    }


def _offset_differences(
    expected: dict[str, Any],
    actual: dict[str, Any],
) -> tuple[float, float]:
    position_difference_m = max(
        abs(float(expected[key]) - float(actual[key]))
        for key in ("x_m", "y_m", "z_m")
    )
    angle_difference_rad = max(
        abs(
            math.atan2(
                math.sin(float(expected[key]) - float(actual[key])),
                math.cos(float(expected[key]) - float(actual[key])),
            )
        )
        for key in ("roll_rad", "pitch_rad", "yaw_rad")
    )
    return position_difference_m, angle_difference_rad


def read_trial_sources(
    rosbag_dir: Path,
    samples: list[SampleMetadata],
) -> tuple[
    dict[tuple[str, int], list[SourceImageEvent]],
    set[int],
    Buffer,
    dict[str, int],
]:
    """필요한 image/controller message와 전체 TF tree를 trial MCAP에서 읽는다."""
    image_keys = {
        (CAMERA_TOPICS[camera], int(stamp))
        for sample in samples
        for camera, stamp in sample.common["timestamps"]["images"].items()
    }
    controller_stamps = {
        int(sample.common["timestamps"]["controller_stamp_ns"])
        for sample in samples
    }
    images: dict[tuple[str, int], list[SourceImageEvent]] = {}
    seen_controllers: set[int] = set()
    counts = {"image": 0, "controller": 0, "tf": 0}
    buffer = Buffer(cache_time=Duration(seconds=86_400.0))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(rosbag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in set(CAMERA_TOPICS.values()) | {CONTROLLER_TOPIC} | TF_TOPICS
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(message_types)))
    while reader.has_next():
        topic, serialized, record_stamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic in CAMERA_TOPICS.values():
            counts["image"] += 1
            key = (topic, int(Time.from_msg(message.header.stamp).nanoseconds))
            if key in image_keys:
                images.setdefault(key, []).append(
                    SourceImageEvent(
                        topic=topic,
                        header_stamp_ns=key[1],
                        record_stamp_ns=int(record_stamp_ns),
                        jpeg=encoded_jpeg(message),
                    )
                )
        elif topic == CONTROLLER_TOPIC:
            counts["controller"] += 1
            stamp_ns = int(Time.from_msg(message.header.stamp).nanoseconds)
            if stamp_ns in controller_stamps:
                seen_controllers.add(stamp_ns)
        else:
            is_static = topic == "/tf_static"
            for transform in message.transforms:
                counts["tf"] += 1
                if is_static:
                    buffer.set_transform_static(transform, "portoffset_validator")
                else:
                    buffer.set_transform(transform, "portoffset_validator")
    return images, seen_controllers, buffer, counts


def validate_sample(
    sample: SampleMetadata,
    dataset_dir: Path,
    images: dict[tuple[str, int], list[SourceImageEvent]],
    controllers: set[int],
    tf_buffer: Buffer,
    record_clock: str,
    max_record_delay_ns: int | None,
) -> dict[str, Any]:
    """한 sample의 영상, source 시각 차이, TF와 label을 검증한다."""
    record = sample.common
    timestamps = record["timestamps"]
    tolerance_ns = int(timestamps["sync_tolerance_ns"])
    image_stamps = {name: int(value) for name, value in timestamps["images"].items()}
    capture_ns = int(timestamps["capture_stamp_ns"])
    controller_ns = int(timestamps["controller_stamp_ns"])
    camera_difference_ns = max(image_stamps.values()) - min(image_stamps.values())
    controller_difference_ns = abs(controller_ns - capture_ns)
    errors: list[str] = []
    warnings: list[str] = []

    for camera, camera_record in sample.records.items():
        for field in ("timestamps", "location", "label", "collection", "plug_reference"):
            if camera_record.get(field) != record.get(field):
                errors.append(f"{camera} metadata field differs: {field}")

    if capture_ns != image_stamps.get("center"):
        errors.append("capture_stamp_ns does not equal center image timestamp")
    if camera_difference_ns > tolerance_ns:
        errors.append("camera timestamp difference exceeds tolerance")
    if controller_difference_ns > tolerance_ns:
        errors.append("controller timestamp difference exceeds tolerance")
    if controller_ns not in controllers:
        errors.append("controller message with recorded timestamp is missing from MCAP")

    image_checks = {}
    for camera, stamp_ns in image_stamps.items():
        key = (CAMERA_TOPICS[camera], stamp_ns)
        source_events = images.get(key, [])
        check = {
            "topic": key[0],
            "timestamp_ns": stamp_ns,
            "mcap_message_found": bool(source_events),
        }
        if not source_events:
            errors.append(f"{camera} image timestamp is missing from MCAP")
        if camera in sample.records:
            image_path = dataset_dir / sample.records[camera]["image"]
            saved = image_path.read_bytes() if image_path.is_file() else b""
            event_check = validate_image_event(
                camera=camera,
                dataset_stamp_ns=stamp_ns,
                dataset_jpeg=saved,
                source_events=source_events,
                record_clock=record_clock,
                max_record_delay_ns=max_record_delay_ns,
            )
            check["jpeg_path"] = str(image_path)
            check["jpeg_matches_mcap"] = event_check["jpeg_matches_source"]
            check["event"] = event_check
            errors.extend(f"{camera}: {error}" for error in event_check["errors"])
            warnings.extend(
                f"{camera}: {warning}" for warning in event_check["warnings"]
            )
        image_checks[camera] = check

    port_candidates, plug_candidates = _candidate_frames(record)
    tf_metadata = timestamps.get("tf", {})
    port_stamp_ns = int(tf_metadata.get("port", {}).get("stamp_ns", capture_ns))
    plug_stamp_ns = int(tf_metadata.get("plug", {}).get("stamp_ns", capture_ns))
    plug_difference_ns = abs(plug_stamp_ns - capture_ns)
    if plug_difference_ns > tolerance_ns:
        errors.append("plug TF timestamp difference exceeds tolerance")

    transform_checks: dict[str, Any] = {}
    try:
        port_snapshot, port_frame = _lookup_first(tf_buffer, port_candidates, port_stamp_ns)
        port_capture, _ = _lookup_first(tf_buffer, [port_frame], capture_ns)
        movement_m, movement_rad = _transform_difference(port_snapshot, port_capture)
        transform_checks["port"] = {
            "frame": port_frame,
            "snapshot_timestamp_ns": port_stamp_ns,
            "movement_during_trial_m": movement_m,
            "movement_during_trial_rad": movement_rad,
        }
        if movement_m > POSITION_TOLERANCE_M or movement_rad > ANGLE_TOLERANCE_RAD:
            errors.append("port entrance moved after its trial snapshot")
        stored = tf_metadata.get("port", {}).get("transform")
        if stored:
            difference_m, difference_rad = _stored_transform_difference(port_snapshot, stored)
            transform_checks["port"].update(
                metadata_difference_m=difference_m,
                metadata_difference_rad=difference_rad,
            )
            if difference_m > POSITION_TOLERANCE_M or difference_rad > ANGLE_TOLERANCE_RAD:
                errors.append("port transform differs from metadata")
    except TransformException as exc:
        errors.append(f"port TF lookup failed: {exc}")
        port_snapshot = None

    try:
        plug_capture, plug_frame = _lookup_first(tf_buffer, plug_candidates, capture_ns)
        transform_checks["plug"] = {
            "frame": plug_frame,
            "capture_timestamp_ns": capture_ns,
            "timestamp_difference_ns": plug_difference_ns,
            "timestamp_difference_ms": plug_difference_ns / 1_000_000.0,
        }
        stored = tf_metadata.get("plug", {}).get("transform")
        if stored:
            difference_m, difference_rad = _stored_transform_difference(plug_capture, stored)
            transform_checks["plug"].update(
                metadata_difference_m=difference_m,
                metadata_difference_rad=difference_rad,
            )
            if difference_m > POSITION_TOLERANCE_M or difference_rad > ANGLE_TOLERANCE_RAD:
                errors.append("plug transform differs from metadata")
    except TransformException as exc:
        errors.append(f"plug TF lookup failed: {exc}")
        plug_capture = None

    label_checks: dict[str, float] = {}
    if port_snapshot is not None and plug_capture is not None:
        plug_reference = record.get("plug_reference", {})
        local_offset = plug_reference.get("local_offset_xyz_m")
        if local_offset is None:
            task_text = json.dumps(record.get("task", {})).lower()
            if "sfp" in task_text:
                local_offset = list(SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME)
                warnings.append(
                    "plug_reference offset missing; the configured SFP offset was used"
                )
            else:
                local_offset = [0.0, 0.0, 0.0]
                warnings.append("plug_reference offset missing; zero offset was used")
        recomputed = _recomputed_offsets(port_snapshot, plug_capture, local_offset)
        for name in ("location", "label"):
            position_difference_m, angle_difference_rad = _offset_differences(
                record[name],
                recomputed[name],
            )
            label_checks[f"{name}_maximum_position_difference_m"] = (
                position_difference_m
            )
            label_checks[f"{name}_maximum_angle_difference_rad"] = (
                angle_difference_rad
            )
            if (
                position_difference_m > POSITION_TOLERANCE_M
                or angle_difference_rad > ANGLE_TOLERANCE_RAD
            ):
                errors.append(f"{name} differs from MCAP TF reconstruction")

    return {
        "sample_id": sample.sample_id,
        "status": "PASS" if not errors else "FAIL",
        "capture_timestamp_ns": capture_ns,
        "sync_tolerance_ns": tolerance_ns,
        "sync_tolerance_ms": tolerance_ns / 1_000_000.0,
        "time_differences": {
            "camera_ns": camera_difference_ns,
            "camera_ms": camera_difference_ns / 1_000_000.0,
            "controller_ns": controller_difference_ns,
            "controller_ms": controller_difference_ns / 1_000_000.0,
            "plug_tf_ns": plug_difference_ns,
            "plug_tf_ms": plug_difference_ns / 1_000_000.0,
        },
        "images": image_checks,
        "transforms": transform_checks,
        "labels": label_checks,
        "errors": errors,
        "warnings": warnings,
    }


def validate_trial(
    dataset_dir: Path,
    rosbag_dir: Path,
    sample_ids: set[str] | None = None,
    max_record_delay_ms: float | None = None,
) -> dict[str, Any]:
    """하나의 trial MCAP과 연결된 모든 dataset sample을 검증한다."""
    if max_record_delay_ms is not None and (
        not math.isfinite(max_record_delay_ms) or max_record_delay_ms < 0
    ):
        raise ValueError("max_record_delay_ms must be finite and non-negative")
    dataset_dir = dataset_dir.expanduser().resolve()
    rosbag_dir = rosbag_dir.expanduser().resolve()
    if rosbag_dir.is_file() and rosbag_dir.suffix == ".mcap":
        rosbag_dir = rosbag_dir.parent
    samples, warnings = select_trial_samples(
        load_samples(dataset_dir),
        rosbag_dir,
        sample_ids,
    )
    record_clock = read_rosbag_record_clock(rosbag_dir)
    if record_clock != SIMULATION_CLOCK:
        warnings.append(
            "rosbag record clock is not marked as ROS simulation time; "
            "source-to-MCAP record delay is unavailable"
        )
    max_record_delay_ns = (
        int(max_record_delay_ms * 1_000_000)
        if max_record_delay_ms is not None
        else None
    )
    images, controllers, tf_buffer, counts = read_trial_sources(rosbag_dir, samples)
    results = [
        validate_sample(
            sample,
            dataset_dir,
            images,
            controllers,
            tf_buffer,
            record_clock,
            max_record_delay_ns,
        )
        for sample in samples
    ]
    passed = sum(result["status"] == "PASS" for result in results)
    event_results = [
        image["event"]
        for result in results
        for image in result["images"].values()
        if "event" in image
    ]
    passed_events = sum(event["status"] == "PASS" for event in event_results)
    return {
        "schema_version": 2,
        "status": "PASS" if passed == len(results) else "FAIL",
        "dataset_dir": str(dataset_dir),
        "rosbag_dir": str(rosbag_dir),
        "units": {
            "time": "nanoseconds and milliseconds",
            "position": "meters",
            "angle": "radians",
        },
        "transform_tolerances": {
            "position_m": POSITION_TOLERANCE_M,
            "angle_rad": ANGLE_TOLERANCE_RAD,
        },
        "camera_event_validation": {
            "source_boundary": "ros_gz_bridge_output",
            "simulator_capture_stamp_assumption": (
                "sensor_msgs/Image.header.stamp is the Gazebo render/capture event time"
            ),
            "record_clock": record_clock,
            "max_record_delay_ms": max_record_delay_ms,
            "event_count": len(event_results),
            "passed_event_count": passed_events,
            "failed_event_count": len(event_results) - passed_events,
            "record_delay": summarize_record_delays(event_results),
        },
        "sample_count": len(results),
        "passed_sample_count": passed,
        "failed_sample_count": len(results) - passed,
        "source_message_counts": counts,
        "warnings": warnings,
        "samples": results,
    }
