"""동기화된 camera image와 img2pos XYZ label 저장."""

from __future__ import annotations

import hashlib
import json
import os
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import fcntl

import cv2
import numpy as np
from geometry_msgs.msg import Transform

from phy_policy.data_generator.geometry import pose_matrix, quaternion_matrix


SFP_REFERENCE_OFFSET = np.array([0.0, 0.0021125, 0.0], dtype=float)


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
    margin = policy.visibility_margin_px
    projections = []
    for point_camera in points_camera:
        depth = float(point_camera[2])
        if depth <= 1e-6:
            return {"visible": False, "reason": "behind_camera", "depth_m": depth}
        u = float(intrinsic[0, 0] * point_camera[0] / depth + intrinsic[0, 2])
        v = float(intrinsic[1, 1] * point_camera[1] / depth + intrinsic[1, 2])
        projections.append({"u_px": u, "v_px": v, "depth_m": depth})
    return {
        "visible": all(
            margin <= u < float(message.width) - margin
            and margin <= v < float(message.height) - margin
            for u, v in (
                (projection["u_px"], projection["v_px"])
                for projection in projections
            )
        ),
        "points": projections,
    }


def _port_projection(policy, observation, camera: str, port_tf: Transform) -> dict[str, Any]:
    """포트 위치가 지정 camera의 유효 영상 영역에 보이는지 검사한다."""
    return _points_projection(
        policy,
        observation,
        camera,
        [[port_tf.translation.x, port_tf.translation.y, port_tf.translation.z]],
    )


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
                f"  angle_limit_deg: {np.rad2deg(policy.board_angle_limit):g}",
            ]
        )
    elif policy.collection_policy == "descent":
        sampling.extend(
            [
                "  approach_distance_mm: "
                f"[{policy.descent_start_distance * 1000.0:g}, "
                f"{policy.base_z_offset * 1000.0:g}]",
                f"  lateral_limit_mm: {policy.descent_lateral_limit * 1000.0:g}",
                f"  angle_limit_deg: {np.rad2deg(policy.descent_angle_limit):g}",
            ]
        )
    else:
        sampling.append(
            "  position_tiers_mm: ["
            + ", ".join(f"{value * 1000.0:g}" for value in policy.sampling_tiers_m)
            + "]"
        )
    content = "\n".join(
        [
            "schema_version: 4",
            "task: img2pos",
            f"version: {policy.dataset_version or 'default'}",
            "sample_unit: synchronized_capture",
            "input: synchronized_rgb_images",
            "samples: samples.jsonl",
            "metadata: metadata.jsonl",
            "image_layout: images/<split>/<camera>/*.jpg",
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
            *sampling,
            "",
        ]
    )
    with _dataset_write_lock(policy):
        temporary = policy.dataset_dir / f".data.yaml.{os.getpid()}.tmp"
        temporary.write_text(content, encoding="utf-8")
        temporary.replace(policy.dataset_dir / "data.yaml")


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
    visible = [
        camera for camera, result in visibility.items() if result.get("visible", False)
    ]
    required_cameras = policy.min_visible_cameras
    if len(visible) < required_cameras:
        detail = {
            camera: result.get("reason")
            or [
                [round(point["u_px"], 1), round(point["v_px"], 1)]
                for point in result.get("points", ())
            ]
            for camera, result in visibility.items()
        }
        return False, (
            f"포트 가시성 부족: visible={visible}, "
            f"required={required_cameras}, projection={detail}"
        )

    trial_id = _trial_id(policy, episode_name)
    split = split_for_trial(policy, trial_id)
    capture_id = f"{episode_name}_collect_{step_idx:06d}"
    images: dict[str, str] = {}
    written: list[Path] = []
    failures: list[str] = []
    for camera in visibility:
        image = image_to_bgr(policy, image_for_camera(observation, camera), camera)
        if image is None:
            failures.append(f"{camera}: image 변환 실패")
            continue
        path = policy.dataset_dir / "images" / split / camera / f"{capture_id}_{camera}.jpg"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            path.unlink(missing_ok=True)
            failures.append(f"{camera}: JPEG 저장 실패 ({path})")
            continue
        written.append(path)
        images[camera] = str(path.relative_to(policy.dataset_dir))
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
        "settle_orientation_error_deg": float(
            np.rad2deg(settle["orientation_error_rad"])
        ),
        "settle_wait_ms": float(settle["wait_ns"] / 1e6),
    }
    try:
        with _dataset_write_lock(policy):
            with policy.samples_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    except OSError as exc:
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"samples.jsonl 저장 실패: {exc}"
    policy.capture_count += 1
    return True, (
        f"capture_id={capture_id}, cameras={sorted(images)}, "
        f"saved_count={policy.capture_count}"
    )
