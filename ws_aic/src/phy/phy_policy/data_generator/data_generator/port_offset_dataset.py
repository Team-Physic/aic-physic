from __future__ import annotations

"""Dataset and label writers for PortOffsetCollect."""

import json
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from geometry_msgs.msg import Pose, Transform


def _connector_dir_for_task(task) -> str:
    """저장 경로에 사용할 커넥터 타입 디렉터리 이름을 반환한다."""
    for value in (
        getattr(task, "port_type", ""),
        getattr(task, "port_name", ""),
        getattr(task, "plug_type", ""),
        getattr(task, "plug_name", ""),
    ):
        text = str(value).lower()
        if "sfp" in text:
            return "SFP"
        if "sc" in text:
            return "SC"
    return "UNKNOWN"


def _split_for_sample(self, sample_index: int) -> str:
    """sample index를 설정된 비율에 따라 train 또는 validation으로 나눈다."""
    if self._rpy_val_ratio <= 0.0:
        return "train"
    period = max(1, round(1.0 / self._rpy_val_ratio))
    return "val" if sample_index % period == 0 else "train"

def _port_projection_for_camera(
    self,
    obs,
    camera_name: str,
    port_tf: Transform,
) -> dict[str, Any]:
    """base_link의 port 위치를 지정 camera 영상 좌표로 투영한다."""
    img_msg = self._image_msg_for_camera(obs, camera_name)
    k = self._camera_intrinsic_matrix(self._camera_info_for_camera(obs, camera_name))
    if img_msg is None or img_msg.width == 0 or img_msg.height == 0 or k is None:
        return {"visible": False, "reason": "missing_image_or_intrinsics"}

    try:
        t_cam_base = self._base_to_camera_optical_matrix(obs, camera_name)
    except Exception as exc:
        return {"visible": False, "reason": f"camera_transform_error: {exc}"}

    point_base = np.array(
        [
            port_tf.translation.x,
            port_tf.translation.y,
            port_tf.translation.z,
            1.0,
        ],
        dtype=float,
    )
    point_cam = t_cam_base @ point_base
    depth = float(point_cam[2])
    if depth <= 1e-6:
        return {"visible": False, "reason": "behind_camera", "depth_m": depth}

    u = float(k[0, 0] * point_cam[0] / depth + k[0, 2])
    v = float(k[1, 1] * point_cam[1] / depth + k[1, 2])
    margin = self._rpy_visibility_margin_px
    visible = (
        margin <= u < float(img_msg.width) - margin
        and margin <= v < float(img_msg.height) - margin
    )
    return {
        "visible": bool(visible),
        "u_px": u,
        "v_px": v,
        "depth_m": depth,
        "width": int(img_msg.width),
        "height": int(img_msg.height),
        "margin_px": margin,
    }

def _scenario_metadata(self, task) -> dict[str, Any]:
    """현재 task ID에 대응하는 randomization metadata를 읽는다."""
    try:
        params = json.loads(
            self._scenario_params_file.read_text(encoding="utf-8")
        )
        return params.get(task.id, {}) or {}
    except Exception:
        return {}

def _stamp_ns(stamp) -> int | None:
    """ROS builtin time message를 nanosecond 정수로 변환한다."""
    if stamp is None:
        return None
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None

def _observation_sync_metadata(self, obs) -> tuple[bool, dict[str, Any]]:
    """camera와 controller source timestamp를 검사하고 공통 기록을 만든다."""
    tolerance_ns = int(self.collect_sync_tolerance_ns)
    timestamps: dict[str, Any] = {
        "clock": "ros",
        "unit": "nanoseconds",
        "sync_tolerance_ns": tolerance_ns,
        "sync_valid": False,
    }
    if obs is None:
        timestamps["rejection_reason"] = "missing_observation"
        return False, timestamps

    image_stamps = {
        camera_name: _stamp_ns(
            getattr(
                getattr(self._image_msg_for_camera(obs, camera_name), "header", None),
                "stamp",
                None,
            )
        )
        for camera_name in ("left", "center", "right")
    }
    controller_stamp = _stamp_ns(
        getattr(
            getattr(getattr(obs, "controller_state", None), "header", None),
            "stamp",
            None,
        )
    )
    timestamps["images"] = image_stamps
    timestamps["controller_stamp_ns"] = controller_stamp

    missing_sources = [
        f"image:{name}" for name, stamp in image_stamps.items() if not stamp
    ]
    if not controller_stamp:
        missing_sources.append("controller")
    if missing_sources:
        timestamps["rejection_reason"] = "missing_or_zero_timestamp"
        timestamps["missing_sources"] = missing_sources
        return False, timestamps

    valid_image_stamps = {
        name: int(stamp) for name, stamp in image_stamps.items() if stamp is not None
    }
    capture_stamp = valid_image_stamps["center"]
    camera_time_difference = max(valid_image_stamps.values()) - min(valid_image_stamps.values())
    controller_time_difference = abs(int(controller_stamp or 0) - capture_stamp)
    timestamps["capture_stamp_ns"] = capture_stamp
    timestamps["skew_ns"] = {
        "camera": int(camera_time_difference),
        "controller": int(controller_time_difference),
    }
    if camera_time_difference > tolerance_ns:
        timestamps["rejection_reason"] = "camera_time_difference_exceeded"
        return False, timestamps
    if controller_time_difference > tolerance_ns:
        timestamps["rejection_reason"] = "controller_time_difference_exceeded"
        return False, timestamps

    timestamps["sync_valid"] = True
    return True, timestamps

def _wait_for_synchronized_observation(
    self,
    get_observation,
) -> tuple[Any | None, dict[str, Any]]:
    """허용 오차를 만족하는 새 Observation을 제한 시간 동안 순차 대기한다."""
    start_ns = time.monotonic_ns()
    timeout_ns = int(self.collect_sync_wait_timeout_sec * 1_000_000_000)
    deadline_ns = start_ns + timeout_ns
    waiting_logged = False
    timestamps: dict[str, Any] = {
        "sync_valid": False,
        "rejection_reason": "missing_observation",
    }

    while True:
        obs = get_observation()
        sync_valid, timestamps = self._observation_sync_metadata(obs)
        now_ns = time.monotonic_ns()
        timestamps.setdefault("wait_ns", {})["observation"] = now_ns - start_ns
        if sync_valid:
            return obs, timestamps
        if now_ns >= deadline_ns:
            timestamps["rejection_reason"] = (
                "observation_sync_timeout:"
                f"{timestamps.get('rejection_reason', 'unknown')}"
            )
            return None, timestamps
        if not waiting_logged:
            self.get_logger().info(
                self._collect_log_text(
                    "[PortOffsetCollect] Waiting for synchronized Observation: "
                    f"timeout={self.collect_sync_wait_timeout_sec:.3f}s",
                    "cyan",
                )
            )
            waiting_logged = True
        remaining_sec = (deadline_ns - now_ns) / 1_000_000_000.0
        self.sleep_for(min(self.collect_sync_poll_sec, remaining_sec))

def _tf_sync_metadata(
    self,
    timestamps: dict[str, Any],
    transforms: dict[str, Any],
    static_sources: set[str] | None = None,
) -> tuple[bool, dict[str, Any]]:
    """capture 시각으로 조회한 TF timestamp를 검사해 동기화 기록을 완성한다."""
    capture_stamp = int(timestamps.get("capture_stamp_ns", 0))
    tolerance_ns = int(self.collect_sync_tolerance_ns)
    static_sources = static_sources or set()
    tf_records: dict[str, Any] = {}
    tf_time_differences = []

    for name, stamped in transforms.items():
        header = getattr(stamped, "header", None)
        if header is None:
            timestamps["sync_valid"] = False
            timestamps["rejection_reason"] = f"missing_tf_timestamp:{name}"
            return False, timestamps
        stamp_ns = _stamp_ns(getattr(header, "stamp", None))
        if stamp_ns is None:
            timestamps["sync_valid"] = False
            timestamps["rejection_reason"] = f"missing_tf_timestamp:{name}"
            return False, timestamps
        is_static_snapshot = name in static_sources
        is_static = stamp_ns == 0 or is_static_snapshot
        time_difference_ns = 0 if is_static else abs(int(stamp_ns) - capture_stamp)
        tf_records[name] = {
            "stamp_ns": stamp_ns,
            "parent_frame_id": str(getattr(header, "frame_id", "")),
            "child_frame_id": str(getattr(stamped, "child_frame_id", "")),
            "is_static": is_static,
            "is_static_snapshot": is_static_snapshot,
            "skew_ns": int(time_difference_ns),
            "transform": {
                "translation_m": {
                    "x": float(stamped.transform.translation.x),
                    "y": float(stamped.transform.translation.y),
                    "z": float(stamped.transform.translation.z),
                },
                "rotation_xyzw": {
                    "x": float(stamped.transform.rotation.x),
                    "y": float(stamped.transform.rotation.y),
                    "z": float(stamped.transform.rotation.z),
                    "w": float(stamped.transform.rotation.w),
                },
            },
        }
        tf_time_differences.append(int(time_difference_ns))

    max_tf_time_difference = max(tf_time_differences, default=0)
    timestamps["tf"] = tf_records
    timestamps.setdefault("skew_ns", {})["tf"] = max_tf_time_difference
    if max_tf_time_difference > tolerance_ns:
        timestamps["sync_valid"] = False
        timestamps["rejection_reason"] = "tf_time_difference_exceeded"
        return False, timestamps

    timestamps["sync_valid"] = True
    timestamps.pop("rejection_reason", None)
    return True, timestamps

def _save_xyz_rpy_sample(
    self,
    episode_name: str,
    task,
    phase: str,
    step_idx: int,
    obs,
    port_tf: Transform,
    plug_tf: Transform,
    pose: Pose,
    extras: dict[str, Any],
    detections_by_camera: Optional[dict[str, Optional[dict[str, Any]]]],
) -> tuple[bool, str]:
    """수집 시각 일치 조건을 통과한 영상과 label을 저장하고 결과 이유를 반환한다."""
    if obs is None:
        return False, "Observation을 받지 못함"

    timestamps = dict(extras.get("timestamps", {}))
    if not timestamps.get("sync_valid", False):
        return (
            False,
            "수집 시각 일치 검사 실패: "
            f"{timestamps.get('rejection_reason', 'unknown reason')}",
        )
    timestamps["dataset_write_stamp_ns"] = int(self.get_clock().now().nanoseconds)

    projections = {
        camera_name: self._port_projection_for_camera(obs, camera_name, port_tf)
        for camera_name in ("left", "center", "right")
    }
    visible_cameras = [
        camera_name
        for camera_name, projection in projections.items()
        if projection.get("visible", False)
    ]
    if len(visible_cameras) < self._rpy_min_visible_cameras:
        return (
            False,
            "포트 가시성 부족: "
            f"visible={visible_cameras}, required={self._rpy_min_visible_cameras}",
        )

    sample_index = self._rpy_sample_count
    split = self._split_for_sample(sample_index)
    connector_dir = _connector_dir_for_task(task)
    sample_id = f"{episode_name}_{phase}_{step_idx:06d}"
    image_records = {}
    written_paths: list[Path] = []
    write_failures: list[str] = []

    for camera_name in visible_cameras:
        bgr = self._image_msg_to_bgr(
            self._image_msg_for_camera(obs, camera_name),
            camera_name,
        )
        if bgr is None:
            write_failures.append(f"{camera_name}: image 변환 실패")
            continue

        stem = f"{sample_id}_{camera_name}"
        image_path = (
            self._rpy_dataset_dir
            / "images"
            / split
            / connector_dir
            / camera_name
            / f"{stem}.jpg"
        )
        metadata_path = (
            self._rpy_dataset_dir
            / "metadata"
            / split
            / connector_dir
            / camera_name
            / f"{stem}.json"
        )
        image_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.parent.mkdir(parents=True, exist_ok=True)

        if not cv2.imwrite(str(image_path), bgr):
            write_failures.append(f"{camera_name}: JPEG 저장 실패 ({image_path})")
            continue
        written_paths.append(image_path)

        label_record = {
            "sample_id": sample_id,
            "collection": dict(self._collection_metadata),
            "camera": camera_name,
            "connector": connector_dir,
            "image": str(image_path.relative_to(self._rpy_dataset_dir)),
            "task": {
                "id": task.id,
                "port_type": task.port_type,
                "port_name": task.port_name,
                "target_module_name": task.target_module_name,
                "cable_name": task.cable_name,
                "plug_name": task.plug_name,
            },
            "scenario": self._scenario_metadata(task),
            "plug_reference": dict(extras.get("plug_reference", {})),
            "command": {
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation_xyzw": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            },
            "location": {
                "x_m": float(extras.get("location", {}).get("x_m", 0.0)),
                "y_m": float(extras.get("location", {}).get("y_m", 0.0)),
                "z_m": float(extras.get("location", {}).get("z_m", 0.0)),
                "roll_rad": float(extras.get("location", {}).get("roll_rad", 0.0)),
                "pitch_rad": float(extras.get("location", {}).get("pitch_rad", 0.0)),
                "yaw_rad": float(extras.get("location", {}).get("yaw_rad", 0.0)),
            },
            "label": {
                "x_m": float(extras.get("label", {}).get("x_m", 0.0)),
                "y_m": float(extras.get("label", {}).get("y_m", 0.0)),
                "z_m": float(extras.get("label", {}).get("z_m", 0.0)),
                "roll_rad": float(extras.get("label", {}).get("roll_rad", 0.0)),
                "pitch_rad": float(extras.get("label", {}).get("pitch_rad", 0.0)),
                "yaw_rad": float(extras.get("label", {}).get("yaw_rad", 0.0)),
            },
            "collect": {
                "pattern": str(extras.get("collect_pattern", self.collect_pattern)),
                "local_x_m": float(extras.get("collect_local_x", 0.0)),
                "local_y_m": float(extras.get("collect_local_y", 0.0)),
                "local_z_m": float(extras.get("collect_local_z", 0.0)),
                "local_roll_rad": float(extras.get("collect_local_roll", 0.0)),
                "local_pitch_rad": float(extras.get("collect_local_pitch", 0.0)),
                "local_yaw_rad": float(extras.get("collect_local_yaw", 0.0)),
                "local_roll_deg": float(extras.get("collect_local_roll_deg", 0.0)),
                "local_pitch_deg": float(extras.get("collect_local_pitch_deg", 0.0)),
                "local_yaw_deg": float(extras.get("collect_local_yaw_deg", 0.0)),
                "distance_m": float(
                    extras.get("collect_distance", extras.get("collect_radius", 0.0))
                ),
            },
            "visibility": {
                "camera": projections[camera_name],
                "visible_cameras": visible_cameras,
            },
            "timestamps": timestamps,
        }
        try:
            metadata_path.write_text(
                json.dumps(self._json_safe(label_record), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except OSError as exc:
            image_path.unlink(missing_ok=True)
            written_paths.remove(image_path)
            write_failures.append(f"{camera_name}: metadata 저장 실패 ({exc})")
            continue
        written_paths.append(metadata_path)
        image_records[camera_name] = str(image_path.relative_to(self._rpy_dataset_dir))

    if len(image_records) < self._rpy_min_visible_cameras:
        for path in written_paths:
            path.unlink(missing_ok=True)
        return (
            False,
            "필요한 camera 파일 수를 저장하지 못함: "
            f"written={list(image_records)}, required={self._rpy_min_visible_cameras}, "
            f"details={write_failures}",
        )

    metadata = {
            "sample_id": sample_id,
            "collection": dict(self._collection_metadata),
            "split": split,
            "connector": connector_dir,
            "phase": phase,
            "step_index": int(step_idx),
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "images": image_records,
            "metadata_dir": f"metadata/{split}/{connector_dir}",
            "image_layout": "images/<split>/<connector>/<camera>",
            "metadata_layout": "metadata/<split>/<connector>/<camera>",
            "visible_cameras": visible_cameras,
            "timestamps": timestamps,
        }
    try:
        with self._rpy_metadata_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(self._json_safe(metadata), ensure_ascii=False) + "\n")
    except OSError as exc:
        for path in written_paths:
            path.unlink(missing_ok=True)
        return False, f"metadata.jsonl 저장 실패: {exc}"
    self._rpy_sample_count += 1
    return (
        True,
        f"sample_id={sample_id}, cameras={sorted(image_records)}, "
        f"saved_count={self._rpy_sample_count}",
    )

def _save_vision_offset_sample(
    self,
    episode_name: str,
    task,
    phase: str,
    step_idx: int,
    obs,
    port_tf: Transform,
    plug_tf: Transform,
    pose: Pose,
    extras: dict[str, Any],
    detections_by_camera: Optional[dict[str, Optional[dict[str, Any]]]] = None,
) -> tuple[bool, str]:
    """기존 vision-offset 저장 API를 XYZ/RPY sample 저장기로 연결한다."""
    return self._save_xyz_rpy_sample(
        episode_name,
        task,
        phase,
        step_idx,
        obs,
        port_tf,
        plug_tf,
        pose,
        extras,
        detections_by_camera,
    )
