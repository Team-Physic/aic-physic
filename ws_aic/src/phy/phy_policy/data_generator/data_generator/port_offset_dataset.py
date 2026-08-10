from __future__ import annotations

"""Dataset and label writers for PortOffsetCollect."""

import hashlib
import json
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from geometry_msgs.msg import Pose, Transform


def _connector_for_task(task) -> str:
    """학습 sample에 기록할 커넥터 타입을 반환한다."""
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


def _split_for_trial(self, trial_id: str) -> str:
    """같은 trial의 모든 sample을 안정적으로 동일한 split에 배정한다."""
    val_ratio = min(max(float(self._rpy_val_ratio), 0.0), 1.0)
    if val_ratio <= 0.0:
        return "train"
    if val_ratio >= 1.0:
        return "val"
    digest = hashlib.sha256(trial_id.encode("utf-8")).digest()
    fraction = int.from_bytes(digest[:8], "big") / float(1 << 64)
    return "val" if fraction < val_ratio else "train"


def _trial_id(self, episode_name: str) -> str:
    """수집 실행 metadata에서 trial 식별자를 만들고 없으면 episode 이름을 사용한다."""
    run_id = str(self._collection_metadata.get("run_id", "")).strip()
    trial_index = str(self._collection_metadata.get("trial_index", "")).strip()
    if run_id and trial_index:
        return f"{run_id}:{trial_index}"
    return episode_name


def _target_xyz_m(extras: dict[str, Any]) -> list[float] | None:
    """수집 계산 결과에서 img2pos 학습용 XYZ correction label만 추출한다."""
    label = extras.get("label")
    if not isinstance(label, dict):
        return None
    try:
        target = [float(label[name]) for name in ("x_m", "y_m", "z_m")]
    except (KeyError, TypeError, ValueError):
        return None
    return target if all(np.isfinite(value) for value in target) else None


def _max_sync_skew_ns(timestamps: dict[str, Any]) -> int:
    """저장 승인에 사용된 source별 시각 차이 중 최댓값을 반환한다."""
    values = timestamps.get("skew_ns", {})
    if not isinstance(values, dict):
        return 0
    skews = []
    for value in values.values():
        try:
            skews.append(int(value))
        except (TypeError, ValueError):
            continue
    return max(skews, default=0)


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
    self, get_observation,
    min_capture_stamp_ns: int | None = None,
) -> tuple[Any | None, dict[str, Any]]:
    """허용 오차와 최소 capture 시각을 만족하는 Observation을 순차 대기한다."""
    start_ns = time.monotonic_ns()
    timeout_ns = int(self.collect_sync_wait_timeout_sec * 1_000_000_000)
    deadline_ns = start_ns + timeout_ns
    waiting_logged = False
    timestamps: dict[str, Any] = {"sync_valid": False, "rejection_reason": "missing_observation"}

    while True:
        obs = get_observation()
        sync_valid, timestamps = self._observation_sync_metadata(obs)
        capture_stamp_ns = timestamps.get("capture_stamp_ns")
        if sync_valid and min_capture_stamp_ns is not None and capture_stamp_ns is not None:
            sync_valid = int(capture_stamp_ns) > int(min_capture_stamp_ns)
            if not sync_valid:
                timestamps["sync_valid"] = False
                timestamps["rejection_reason"] = "capture_not_after_command"
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

def _save_img2pos_sample(
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
    """승인된 영상을 compact img2pos sample과 함께 저장하고 결과를 반환한다."""
    if obs is None:
        return False, "Observation을 받지 못함"

    timestamps = dict(extras.get("timestamps", {}))
    if not timestamps.get("sync_valid", False):
        return (
            False,
            "수집 시각 일치 검사 실패: "
            f"{timestamps.get('rejection_reason', 'unknown reason')}",
        )
    try:
        capture_stamp_ns = int(timestamps.get("capture_stamp_ns", 0))
    except (TypeError, ValueError):
        capture_stamp_ns = 0
    if capture_stamp_ns <= 0:
        return False, "유효한 camera capture timestamp가 없음"

    target_xyz_m = _target_xyz_m(extras)
    if target_xyz_m is None:
        return False, "유효한 XYZ correction label이 없음"

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

    trial_id = self._trial_id(episode_name)
    split = self._split_for_trial(trial_id)
    connector = _connector_for_task(task)
    capture_id = f"{episode_name}_{phase}_{step_idx:06d}"
    sample_records: list[dict[str, Any]] = []
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

        sample_id = f"{capture_id}_{camera_name}"
        image_path = (
            self._rpy_dataset_dir
            / "images"
            / split
            / camera_name
            / f"{sample_id}.jpg"
        )
        image_path.parent.mkdir(parents=True, exist_ok=True)

        if not cv2.imwrite(str(image_path), bgr):
            write_failures.append(f"{camera_name}: JPEG 저장 실패 ({image_path})")
            continue
        written_paths.append(image_path)
        sample_records.append({
            "id": sample_id,
            "capture_id": capture_id,
            "trial_id": trial_id,
            "split": split,
            "image": str(image_path.relative_to(self._rpy_dataset_dir)),
            "camera": camera_name,
            "connector": connector,
            "target_xyz_m": target_xyz_m,
            "capture_stamp_ns": capture_stamp_ns,
            "max_sync_skew_ns": _max_sync_skew_ns(timestamps),
        })

    if len(sample_records) < self._rpy_min_visible_cameras:
        for path in written_paths:
            path.unlink(missing_ok=True)
        return (
            False,
            "필요한 camera 파일 수를 저장하지 못함: "
            f"written={[record['camera'] for record in sample_records]}, "
            f"required={self._rpy_min_visible_cameras}, "
            f"details={write_failures}",
        )

    try:
        with self._img2pos_samples_path.open("a", encoding="utf-8") as f:
            f.write("".join(
                json.dumps(record, ensure_ascii=False) + "\n"
                for record in sample_records
            ))
    except OSError as exc:
        for path in written_paths:
            path.unlink(missing_ok=True)
        return False, f"samples.jsonl 저장 실패: {exc}"
    self._rpy_sample_count += 1
    return (
        True,
        f"capture_id={capture_id}, "
        f"cameras={sorted(record['camera'] for record in sample_records)}, "
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
    """기존 vision-offset 저장 API를 compact img2pos 저장기로 연결한다."""
    return self._save_img2pos_sample(
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
