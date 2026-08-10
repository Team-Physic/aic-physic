"""동기화된 camera image와 img2pos XYZ label 저장."""

from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any

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


def _port_projection(policy, observation, camera: str, port_tf: Transform) -> dict[str, Any]:
    """포트 위치가 지정 camera의 유효 영상 영역에 보이는지 검사한다."""
    message = image_for_camera(observation, camera)
    intrinsic = _camera_matrix(camera_info_for(observation, camera))
    if message is None or message.width == 0 or message.height == 0 or intrinsic is None:
        return {"visible": False, "reason": "missing_image_or_intrinsics"}
    try:
        point_camera = _base_to_camera(policy, observation, camera) @ np.array(
            [port_tf.translation.x, port_tf.translation.y, port_tf.translation.z, 1.0],
            dtype=float,
        )
    except Exception as exc:
        return {"visible": False, "reason": f"camera_transform_error: {exc}"}
    depth = float(point_camera[2])
    if depth <= 1e-6:
        return {"visible": False, "reason": "behind_camera", "depth_m": depth}
    u = float(intrinsic[0, 0] * point_camera[0] / depth + intrinsic[0, 2])
    v = float(intrinsic[1, 1] * point_camera[1] / depth + intrinsic[1, 2])
    margin = policy.visibility_margin_px
    return {
        "visible": bool(
            margin <= u < float(message.width) - margin
            and margin <= v < float(message.height) - margin
        ),
        "u_px": u,
        "v_px": v,
        "depth_m": depth,
    }


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
    if camera_skew > policy.sync_tolerance_ns:
        timestamps["rejection_reason"] = "camera_time_difference_exceeded"
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
    return f"{run_id}:{trial_index}" if run_id and trial_index else episode_name


def split_for_trial(policy, trial_id: str) -> str:
    """같은 trial을 안정적으로 동일한 train/val split에 배정한다."""
    ratio = min(max(float(policy.val_ratio), 0.0), 1.0)
    if ratio <= 0.0:
        return "train"
    if ratio >= 1.0:
        return "val"
    fraction = int.from_bytes(hashlib.sha256(trial_id.encode()).digest()[:8], "big") / float(1 << 64)
    return "val" if fraction < ratio else "train"


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


def write_manifest(policy) -> None:
    """compact img2pos 데이터셋 schema를 data.yaml에 기록한다."""
    policy.dataset_dir.mkdir(parents=True, exist_ok=True)
    (policy.dataset_dir / "data.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                "task: img2pos",
                f"version: {policy.dataset_version or 'default'}",
                "input: rgb_image",
                "samples: samples.jsonl",
                "image_layout: images/<split>/<camera>/*.jpg",
                "cameras: [left, center, right]",
                "target:",
                "  name: correction_xyz",
                "  definition: port_entrance - plug_reference",
                "  frame: base_link",
                "  unit: meter",
                "split:",
                "  group_by: trial_id",
                f"  validation_ratio: {policy.val_ratio:.6f}",
                "",
            ]
        ),
        encoding="utf-8",
    )


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
) -> tuple[bool, str]:
    """가시 camera JPEG와 최소 img2pos JSONL row를 저장한다."""
    if observation is None:
        return False, "Observation을 받지 못함"
    if not timestamps.get("sync_valid", False):
        return False, f"수집 시각 일치 검사 실패: {timestamps.get('rejection_reason', 'unknown reason')}"
    capture_stamp = int(timestamps.get("capture_stamp_ns", 0))
    if capture_stamp <= 0:
        return False, "유효한 camera capture timestamp가 없음"
    if len(label_xyz) != 3 or not all(np.isfinite(float(value)) for value in label_xyz):
        return False, "유효한 XYZ correction label이 없음"
    label_xyz = [float(value) for value in label_xyz]
    visible = [
        camera
        for camera in ("left", "center", "right")
        if _port_projection(policy, observation, camera, port_tf).get("visible", False)
    ]
    if len(visible) < policy.min_visible_cameras:
        return False, f"포트 가시성 부족: visible={visible}, required={policy.min_visible_cameras}"

    trial_id = _trial_id(policy, episode_name)
    split = split_for_trial(policy, trial_id)
    capture_id = f"{episode_name}_collect_{step_idx:06d}"
    records: list[dict[str, Any]] = []
    written: list[Path] = []
    failures: list[str] = []
    for camera in visible:
        image = image_to_bgr(policy, image_for_camera(observation, camera), camera)
        if image is None:
            failures.append(f"{camera}: image 변환 실패")
            continue
        sample_id = f"{capture_id}_{camera}"
        path = policy.dataset_dir / "images" / split / camera / f"{sample_id}.jpg"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            failures.append(f"{camera}: JPEG 저장 실패 ({path})")
            continue
        written.append(path)
        records.append(
            {
                "id": sample_id,
                "capture_id": capture_id,
                "trial_id": trial_id,
                "split": split,
                "image": str(path.relative_to(policy.dataset_dir)),
                "camera": camera,
                "connector": _connector(task),
                "target_xyz_m": label_xyz,
                "capture_stamp_ns": capture_stamp,
                "max_sync_skew_ns": _max_skew(timestamps),
            }
        )
    if len(records) < policy.min_visible_cameras:
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"camera 저장 부족: required={policy.min_visible_cameras}, details={failures}"
    try:
        with policy.samples_path.open("a", encoding="utf-8") as stream:
            for record in records:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    except OSError as exc:
        for path in written:
            path.unlink(missing_ok=True)
        return False, f"samples.jsonl 저장 실패: {exc}"
    policy.capture_count += 1
    cameras = sorted(record["camera"] for record in records)
    return True, f"capture_id={capture_id}, cameras={cameras}, saved_count={policy.capture_count}"


def upload_to_hub(policy, task, status: str, collect_steps: int) -> dict[str, Any]:
    """수집 완료 dataset을 설정된 Hugging Face branch에 업로드한다."""
    if not policy.push_to_hub:
        return {"enabled": False, "success": False, "reason": "disabled"}
    if status != "ok" or collect_steps <= 0:
        return {"enabled": True, "success": False, "reason": f"skipped_{status}"}
    port_type = str(task.port_type).strip().lower()
    if policy.upload_on_port_type and port_type != policy.upload_on_port_type:
        return {"enabled": True, "success": False, "reason": f"waiting_for_{policy.upload_on_port_type}"}
    if not policy.hf_repo_id:
        reason = "AIC_IMG2POS_HF_REPO_ID is not set"
        policy.get_logger().warn(f"[PortOffsetCollect] HF upload skipped: {reason}")
        return {"enabled": True, "success": False, "reason": reason}
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        api.whoami()
        api.create_repo(
            repo_id=policy.hf_repo_id,
            repo_type="dataset",
            private=policy.hf_private,
            exist_ok=True,
        )
        if policy.hf_revision != "main":
            branches = [ref.name for ref in api.list_repo_refs(policy.hf_repo_id, repo_type="dataset").branches]
            if policy.hf_revision not in branches:
                api.create_branch(
                    repo_id=policy.hf_repo_id,
                    repo_type="dataset",
                    branch=policy.hf_revision,
                )
        api.upload_large_folder(
            repo_id=policy.hf_repo_id,
            repo_type="dataset",
            revision=policy.hf_revision,
            folder_path=str(policy.dataset_dir),
            ignore_patterns=["*.tmp", "*.lock", "__pycache__/*", ".DS_Store"],
            private=policy.hf_private,
        )
        url = f"https://huggingface.co/datasets/{policy.hf_repo_id}/tree/{policy.hf_revision}"
        policy.get_logger().info(f"[PortOffsetCollect] HF upload complete: {url}")
        return {"enabled": True, "success": True, "url": url}
    except Exception as exc:
        policy.get_logger().error(f"[PortOffsetCollect] HF upload failed: {exc}")
        return {"enabled": True, "success": False, "reason": str(exc)}
