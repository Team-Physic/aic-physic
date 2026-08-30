"""YOLO port class, multi-camera triangulation, approach tracking."""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from itertools import combinations
from typing import Any, Callable

import cv2
import numpy as np

from .geometry import transform_matrix
from .final_policy_reid import AppearanceReID


CAMERAS = ("left", "center", "right")
CAMERA_FRAMES = {
    camera: os.environ.get(
        f"AIC_{camera.upper()}_CAMERA_FRAME", f"{camera}_camera/optical"
    )
    for camera in CAMERAS
}
CLASS_PATTERN = re.compile(r"^SFP_(\d+)(\d)$", re.IGNORECASE)
VALID_RAILS = range(5)
VALID_PORTS = range(2)
ANSI_BLUE = "\033[1;34m"
ANSI_RESET = "\033[0m"


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def observation_stamp_ns(observation) -> int:
    """Observation의 center image 촬영 시각을 nanosecond로 반환한다."""
    if observation is None or observation.center_image is None:
        return 0
    return _stamp_ns(observation.center_image.header.stamp)


def _image_for_camera(observation, camera: str):
    return getattr(observation, f"{camera}_image")


def _camera_info_for(observation, camera: str):
    return getattr(observation, f"{camera}_camera_info")


def _image_to_bgr(policy, message, camera: str) -> np.ndarray | None:
    if message is None or message.width == 0 or message.height == 0:
        return None
    try:
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
    except ValueError:
        policy.get_logger().warn(f"FinalPolicy: invalid {camera} image buffer")
        return None
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


@dataclass(frozen=True)
class TargetSpec:
    port_type: str
    rail_index: int
    port_index: int
    class_name: str


@dataclass
class PortEstimate:
    class_name: str
    xyz: np.ndarray
    normal: np.ndarray
    stamp: Any
    stamp_ns: int
    detections: dict[str, dict[str, Any]]
    images: dict[str, np.ndarray]
    reprojection_rms_px: float
    reid_score: float | None = None


def parse_model_class(class_name: str) -> TargetSpec:
    """SFP_41을 port type SFP, rail 4, port 1로 변환하고 범위를 검증한다."""
    normalized = str(class_name).strip().upper()
    match = CLASS_PATTERN.fullmatch(normalized)
    if match is None:
        raise ValueError(f"unsupported YOLO port class: {class_name!r}")
    rail_text, port_text = match.groups()
    port_type = "SFP"
    rail_index, port_index = int(rail_text), int(port_text)
    if rail_index not in VALID_RAILS:
        raise ValueError(f"invalid {port_type} rail index: {rail_index}")
    if port_index not in VALID_PORTS:
        raise ValueError(f"invalid {port_type} port index: {port_index}")
    return TargetSpec(port_type, rail_index, port_index, normalized)


def target_from_task(task) -> TargetSpec:
    """Task의 port/module 이름을 YOLO target specification으로 변환한다."""
    port_type = str(task.port_type).strip().upper()
    if port_type != "SFP":
        raise ValueError(
            f"unsupported task port type: {task.port_type!r}; "
            "FinalPolicy currently supports SFP only"
        )
    module = re.fullmatch(r"nic_card_mount_(\d+)", str(task.target_module_name))
    port = re.fullmatch(r"sfp_port_(\d+)", str(task.port_name))
    if module is None or port is None:
        raise ValueError("SFP task requires nic_card_mount_<rail>/sfp_port_<port>")
    rail_index, port_index = int(module.group(1)), int(port.group(1))
    return parse_model_class(f"{port_type}_{rail_index}{port_index}")


def triangulate_point(
    projection_a: np.ndarray,
    projection_b: np.ndarray,
    uv_a: np.ndarray,
    uv_b: np.ndarray,
) -> np.ndarray:
    """두 camera pixel과 projection matrix로 base_link 3D point를 복원한다."""
    homogeneous = cv2.triangulatePoints(
        np.asarray(projection_a, dtype=np.float64),
        np.asarray(projection_b, dtype=np.float64),
        np.asarray(uv_a, dtype=np.float64).reshape(2, 1),
        np.asarray(uv_b, dtype=np.float64).reshape(2, 1),
    )[:, 0]
    if abs(float(homogeneous[3])) < 1e-12:
        return np.full(3, np.nan)
    return homogeneous[:3] / homogeneous[3]


def project_point(projection: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    """base_link 3D point를 camera pixel로 투영한다."""
    pixel = np.asarray(projection, dtype=float) @ np.append(xyz, 1.0)
    if abs(float(pixel[2])) < 1e-12:
        return np.full(2, np.nan)
    return pixel[:2] / pixel[2]


def plane_normal(corners: np.ndarray, viewpoint: np.ndarray) -> np.ndarray:
    """3D port corner plane의 normal을 계산하고 camera 쪽으로 부호를 선택한다."""
    points = np.asarray(corners, dtype=float).reshape(-1, 3)
    if len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("at least three finite corners are required")
    centered = points - np.mean(points, axis=0)
    _, singular_values, basis = np.linalg.svd(centered, full_matrices=False)
    if len(singular_values) < 2 or float(singular_values[1]) < 1e-6:
        raise ValueError("degenerate port corners")
    normal = basis[-1]
    normal /= max(float(np.linalg.norm(normal)), 1e-12)
    if float(np.dot(normal, np.asarray(viewpoint) - np.mean(points, axis=0))) < 0.0:
        normal = -normal
    return normal


def track_keypoints(
    previous_gray: np.ndarray,
    current_gray: np.ndarray,
    previous_points: np.ndarray,
    forward_backward_max_px: float,
) -> tuple[np.ndarray, float] | None:
    """Pyramidal KLT와 forward-backward 검사로 keypoint의 current pixel을 반환한다."""
    points = np.asarray(previous_points, dtype=np.float32).reshape(-1, 1, 2)
    if len(points) < 3 or previous_gray.shape != current_gray.shape:
        return None
    current, status_forward, _ = cv2.calcOpticalFlowPyrLK(
        previous_gray,
        current_gray,
        points,
        None,
        winSize=(21, 21),
        maxLevel=3,
    )
    if current is None or status_forward is None:
        return None
    backward, status_backward, _ = cv2.calcOpticalFlowPyrLK(
        current_gray,
        previous_gray,
        current,
        None,
        winSize=(21, 21),
        maxLevel=3,
    )
    if backward is None or status_backward is None:
        return None
    valid = status_forward[:, 0].astype(bool) & status_backward[:, 0].astype(bool)
    errors = np.linalg.norm(points[:, 0] - backward[:, 0], axis=1)
    valid &= np.isfinite(errors) & (errors <= forward_backward_max_px)
    if int(np.count_nonzero(valid)) != len(points):
        return None
    return current[:, 0][valid], float(np.sqrt(np.mean(np.square(errors[valid]))))


DebugImageCallback = Callable[[str, Any, np.ndarray, Any], None]


class PortVision:
    """YOLO exact-class detection을 base_link pose와 연속 track으로 변환한다."""

    def __init__(
        self,
        policy,
        target: TargetSpec,
        model=None,
        debug_image_callback: DebugImageCallback | None = None,
    ):
        self.policy = policy
        self.target = target
        self.model = model
        self.debug_image_callback = debug_image_callback
        self.reid = AppearanceReID(policy.get_logger())
        self._device_logged = False
        self.confidence = _env_float("AIC_YOLO_CONFIDENCE", 0.8)
        self.keypoint_confidence = _env_float("AIC_YOLO_KEYPOINT_CONFIDENCE", 0.25)
        self.device = os.environ.get("AIC_YOLO_DEVICE", "").strip() or None
        self.sync_max_ns = int(
            _env_float("AIC_TRIANGULATION_SYNC_THRESHOLD_MS", 1.0) * 1_000_000
        )
        self.reprojection_max_px = _env_float(
            "AIC_TRIANGULATION_REPROJECTION_MAX_PX", 30.0
        )
        self.forward_backward_max_px = _env_float(
            "AIC_TRACK_FORWARD_BACKWARD_MAX_PX", 2.0
        )
        self.track_reprojection_max_px = _env_float(
            "AIC_TRACK_REPROJECTION_MAX_ERROR_PX", 30.0
        )
        self.track_max_3d_jump_m = _env_float("AIC_TRACK_MAX_3D_JUMP_M", 0.03)
        self.workspace_min = self._env_vector(
            "AIC_FINAL_POLICY_WORKSPACE_MIN", "-1.0,-1.0,-0.1"
        )
        self.workspace_max = self._env_vector(
            "AIC_FINAL_POLICY_WORKSPACE_MAX", "1.0,1.0,1.5"
        )

    @staticmethod
    def _env_vector(name: str, default: str) -> np.ndarray:
        try:
            values = [float(value.strip()) for value in os.environ.get(name, default).split(",")]
        except ValueError:
            values = [float(value) for value in default.split(",")]
        return np.asarray(values if len(values) == 3 else default.split(","), dtype=float)

    def load_model(self) -> bool:
        """YOLO와 선택된 appearance encoder를 Policy 시작 전에 load한다."""
        if self.model is None:
            variable = "AIC_SFP_YOLO_MODEL_PATH"
            model_path = os.environ.get(variable, "").strip()
            if not model_path:
                self.policy.get_logger().error(f"FinalPolicy: {variable} is required")
                return False
            try:
                from ultralytics import YOLO

                self.model = YOLO(model_path)
            except Exception as exc:
                self.policy.get_logger().error(f"FinalPolicy: YOLO load failed: {exc}")
                return False
        try:
            self.reid.load()
        except Exception as exc:
            self.policy.get_logger().error(f"FinalPolicy: ReID load failed: {exc}")
            return False
        self.policy.get_logger().info(
            f"{ANSI_BLUE}FinalPolicy: YOLO requested device: "
            f"{self.device or 'auto'}{ANSI_RESET}"
        )
        return True

    def _projection_data(self, observation) -> dict[str, Any] | None:
        image_messages = {
            camera: _image_for_camera(observation, camera)
            for camera in CAMERAS
        }
        stamps = {
            camera: _stamp_ns(image_messages[camera].header.stamp)
            for camera in CAMERAS
        }
        if (
            not all(stamps.values())
            or max(stamps.values()) - min(stamps.values()) > self.sync_max_ns
        ):
            return None
        reference_stamp = observation.center_image.header.stamp
        images: dict[str, np.ndarray] = {}
        projections: dict[str, np.ndarray] = {}
        transforms: dict[str, np.ndarray] = {}
        camera_origins: dict[str, np.ndarray] = {}
        for camera in CAMERAS:
            image = _image_to_bgr(self.policy, image_messages[camera], camera)
            camera_info = _camera_info_for(observation, camera)
            if image is None or camera_info is None or len(camera_info.k) < 9:
                return None
            intrinsic = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
            stamped = self.policy.lookup_transform_at(
                CAMERA_FRAMES[camera], "base_link", reference_stamp
            )
            transform = stamped.transform
            camera_from_base = transform_matrix(
                [transform.translation.x, transform.translation.y, transform.translation.z],
                [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ],
            )
            images[camera] = image
            transforms[camera] = camera_from_base
            projections[camera] = intrinsic @ camera_from_base[:3, :]
            camera_origins[camera] = np.linalg.inv(camera_from_base)[:3, 3]
        return {
            "stamp": reference_stamp,
            "stamp_ns": _stamp_ns(reference_stamp),
            "headers": {
                camera: image_messages[camera].header for camera in CAMERAS
            },
            "images": images,
            "projections": projections,
            "transforms": transforms,
            "camera_origins": camera_origins,
        }

    @staticmethod
    def _numpy(value) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value)

    def _detect(
        self,
        images: dict[str, np.ndarray],
        headers: dict[str, Any] | None = None,
    ) -> dict[str, list[dict[str, Any]]]:
        if self.model is None:
            return {camera: [] for camera in CAMERAS}
        source = [images[camera] for camera in CAMERAS]
        predict = getattr(self.model, "predict", self.model)
        arguments = {
            "source": source,
            "conf": self.confidence,
            "verbose": False,
        }
        if self.device is not None:
            arguments["device"] = self.device
        results = predict(**arguments)
        if not self._device_logged:
            predictor = getattr(self.model, "predictor", None)
            selected = getattr(predictor, "device", self.device or "auto")
            self.policy.get_logger().info(
                f"{ANSI_BLUE}FinalPolicy: YOLO selected device: "
                f"{selected}{ANSI_RESET}"
            )
            self._device_logged = True
        detections = {camera: [] for camera in CAMERAS}
        for camera, result in zip(CAMERAS, results):
            if self.debug_image_callback is not None:
                try:
                    self.debug_image_callback(
                        camera,
                        result,
                        images[camera],
                        headers.get(camera) if headers is not None else None,
                    )
                except Exception as exc:
                    self.policy.get_logger().warn(
                        f"FinalPolicy: debug image publish failed: {exc}"
                    )
            if result.boxes is None or result.keypoints is None:
                continue
            points_all = self._numpy(result.keypoints.xy)
            confidence_all = (
                self._numpy(result.keypoints.conf)
                if getattr(result.keypoints, "conf", None) is not None
                else None
            )
            names = getattr(result, "names", getattr(self.model, "names", {}))
            for index, box in enumerate(result.boxes):
                class_index = int(self._numpy(box.cls).reshape(-1)[0])
                class_name = str(
                    names[class_index] if isinstance(names, dict) else names[class_index]
                ).strip().upper()
                if class_name != self.target.class_name or index >= len(points_all):
                    continue
                try:
                    parsed = parse_model_class(class_name)
                except ValueError:
                    continue
                if parsed != self.target:
                    continue
                keypoints = np.asarray(points_all[index], dtype=float)[:4]
                if keypoints.shape != (4, 2) or not np.isfinite(keypoints).all():
                    continue
                if confidence_all is not None:
                    point_confidence = np.asarray(confidence_all[index], dtype=float)[:4]
                    if len(point_confidence) < 4 or np.any(
                        point_confidence < self.keypoint_confidence
                    ):
                        continue
                detections[camera].append(
                    {
                        "class_name": class_name,
                        "confidence": float(self._numpy(box.conf).reshape(-1)[0]),
                        "keypoint_confidence": (
                            point_confidence
                            if confidence_all is not None
                            else np.ones(4, dtype=float)
                        ),
                        "keypoints": keypoints,
                        "uv": np.mean(keypoints, axis=0),
                    }
                )
        return detections

    def _estimate_candidates(
        self,
        data: dict[str, Any],
        detections: dict[str, list[dict[str, Any]]],
    ) -> list[PortEstimate]:
        cameras = [camera for camera in CAMERAS if detections.get(camera)]
        candidates: list[PortEstimate] = []
        for camera_a, camera_b in combinations(cameras, 2):
            for detection_a in detections[camera_a]:
                for detection_b in detections[camera_b]:
                    corners = np.asarray(
                        [
                            triangulate_point(
                                data["projections"][camera_a],
                                data["projections"][camera_b],
                                detection_a["keypoints"][index],
                                detection_b["keypoints"][index],
                            )
                            for index in range(4)
                        ]
                    )
                    xyz = np.mean(corners, axis=0)
                    if not np.isfinite(xyz).all():
                        continue
                    if np.any(xyz < self.workspace_min) or np.any(xyz > self.workspace_max):
                        continue
                    matched = {camera_a: detection_a, camera_b: detection_b}
                    errors: list[float] = []
                    valid = True
                    for camera in cameras:
                        camera_xyz = data["transforms"][camera] @ np.append(xyz, 1.0)
                        if float(camera_xyz[2]) <= 1e-6:
                            valid = False
                            break
                        projected = project_point(data["projections"][camera], xyz)
                        detection = matched.get(camera)
                        if detection is None:
                            detection = min(
                                detections[camera],
                                key=lambda candidate: float(
                                    np.linalg.norm(candidate["uv"] - projected)
                                ),
                            )
                            matched[camera] = detection
                        error = float(np.linalg.norm(detection["uv"] - projected))
                        if not np.isfinite(error) or error > self.reprojection_max_px:
                            valid = False
                            break
                        errors.append(error)
                    if not valid:
                        continue
                    viewpoint = np.mean(
                        [data["camera_origins"][camera] for camera in matched], axis=0
                    )
                    try:
                        normal = plane_normal(corners, viewpoint)
                    except ValueError:
                        continue
                    candidates.append(
                        PortEstimate(
                            class_name=self.target.class_name,
                            xyz=xyz,
                            normal=normal,
                            stamp=data["stamp"],
                            stamp_ns=data["stamp_ns"],
                            detections=matched,
                            images={camera: data["images"][camera] for camera in matched},
                            reprojection_rms_px=float(
                                np.sqrt(np.mean(np.square(errors)))
                            ),
                        )
                    )
        return sorted(candidates, key=lambda candidate: candidate.reprojection_rms_px)

    def estimate(self, observation) -> PortEstimate | None:
        """동기화 Observation에서 geometry와 ReID를 통과한 target을 반환한다."""
        if observation is None:
            return None
        try:
            data = self._projection_data(observation)
        except Exception as exc:
            self.policy.get_logger().warn(f"FinalPolicy: camera TF lookup failed: {exc}")
            return None
        if data is None:
            return None
        candidates = self._estimate_candidates(
            data, self._detect(data["images"], data["headers"])
        )
        return self.reid.select(candidates)

    def lock_identity(self, estimate: PortEstimate) -> bool:
        """Exact-class initial lock의 appearance를 recovery memory로 고정한다."""
        try:
            return self.reid.remember(estimate)
        except Exception as exc:
            self.policy.get_logger().error(
                f"FinalPolicy: initial ReID descriptor failed: {exc}"
            )
            return False

    def track(self, observation, previous: PortEstimate) -> PortEstimate | None:
        """YOLO 호출 없이 optical flow로 직전 target keypoint를 갱신한다."""
        if observation is None:
            return None
        try:
            data = self._projection_data(observation)
        except Exception:
            return None
        if data is None or data["stamp_ns"] <= previous.stamp_ns:
            return None
        selected: dict[str, list[dict[str, Any]]] = {}
        for camera, previous_detection in previous.detections.items():
            previous_gray = cv2.cvtColor(previous.images[camera], cv2.COLOR_BGR2GRAY)
            current_gray = cv2.cvtColor(data["images"][camera], cv2.COLOR_BGR2GRAY)
            tracked = track_keypoints(
                previous_gray,
                current_gray,
                previous_detection["keypoints"],
                self.forward_backward_max_px,
            )
            if tracked is None:
                continue
            tracked_points, _ = tracked
            flow_center = np.mean(tracked_points, axis=0)
            projected = project_point(data["projections"][camera], previous.xyz)
            projection_error = float(np.linalg.norm(flow_center - projected))
            if projection_error > self.track_reprojection_max_px:
                continue
            selected[camera] = [
                {
                    "class_name": previous.class_name,
                    "confidence": previous_detection.get("confidence", 1.0),
                    "keypoints": tracked_points,
                    "uv": flow_center,
                }
            ]
        if len(selected) < 2:
            return None
        candidates = self._estimate_candidates(data, selected)
        if not candidates:
            return None
        current = candidates[0]
        if float(np.linalg.norm(current.xyz - previous.xyz)) > self.track_max_3d_jump_m:
            return None
        return current
