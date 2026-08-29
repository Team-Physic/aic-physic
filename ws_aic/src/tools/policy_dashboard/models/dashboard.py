"""Thread-safe application state shared by ROS and HTTP."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from math import asin, atan2, degrees, sqrt
from typing import Any

CAMERAS = ("left", "center", "right")
IMAGE_TOPICS = {
    camera: f"/final_policy/yolo/{camera}/image" for camera in CAMERAS
}
POINT_TOPIC = "/final_policy/triangulated_port_xyz"
TASK_TOPIC = "/final_policy/task"
CONTROLLER_STATE_TOPIC = "/aic_controller/controller_state"
WRENCH_TOPIC = "/fts_broadcaster/wrench"
STALE_AFTER_SECONDS = 2.0


@dataclass(frozen=True)
class CameraFrame:
    """Latest browser-ready image and its display metadata."""

    jpeg: bytes
    sequence: int
    width: int
    height: int
    fps: float
    stamp: str
    received_at: float


@dataclass(frozen=True)
class PointValue:
    """Latest triangulated port position."""

    sequence: int
    frame_id: str
    x: float
    y: float
    z: float
    stamp: str
    received_at: float


@dataclass(frozen=True)
class OrientationValue:
    """Orientation and local +Z direction expressed in the base frame."""

    frame_id: str
    quaternion: tuple[float, float, float, float]
    rpy_degrees: tuple[float, float, float]
    direction: tuple[float, float, float]
    stamp: str
    received_at: float


@dataclass(frozen=True)
class PositionValue:
    """Latest frame position expressed in ``base_link``."""

    sequence: int
    frame_id: str
    xyz: tuple[float, float, float]
    stamp: str
    received_at: float


@dataclass(frozen=True)
class TaskValue:
    """Frames derived from the active insertion task."""

    trial_index: int
    task_id: str
    cable_frame: str
    port_frame: str
    received_at: float


@dataclass(frozen=True)
class WrenchValue:
    """Latest force-torque sample."""

    sequence: int
    frame_id: str
    force: tuple[float, float, float]
    torque: tuple[float, float, float]
    chart_torque: tuple[float, float, float]
    stamp: str
    received_at: float


def _stamp_text(stamp: Any) -> str:
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    return f"{seconds}.{nanoseconds:09d}"


def _orientation_values(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("orientation quaternion has zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    direction = (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )
    return (
        (x, y, z, w),
        (degrees(roll), degrees(pitch), degrees(yaw)),
        direction,
    )


class DashboardState:
    """Own the latest policy telemetry without depending on a UI framework."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, CameraFrame] = {}
        self._point: PointValue | None = None
        self._orientations: dict[str, OrientationValue] = {}
        self._positions: dict[str, PositionValue] = {}
        self._task: TaskValue | None = None
        self._trial_index = 0
        self._wrench: WrenchValue | None = None

    def update_frame(
        self,
        camera: str,
        jpeg: bytes,
        width: int,
        height: int,
        stamp: Any,
    ) -> None:
        now = time.monotonic()
        with self._lock:
            previous = self._frames.get(camera)
            sequence = 1 if previous is None else previous.sequence + 1
            fps = 0.0
            if previous is not None:
                instantaneous = 1.0 / max(now - previous.received_at, 1e-6)
                fps = (
                    instantaneous
                    if previous.fps == 0.0
                    else 0.8 * previous.fps + 0.2 * instantaneous
                )
            self._frames[camera] = CameraFrame(
                jpeg=jpeg,
                sequence=sequence,
                width=width,
                height=height,
                fps=fps,
                stamp=_stamp_text(stamp),
                received_at=now,
            )

    def update_point(
        self,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        stamp: Any,
    ) -> None:
        now = time.monotonic()
        with self._lock:
            sequence = 1 if self._point is None else self._point.sequence + 1
            self._point = PointValue(
                sequence=sequence,
                frame_id=frame_id or "?",
                x=float(x),
                y=float(y),
                z=float(z),
                stamp=_stamp_text(stamp),
                received_at=now,
            )

    def update_task(
        self,
        task_id: str,
        cable_frame: str,
        port_frame: str,
    ) -> int:
        """Start an observed trial and discard its predecessor's live values.

        FinalPolicy publishes exactly one active task for each insertion trial.
        Counting those publications gives the dashboard a stable trial boundary
        even when consecutive trials reuse the same task ID and frames.
        """
        with self._lock:
            self._trial_index += 1
            self._task = TaskValue(
                trial_index=self._trial_index,
                task_id=task_id or "?",
                cable_frame=cable_frame,
                port_frame=port_frame,
                received_at=time.monotonic(),
            )
            self._point = None
            self._positions.clear()
            return self._trial_index

    def update_position(
        self,
        source: str,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        stamp: Any,
    ) -> None:
        """Update a source position used by the interactive 3D viewer."""
        now = time.monotonic()
        with self._lock:
            previous = self._positions.get(source)
            sequence = 1 if previous is None else previous.sequence + 1
            self._positions[source] = PositionValue(
                sequence=sequence,
                frame_id=frame_id,
                xyz=(float(x), float(y), float(z)),
                stamp=_stamp_text(stamp),
                received_at=now,
            )

    def update_orientation(
        self,
        source: str,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        w: float,
        stamp: Any,
    ) -> None:
        quaternion, rpy_degrees, direction = _orientation_values(x, y, z, w)
        value = OrientationValue(
            frame_id=frame_id,
            quaternion=quaternion,
            rpy_degrees=rpy_degrees,
            direction=direction,
            stamp=_stamp_text(stamp),
            received_at=time.monotonic(),
        )
        with self._lock:
            self._orientations[source] = value

    def clear_orientation(self, source: str) -> None:
        """Discard an orientation whose source frame is no longer active."""
        with self._lock:
            self._orientations.pop(source, None)

    def clear_position(self, source: str) -> None:
        """Discard a position whose source frame is no longer active."""
        with self._lock:
            self._positions.pop(source, None)

    def update_wrench(
        self,
        frame_id: str,
        force: tuple[float, float, float],
        torque: tuple[float, float, float],
        stamp: Any,
        chart_torque: tuple[float, float, float] | None = None,
    ) -> None:
        now = time.monotonic()
        with self._lock:
            sequence = 1 if self._wrench is None else self._wrench.sequence + 1
            self._wrench = WrenchValue(
                sequence=sequence,
                frame_id=frame_id or "?",
                force=tuple(map(float, force)),
                torque=tuple(map(float, torque)),
                chart_torque=tuple(
                    map(float, chart_torque if chart_torque is not None else torque)
                ),
                stamp=_stamp_text(stamp),
                received_at=now,
            )

    def frame(self, camera: str) -> CameraFrame | None:
        with self._lock:
            return self._frames.get(camera)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            frames = dict(self._frames)
            point = self._point
            orientations = dict(self._orientations)
            positions = dict(self._positions)
            task = self._task
            wrench = self._wrench

        cameras: dict[str, dict[str, Any]] = {}
        for camera in CAMERAS:
            frame = frames.get(camera)
            if frame is None:
                cameras[camera] = {
                    "status": "waiting",
                    "topic": IMAGE_TOPICS[camera],
                    "sequence": 0,
                }
                continue
            age = max(0.0, now - frame.received_at)
            cameras[camera] = {
                "status": "live" if age <= STALE_AFTER_SECONDS else "stale",
                "topic": IMAGE_TOPICS[camera],
                "sequence": frame.sequence,
                "width": frame.width,
                "height": frame.height,
                "fps": round(frame.fps, 1),
                "stamp": frame.stamp,
                "age_seconds": round(age, 2),
            }

        point_payload = None
        if point is not None:
            point_age = max(0.0, now - point.received_at)
            point_payload = {
                "status": (
                    "live" if point_age <= STALE_AFTER_SECONDS else "stale"
                ),
                "sequence": point.sequence,
                "frame_id": point.frame_id,
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "xyz": (point.x, point.y, point.z),
                "stamp": point.stamp,
                "age_seconds": round(point_age, 2),
            }

        spatial_payload: dict[str, Any] = {
            "task": None,
            "port": {"status": "waiting", "sequence": 0},
            "ee": {"status": "waiting", "sequence": 0},
            "cable": {"status": "waiting", "sequence": 0},
            "triangulated_port": point_payload
            or {"status": "waiting", "sequence": 0},
        }
        if task is not None:
            spatial_payload["task"] = {
                "trial_index": task.trial_index,
                "id": task.task_id,
                "cable_frame": task.cable_frame,
                "port_frame": task.port_frame,
            }
        for source in ("port", "ee", "cable"):
            position = positions.get(source)
            if position is None:
                continue
            age = max(0.0, now - position.received_at)
            spatial_payload[source] = {
                "status": (
                    "live" if age <= STALE_AFTER_SECONDS else "stale"
                ),
                "sequence": position.sequence,
                "frame_id": position.frame_id,
                "xyz": position.xyz,
                "stamp": position.stamp,
                "age_seconds": round(age, 2),
            }

        orientation_payload: dict[str, dict[str, Any]] = {}
        expected_frames = {"ee": "gripper/tcp", "cable": "cable tip"}
        for source, expected_frame in expected_frames.items():
            orientation = orientations.get(source)
            if orientation is None:
                orientation_payload[source] = {
                    "status": "waiting",
                    "frame_id": expected_frame,
                }
                continue
            age = max(0.0, now - orientation.received_at)
            orientation_payload[source] = {
                "status": "live" if age <= STALE_AFTER_SECONDS else "stale",
                "frame_id": orientation.frame_id,
                "quaternion": orientation.quaternion,
                "rpy_degrees": orientation.rpy_degrees,
                "direction": orientation.direction,
                "stamp": orientation.stamp,
                "age_seconds": round(age, 2),
            }

        wrench_payload: dict[str, Any] = {
            "status": "waiting",
            "topic": WRENCH_TOPIC,
            "sequence": 0,
        }
        if wrench is not None:
            age = max(0.0, now - wrench.received_at)
            force_magnitude = sqrt(sum(value * value for value in wrench.force))
            torque_magnitude = sqrt(sum(value * value for value in wrench.torque))
            wrench_payload = {
                "status": "live" if age <= STALE_AFTER_SECONDS else "stale",
                "topic": WRENCH_TOPIC,
                "sequence": wrench.sequence,
                "frame_id": wrench.frame_id,
                "force": wrench.force,
                "torque": wrench.torque,
                "chart_torque": wrench.chart_torque,
                "force_magnitude": force_magnitude,
                "torque_magnitude": torque_magnitude,
                "stamp": wrench.stamp,
                "age_seconds": round(age, 2),
            }
        return {
            "cameras": cameras,
            "point": point_payload,
            "point_topic": POINT_TOPIC,
            "task_topic": TASK_TOPIC,
            "orientation": orientation_payload,
            "orientation_axis": "+Z",
            "spatial": spatial_payload,
            "controller_state_topic": CONTROLLER_STATE_TOPIC,
            "wrench": wrench_payload,
        }
