"""Camera subscriptions and browser image encoding."""

from __future__ import annotations

from functools import partial

import cv2
import numpy as np
from models import CAMERAS, IMAGE_TOPICS, DashboardState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Convert a packed rgb8/bgr8 ROS image into owned BGR pixels."""

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    if encoding not in {"rgb8", "bgr8"}:
        raise ValueError(f"unsupported image encoding: {message.encoding!r}")
    row_bytes = width * 3
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than {row_bytes}")
    buffer = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if buffer.size < required:
        raise ValueError(
            f"image buffer has {buffer.size} bytes; expected at least {required}"
        )
    packed = buffer[:required].reshape(height, step)[:, :row_bytes]
    image = packed.reshape(height, width, 3)
    return image.copy() if encoding == "bgr8" else image[:, :, ::-1].copy()


def image_message_to_jpeg(
    message: Image,
    quality: int = 85,
) -> tuple[bytes, int, int]:
    """Encode a ROS image as JPEG bytes for an MJPEG response."""

    if not 1 <= quality <= 100:
        raise ValueError("JPEG quality must be between 1 and 100")
    image = image_message_to_bgr(message)
    encoded, buffer = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not encoded:
        raise ValueError("JPEG encoding failed")
    height, width = image.shape[:2]
    return buffer.tobytes(), width, height


class ImageController:
    """Receive the three policy debug images."""

    def __init__(
        self,
        node: Node,
        state: DashboardState,
        jpeg_quality: int,
    ) -> None:
        self._node = node
        self._state = state
        self._jpeg_quality = jpeg_quality
        self._subscriptions = [
            node.create_subscription(
                Image,
                IMAGE_TOPICS[camera],
                partial(self._on_image, camera),
                qos_profile_sensor_data,
            )
            for camera in CAMERAS
        ]
        for camera, topic in IMAGE_TOPICS.items():
            node.get_logger().info(f"{camera} YOLO image: {topic}")

    def _on_image(self, camera: str, message: Image) -> None:
        try:
            jpeg, width, height = image_message_to_jpeg(
                message, quality=self._jpeg_quality
            )
        except ValueError as exc:
            self._node.get_logger().warn(f"invalid {camera} YOLO image: {exc}")
            return
        self._state.update_frame(
            camera, jpeg, width, height, message.header.stamp
        )
