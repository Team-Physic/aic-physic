"""PyQt dashboard for FinalPolicy YOLO debug image topics."""

from __future__ import annotations

import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .rendering import image_message_to_rgb


CAMERAS = ("left", "center", "right")
IMAGE_TOPICS = {
    camera: f"/final_policy/yolo/{camera}/image" for camera in CAMERAS
}
POINT_TOPIC = "/final_policy/triangulated_port_xyz"


class CameraPanel(QFrame):
    """One camera image with connection, timestamp, and measured FPS state."""

    def __init__(self, camera: str, parent=None):
        super().__init__(parent)
        self.camera = camera
        self._image: QImage | None = None
        self._last_frame_at: float | None = None
        self._fps = 0.0
        self.setObjectName("cameraPanel")

        self.title = QLabel(camera.upper())
        self.title.setObjectName("cameraTitle")
        self.metadata = QLabel(f"waiting · {IMAGE_TOPICS[camera]}")
        self.metadata.setObjectName("cameraMetadata")
        self.image_label = QLabel("Waiting for YOLO result…")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setObjectName("cameraImage")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(self.title)
        layout.addWidget(self.image_label, 1)
        layout.addWidget(self.metadata)

    def set_frame(self, rgb: np.ndarray, stamp) -> None:
        now = time.monotonic()
        if self._last_frame_at is not None:
            instantaneous = 1.0 / max(now - self._last_frame_at, 1e-6)
            self._fps = instantaneous if self._fps == 0.0 else 0.8 * self._fps + 0.2 * instantaneous
        self._last_frame_at = now

        height, width = rgb.shape[:2]
        self._image = QImage(
            rgb.data,
            width,
            height,
            int(rgb.strides[0]),
            QImage.Format_RGB888,
        ).copy()
        seconds = int(getattr(stamp, "sec", 0))
        nanoseconds = int(getattr(stamp, "nanosec", 0))
        self.metadata.setText(
            f"{width}×{height} · {self._fps:.1f} FPS · {seconds}.{nanoseconds:09d}"
        )
        self._render()

    def _render(self) -> None:
        if self._image is None:
            return
        pixmap = QPixmap.fromImage(self._image).scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(pixmap)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render()


class DashboardWindow(QMainWindow):
    """Three-camera FinalPolicy visualization window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PHY Dashboard · FinalPolicy YOLO")
        self.resize(1720, 620)

        heading = QLabel("FinalPolicy · Live YOLO Pose")
        heading.setObjectName("heading")
        self.point_label = QLabel(f"Waiting for {POINT_TOPIC}")
        self.point_label.setObjectName("pointStatus")
        header = QHBoxLayout()
        header.addWidget(heading)
        header.addStretch(1)
        header.addWidget(self.point_label)

        self.panels = {camera: CameraPanel(camera) for camera in CAMERAS}
        cameras = QHBoxLayout()
        cameras.setSpacing(8)
        for camera in CAMERAS:
            cameras.addWidget(self.panels[camera], 1)

        root = QVBoxLayout()
        root.setContentsMargins(12, 12, 12, 12)
        root.addLayout(header)
        root.addLayout(cameras, 1)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #12161d; color: #e8edf4; }
            QLabel#heading { font-size: 22px; font-weight: 700; }
            QLabel#pointStatus { color: #8fd3ff; font-family: monospace; }
            QFrame#cameraPanel { background: #1b222c; border: 1px solid #303b49; border-radius: 6px; }
            QLabel#cameraTitle { font-size: 15px; font-weight: 700; color: #8fd3ff; }
            QLabel#cameraImage { background: #080b0f; color: #687483; }
            QLabel#cameraMetadata { color: #a9b4c2; font-family: monospace; }
            """
        )

    def set_camera_frame(self, camera: str, image: np.ndarray, stamp) -> None:
        self.panels[camera].set_frame(image, stamp)

    def set_point(self, message: PointStamped) -> None:
        point = message.point
        self.point_label.setText(
            f"port@{message.header.frame_id or '?'}  "
            f"x={point.x:+.4f}  y={point.y:+.4f}  z={point.z:+.4f} m"
        )


class DashboardNode(Node):
    """ROS subscriptions feeding the Qt widgets from the Qt timer thread."""

    def __init__(self, window: DashboardWindow):
        super().__init__("phy_dashboard")
        self.window = window
        self._image_subscriptions = [
            self.create_subscription(
                Image,
                IMAGE_TOPICS[camera],
                lambda message, camera=camera: self._on_image(camera, message),
                qos_profile_sensor_data,
            )
            for camera in CAMERAS
        ]
        self._point_subscription = self.create_subscription(
            PointStamped, POINT_TOPIC, window.set_point, 10
        )
        for camera, topic in IMAGE_TOPICS.items():
            self.get_logger().info(f"{camera} YOLO image: {topic}")

    def _on_image(self, camera: str, message: Image) -> None:
        try:
            image = image_message_to_rgb(message)
        except ValueError as exc:
            self.get_logger().warn(f"invalid {camera} YOLO image: {exc}")
            return
        self.window.set_camera_frame(camera, image, message.header.stamp)


def main(args=None) -> None:
    """Run ROS callbacks cooperatively inside the Qt event loop."""
    rclpy.init(args=args)
    app = QApplication([sys.argv[0]])
    app.setStyle("Fusion")
    window = DashboardWindow()
    node = DashboardNode(window)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(10)
    signal.signal(signal.SIGINT, lambda _signal, _frame: window.close())
    window.show()
    try:
        app.exec_()
    finally:
        timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
