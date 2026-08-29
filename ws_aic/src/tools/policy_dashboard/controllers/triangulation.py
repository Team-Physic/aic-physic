"""Triangulated port point telemetry."""

from __future__ import annotations

from geometry_msgs.msg import PointStamped
from models import POINT_TOPIC, DashboardState
from rclpy.node import Node


class TriangulationController:
    """Receive the latest triangulated port point."""

    def __init__(self, node: Node, state: DashboardState) -> None:
        self._state = state
        self._subscription = node.create_subscription(
            PointStamped, POINT_TOPIC, self._on_point, 10
        )
        node.get_logger().info(f"triangulated port: {POINT_TOPIC}")

    def _on_point(self, message: PointStamped) -> None:
        point = message.point
        self._state.update_point(
            message.header.frame_id,
            point.x,
            point.y,
            point.z,
            message.header.stamp,
        )
