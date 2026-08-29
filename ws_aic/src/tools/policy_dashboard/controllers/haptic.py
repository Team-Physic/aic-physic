"""Force-torque sensor controller."""

from __future__ import annotations

import time
from math import exp, pi

from geometry_msgs.msg import WrenchStamped
from models import WRENCH_TOPIC, DashboardState
from rclpy.node import Node


TORQUE_CHART_CUTOFF_HZ = 0.3
FILTER_RESET_AFTER_SECONDS = 0.5


class HapticController:
    """Keep raw F/T readings and prepare a display-only torque signal."""

    def __init__(self, node: Node, state: DashboardState) -> None:
        self._state = state
        self._filtered_torque: tuple[float, float, float] | None = None
        self._last_received_at: float | None = None
        self._subscription = node.create_subscription(
            WrenchStamped, WRENCH_TOPIC, self._on_wrench, 10
        )
        node.get_logger().info(
            f"force/torque: {WRENCH_TOPIC} "
            f"(raw values, torque chart LPF {TORQUE_CHART_CUTOFF_HZ:g} Hz)"
        )

    def _on_wrench(self, message: WrenchStamped) -> None:
        force = message.wrench.force
        torque = message.wrench.torque
        raw_torque = (torque.x, torque.y, torque.z)
        now = time.monotonic()
        elapsed = (
            None
            if self._last_received_at is None
            else now - self._last_received_at
        )
        if (
            self._filtered_torque is None
            or elapsed is None
            or elapsed > FILTER_RESET_AFTER_SECONDS
        ):
            self._filtered_torque = raw_torque
        else:
            alpha = 1.0 - exp(-2.0 * pi * TORQUE_CHART_CUTOFF_HZ * elapsed)
            self._filtered_torque = tuple(
                previous + alpha * (sample - previous)
                for previous, sample in zip(
                    self._filtered_torque, raw_torque, strict=True
                )
            )
        self._last_received_at = now
        self._state.update_wrench(
            message.header.frame_id,
            (force.x, force.y, force.z),
            raw_torque,
            message.header.stamp,
            chart_torque=self._filtered_torque,
        )
