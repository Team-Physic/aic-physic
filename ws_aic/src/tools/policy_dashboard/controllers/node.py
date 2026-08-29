"""Composition node for independent ROS telemetry controllers."""

from __future__ import annotations

from models import DashboardState
from rclpy.node import Node
from rclpy.parameter import Parameter

from .haptic import HapticController
from .image import ImageController
from .pose import PoseController
from .triangulation import TriangulationController


class DashboardNode(Node):
    """Own the ROS node and compose feature-specific input adapters."""

    def __init__(
        self,
        state: DashboardState,
        jpeg_quality: int = 85,
        cable_frame: str = "",
    ) -> None:
        super().__init__("policy_dashboard")
        result = self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )[0]
        if not result.successful:
            self.get_logger().warn(
                f"failed to enable simulation time: {result.reason}"
            )
        self.image = ImageController(self, state, jpeg_quality)
        self.triangulation = TriangulationController(self, state)
        self.pose = PoseController(self, state, cable_frame)
        self.haptic = HapticController(self, state)
