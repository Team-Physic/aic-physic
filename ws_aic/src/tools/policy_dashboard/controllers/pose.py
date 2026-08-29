"""Task-aware ROS pose and TF adapter for dashboard spatial data."""

from __future__ import annotations

from aic_control_interfaces.msg import ControllerState
from aic_task_interfaces.msg import Task
from models import CONTROLLER_STATE_TOPIC, TASK_TOPIC, DashboardState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PoseController:
    """Resolve task, EE, cable, and port poses into dashboard state."""

    def __init__(
        self,
        node: Node,
        state: DashboardState,
        cable_frame: str = "",
    ) -> None:
        self._node = node
        self._state = state
        self._fixed_cable_frame = cable_frame.strip().strip("/")
        self._task_cable_frame = ""
        self._task_port_frame = ""
        self._selected_cable_frame = ""
        # Bind the buffer to the node's simulation clock so tf2 clears cached
        # transforms when a new trial resets /clock backwards.
        self._tf_buffer = Buffer(node=node)
        self._tf_listener = TransformListener(
            self._tf_buffer, node, spin_thread=False
        )
        task_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._task_subscription = node.create_subscription(
            Task, TASK_TOPIC, self._on_task, task_qos
        )
        self._controller_subscription = node.create_subscription(
            ControllerState,
            CONTROLLER_STATE_TOPIC,
            self._on_controller_state,
            10,
        )
        self._timer = node.create_timer(0.1, self._update_spatial_transforms)

        node.get_logger().info(f"active task: {TASK_TOPIC}")
        node.get_logger().info(f"EE orientation: {CONTROLLER_STATE_TOPIC}")
        if self._fixed_cable_frame:
            node.get_logger().info(
                f"cable orientation frame override: {self._fixed_cable_frame}"
            )
        else:
            node.get_logger().info(
                "cable orientation frame: waiting for active task"
            )

    def _on_task(self, message: Task) -> None:
        cable_name = message.cable_name.strip().strip("/")
        plug_name = message.plug_name.strip().strip("/")
        module_name = message.target_module_name.strip().strip("/")
        port_name = message.port_name.strip().strip("/")
        fields = (cable_name, plug_name, module_name, port_name)
        if any(not value or "/" in value for value in fields):
            self._node.get_logger().warn(
                "active task has invalid cable, plug, module, or port names"
            )
            return
        cable_frame = self._fixed_cable_frame or f"{cable_name}/{plug_name}_link"
        port_frame = f"task_board/{module_name}/{port_name}_link"
        self._task_cable_frame = cable_frame
        self._task_port_frame = port_frame
        self._selected_cable_frame = ""
        self._state.clear_orientation("cable")
        trial_index = self._state.update_task(
            message.id, cable_frame, port_frame
        )
        self._node.get_logger().info(
            f"observed trial {trial_index}, active task {message.id}: "
            f"cable={cable_frame}, port={port_frame}"
        )

    def _on_controller_state(self, message: ControllerState) -> None:
        position = message.tcp_pose.position
        self._state.update_position(
            "ee",
            "gripper/tcp",
            position.x,
            position.y,
            position.z,
            message.header.stamp,
        )
        orientation = message.tcp_pose.orientation
        try:
            self._state.update_orientation(
                "ee",
                "gripper/tcp",
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
                message.header.stamp,
            )
        except ValueError as exc:
            self._node.get_logger().warn(f"invalid EE orientation: {exc}")

    def _update_spatial_transforms(self) -> None:
        frame_id = self._fixed_cable_frame or self._task_cable_frame
        if frame_id:
            try:
                stamped = self._tf_buffer.lookup_transform(
                    "base_link", frame_id, Time()
                )
            except TransformException:
                stamped = None
            if stamped is not None:
                translation = stamped.transform.translation
                self._state.update_position(
                    "cable",
                    frame_id,
                    translation.x,
                    translation.y,
                    translation.z,
                    stamped.header.stamp,
                )
                orientation = stamped.transform.rotation
                try:
                    self._state.update_orientation(
                        "cable",
                        frame_id,
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                        stamped.header.stamp,
                    )
                except ValueError as exc:
                    self._node.get_logger().warn(
                        f"invalid cable orientation: {exc}"
                    )
                if frame_id != self._selected_cable_frame:
                    self._selected_cable_frame = frame_id
                    self._node.get_logger().info(
                        f"using cable frame: {frame_id}"
                    )

        if not self._task_port_frame:
            return
        try:
            port = self._tf_buffer.lookup_transform(
                "base_link", self._task_port_frame, Time()
            )
        except TransformException:
            return
        translation = port.transform.translation
        self._state.update_position(
            "port",
            self._task_port_frame,
            translation.x,
            translation.y,
            translation.z,
            port.header.stamp,
        )
