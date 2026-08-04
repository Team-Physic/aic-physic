import importlib.util
from typing import Any, Optional

import cv2
import numpy as np
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation

try:
    _LEROBOT_AVAILABLE = importlib.util.find_spec("lerobot") is not None
except Exception:
    _LEROBOT_AVAILABLE = False


_IMG_H = 256
_IMG_W = 288

LEROBOT_FEATURES = {
    "observation.state": {
        "dtype": "float32",
        "shape": (35,),
        "names": [
            "tcp_pose.position.x",
            "tcp_pose.position.y",
            "tcp_pose.position.z",
            "tcp_pose.orientation.x",
            "tcp_pose.orientation.y",
            "tcp_pose.orientation.z",
            "tcp_pose.orientation.w",
            "tcp_velocity.linear.x",
            "tcp_velocity.linear.y",
            "tcp_velocity.linear.z",
            "tcp_velocity.angular.x",
            "tcp_velocity.angular.y",
            "tcp_velocity.angular.z",
            "tcp_error.x",
            "tcp_error.y",
            "tcp_error.z",
            "tcp_error.rx",
            "tcp_error.ry",
            "tcp_error.rz",
            "joint_positions.0",
            "joint_positions.1",
            "joint_positions.2",
            "joint_positions.3",
            "joint_positions.4",
            "joint_positions.5",
            "joint_positions.6",
            "force.x",
            "force.y",
            "force.z",
            "torque.x",
            "torque.y",
            "torque.z",
            "gripper_offset.x",
            "gripper_offset.y",
            "gripper_offset.z",
        ],
    },
    "action": {
        "dtype": "float32",
        "shape": (7,),
        "names": [
            "position.x",
            "position.y",
            "position.z",
            "orientation.x",
            "orientation.y",
            "orientation.z",
            "orientation.w",
        ],
    },
    "observation.plug_to_port": {
        "dtype": "float32",
        "shape": (7,),
        "names": [
            "translation.x",
            "translation.y",
            "translation.z",
            "rotation.x",
            "rotation.y",
            "rotation.z",
            "rotation.w",
        ],
    },
    "observation.images.left_camera": {
        "dtype": "video",
        "shape": (_IMG_H, _IMG_W, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.images.center_camera": {
        "dtype": "video",
        "shape": (_IMG_H, _IMG_W, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.images.right_camera": {
        "dtype": "video",
        "shape": (_IMG_H, _IMG_W, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.scenario_params": {
        "dtype": "float32",
        "shape": (11,),
        "names": [
            "trial_type",
            "rail_idx",
            "board_x",
            "board_y",
            "board_yaw",
            "gripper_offset_x",
            "gripper_offset_y",
            "gripper_offset_z",
            "nic_translation",
            "nic_yaw",
            "sc_translation",
        ],
    },
    "observation.stiffness": {
        "dtype": "float32",
        "shape": (6,),
        "names": ["x", "y", "z", "rx", "ry", "rz"],
    },
    "observation.damping": {
        "dtype": "float32",
        "shape": (6,),
        "names": ["x", "y", "z", "rx", "ry", "rz"],
    },
    "insertion_success": {
        "dtype": "int64",
        "shape": (1,),
        "names": None,
    },
    "phase": {
        "dtype": "string",
        "shape": (1,),
        "names": None,
    },
}


def compute_plug_to_port(port_tf: Transform, plug_tf: Transform) -> np.ndarray:
    """Express the plug pose in the port coordinate frame."""
    port_translation = np.array(
        [
            port_tf.translation.x,
            port_tf.translation.y,
            port_tf.translation.z,
        ]
    )
    port_quaternion = np.array(
        [
            port_tf.rotation.x,
            port_tf.rotation.y,
            port_tf.rotation.z,
            port_tf.rotation.w,
        ]
    )
    plug_translation = np.array(
        [
            plug_tf.translation.x,
            plug_tf.translation.y,
            plug_tf.translation.z,
        ]
    )
    plug_quaternion = np.array(
        [
            plug_tf.rotation.x,
            plug_tf.rotation.y,
            plug_tf.rotation.z,
            plug_tf.rotation.w,
        ]
    )
    port_rotation = Rotation.from_quat(port_quaternion)
    plug_rotation = Rotation.from_quat(plug_quaternion)
    relative_translation = port_rotation.inv().apply(
        plug_translation - port_translation
    )
    relative_quaternion = (port_rotation.inv() * plug_rotation).as_quat()
    return np.concatenate(
        [relative_translation, relative_quaternion]
    ).astype(np.float32)


def decode_image(image_msg, h: int = _IMG_H, w: int = _IMG_W) -> np.ndarray:
    """Convert a ROS Image message into a resized RGB numpy array."""
    if image_msg.width == 0 or image_msg.height == 0:
        return np.zeros((h, w, 3), dtype=np.uint8)
    image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
        image_msg.height,
        image_msg.width,
        3,
    )
    if image_msg.encoding != "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)


class LeRobotRecorder:
    """Record episode steps directly into a LeRobotDataset."""

    def __init__(self, dataset: "_LeRobotDataset", scenario_params_vec: np.ndarray):
        if not _LEROBOT_AVAILABLE:
            raise RuntimeError("lerobot package is not installed.")
        self.dataset = dataset
        self.scenario_params_vec = scenario_params_vec.astype(np.float32)
        self._gripper_offset_xyz = self.scenario_params_vec[5:8]
        self._episode_frame_count = 0

    def _build_state(self, observation: Observation) -> np.ndarray:
        controller_state = observation.controller_state
        wrench = observation.wrist_wrench.wrench
        state = [
            controller_state.tcp_pose.position.x,
            controller_state.tcp_pose.position.y,
            controller_state.tcp_pose.position.z,
            controller_state.tcp_pose.orientation.x,
            controller_state.tcp_pose.orientation.y,
            controller_state.tcp_pose.orientation.z,
            controller_state.tcp_pose.orientation.w,
            controller_state.tcp_velocity.linear.x,
            controller_state.tcp_velocity.linear.y,
            controller_state.tcp_velocity.linear.z,
            controller_state.tcp_velocity.angular.x,
            controller_state.tcp_velocity.angular.y,
            controller_state.tcp_velocity.angular.z,
            float(controller_state.tcp_error[0]),
            float(controller_state.tcp_error[1]),
            float(controller_state.tcp_error[2]),
            float(controller_state.tcp_error[3]),
            float(controller_state.tcp_error[4]),
            float(controller_state.tcp_error[5]),
        ]
        state.extend(float(position) for position in observation.joint_states.position)
        state.extend(
            [
                wrench.force.x,
                wrench.force.y,
                wrench.force.z,
                wrench.torque.x,
                wrench.torque.y,
                wrench.torque.z,
                float(self._gripper_offset_xyz[0]),
                float(self._gripper_offset_xyz[1]),
                float(self._gripper_offset_xyz[2]),
            ]
        )
        return np.array(state, dtype=np.float32)

    def record_step(
        self,
        phase: str,
        task: Task,
        obs: Observation,
        action: MotionUpdate,
        port_tf: Transform,
        plug_tf: Transform,
        gripper_tf: Transform,
        extras: dict[str, Any],
        stiffness: Optional[list[float]] = None,
        damping: Optional[list[float]] = None,
    ) -> None:
        del gripper_tf, extras
        task_name = (
            "sfp_insertion"
            if "sfp" in task.port_type.lower()
            else "sc_insertion"
        )
        selected_stiffness = np.array(
            stiffness if stiffness is not None else [0.0] * 6,
            dtype=np.float32,
        )
        selected_damping = np.array(
            damping if damping is not None else [0.0] * 6,
            dtype=np.float32,
        )

        self.dataset.add_frame(
            {
                "observation.state": self._build_state(obs),
                "action": np.array(
                    [
                        action.pose.position.x,
                        action.pose.position.y,
                        action.pose.position.z,
                        action.pose.orientation.x,
                        action.pose.orientation.y,
                        action.pose.orientation.z,
                        action.pose.orientation.w,
                    ],
                    dtype=np.float32,
                ),
                "observation.plug_to_port": compute_plug_to_port(
                    port_tf,
                    plug_tf,
                ),
                "observation.scenario_params": self.scenario_params_vec,
                "observation.stiffness": selected_stiffness,
                "observation.damping": selected_damping,
                "observation.images.left_camera": decode_image(obs.left_image),
                "observation.images.center_camera": decode_image(obs.center_image),
                "observation.images.right_camera": decode_image(obs.right_image),
                "insertion_success": np.array([0], dtype=np.int64),
                "phase": phase,
                "task": task_name,
            }
        )
        self._episode_frame_count += 1

    def record_terminal_step(
        self,
        phase: str,
        task: Task,
        obs: Observation,
        port_tf: Transform,
        plug_tf: Transform,
        gripper_tf: Transform,
        extras: dict[str, Any],
        stiffness: Optional[list[float]] = None,
        damping: Optional[list[float]] = None,
    ) -> None:
        action = MotionUpdate()
        action.header.frame_id = "base_link"
        action.trajectory_generation_mode = TrajectoryGenerationMode(
            mode=TrajectoryGenerationMode.MODE_UNSPECIFIED
        )
        self.record_step(
            phase=phase,
            task=task,
            obs=obs,
            action=action,
            port_tf=port_tf,
            plug_tf=plug_tf,
            gripper_tf=gripper_tf,
            extras=extras,
            stiffness=stiffness,
            damping=damping,
        )

    def save_episode(self, insertion_success: bool = False) -> None:
        if self._episode_frame_count > 0 and insertion_success:
            self.dataset.writer.episode_buffer["insertion_success"][-1] = np.array(
                [1],
                dtype=np.int64,
            )
        self._episode_frame_count = 0
        self.dataset.save_episode()
