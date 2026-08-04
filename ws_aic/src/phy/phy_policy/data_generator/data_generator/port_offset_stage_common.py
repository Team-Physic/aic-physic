from __future__ import annotations

"""PortOffsetCollect stage가 공유하는 pose 제어 처리."""

import os
from typing import Any

import numpy as np
from aic_model.policy import (
    MoveRobotCallback,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from tf2_ros import TransformException

from data_generator.port_offset_config import (
    APPROACH_DAMPING,
    APPROACH_DT,
    APPROACH_NEAR_DAMPING,
    APPROACH_NEAR_STIFFNESS,
    APPROACH_NEAR_Z_OFFSET_M,
    APPROACH_RETRY_DT,
    APPROACH_SETTLE_S,
    APPROACH_STEPS,
    APPROACH_STIFFNESS,
    APPROACH_TCP_OFFSET,
    APPROACH_VISION_RETRIES,
    DAMPING_DEFAULT,
    INITIAL_LIFT_DT,
    INITIAL_LIFT_M,
    INITIAL_LIFT_SETTLE_S,
    INITIAL_LIFT_STEPS,
    STIFFNESS_DEFAULT,
)
from data_generator.port_offset_geometry import interp_profile


def _copy_quaternion(quat: Quaternion) -> Quaternion:
    """ROS quaternion message를 값 복사해 새 객체로 반환한다."""
    return Quaternion(
        x=float(quat.x),
        y=float(quat.y),
        z=float(quat.z),
        w=float(quat.w),
    )


def _copy_pose(pose: Pose) -> Pose:
    """ROS pose message의 위치와 자세를 깊은 값 복사한다."""
    return Pose(
        position=Point(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
        ),
        orientation=_copy_quaternion(pose.orientation),
    )


def _tcp_pose(observation) -> Pose | None:
    """observation에서 현재 TCP pose를 안전하게 복사한다."""
    if observation is None:
        return None
    return _copy_pose(observation.controller_state.tcp_pose)


def _follow_pose(
    self,
    *,
    move_robot: MoveRobotCallback,
    start_pose: Pose,
    target_pose: Pose,
    steps: int,
    stiffness: list[float],
    damping: list[float],
    dt: float,
    label: str,
) -> None:
    """현재 TCP pose에서 목표 pose까지 위치를 S-curve로 보간해 순차 명령한다."""
    start = np.array(
        [start_pose.position.x, start_pose.position.y, start_pose.position.z],
        dtype=float,
    )
    target = np.array(
        [target_pose.position.x, target_pose.position.y, target_pose.position.z],
        dtype=float,
    )
    step_count = max(1, int(steps))
    for index in range(step_count):
        fraction = interp_profile((index + 1) / float(step_count), quintic=True)
        position = start * (1.0 - fraction) + target * fraction
        pose = Pose(
            position=Point(
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            ),
            orientation=_copy_quaternion(target_pose.orientation),
        )
        self.set_pose_target(
            move_robot,
            pose,
            stiffness=stiffness,
            damping=damping,
        )
        if index == 0 or index == step_count - 1:
            self.get_logger().info(
                f"{label}: waypoint {index + 1}/{step_count} "
                f"tcp=({position[0]:+.4f}, {position[1]:+.4f}, {position[2]:+.4f})"
            )
        self.sleep_for(dt)


def _configure_port_collect_control(self, task: Task) -> dict[str, Any]:
    """포트 타입별 접근/수집 제어 파라미터를 설정하고 stage context 일부를 반환한다."""
    port_kw = "sfp" if "sfp" in task.port_type.lower() else "sc"
    if port_kw == "sc":
        self._planner.i_gain = 0.07
        self._planner.max_integrator_windup = 0.06
        approach_stiffness = [280.0, 250.0, 250.0, 50.0, 50.0, 50.0]
        approach_damping = [87.0, 80.0, 80.0, 20.0, 20.0, 20.0]
    else:
        self._planner.i_gain = float(
            os.environ.get("AIC_CAPTURE_CHEATCODE_I_GAIN", "0.15")
        )
        self._planner.max_integrator_windup = 0.08
        approach_stiffness = STIFFNESS_DEFAULT
        approach_damping = DAMPING_DEFAULT

    return {
        "port_kw": port_kw,
        "approach_stiffness": APPROACH_STIFFNESS,
        "approach_damping": APPROACH_DAMPING,
        "lift_stiffness": APPROACH_NEAR_STIFFNESS,
        "lift_damping": APPROACH_NEAR_DAMPING,
        "collect_stiffness": approach_stiffness,
        "collect_damping": approach_damping,
    }
