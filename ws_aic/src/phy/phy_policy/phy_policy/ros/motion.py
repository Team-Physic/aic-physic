"""Motion primitives shared by PHY runtime policies."""

from __future__ import annotations

import os

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion


LIFT_STIFFNESS = [140.0, 140.0, 140.0, 40.0, 40.0, 40.0]
LIFT_DAMPING = [65.0, 65.0, 65.0, 16.0, 16.0, 16.0]
APPROACH_STIFFNESS = [180.0, 180.0, 180.0, 45.0, 45.0, 45.0]
APPROACH_DAMPING = [75.0, 75.0, 75.0, 18.0, 18.0, 18.0]


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


LIFT_M = _env_float("AIC_DISTANCE_INITIAL_LIFT_M", 0.050)
LIFT_STEPS = _env_int("AIC_DISTANCE_INITIAL_LIFT_STEPS", 40)
LIFT_DT = _env_float("AIC_DISTANCE_INITIAL_LIFT_DT", 0.05)
LIFT_SETTLE_S = _env_float("AIC_DISTANCE_INITIAL_LIFT_SETTLE_S", 0.50)
APPROACH_STEPS = _env_int("AIC_APPROACH_STEPS", 80)
APPROACH_DT = _env_float("AIC_APPROACH_DT", 0.05)
APPROACH_SETTLE_S = _env_float("AIC_APPROACH_SETTLE_S", 0.50)


def _copy_pose(pose: Pose) -> Pose:
    """Copy a ROS Pose by value."""
    return Pose(
        position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        orientation=Quaternion(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w,
        ),
    )


def _interpolate_quaternion(start: Pose, target: Pose, fraction: float) -> np.ndarray:
    """Interpolate two poses along the shortest quaternion path."""
    start_q = np.array(
        [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w],
        dtype=float,
    )
    target_q = np.array(
        [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w],
        dtype=float,
    )
    start_q /= max(float(np.linalg.norm(start_q)), 1e-12)
    target_q /= max(float(np.linalg.norm(target_q)), 1e-12)
    cosine = float(np.dot(start_q, target_q))
    if cosine < 0.0:
        target_q = -target_q
        cosine = -cosine
    if cosine > 0.9995:
        quaternion = start_q + fraction * (target_q - start_q)
        return quaternion / max(float(np.linalg.norm(quaternion)), 1e-12)
    angle = float(np.arccos(np.clip(cosine, -1.0, 1.0)))
    sine = float(np.sin(angle))
    return (
        np.sin((1.0 - fraction) * angle) / sine * start_q
        + np.sin(fraction * angle) / sine * target_q
    )


def _tcp_pose(observation) -> Pose | None:
    """Return a value copy of the current observation TCP pose."""
    return None if observation is None else _copy_pose(observation.controller_state.tcp_pose)


def _follow(
    policy,
    move_robot,
    start: Pose,
    target: Pose,
    steps: int,
    dt: float,
    label: str,
    stiffness,
    damping,
    *,
    step_guard=None,
) -> bool:
    """Send an S-curve pose path, optionally gating each waypoint."""
    count = max(1, steps)
    start_xyz = np.array([start.position.x, start.position.y, start.position.z])
    target_xyz = np.array([target.position.x, target.position.y, target.position.z])
    for index in range(count):
        t = (index + 1) / count
        fraction = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
        xyz = start_xyz * (1.0 - fraction) + target_xyz * fraction
        quaternion = _interpolate_quaternion(start, target, fraction)
        pose = _copy_pose(target)
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = map(float, quaternion)
        if step_guard is not None and not step_guard(index, pose):
            policy.get_logger().info(
                f"{label}: stopped by waypoint guard before {index + 1}/{count}"
            )
            return False
        policy.set_pose_target(
            move_robot, pose, stiffness=stiffness, damping=damping
        )
        if index in {0, count - 1}:
            policy.get_logger().info(
                f"{label}: waypoint {index + 1}/{count} "
                f"tcp=({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})"
            )
        policy.sleep_for(dt)
    return True
