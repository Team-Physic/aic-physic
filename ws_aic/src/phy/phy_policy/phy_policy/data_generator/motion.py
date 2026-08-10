"""Ground-truth TF 기반 로봇 이동과 pose sampling."""

from __future__ import annotations

import os
import time
from typing import Any

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from tf2_ros import TransformException

from phy_policy.data_generator import dataset
from phy_policy.data_generator.geometry import (
    axis_angle_quaternion,
    multiply_quaternions,
    quaternion_matrix,
)


STIFFNESS = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
DAMPING = [40.0, 40.0, 40.0, 20.0, 20.0, 20.0]
LIFT_STIFFNESS = [140.0, 140.0, 140.0, 40.0, 40.0, 40.0]
LIFT_DAMPING = [65.0, 65.0, 65.0, 16.0, 16.0, 16.0]
APPROACH_STIFFNESS = [180.0, 180.0, 180.0, 45.0, 45.0, 45.0]
APPROACH_DAMPING = [75.0, 75.0, 75.0, 18.0, 18.0, 18.0]


def _env_float(name: str, default: float) -> float:
    """환경변수를 float로 읽고 잘못되면 기본값을 반환한다."""
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    """환경변수를 int로 읽고 잘못되면 기본값을 반환한다."""
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


LIFT_M = _env_float("AIC_DISTANCE_INITIAL_LIFT_M", 0.050)
LIFT_STEPS = _env_int("AIC_DISTANCE_INITIAL_LIFT_STEPS", 40)
LIFT_DT = _env_float("AIC_DISTANCE_INITIAL_LIFT_DT", 0.05)
LIFT_SETTLE_S = _env_float("AIC_DISTANCE_INITIAL_LIFT_SETTLE_S", 0.50)
APPROACH_Z_M = _env_float("AIC_APPROACH_NEAR_Z_OFFSET_M", 0.020)
APPROACH_STEPS = _env_int("AIC_APPROACH_STEPS", 80)
APPROACH_DT = _env_float("AIC_APPROACH_DT", 0.05)
APPROACH_SETTLE_S = _env_float("AIC_APPROACH_SETTLE_S", 0.50)


class Planner:
    """plug 기준점을 port 접근축 위 목표점으로 이동시키는 GT planner."""

    def __init__(self, i_gain: float = 0.15, windup: float = 0.15):
        """누적 XY 보정 gain과 제한값을 설정한다."""
        self.i_gain = i_gain
        self.windup = windup
        self.reset()

    def reset(self) -> None:
        """누적 XY 보정값을 초기화한다."""
        self.integrator = np.zeros(2, dtype=float)

    def build_pose(
        self,
        port_tf: Transform,
        plug_tf: Transform,
        gripper_tf: Transform,
        *,
        z_offset: float,
        reset_integrator: bool,
    ) -> tuple[Pose, dict[str, Any]]:
        """현재 TF에서 plug가 목표 port pose에 도달할 TCP pose를 계산한다."""
        port_q = (
            port_tf.rotation.x,
            port_tf.rotation.y,
            port_tf.rotation.z,
            port_tf.rotation.w,
        )
        plug_q = (
            plug_tf.rotation.x,
            plug_tf.rotation.y,
            plug_tf.rotation.z,
            plug_tf.rotation.w,
        )
        gripper_q = (
            gripper_tf.rotation.x,
            gripper_tf.rotation.y,
            gripper_tf.rotation.z,
            gripper_tf.rotation.w,
        )
        plug_inverse = (-plug_q[0], -plug_q[1], -plug_q[2], plug_q[3])
        target_q = multiply_quaternions(
            multiply_quaternions(port_q, plug_inverse), gripper_q
        )
        port_xyz = np.array(
            [port_tf.translation.x, port_tf.translation.y, port_tf.translation.z],
            dtype=float,
        )
        plug_xyz = np.array(
            [plug_tf.translation.x, plug_tf.translation.y, plug_tf.translation.z],
            dtype=float,
        )
        gripper_xyz = np.array(
            [
                gripper_tf.translation.x,
                gripper_tf.translation.y,
                gripper_tf.translation.z,
            ],
            dtype=float,
        )
        port_axis = quaternion_matrix(*port_q) @ np.array([0.0, 0.0, -1.0])
        norm = float(np.linalg.norm(port_axis))
        port_axis = port_axis / norm if norm > 1e-9 else np.array([0.0, 0.0, -1.0])
        if reset_integrator:
            self.reset()
        else:
            self.integrator = np.clip(
                self.integrator + port_xyz[:2] - plug_xyz[:2],
                -self.windup,
                self.windup,
            )
        correction = np.array(
            [self.i_gain * self.integrator[0], self.i_gain * self.integrator[1], 0.0]
        )
        correction -= port_axis * float(np.dot(correction, port_axis))
        target_xyz = (
            port_xyz
            + port_axis * z_offset
            + correction
            + gripper_xyz
            - plug_xyz
        )
        pose = Pose(
            position=Point(x=float(target_xyz[0]), y=float(target_xyz[1]), z=float(target_xyz[2])),
            orientation=Quaternion(
                x=float(target_q[0]),
                y=float(target_q[1]),
                z=float(target_q[2]),
                w=float(target_q[3]),
            ),
        )
        return pose, {"target_xyz": target_xyz, "port_axis": port_axis}


def build_samples(policy) -> list[dict[str, float]]:
    """첫 영점과 stratified XYZ/RPY pose sample을 생성한다."""
    zero = {name: 0.0 for name in ("x", "y", "z", "roll", "pitch", "yaw")}
    if policy.collect_steps <= 1:
        return [zero]
    count = policy.collect_steps - 1

    def axis(low: float, high: float) -> np.ndarray:
        """한 축을 같은 구간으로 나눠 각 구간에서 sample을 뽑는다."""
        edges = np.linspace(low, high, count + 1)
        values = policy.rng.uniform(edges[:-1], edges[1:])
        policy.rng.shuffle(values)
        return values

    values = {
        name: axis(*policy.sample_ranges[name])
        for name in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    samples = [zero]
    for index in range(count):
        rpy = np.array([values[name][index] for name in ("roll", "pitch", "yaw")])
        norm = float(np.linalg.norm(rpy))
        if policy.rpy_norm_max > 0.0 and norm > policy.rpy_norm_max:
            rpy *= policy.rpy_norm_max / norm
        samples.append(
            {
                "x": float(values["x"][index]),
                "y": float(values["y"][index]),
                "z": float(values["z"][index]),
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            }
        )
    samples[1:] = sorted(
        samples[1:],
        key=lambda sample: np.linalg.norm([sample["x"], sample["y"], sample["z"]]),
    )
    return samples


def _port_axes(port_tf: Transform, port_axis) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """포트 local XYZ 축을 base_link 방향 벡터로 변환한다."""
    rotation = quaternion_matrix(
        port_tf.rotation.x,
        port_tf.rotation.y,
        port_tf.rotation.z,
        port_tf.rotation.w,
    )
    x_axis, y_axis = rotation[:, 0].copy(), rotation[:, 1].copy()
    z_axis = np.asarray(port_axis, dtype=float)
    z_norm = float(np.linalg.norm(z_axis))
    z_axis = z_axis / z_norm if z_norm > 1e-9 else rotation[:, 2].copy()
    for basis in (x_axis, y_axis):
        basis -= z_axis * float(np.dot(basis, z_axis))
        norm = float(np.linalg.norm(basis))
        if norm > 1e-9:
            basis /= norm
    return x_axis, y_axis, z_axis


def _apply_sample(policy, pose: Pose, port_tf: Transform, port_axis, index: int):
    """현재 stratified XYZ/RPY sample을 TCP 목표 pose에 적용한다."""
    sample = policy.samples[index % len(policy.samples)]
    x_axis, y_axis, z_axis = _port_axes(port_tf, port_axis)
    offset = sample["x"] * x_axis + sample["y"] * y_axis + sample["z"] * z_axis
    pose.position.x += float(offset[0])
    pose.position.y += float(offset[1])
    pose.position.z += float(offset[2])
    delta = multiply_quaternions(
        axis_angle_quaternion(z_axis, sample["yaw"]),
        multiply_quaternions(
            axis_angle_quaternion(y_axis, sample["pitch"]),
            axis_angle_quaternion(x_axis, sample["roll"]),
        ),
    )
    base = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    quaternion = np.asarray(multiply_quaternions(delta, base), dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm > 1e-9:
        quaternion /= norm
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
    detail = {
        "x_m": sample["x"],
        "y_m": sample["y"],
        "z_m": sample["z"],
        "roll_deg": float(np.rad2deg(sample["roll"])),
        "pitch_deg": float(np.rad2deg(sample["pitch"])),
        "yaw_deg": float(np.rad2deg(sample["yaw"])),
    }
    return pose, detail


def _copy_pose(pose: Pose) -> Pose:
    """ROS Pose를 값 복사한다."""
    return Pose(
        position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        orientation=Quaternion(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w,
        ),
    )


def _tcp_pose(observation) -> Pose | None:
    """Observation의 현재 TCP pose를 값 복사해 반환한다."""
    return None if observation is None else _copy_pose(observation.controller_state.tcp_pose)


def _follow(policy, move_robot, start: Pose, target: Pose, steps: int, dt: float, label: str, stiffness, damping):
    """현재 TCP에서 목표 pose까지 S-curve 위치 명령을 보낸다."""
    start_xyz = np.array([start.position.x, start.position.y, start.position.z])
    target_xyz = np.array([target.position.x, target.position.y, target.position.z])
    for index in range(max(1, steps)):
        t = (index + 1) / max(1, steps)
        fraction = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
        xyz = start_xyz * (1.0 - fraction) + target_xyz * fraction
        pose = _copy_pose(target)
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        policy.set_pose_target(move_robot, pose, stiffness, damping)
        if index in {0, max(1, steps) - 1}:
            policy.get_logger().info(
                f"{label}: waypoint {index + 1}/{max(1, steps)} "
                f"tcp=({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})"
            )
        policy.sleep_for(dt)


def control_for(task, planner: Planner) -> dict[str, list[float]]:
    """connector 타입에 맞는 접근·수집 impedance 설정을 반환한다."""
    if "sfp" not in task.port_type.lower():
        planner.i_gain, planner.windup = 0.07, 0.06
        collect_stiffness = [280.0, 250.0, 250.0, 50.0, 50.0, 50.0]
        collect_damping = [87.0, 80.0, 80.0, 20.0, 20.0, 20.0]
    else:
        planner.i_gain = float(os.environ.get("AIC_CAPTURE_CHEATCODE_I_GAIN", "0.15"))
        planner.windup = 0.08
        collect_stiffness, collect_damping = STIFFNESS, DAMPING
    return {
        "lift_stiffness": LIFT_STIFFNESS,
        "lift_damping": LIFT_DAMPING,
        "approach_stiffness": APPROACH_STIFFNESS,
        "approach_damping": APPROACH_DAMPING,
        "collect_stiffness": collect_stiffness,
        "collect_damping": collect_damping,
    }


def lift(policy, context, get_observation, move_robot) -> bool:
    """초기 TCP를 위로 들어 task board 관측을 확보한다."""
    if abs(LIFT_M) < 1e-9:
        return True
    start = _tcp_pose(get_observation())
    if start is None:
        policy.get_logger().error("[PortOffsetCollect] lift failed: missing TCP pose")
        return False
    target = _copy_pose(start)
    target.position.z += LIFT_M
    _follow(
        policy, move_robot, start, target, LIFT_STEPS, LIFT_DT, "lift_up",
        context["lift_stiffness"], context["lift_damping"],
    )
    policy.sleep_for(LIFT_SETTLE_S)
    context["counts"]["lift_up"] += 1
    return True


def approach(policy, context, get_observation, move_robot) -> bool:
    """GT port/plug TF로 port 근처 pose에 접근한다."""
    start = _tcp_pose(get_observation())
    if start is None:
        policy.get_logger().error("[PortOffsetCollect] approach failed: missing TCP pose")
        return False
    try:
        plug_tf = dataset.shift_origin(
            policy.lookup_transform("base_link", context["cable_tip_frame"]),
            context["plug_offset"],
        )
        target, state = policy.planner.build_pose(
            context["port_snapshot"].transform,
            plug_tf,
            policy.lookup_transform("base_link", "gripper/tcp"),
            z_offset=APPROACH_Z_M,
            reset_integrator=True,
        )
    except TransformException as exc:
        policy.get_logger().error(f"[PortOffsetCollect] approach TF failed: {exc}")
        return False
    xyz = state["target_xyz"]
    policy.get_logger().info(
        f"[PortOffsetCollect] approach target=({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})"
    )
    _follow(
        policy, move_robot, start, target, APPROACH_STEPS, APPROACH_DT, "approach",
        context["approach_stiffness"], context["approach_damping"],
    )
    policy.sleep_for(APPROACH_SETTLE_S)
    context["counts"]["approach"] += 1
    return True


def _skew_text(timestamps: dict[str, Any]) -> str:
    """source별 nanosecond 시각 차이를 millisecond 문자열로 바꾼다."""
    return ", ".join(
        f"{name}={int(value) / 1e6:.3f} ms"
        for name, value in sorted(timestamps.get("skew_ns", {}).items())
    ) or "unavailable"


def failure_reason(reason: str) -> str:
    """내부 timestamp 실패 분류를 사람이 읽을 문장으로 바꾼다."""
    timed_out = reason.startswith("observation_sync_timeout:")
    key = reason.split(":", 1)[1] if timed_out else reason
    descriptions = {
        "missing_observation": "camera/controller Observation을 받지 못함",
        "missing_or_zero_timestamp": "camera 또는 controller source 시각이 없거나 0임",
        "camera_time_difference_exceeded": "세 camera 촬영 시각 차이가 허용 범위를 초과함",
        "controller_time_difference_exceeded": "controller와 center camera 시각 차이가 허용 범위를 초과함",
        "capture_not_after_command": "center camera frame이 현재 명령보다 새롭지 않음",
        "tf_time_difference_exceeded": "plug TF와 center camera 시각 차이가 허용 범위를 초과함",
    }
    detail = descriptions.get(key, key.replace("_", " "))
    return f"대기시간 내 일치하는 Observation을 찾지 못함: {detail}" if timed_out else detail


def collect(policy, context, get_observation, move_robot) -> bool:
    """stratified pose마다 동기화 image와 촬영 시점 XYZ label을 저장한다."""
    for index in range(policy.collect_steps):
        try:
            port_tf = context["port_snapshot"].transform
            plug_tf = dataset.shift_origin(
                policy.lookup_transform("base_link", context["cable_tip_frame"]),
                context["plug_offset"],
            )
            pose, state = policy.planner.build_pose(
                port_tf,
                plug_tf,
                policy.lookup_transform("base_link", "gripper/tcp"),
                z_offset=policy.base_z_offset,
                reset_integrator=False,
            )
            pose, sample = _apply_sample(policy, pose, port_tf, state["port_axis"], index)
            policy.get_logger().info(
                f"COLLECT {index}/{policy.collect_steps}: "
                f"xyz=({sample['x_m']*1e3:+.1f}, {sample['y_m']*1e3:+.1f}, {sample['z_m']*1e3:+.1f})mm "
                f"rpy=({sample['roll_deg']:+.1f}, {sample['pitch_deg']:+.1f}, {sample['yaw_deg']:+.1f})deg"
            )
            command_stamp = policy.set_pose_target(
                move_robot, pose, context["collect_stiffness"], context["collect_damping"]
            )
            observation, timestamps = dataset.wait_for_observation(
                policy, get_observation, command_stamp
            )
            if observation is None:
                policy.get_logger().error(
                    policy.log_text(
                        f"[PortOffsetCollect] CAPTURE FAILED: {failure_reason(timestamps['rejection_reason'])}; "
                        f"skew={_skew_text(timestamps)}",
                        "red",
                    )
                )
                policy.sleep_for(policy.step_sleep_s)
                continue
            capture_stamp = observation.center_image.header.stamp
            wait_start = time.monotonic_ns()
            plug_stamped = policy.lookup_transform_at(
                "base_link", context["cable_tip_frame"], capture_stamp
            )
            timestamps.setdefault("wait_ns", {})["tf"] = time.monotonic_ns() - wait_start
            valid, timestamps = dataset.tf_sync(
                policy,
                timestamps,
                {"port": context["port_snapshot"], "plug": plug_stamped},
                static_sources={"port"},
            )
            if not valid:
                policy.get_logger().error(
                    policy.log_text(
                        f"[PortOffsetCollect] CAPTURE FAILED: {failure_reason(timestamps['rejection_reason'])}; "
                        f"skew={_skew_text(timestamps)}",
                        "red",
                    )
                )
                policy.sleep_for(policy.step_sleep_s)
                continue
            plug_at_capture = dataset.shift_origin(plug_stamped.transform, context["plug_offset"])
            saved, detail = dataset.save_sample(
                policy,
                episode_name=context["episode_name"],
                task=context["task"],
                step_idx=context["counts"]["collect"],
                observation=observation,
                port_tf=port_tf,
                timestamps=timestamps,
                label_xyz=dataset.target_xyz(port_tf, plug_at_capture),
            )
            if saved:
                context["counts"]["collect"] += 1
                policy.get_logger().info(
                    policy.log_text(
                        f"[PortOffsetCollect] CAPTURE SAVED: {detail}; "
                        f"capture_stamp_ns={timestamps['capture_stamp_ns']}; skew={_skew_text(timestamps)}",
                        "green",
                    )
                )
            else:
                policy.get_logger().error(policy.log_text(f"[PortOffsetCollect] CAPTURE FAILED: {detail}", "red"))
        except TransformException as exc:
            policy.get_logger().error(
                policy.log_text(f"[PortOffsetCollect] CAPTURE FAILED: TF lookup: {exc}", "red")
            )
        policy.sleep_for(policy.step_sleep_s)
    return True
