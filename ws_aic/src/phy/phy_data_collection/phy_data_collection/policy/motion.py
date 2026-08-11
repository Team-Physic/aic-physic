"""Ground-truth TF 기반 로봇 이동과 pose sampling."""

from __future__ import annotations

import os
import time
from typing import Any

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from tf2_ros import TransformException

from . import dataset
from .geometry import (
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
MIN_CLEARANCE_M = 0.020
APPROACH_STEPS = _env_int("AIC_APPROACH_STEPS", 80)
APPROACH_DT = _env_float("AIC_APPROACH_DT", 0.05)
APPROACH_SETTLE_S = _env_float("AIC_APPROACH_SETTLE_S", 0.50)


class Planner:
    """plug 기준점을 port 접근축 위 목표점으로 이동시키는 GT planner."""

    def build_pose(
        self,
        port_tf: Transform,
        plug_tf: Transform,
        gripper_tf: Transform,
        *,
        z_offset: float,
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
        port_rotation = quaternion_matrix(*port_q)
        plug_rotation = quaternion_matrix(*plug_q)
        port_axis = port_rotation @ np.array([0.0, 0.0, -1.0])
        norm = float(np.linalg.norm(port_axis))
        port_axis = port_axis / norm if norm > 1e-9 else np.array([0.0, 0.0, -1.0])
        target_plug_xyz = port_xyz + port_axis * z_offset
        target_tcp_from_plug = (
            port_rotation @ plug_rotation.T @ (gripper_xyz - plug_xyz)
        )
        target_xyz = target_plug_xyz + target_tcp_from_plug
        pose = Pose(
            position=Point(x=float(target_xyz[0]), y=float(target_xyz[1]), z=float(target_xyz[2])),
            orientation=Quaternion(
                x=float(target_q[0]),
                y=float(target_q[1]),
                z=float(target_q[2]),
                w=float(target_q[3]),
            ),
        )
        return pose, {
            "target_xyz": target_xyz,
            "port_axis": port_axis,
            "target_tcp_from_plug": target_tcp_from_plug,
        }


def _axis_samples(policy, low: float, high: float, count: int) -> np.ndarray:
    """축 범위를 같은 구간으로 나눠 무작위 값을 하나씩 뽑는다."""
    edges = np.linspace(low, high, count + 1)
    values = policy.rng.uniform(edges[:-1], edges[1:])
    policy.rng.shuffle(values)
    return values


def _build_view_samples(policy, *, descending: bool) -> list[dict[str, float]]:
    """board-view 또는 descent 정책의 거리·횡방향·각도 sample을 만든다."""
    count = policy.collect_steps
    if descending:
        distance_min, distance_max = policy.base_z_offset, policy.descent_start_distance
        lateral_limit = policy.descent_lateral_limit
        angle_limit = policy.descent_angle_limit
    else:
        distance_min, distance_max = policy.board_distance_range
        lateral_limit = policy.board_lateral_limit
        angle_limit = policy.board_angle_limit
    distances = _axis_samples(policy, distance_min, distance_max, count)
    if descending:
        distances = np.sort(distances)[::-1]
        distances[0] = distance_max
        if count > 1:
            distances[-1] = distance_min
    values = {
        "x": _axis_samples(policy, -lateral_limit, lateral_limit, count),
        "y": _axis_samples(policy, -lateral_limit, lateral_limit, count),
        "roll": _axis_samples(policy, -angle_limit, angle_limit, count),
        "pitch": _axis_samples(policy, -angle_limit, angle_limit, count),
        "yaw": _axis_samples(policy, -angle_limit, angle_limit, count),
    }
    return [
        {
            "x": float(values["x"][index]),
            "y": float(values["y"][index]),
            "z": 0.0,
            "roll": float(values["roll"][index]),
            "pitch": float(values["pitch"][index]),
            "yaw": float(values["yaw"][index]),
            "tier_m": None,
            "distance_m": float(distances[index]),
        }
        for index in range(count)
    ]


def build_samples(policy) -> list[dict[str, float | None]]:
    """선택된 수집 정책에 맞는 stratified 거리·XYZ·RPY sample을 생성한다."""
    if policy.collection_policy == "board-view":
        return _build_view_samples(policy, descending=False)
    if policy.collection_policy == "descent":
        return _build_view_samples(policy, descending=True)

    tiers = np.asarray(policy.sampling_tiers_m, dtype=float)
    weights = np.asarray(policy.sampling_tier_weights, dtype=float)
    raw_counts = weights / weights.sum() * policy.collect_steps
    counts = np.floor(raw_counts).astype(int)
    for index in np.argsort(-(raw_counts - counts))[: policy.collect_steps - int(counts.sum())]:
        counts[index] += 1

    allocated = np.flatnonzero(counts > 0)
    largest_index = int(allocated[np.argmax(tiers[allocated])])
    zero = {
        **{name: 0.0 for name in ("x", "y", "z", "roll", "pitch", "yaw")},
        "tier_m": float(tiers[largest_index]),
    }
    counts[largest_index] -= 1
    samples: list[dict[str, float]] = [zero]
    largest_tier = float(np.max(tiers))

    for tier_index in np.argsort(-tiers):
        tier = tiers[tier_index]
        count = counts[tier_index]
        if count <= 0:
            continue
        scale = float(tier) / largest_tier
        values = {
            name: _axis_samples(
                policy,
                policy.sample_ranges[name][0] * scale,
                policy.sample_ranges[name][1] * scale,
                int(count),
            )
            for name in ("x", "y", "z", "roll", "pitch", "yaw")
        }
        for index in range(int(count)):
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
                    "tier_m": float(tier),
                    "distance_m": policy.base_z_offset,
                }
            )
    samples[0]["distance_m"] = policy.base_z_offset
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


def _apply_sample(policy, pose: Pose, port_tf: Transform, state: dict, index: int):
    """현재 stratified XYZ/RPY sample을 TCP 목표 pose에 적용한다."""
    sample = policy.samples[index % len(policy.samples)]
    port_axis = state["port_axis"]
    x_axis, y_axis, z_axis = _port_axes(port_tf, port_axis)
    sample_offset = sample["x"] * x_axis + sample["y"] * y_axis + sample["z"] * z_axis
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
    tcp_from_plug = np.asarray(state["target_tcp_from_plug"], dtype=float)
    position_correction = quaternion_matrix(*delta) @ tcp_from_plug - tcp_from_plug
    position_offset = sample_offset + position_correction
    pose.position.x += float(position_offset[0])
    pose.position.y += float(position_offset[1])
    pose.position.z += float(position_offset[2])
    detail = {
        "x_m": sample["x"],
        "y_m": sample["y"],
        "z_m": sample["z"],
        "roll_rad": float(sample["roll"]),
        "pitch_rad": float(sample["pitch"]),
        "yaw_rad": float(sample["yaw"]),
        "tier_m": sample["tier_m"],
        "distance_m": sample["distance_m"],
    }
    return pose, detail


def _actual_sampling_offset(
    label_xyz: list[float],
    port_tf: Transform,
    port_axis,
    minimum_clearance_m: float,
) -> list[float]:
    """실제 TF label을 안전거리 기준 port-local sampling offset으로 변환한다."""
    x_axis, y_axis, z_axis = _port_axes(port_tf, port_axis)
    label = np.asarray(label_xyz, dtype=float)
    return [
        -float(np.dot(label, x_axis)),
        -float(np.dot(label, y_axis)),
        -float(np.dot(label, z_axis)) - minimum_clearance_m,
    ]


def _board_view_pose(policy, context, index: int):
    """보드 중심과 center camera extrinsic으로 무작위 관측 TCP pose를 역산한다."""
    sample = policy.samples[index % len(policy.samples)]
    pose = Pose(
        position=Point(),
        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    )
    x_axis = np.array([1.0, 0.0, 0.0])
    y_axis = np.array([0.0, 1.0, 0.0])
    z_axis = np.array([0.0, 0.0, 1.0])
    delta = multiply_quaternions(
        axis_angle_quaternion(z_axis, sample["yaw"]),
        multiply_quaternions(
            axis_angle_quaternion(y_axis, sample["pitch"]),
            axis_angle_quaternion(x_axis, sample["roll"]),
        ),
    )
    quaternion = np.asarray(
        multiply_quaternions(delta, (1.0, 0.0, 0.0, 0.0)), dtype=float
    )
    quaternion /= max(float(np.linalg.norm(quaternion)), 1e-12)
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = map(float, quaternion)
    tcp_rotation = quaternion_matrix(*quaternion)
    tcp_to_optical = np.linalg.inv(policy._tool0_tcp) @ policy._tool0_optical["center"]
    camera_rotation = tcp_rotation @ tcp_to_optical[:3, :3]
    camera_offset = tcp_rotation @ tcp_to_optical[:3, 3]
    board_tf = context["board_snapshot"].transform
    board_center = np.array(
        [board_tf.translation.x, board_tf.translation.y, board_tf.translation.z],
        dtype=float,
    )
    camera_origin = (
        board_center
        - camera_rotation[:, 2] * sample["distance_m"]
        + camera_rotation[:, 0] * sample["x"]
        + camera_rotation[:, 1] * sample["y"]
    )
    tcp_xyz = camera_origin - camera_offset
    pose.position.x, pose.position.y, pose.position.z = map(float, tcp_xyz)
    port_tf = context["port_snapshot"].transform
    port_rotation = quaternion_matrix(
        port_tf.rotation.x,
        port_tf.rotation.y,
        port_tf.rotation.z,
        port_tf.rotation.w,
    )
    port_axis = port_rotation @ np.array([0.0, 0.0, -1.0])
    return pose, {"port_axis": port_axis}, {
        "x_m": sample["x"],
        "y_m": sample["y"],
        "z_m": 0.0,
        "roll_rad": float(sample["roll"]),
        "pitch_rad": float(sample["pitch"]),
        "yaw_rad": float(sample["yaw"]),
        "tier_m": None,
        "distance_m": sample["distance_m"],
    }


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


def _interpolate_quaternion(start: Pose, target: Pose, fraction: float) -> np.ndarray:
    """두 pose의 quaternion을 최단 회전 경로로 보간한다."""
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
    """Observation의 현재 TCP pose를 값 복사해 반환한다."""
    return None if observation is None else _copy_pose(observation.controller_state.tcp_pose)


def _wrist_force_in_base(observation) -> tuple[np.ndarray, int] | None:
    """tare를 뺀 wrist force를 base_link 좌표계로 변환한다."""
    if observation is None:
        return None
    wrist = getattr(observation, "wrist_wrench", None)
    controller = getattr(observation, "controller_state", None)
    if wrist is None or controller is None:
        return None
    raw = np.array(
        [wrist.wrench.force.x, wrist.wrench.force.y, wrist.wrench.force.z],
        dtype=float,
    )
    tare_force = controller.fts_tare_offset.wrench.force
    tare = np.array([tare_force.x, tare_force.y, tare_force.z], dtype=float)
    pose = controller.tcp_pose
    rotation = quaternion_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    force = rotation @ (raw - tare)
    stamp = dataset._stamp_ns(getattr(wrist.header, "stamp", None))
    if stamp is None or stamp <= 0 or not np.all(np.isfinite(force)):
        return None
    return force, stamp


class HapticGuard:
    """정지 baseline 대비 추가 force가 지속되는지 추적한다."""

    def __init__(self, baseline_force: np.ndarray, threshold_n: float, duration_s: float):
        self.baseline_force = np.asarray(baseline_force, dtype=float)
        self.threshold_n = float(threshold_n)
        self.duration_ns = int(float(duration_s) * 1e9)
        self.last_stamp_ns = 0
        self.above_since_ns: int | None = None
        self.peak_delta_force_n = 0.0
        self.last_delta_force_n = 0.0

    @property
    def baseline_force_n(self) -> float:
        """정지 baseline force 벡터의 크기를 반환한다."""
        return float(np.linalg.norm(self.baseline_force))

    def observe(self, observation) -> bool:
        """새 force frame을 반영하고 지속 임계치 초과 여부를 반환한다."""
        reading = _wrist_force_in_base(observation)
        if reading is None:
            return False
        force, stamp_ns = reading
        if stamp_ns <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp_ns
        self.last_delta_force_n = float(np.linalg.norm(force - self.baseline_force))
        self.peak_delta_force_n = max(
            self.peak_delta_force_n, self.last_delta_force_n
        )
        if self.last_delta_force_n <= self.threshold_n:
            self.above_since_ns = None
            return False
        if self.above_since_ns is None or stamp_ns < self.above_since_ns:
            self.above_since_ns = stamp_ns
        return stamp_ns - self.above_since_ns >= self.duration_ns

    def metrics(self) -> dict[str, float]:
        """저장·로그에 사용할 haptic 측정값을 반환한다."""
        return {
            "baseline_force_n": self.baseline_force_n,
            "peak_delta_force_n": self.peak_delta_force_n,
            "last_delta_force_n": self.last_delta_force_n,
        }


def prepare_haptic_guard(policy, get_observation) -> HapticGuard | None:
    """이동 직전 정지 force frame들의 중앙값으로 baseline을 만든다."""
    samples: list[np.ndarray] = []
    last_stamp_ns = 0
    deadline = time.monotonic() + policy.haptic_baseline_timeout_s
    while len(samples) < policy.haptic_baseline_samples and time.monotonic() <= deadline:
        reading = _wrist_force_in_base(get_observation())
        if reading is not None:
            force, stamp_ns = reading
            if stamp_ns != last_stamp_ns:
                samples.append(force)
                last_stamp_ns = stamp_ns
        if len(samples) < policy.haptic_baseline_samples:
            policy.sleep_for(policy.settle_poll_s)
    if len(samples) < policy.haptic_baseline_samples:
        policy.get_logger().error(
            "[PortOffsetCollect] haptic baseline unavailable: "
            f"frames={len(samples)}/{policy.haptic_baseline_samples}"
        )
        return None
    guard = HapticGuard(
        np.median(np.asarray(samples), axis=0),
        policy.haptic_force_threshold_n,
        policy.haptic_contact_duration_s,
    )
    vector = guard.baseline_force
    policy.get_logger().info(
        "[PortOffsetCollect] haptic baseline: "
        f"force=({vector[0]:+.2f}, {vector[1]:+.2f}, {vector[2]:+.2f})N, "
        f"magnitude={guard.baseline_force_n:.2f}N"
    )
    return guard


def _pose_error(current: Pose, target: Pose) -> tuple[float, float]:
    """현재 TCP와 목표 pose의 위치·quaternion 각도 오차를 반환한다."""
    current_xyz = np.array([current.position.x, current.position.y, current.position.z])
    target_xyz = np.array([target.position.x, target.position.y, target.position.z])
    current_q = np.array(
        [current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w]
    )
    target_q = np.array(
        [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w]
    )
    current_norm = float(np.linalg.norm(current_q))
    target_norm = float(np.linalg.norm(target_q))
    if current_norm <= 1e-9 or target_norm <= 1e-9:
        return float(np.linalg.norm(current_xyz - target_xyz)), float("inf")
    cosine = float(np.clip(abs(np.dot(current_q / current_norm, target_q / target_norm)), 0.0, 1.0))
    return float(np.linalg.norm(current_xyz - target_xyz)), 2.0 * float(np.arccos(cosine))


def _controller_tracking_error(controller) -> tuple[float, float]:
    """controller가 계산한 current-to-reference TCP 오차를 반환한다."""
    error = np.asarray(getattr(controller, "tcp_error", ()), dtype=float)
    if error.shape != (6,) or not np.all(np.isfinite(error)):
        return float("inf"), float("inf")
    return float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))


def wait_for_pose_convergence(
    policy,
    get_observation,
    target: Pose,
    command_stamp_ns: int,
    haptic_guard: HapticGuard | None = None,
):
    """controller reference와 실제 TCP 움직임이 함께 멈출 때까지 기다린다."""
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(policy.settle_timeout_s * 1e9)
    stable = 0
    last_stamp = 0
    last_reference = None
    last_current = None
    position_delta = float("inf")
    orientation_delta = float("inf")
    tracking_position_error = float("inf")
    tracking_orientation_error = float("inf")
    command_position_delta = float("inf")
    command_orientation_delta = float("inf")
    while time.monotonic_ns() <= deadline_ns:
        observation = get_observation()
        if haptic_guard is not None and haptic_guard.observe(observation):
            return None, {
                "failure_reason": "haptic_contact",
                "position_error_m": position_delta,
                "orientation_error_rad": orientation_delta,
                "tracking_position_error_m": tracking_position_error,
                "tracking_orientation_error_rad": tracking_orientation_error,
                "command_position_delta_m": command_position_delta,
                "command_orientation_delta_rad": command_orientation_delta,
                "wait_ns": time.monotonic_ns() - start_ns,
                "stable_observations": stable,
                **haptic_guard.metrics(),
            }
        controller = getattr(observation, "controller_state", None)
        header = getattr(controller, "header", None)
        stamp = dataset._stamp_ns(getattr(header, "stamp", None))
        if stamp and stamp > command_stamp_ns and stamp != last_stamp:
            last_stamp = stamp
            current = _copy_pose(controller.tcp_pose)
            reference = _copy_pose(controller.reference_tcp_pose)
            tracking_position_error, tracking_orientation_error = (
                _controller_tracking_error(controller)
            )
            command_position_delta, command_orientation_delta = _pose_error(reference, target)
            if last_reference is not None and last_current is not None:
                reference_position_delta, reference_orientation_delta = _pose_error(
                    reference, last_reference
                )
                position_delta, orientation_delta = _pose_error(current, last_current)
                reference_stable = (
                    reference_position_delta <= 1e-5
                    and reference_orientation_delta <= 1e-4
                )
                within_tolerance = (
                    position_delta <= policy.settle_position_tolerance_m
                    and orientation_delta <= policy.settle_orientation_tolerance_rad
                    and reference_stable
                )
                stable = stable + 1 if within_tolerance else 0
                if stable >= policy.settle_stable_observations:
                    return stamp, {
                        "position_error_m": position_delta,
                        "orientation_error_rad": orientation_delta,
                        "tracking_position_error_m": tracking_position_error,
                        "tracking_orientation_error_rad": tracking_orientation_error,
                        "command_position_delta_m": command_position_delta,
                        "command_orientation_delta_rad": command_orientation_delta,
                        "wait_ns": time.monotonic_ns() - start_ns,
                        "stable_observations": stable,
                    }
            last_reference = reference
            last_current = current
        policy.sleep_for(policy.settle_poll_s)
    return None, {
        "position_error_m": position_delta,
        "orientation_error_rad": orientation_delta,
        "tracking_position_error_m": tracking_position_error,
        "tracking_orientation_error_rad": tracking_orientation_error,
        "command_position_delta_m": command_position_delta,
        "command_orientation_delta_rad": command_orientation_delta,
        "wait_ns": time.monotonic_ns() - start_ns,
        "stable_observations": stable,
    }


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
    get_observation=None,
    haptic_guard: HapticGuard | None = None,
) -> bool:
    """현재 TCP에서 목표 pose까지 S-curve 위치·자세 명령을 보낸다."""
    start_xyz = np.array([start.position.x, start.position.y, start.position.z])
    target_xyz = np.array([target.position.x, target.position.y, target.position.z])
    for index in range(max(1, steps)):
        t = (index + 1) / max(1, steps)
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
        policy.set_pose_target(move_robot, pose, stiffness, damping)
        if index in {0, max(1, steps) - 1}:
            policy.get_logger().info(
                f"{label}: waypoint {index + 1}/{max(1, steps)} "
                f"tcp=({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})"
            )
        policy.sleep_for(dt)
        observation = get_observation() if haptic_guard is not None else None
        if haptic_guard is not None and haptic_guard.observe(observation):
            current = _tcp_pose(observation)
            if current is not None:
                policy.set_pose_target(move_robot, current, stiffness, damping)
            policy.get_logger().error(
                "[PortOffsetCollect] HAPTIC CONTACT: "
                f"stage={label}, delta={haptic_guard.last_delta_force_n:.2f}N, "
                f"peak={haptic_guard.peak_delta_force_n:.2f}N, "
                f"threshold={haptic_guard.threshold_n:.2f}N"
            )
            return False
    return True


def retreat_to_pose(policy, get_observation, move_robot, target: Pose, stiffness, damping) -> bool:
    """접촉 시 현재 자세에서 직전 출발 자세로 역경로 후퇴한다."""
    current = _tcp_pose(get_observation())
    if current is None:
        policy.get_logger().error(
            "[PortOffsetCollect] haptic retreat failed: missing TCP pose"
        )
        return False
    distance = float(
        np.linalg.norm(
            np.array(
                [
                    target.position.x - current.position.x,
                    target.position.y - current.position.y,
                    target.position.z - current.position.z,
                ]
            )
        )
    )
    steps = min(120, max(10, int(np.ceil(distance / 0.003))))
    policy.get_logger().info(
        f"[PortOffsetCollect] haptic retreat: distance={distance * 1e3:.1f}mm"
    )
    followed = _follow(
        policy,
        move_robot,
        current,
        target,
        steps,
        0.03,
        "haptic_retreat",
        stiffness,
        damping,
    )
    policy.sleep_for(0.3)
    return followed


def control_for(task) -> dict[str, list[float]]:
    """connector 타입에 맞는 접근·수집 impedance 설정을 반환한다."""
    if "sfp" not in task.port_type.lower():
        collect_stiffness = [280.0, 250.0, 250.0, 50.0, 50.0, 50.0]
        collect_damping = [87.0, 80.0, 80.0, 20.0, 20.0, 20.0]
    else:
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
        )
    except TransformException as exc:
        policy.get_logger().error(f"[PortOffsetCollect] approach TF failed: {exc}")
        return False
    xyz = state["target_xyz"]
    policy.get_logger().info(
        f"[PortOffsetCollect] approach target=({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})"
    )
    guard = (
        prepare_haptic_guard(policy, get_observation)
        if policy.haptic_guard_enabled
        else None
    )
    if policy.haptic_guard_enabled and guard is None:
        return False
    if not _follow(
        policy, move_robot, start, target, APPROACH_STEPS, APPROACH_DT, "approach",
        context["approach_stiffness"], context["approach_damping"],
        get_observation=get_observation,
        haptic_guard=guard,
    ):
        context["counts"]["haptic_contacts"] += 1
        retreat_to_pose(
            policy,
            get_observation,
            move_robot,
            start,
            context["approach_stiffness"],
            context["approach_damping"],
        )
        return False
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
        "camera_timestamp_mismatch": "세 camera의 촬영 시각이 서로 다름",
        "controller_time_difference_exceeded": "controller와 center camera 시각 차이가 허용 범위를 초과함",
        "capture_not_after_command": "center camera frame이 현재 명령보다 새롭지 않음",
        "tf_time_difference_exceeded": "plug TF와 해당 camera 촬영 시각 차이가 허용 범위를 초과함",
    }
    detail = descriptions.get(key, key.replace("_", " "))
    return f"대기시간 내 일치하는 Observation을 찾지 못함: {detail}" if timed_out else detail


def collect(policy, context, get_observation, move_robot) -> bool:
    """선택 정책의 목표 pose 수렴 후 촬영 시점 XYZ label과 image를 저장한다."""
    max_attempts_per_sample = policy.max_attempts
    waypoint_index = 0
    sample_attempts = 0
    while context["counts"]["collect"] < policy.collect_steps:
        if waypoint_index >= len(policy.samples):
            replacements = build_samples(policy)
            if not replacements:
                policy.get_logger().error(
                    "[PortOffsetCollect] replacement waypoint generation failed"
                )
                return False
            policy.samples.extend(replacements)
            policy.get_logger().info(
                "[PortOffsetCollect] generated replacement waypoints: "
                f"count={len(replacements)}"
            )
        if sample_attempts >= max_attempts_per_sample:
            policy.get_logger().error(
                "[PortOffsetCollect] sample attempts exhausted: "
                f"sample={context['counts']['collect'] + 1}/{policy.collect_steps}, "
                f"waypoint={waypoint_index + 1}, "
                f"attempts={max_attempts_per_sample}"
            )
            return False
        index = waypoint_index
        sample_attempts += 1
        context["counts"]["attempts"] += 1
        try:
            port_tf = context["port_snapshot"].transform
            planned_sample = policy.samples[index % len(policy.samples)]
            plug_tf = dataset.shift_origin(
                policy.lookup_transform("base_link", context["cable_tip_frame"]),
                context["plug_offset"],
            )
            if policy.collection_policy == "board-view":
                pose, state, sample = _board_view_pose(policy, context, index)
            else:
                pose, state = policy.planner.build_pose(
                    port_tf,
                    plug_tf,
                    policy.lookup_transform("base_link", "gripper/tcp"),
                    z_offset=planned_sample["distance_m"],
                )
                pose, sample = _apply_sample(policy, pose, port_tf, state, index)
            tier_text = (
                f"tier={sample['tier_m']*1e3:g}mm"
                if sample["tier_m"] is not None
                else f"distance={sample['distance_m']*1e3:.1f}mm"
            )
            policy.get_logger().info(
                f"COLLECT {context['counts']['collect'] + 1}/{policy.collect_steps} "
                f"waypoint={waypoint_index + 1} "
                f"attempt={sample_attempts}/{max_attempts_per_sample} "
                f"policy={policy.collection_policy} {tier_text}: "
                f"xyz=({sample['x_m']*1e3:+.1f}, {sample['y_m']*1e3:+.1f}, {sample['z_m']*1e3:+.1f})mm "
                f"rpy=({sample['roll_rad']:+.4f}, {sample['pitch_rad']:+.4f}, {sample['yaw_rad']:+.4f})rad"
            )
            start = _tcp_pose(get_observation())
            if start is None:
                policy.get_logger().error(
                    "[PortOffsetCollect] view move failed: missing TCP pose"
                )
                continue
            guarded_policy = policy.collection_policy in {"near-port", "descent"}
            haptic_guard = (
                prepare_haptic_guard(policy, get_observation)
                if policy.haptic_guard_enabled and guarded_policy
                else None
            )
            if policy.haptic_guard_enabled and guarded_policy and haptic_guard is None:
                continue
            if policy.collection_policy != "near-port":
                distance = float(
                    np.linalg.norm(
                        np.array(
                            [
                                pose.position.x - start.position.x,
                                pose.position.y - start.position.y,
                                pose.position.z - start.position.z,
                            ]
                        )
                    )
                )
                steps = min(120, max(10, int(np.ceil(distance / 0.003))))
                if not _follow(
                    policy,
                    move_robot,
                    start,
                    pose,
                    steps,
                    0.03,
                    policy.collection_policy,
                    context["collect_stiffness"],
                    context["collect_damping"],
                    get_observation=get_observation,
                    haptic_guard=haptic_guard,
                ):
                    context["counts"]["haptic_contacts"] += 1
                    retreat_to_pose(
                        policy,
                        get_observation,
                        move_robot,
                        start,
                        context["collect_stiffness"],
                        context["collect_damping"],
                    )
                    waypoint_index += 1
                    sample_attempts = 0
                    continue
            command_stamp = policy.set_pose_target(
                move_robot, pose, context["collect_stiffness"], context["collect_damping"]
            )
            settled_stamp, settle = wait_for_pose_convergence(
                policy,
                get_observation,
                pose,
                command_stamp,
                haptic_guard=haptic_guard,
            )
            if settled_stamp is None:
                if settle.get("failure_reason") == "haptic_contact":
                    current = _tcp_pose(get_observation())
                    if current is not None:
                        policy.set_pose_target(
                            move_robot,
                            current,
                            context["collect_stiffness"],
                            context["collect_damping"],
                        )
                    context["counts"]["haptic_contacts"] += 1
                    policy.get_logger().error(
                        "[PortOffsetCollect] HAPTIC CONTACT: "
                        f"stage={policy.collection_policy}_settle, "
                        f"delta={settle['last_delta_force_n']:.2f}N, "
                        f"peak={settle['peak_delta_force_n']:.2f}N, "
                        f"threshold={policy.haptic_force_threshold_n:.2f}N"
                    )
                    retreat_to_pose(
                        policy,
                        get_observation,
                        move_robot,
                        start,
                        context["collect_stiffness"],
                        context["collect_damping"],
                    )
                    waypoint_index += 1
                    sample_attempts = 0
                    continue
                policy.get_logger().error(
                    policy.log_text(
                        "[PortOffsetCollect] CAPTURE FAILED: pose convergence timeout; "
                        f"motion={settle['position_error_m']*1e3:.3f}mm/"
                        f"{settle['orientation_error_rad']:.6f}rad, "
                        f"tracking={settle['tracking_position_error_m']*1e3:.3f}mm/"
                        f"{settle['tracking_orientation_error_rad']:.6f}rad",
                        "red",
                    )
                )
                continue
            if haptic_guard is not None:
                sample["haptic"] = haptic_guard.metrics()
            policy.get_logger().info(
                "[PortOffsetCollect] pose settled: "
                f"motion={settle['position_error_m']*1e3:.3f}mm/"
                f"{settle['orientation_error_rad']:.6f}rad, "
                f"tracking={settle['tracking_position_error_m']*1e3:.3f}mm/"
                f"{settle['tracking_orientation_error_rad']:.6f}rad, "
                f"command_clamp_delta={settle['command_position_delta_m']*1e3:.3f}mm/"
                f"{settle['command_orientation_delta_rad']:.6f}rad"
            )
            observation, timestamps = dataset.wait_for_observation(
                policy, get_observation, settled_stamp
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
            wait_start = time.monotonic_ns()
            plug_stamped = policy.lookup_transform_at(
                "base_link",
                context["cable_tip_frame"],
                dataset.image_for_camera(observation, "center").header.stamp,
            )
            timestamps.setdefault("wait_ns", {})["tf"] = (
                time.monotonic_ns() - wait_start
            )
            valid, timestamps = dataset.tf_sync(
                policy,
                timestamps,
                {
                    "port": context["port_snapshot"],
                    "plug": plug_stamped,
                },
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
            label_xyz = dataset.target_xyz(
                port_tf,
                dataset.shift_origin(plug_stamped.transform, context["plug_offset"]),
            )
            sampling_offset_xyz = _actual_sampling_offset(
                label_xyz,
                port_tf,
                state["port_axis"],
                0.0 if policy.collection_policy == "board-view" else sample["distance_m"],
            )
            if sample["tier_m"] is not None and max(
                abs(value) for value in sampling_offset_xyz
            ) > sample["tier_m"]:
                policy.get_logger().error(
                    policy.log_text(
                        "[PortOffsetCollect] CAPTURE FAILED: actual TF sampling offset is outside "
                        f"the {sample['tier_m']*1e3:g}mm tier; "
                        f"xyz=({sampling_offset_xyz[0]*1e3:+.3f}, "
                        f"{sampling_offset_xyz[1]*1e3:+.3f}, "
                        f"{sampling_offset_xyz[2]*1e3:+.3f})mm",
                        "red",
                    )
                )
                continue
            actual_view_distance_m = -float(
                np.dot(np.asarray(label_xyz, dtype=float), state["port_axis"])
            )
            sample["actual_xyz_m"] = sampling_offset_xyz
            sample["actual_view_distance_m"] = actual_view_distance_m
            saved, detail = dataset.save_sample(
                policy,
                episode_name=context["episode_name"],
                task=context["task"],
                step_idx=context["counts"]["collect"],
                observation=observation,
                port_tf=port_tf,
                timestamps=timestamps,
                label_xyz=label_xyz,
                sample=sample,
                settle=settle,
                annotation_ports=context.get("annotation_ports", ()),
            )
            if saved:
                context["counts"]["collect"] += 1
                waypoint_index += 1
                sample_attempts = 0
                policy.get_logger().info(
                    policy.log_text(
                        f"[PortOffsetCollect] CAPTURE SAVED: {detail}; "
                        f"capture_stamp_ns={timestamps['capture_stamp_ns']}; skew={_skew_text(timestamps)}",
                        "green",
                    )
                )
            elif dataset.is_port_visibility_failure(detail):
                policy.get_logger().info(
                    policy.log_text(
                        "[PortOffsetCollect] WAYPOINT SKIPPED: "
                        f"{detail}; moving to waypoint {waypoint_index + 2}",
                        "cyan",
                    )
                )
                waypoint_index += 1
                sample_attempts = 0
            else:
                policy.get_logger().error(policy.log_text(f"[PortOffsetCollect] CAPTURE FAILED: {detail}", "red"))
        except TransformException as exc:
            policy.get_logger().error(
                policy.log_text(f"[PortOffsetCollect] CAPTURE FAILED: TF lookup: {exc}", "red")
            )
        policy.sleep_for(policy.step_sleep_s)
    return True
