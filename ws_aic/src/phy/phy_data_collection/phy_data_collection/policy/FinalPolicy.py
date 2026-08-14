"""Multiple-card YOLO triangulation과 target-locked approach policy."""

from __future__ import annotations

import os
from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time

from . import motion
from .final_policy_vision import (
    PortEstimate,
    PortVision,
    TargetSpec,
    target_from_task,
)


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


class FinalPolicy(Policy):
    """Task target class를 triangulation하고 같은 port track을 지키며 접근한다."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.sync_wait_timeout_s = max(
            0.0, _env_float("AIC_FINAL_POLICY_TF_TIMEOUT_S", 0.2)
        )
        self.vision_retries = max(1, _env_int("AIC_APPROACH_VISION_RETRIES", 20))
        self.vision_retry_s = max(0.0, _env_float("AIC_APPROACH_RETRY_DT", 0.2))
        self.stand_off_m = max(
            0.0, _env_float("AIC_APPROACH_NEAR_STAND_OFF_M", 0.030)
        )
        self.tcp_offset = np.array(
            [
                _env_float("AIC_APPROACH_TCP_OFFSET_X_M", 0.0),
                _env_float("AIC_APPROACH_TCP_OFFSET_Y_M", 0.015),
                _env_float("AIC_APPROACH_TCP_OFFSET_Z_M", 0.045),
            ],
            dtype=float,
        )
        self.max_approach_distance_m = max(
            0.0, _env_float("AIC_APPROACH_MAX_DISTANCE_M", 0.5)
        )
        self.track_max_misses = max(1, _env_int("AIC_TRACK_MAX_MISSES", 3))
        self.track_reacquire_hits = max(1, _env_int("AIC_TRACK_REACQUIRE_HITS", 2))
        self.track_retry_s = max(0.0, _env_float("AIC_TRACK_RETRY_S", 0.05))
        self._estimate: PortEstimate | None = None
        self._target: TargetSpec | None = None
        self._publisher = parent_node.create_publisher(
            PointStamped, "/final_policy/triangulated_port_xyz", 10
        )

    def lookup_transform_at(self, target: str, source: str, stamp):
        """지정 ROS simulation timestamp의 source-to-target transform을 조회한다."""
        return self._parent_node._tf_buffer.lookup_transform(
            target,
            source,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.sync_wait_timeout_s),
        )

    def _publish_estimate(self, estimate: PortEstimate, label: str) -> None:
        message = PointStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = estimate.stamp
        message.point.x, message.point.y, message.point.z = map(float, estimate.xyz)
        self._publisher.publish(message)
        self.get_logger().info(
            f"[FinalPolicy] {label}: class={estimate.class_name}, "
            f"xyz=({estimate.xyz[0]:+.4f}, {estimate.xyz[1]:+.4f}, "
            f"{estimate.xyz[2]:+.4f}), reproj={estimate.reprojection_rms_px:.2f}px"
        )

    @staticmethod
    def _completed_estimate(future: Future | None) -> PortEstimate | None:
        if future is None or not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _stage_lift_up_detect(
        self,
        vision: PortVision,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
    ) -> bool:
        """기존 lift profile 동안 비동기 target detection을 수행한다."""
        observation = get_observation()
        start = motion._tcp_pose(observation)
        if start is None:
            self.get_logger().error("FinalPolicy: lift failed; missing TCP pose")
            return False
        target_pose = motion._copy_pose(start)
        target_pose.position.z += motion.LIFT_M
        estimate: PortEstimate | None = None
        future: Future | None = None

        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="final-policy-yolo") as pool:
            if observation is not None:
                future = pool.submit(vision.estimate, observation)

            def detect_guard(_index, _pose) -> bool:
                nonlocal estimate, future
                completed = self._completed_estimate(future)
                if completed is not None:
                    estimate = completed
                    return False
                if future is not None and future.done():
                    future = None
                if future is None:
                    current = get_observation()
                    if current is not None:
                        future = pool.submit(vision.estimate, current)
                return True

            motion._follow(
                self,
                move_robot,
                start,
                target_pose,
                motion.LIFT_STEPS,
                motion.LIFT_DT,
                "lift_up",
                motion.LIFT_STIFFNESS,
                motion.LIFT_DAMPING,
                step_guard=detect_guard,
            )
            if estimate is None and future is not None:
                try:
                    estimate = future.result()
                except Exception as exc:
                    self.get_logger().warn(f"FinalPolicy: lift YOLO failed: {exc}")

        for _ in range(self.vision_retries):
            if estimate is not None:
                break
            estimate = vision.estimate(get_observation())
            if estimate is None:
                self.sleep_for(self.vision_retry_s)
        if estimate is None:
            self.get_logger().error(
                f"FinalPolicy: target {vision.target.class_name} was not triangulated"
            )
            return False
        self._estimate = estimate
        self._publish_estimate(estimate, "target locked")
        self.sleep_for(motion.LIFT_SETTLE_S)
        return True

    def _track_guard(
        self,
        vision: PortVision,
        get_observation: GetObservationCallback,
        index: int,
    ) -> bool:
        """같은 class와 keypoint track을 검사해 새 motion command를 허용한다."""
        if self._estimate is None:
            return False
        previous = self._estimate
        reacquire_hits = 0
        provisional = previous
        for _ in range(self.track_max_misses):
            observation = get_observation()
            tracked = vision.track(observation, provisional)
            if tracked is not None:
                self._estimate = tracked
                self._publish_estimate(tracked, f"track waypoint={index + 1}")
                return True
            reacquired = vision.estimate(observation)
            if (
                reacquired is not None
                and reacquired.stamp_ns > provisional.stamp_ns
                and float(np.linalg.norm(reacquired.xyz - previous.xyz))
                <= vision.track_max_3d_jump_m
            ):
                provisional = reacquired
                reacquire_hits += 1
                if reacquire_hits >= self.track_reacquire_hits:
                    self._estimate = provisional
                    self._publish_estimate(
                        provisional, f"track reacquired waypoint={index + 1}"
                    )
                    return True
            else:
                reacquire_hits = 0
            self.sleep_for(self.track_retry_s)
        self.get_logger().error(
            f"FinalPolicy: target track lost at approach waypoint {index + 1}; hold"
        )
        return False

    def _stage_approach(
        self,
        vision: PortVision,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
    ) -> bool:
        """Port normal stand-off pose로 이동하며 target lock을 확인한다."""
        observation = get_observation()
        start = motion._tcp_pose(observation)
        if start is None or self._estimate is None:
            self.get_logger().error("FinalPolicy: approach missing TCP or target pose")
            return False
        target_xyz = (
            self._estimate.xyz
            + self.stand_off_m * self._estimate.normal
            + self.tcp_offset
        )
        start_xyz = np.array(
            [start.position.x, start.position.y, start.position.z], dtype=float
        )
        distance = float(np.linalg.norm(target_xyz - start_xyz))
        if distance > self.max_approach_distance_m:
            self.get_logger().error(
                f"FinalPolicy: approach distance {distance:.3f}m exceeds "
                f"{self.max_approach_distance_m:.3f}m"
            )
            return False
        target_pose = motion._copy_pose(start)
        target_pose.position.x, target_pose.position.y, target_pose.position.z = map(
            float, target_xyz
        )
        self.get_logger().info(
            f"[FinalPolicy] approach: class={self._estimate.class_name}, "
            f"stand_off={self.stand_off_m * 1000.0:.1f}mm, "
            f"normal=({self._estimate.normal[0]:+.3f}, "
            f"{self._estimate.normal[1]:+.3f}, {self._estimate.normal[2]:+.3f}), "
            f"target=({target_xyz[0]:+.4f}, {target_xyz[1]:+.4f}, "
            f"{target_xyz[2]:+.4f})"
        )
        completed = motion._follow(
            self,
            move_robot,
            start,
            target_pose,
            motion.APPROACH_STEPS,
            motion.APPROACH_DT,
            "approach",
            motion.APPROACH_STIFFNESS,
            motion.APPROACH_DAMPING,
            step_guard=lambda index, _pose: self._track_guard(
                vision, get_observation, index
            ),
        )
        if not completed:
            return False
        self.sleep_for(motion.APPROACH_SETTLE_S)
        return True

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """Task target를 lift 중 찾고 같은 YOLO class를 추적하며 approach한다."""
        try:
            self._target = target_from_task(task)
        except ValueError as exc:
            self.get_logger().error(f"FinalPolicy: invalid task: {exc}")
            return False
        self._estimate = None
        vision = PortVision(self, self._target)
        self.get_logger().info(
            f"[FinalPolicy] target: type={self._target.port_type}, "
            f"rail={self._target.rail_index}, port={self._target.port_index}, "
            f"class={self._target.class_name}"
        )
        if not vision.load_model():
            return False
        send_feedback("FinalPolicy: lift_up_detect")
        if not self._stage_lift_up_detect(vision, get_observation, move_robot):
            return False
        send_feedback("FinalPolicy: target_locked_approach")
        if not self._stage_approach(vision, get_observation, move_robot):
            return False
        send_feedback("FinalPolicy: approach complete")
        return True
