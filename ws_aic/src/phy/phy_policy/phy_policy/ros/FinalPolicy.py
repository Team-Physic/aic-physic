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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image

from . import motion
from .final_policy_vision import (
    ANSI_BLUE,
    ANSI_RESET,
    CAMERAS,
    PortEstimate,
    PortVision,
    TargetSpec,
    observation_stamp_ns,
    target_from_task,
)


DEBUG_IMAGE_TOPICS = {
    camera: f"/final_policy/yolo/{camera}/image" for camera in CAMERAS
}
TASK_TOPIC = "/final_policy/task"
ANSI_GREEN = "\033[1;32m"


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


def _matches_locked_target(
    locked: PortEstimate, candidate: PortEstimate, radius_m: float
) -> bool:
    return (
        candidate.class_name == locked.class_name
        and float(np.linalg.norm(candidate.xyz - locked.xyz)) <= radius_m
    )


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
        self.target_lock_hits = max(1, _env_int("AIC_TARGET_LOCK_HITS", 5))
        self.target_lock_radius_m = max(
            0.0, _env_float("AIC_TARGET_LOCK_RADIUS_M", 0.010)
        )
        self._background_yolo_misses = 0
        self._estimate: PortEstimate | None = None
        self._target: TargetSpec | None = None
        self._publisher = parent_node.create_publisher(
            PointStamped, "/final_policy/triangulated_port_xyz", 10
        )
        task_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._task_publisher = parent_node.create_publisher(
            Task, TASK_TOPIC, task_qos
        )
        self._debug_image_publishers = {
            camera: parent_node.create_publisher(
                Image, topic, qos_profile_sensor_data
            )
            for camera, topic in DEBUG_IMAGE_TOPICS.items()
        }
        self._debug_publish_error_logged = False

    def _publish_yolo_debug(self, camera, result, source, header) -> None:
        """subscriber가 있을 때만 Ultralytics overlay를 ROS Image로 발행한다."""
        publisher = self._debug_image_publishers.get(camera)
        if publisher is None or publisher.get_subscription_count() < 1:
            return
        try:
            annotated = result.plot(
                img=np.asarray(source).copy(),
                boxes=True,
                labels=True,
                conf=True,
                kpt_line=True,
            )
            annotated = np.ascontiguousarray(annotated, dtype=np.uint8)
            if annotated.ndim != 3 or annotated.shape[2] != 3:
                raise ValueError(f"invalid overlay shape: {annotated.shape}")
            message = Image()
            if header is not None:
                message.header = header
            message.height, message.width = annotated.shape[:2]
            message.encoding = "bgr8"
            message.is_bigendian = False
            message.step = int(message.width * 3)
            message.data = annotated.tobytes()
            publisher.publish(message)
        except Exception as exc:
            if not self._debug_publish_error_logged:
                self.get_logger().warn(f"FinalPolicy: YOLO overlay failed: {exc}")
                self._debug_publish_error_logged = True

    def lookup_transform_at(self, target: str, source: str, stamp):
        """지정 ROS simulation timestamp의 source-to-target transform을 조회한다."""
        return self._parent_node._tf_buffer.lookup_transform(
            target,
            source,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.sync_wait_timeout_s),
        )

    def _publish_estimate(
        self, estimate: PortEstimate, label: str, color: str = ""
    ) -> None:
        message = PointStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = estimate.stamp
        message.point.x, message.point.y, message.point.z = map(float, estimate.xyz)
        self._publisher.publish(message)
        self.get_logger().info(
            f"{color}[FinalPolicy] {label}: class={estimate.class_name}, "
            f"xyz=({estimate.xyz[0]:+.4f}, {estimate.xyz[1]:+.4f}, "
            f"{estimate.xyz[2]:+.4f}), "
            f"reproj={estimate.reprojection_rms_px:.2f}px{ANSI_RESET}"
        )

    def _stage_lift_up_detect(
        self,
        vision: PortVision,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
    ) -> bool:
        """Lift와 YOLO를 병행하고 가까운 3D 검출을 반복해 target을 고정한다."""
        observation = get_observation()
        start = motion._tcp_pose(observation)
        if start is None:
            self.get_logger().error("FinalPolicy: lift failed; missing TCP pose")
            return False
        target_pose = motion._copy_pose(start)
        target_pose.position.z += motion.LIFT_M
        estimate: PortEstimate | None = None
        future: Future | None = None
        confirmations: list[PortEstimate] = []

        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="final-policy-yolo") as pool:
            def submit_latest() -> None:
                nonlocal future
                current = get_observation()
                if current is not None:
                    future = pool.submit(vision.estimate, current)

            def accept(candidate: PortEstimate | None) -> None:
                nonlocal estimate
                if candidate is None:
                    return
                if confirmations and candidate.stamp_ns <= confirmations[-1].stamp_ns:
                    return
                if confirmations:
                    center = np.median(
                        np.asarray([item.xyz for item in confirmations]), axis=0
                    )
                    if (
                        candidate.class_name != confirmations[-1].class_name
                        or float(np.linalg.norm(candidate.xyz - center))
                        > self.target_lock_radius_m
                    ):
                        confirmations.clear()
                confirmations.append(candidate)
                self._publish_estimate(
                    candidate,
                    f"target confirmation {len(confirmations)}/"
                    f"{self.target_lock_hits}",
                )
                if len(confirmations) >= self.target_lock_hits:
                    estimate = candidate

            submit_latest()

            def detect_guard(_index, _pose) -> bool:
                nonlocal future
                if estimate is not None or future is None or not future.done():
                    return True
                try:
                    accept(future.result())
                except Exception as exc:
                    self.get_logger().warn(f"FinalPolicy: lift YOLO failed: {exc}")
                future = None
                if estimate is None:
                    submit_latest()
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
            for _ in range(self.vision_retries):
                if estimate is not None:
                    break
                if future is None:
                    submit_latest()
                if future is None:
                    self.sleep_for(self.vision_retry_s)
                    continue
                try:
                    accept(future.result())
                except Exception as exc:
                    self.get_logger().warn(f"FinalPolicy: lift YOLO failed: {exc}")
                future = None
                if estimate is None:
                    self.sleep_for(self.vision_retry_s)

        if estimate is None:
            self.get_logger().error(
                f"FinalPolicy: target {vision.target.class_name} did not produce "
                f"{self.target_lock_hits} consistent detections"
            )
            return False
        self._estimate = estimate
        self._publish_estimate(estimate, "target locked")
        self.sleep_for(motion.LIFT_SETTLE_S)
        return True

    def _recover_target(
        self,
        locked: PortEstimate,
        poll_yolo,
        get_observation: GetObservationCallback,
        index: int,
    ) -> bool:
        """정지 이후 촬영된 연속 YOLO 결과만으로 target lock을 복구한다."""
        recovery_min_stamp_ns = observation_stamp_ns(get_observation())
        hits = 0
        previous_stamp_ns = recovery_min_stamp_ns - 1
        provisional: PortEstimate | None = None
        for _ in range(self.vision_retries):
            ready, candidate = poll_yolo(True)
            if (
                ready
                and candidate is not None
                and candidate.stamp_ns > recovery_min_stamp_ns
                and candidate.stamp_ns > previous_stamp_ns
                and _matches_locked_target(
                    locked, candidate, self.target_lock_radius_m
                )
            ):
                hits += 1
                previous_stamp_ns = candidate.stamp_ns
                provisional = candidate
                if hits >= self.track_reacquire_hits:
                    self._background_yolo_misses = 0
                    self._estimate = provisional
                    self._publish_estimate(
                        provisional,
                        f"track reacquired waypoint={index + 1}",
                        ANSI_GREEN,
                    )
                    return True
            else:
                hits = 0
            if hits < self.track_reacquire_hits:
                self.sleep_for(self.vision_retry_s)
        self.get_logger().error(
            f"FinalPolicy: target recovery failed at approach waypoint {index + 1}; hold"
        )
        return False

    def _track_guard(
        self,
        vision: PortVision,
        get_observation: GetObservationCallback,
        locked: PortEstimate,
        poll_yolo,
        index: int,
    ) -> bool:
        """KLT를 즉시 검사하고 background YOLO 불일치 시 이동을 보류한다."""
        if self._estimate is None:
            return False
        for _ in range(self.track_max_misses):
            ready, yolo_estimate = poll_yolo(False)
            observation = get_observation()
            if ready:
                if yolo_estimate is None:
                    self._background_yolo_misses += 1
                elif not _matches_locked_target(
                    locked, yolo_estimate, self.target_lock_radius_m
                ):
                    return self._recover_target(
                        locked, poll_yolo, get_observation, index
                    )
                else:
                    self._background_yolo_misses = 0
                    current_stamp_ns = observation_stamp_ns(observation)
                    if yolo_estimate.stamp_ns == current_stamp_ns:
                        reanchored = yolo_estimate
                    elif yolo_estimate.stamp_ns < current_stamp_ns:
                        reanchored = vision.track(observation, yolo_estimate)
                    else:
                        reanchored = None
                    if reanchored is not None and _matches_locked_target(
                        locked, reanchored, self.target_lock_radius_m
                    ):
                        self._estimate = reanchored
                        self._publish_estimate(
                            reanchored,
                            f"YOLO re-anchor waypoint={index + 1}",
                            ANSI_GREEN,
                        )
                        return True
                if self._background_yolo_misses >= self.track_max_misses:
                    return self._recover_target(
                        locked, poll_yolo, get_observation, index
                    )
            tracked = vision.track(observation, self._estimate)
            if tracked is not None and _matches_locked_target(
                locked, tracked, self.target_lock_radius_m
            ):
                self._estimate = tracked
                self._publish_estimate(
                    tracked, f"track waypoint={index + 1}", ANSI_GREEN
                )
                return True
            self.sleep_for(self.track_retry_s)
        return self._recover_target(locked, poll_yolo, get_observation, index)

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
        locked = self._estimate
        self._background_yolo_misses = 0
        target_xyz = locked.xyz + self.stand_off_m * locked.normal + self.tcp_offset
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
        pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="final-policy-yolo")
        future: Future | None = None

        def submit_latest() -> None:
            nonlocal future
            observation = get_observation()
            if observation is not None:
                future = pool.submit(vision.estimate, observation)

        def poll_yolo(wait: bool) -> tuple[bool, PortEstimate | None]:
            nonlocal future
            if future is None:
                submit_latest()
            if future is None or (not wait and not future.done()):
                return False, None
            try:
                result = future.result()
            except Exception as exc:
                self.get_logger().warn(f"FinalPolicy: background YOLO failed: {exc}")
                result = None
            future = None
            submit_latest()
            return True, result

        submit_latest()
        try:
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
                    vision, get_observation, locked, poll_yolo, index
                ),
            )
        finally:
            if future is not None:
                future.cancel()
            pool.shutdown(wait=True, cancel_futures=True)
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
        self._task_publisher.publish(task)
        self._estimate = None
        vision = PortVision(
            self,
            self._target,
            debug_image_callback=self._publish_yolo_debug,
        )
        self.get_logger().info(
            f"{ANSI_BLUE}[FinalPolicy] target: type={self._target.port_type}, "
            f"rail={self._target.rail_index}, port={self._target.port_index}, "
            f"class={self._target.class_name}{ANSI_RESET}"
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
