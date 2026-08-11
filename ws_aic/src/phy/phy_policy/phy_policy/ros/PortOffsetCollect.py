"""AIC img2pos 데이터 수집 policy 진입점."""

from __future__ import annotations

import json
import os
import re
import signal
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Vector3, Wrench
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tf2_ros import TransformException

from phy_policy.data_generator import dataset, motion
from phy_policy.data_generator.geometry import transform_matrix


TOOL0_TCP_Z = 0.1965
TOOL0_OPTICAL = {
    "left": (
        [-0.100516584, -0.058032593, -0.008935891],
        [-0.113039947, 0.065265728, -0.495722390, 0.858616135],
    ),
    "center": (
        [-0.000000001, -0.116079183, -0.008937891],
        [-0.130528330, 0.000001827, -0.000000288, 0.991444580],
    ),
    "right": (
        [0.100516583, -0.058032595, -0.008935891],
        [-0.113041775, -0.065262563, 0.495721890, 0.858616424],
    ),
}
COLORS = {
    "cyan": "\033[36m",
    "green": "\033[32m",
    "red": "\033[31m",
    "reset": "\033[0m",
    "bold": "\033[1m",
}


def _env_bool(name: str, default: bool) -> bool:
    """환경변수의 일반적인 true/false 문자열을 bool로 변환한다."""
    value = os.environ.get(name)
    return default if value is None else value.strip().lower() not in {"0", "false", "no", "off", ""}


def _env_float(name: str, default: float) -> float:
    """환경변수를 float로 읽고 잘못되면 기본값을 반환한다."""
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _env_optional_int(name: str) -> int | None:
    """환경변수를 int로 읽고 없거나 잘못되면 None을 반환한다."""
    try:
        return int(os.environ[name])
    except (KeyError, ValueError):
        return None


def _env_float_list(name: str, default: str) -> list[float]:
    """쉼표로 구분한 환경변수를 유한한 float 목록으로 변환한다."""
    values = []
    for token in os.environ.get(name, default).split(","):
        try:
            value = float(token.strip())
        except ValueError:
            continue
        if np.isfinite(value):
            values.append(value)
    return values


def _range_from_env(
    min_name: str,
    max_name: str,
    default_min: float,
    default_max: float,
    scale: float,
) -> tuple[float, float]:
    """최소·최대 환경변수를 읽어 정렬하고 지정 비율로 변환한다."""
    try:
        low = float(os.environ.get(min_name, default_min / scale)) * scale
    except ValueError:
        low = default_min
    try:
        high = float(os.environ.get(max_name, default_max / scale)) * scale
    except ValueError:
        high = default_max
    return (low, high) if low <= high else (high, low)


def _dataset_dir() -> Path:
    """version을 반영한 기본 img2pos 데이터셋 경로를 반환한다."""
    root = Path(__file__).resolve().parents[5] / "data" / "img2pos"
    version = os.environ.get("AIC_IMG2POS_DATASET_VERSION", "").strip()
    return root / version if version else root


class PortOffsetCollect(Policy):
    """GT TF로 camera image와 port-minus-plug XYZ label을 수집한다."""

    collection_policy = "near-port"

    def __init__(self, parent_node):
        """현재 img2pos 수집에 필요한 설정과 ROS 실행 상태만 초기화한다."""
        os.environ.setdefault("AIC_COLLECT_STEPS", "1000")
        Policy.__init__(self, parent_node)

        fps = int(os.environ.get("AIC_COLLECT_FPS", "0"))
        self.step_sleep_s = 1.0 / (fps if fps > 0 else 20.0)
        self.capture_root = Path(os.environ.get("AIC_CAPTURE_DIR", "/tmp/aic_episodes"))
        self.collect_steps = max(1, int(os.environ["AIC_COLLECT_STEPS"]))
        self.base_z_offset = max(
            motion.MIN_CLEARANCE_M,
            _env_float("AIC_PORT_COLLECT_BASE_Z_OFFSET_M", motion.MIN_CLEARANCE_M),
        )
        self.sync_tolerance_ns = int(
            max(0.0, _env_float("AIC_COLLECT_SYNC_TOLERANCE_MS", 30.0)) * 1e6
        )
        self.sync_wait_timeout_s = max(
            0.0, _env_float("AIC_COLLECT_SYNC_WAIT_TIMEOUT_SEC", 1.0)
        )
        self.sync_poll_s = max(0.001, _env_float("AIC_COLLECT_SYNC_POLL_SEC", 0.01))
        self.settle_timeout_s = max(0.0, _env_float("AIC_COLLECT_SETTLE_TIMEOUT_SEC", 8.0))
        self.settle_position_tolerance_m = max(
            0.0, _env_float("AIC_COLLECT_SETTLE_POSITION_TOLERANCE_MM", 1.0) / 1000.0
        )
        self.settle_orientation_tolerance_rad = max(
            0.0,
            _env_float(
                "AIC_COLLECT_SETTLE_ORIENTATION_TOLERANCE_RAD",
                0.017453292519943295,
            ),
        )
        self.settle_stable_observations = max(
            1, int(os.environ.get("AIC_COLLECT_SETTLE_STABLE_OBSERVATIONS", "3"))
        )
        self.settle_poll_s = max(0.001, _env_float("AIC_COLLECT_SETTLE_POLL_SEC", 0.02))
        self.max_attempts = max(
            1, int(_env_float("AIC_COLLECT_MAX_ATTEMPTS", 2.0))
        )
        self.color_log = _env_bool("AIC_COLLECT_COLOR_LOG", True) and not os.environ.get("NO_COLOR")
        self.rng = np.random.default_rng(
            _env_optional_int("AIC_COLLECT_RANDOM_SEED")
        )
        self.planner = motion.Planner()
        self.board_distance_range = (
            max(
                self.base_z_offset,
                _env_float("AIC_BOARD_VIEW_DISTANCE_MIN_M", 0.750),
            ),
            max(
                self.base_z_offset,
                _env_float("AIC_BOARD_VIEW_DISTANCE_MAX_M", 0.850),
            ),
        )
        self.board_distance_range = tuple(sorted(self.board_distance_range))
        self.board_lateral_limit = max(
            0.0, _env_float("AIC_BOARD_VIEW_LATERAL_LIMIT_M", 0.030)
        )
        self.board_angle_limit = max(
            0.0,
            _env_float("AIC_BOARD_VIEW_ANGLE_LIMIT_RAD", 0.2617993877991494),
        )
        self.descent_start_distance = max(
            self.base_z_offset,
            _env_float("AIC_DESCENT_START_DISTANCE_M", 0.550),
        )
        self.descent_lateral_limit = max(
            0.0, _env_float("AIC_DESCENT_LATERAL_LIMIT_M", 0.040)
        )
        self.descent_angle_limit = max(
            0.0,
            _env_float("AIC_DESCENT_ANGLE_LIMIT_RAD", 0.3490658503988659),
        )

        self.dataset_dir = Path(
            os.environ.setdefault("AIC_IMG2POS_DATASET_DIR", str(_dataset_dir()))
        ).expanduser()
        self.dataset_version = os.environ.get("AIC_IMG2POS_DATASET_VERSION", "").strip()
        self.samples_path = self.dataset_dir / "samples.jsonl"
        self.run_id = os.environ.get("AIC_PORTOFFSET_RUN_ID", "").strip()
        self.trial_index = _env_optional_int("AIC_PORTOFFSET_TRIAL_INDEX")
        self.val_ratio = _env_float("AIC_IMG2POS_VAL_RATIO", 0.15)
        self.test_ratio = _env_float("AIC_IMG2POS_TEST_RATIO", 0.15)
        self.trial_split = os.environ.get("AIC_IMG2POS_TRIAL_SPLIT", "").strip().lower()
        self.min_visible_cameras = max(1, int(os.environ.get("AIC_IMG2POS_MIN_VISIBLE_CAMERAS", "2")))
        self.auto_annotate_ports = _env_bool(
            "AIC_IMG2POS_AUTO_ANNOTATE_PORTS", False
        )
        self.auto_annotation_visibility = (
            self.auto_annotate_ports
            and _env_bool("AIC_IMG2POS_DEPTH_VISIBILITY", False)
        )
        self.annotation_depth_timeout_s = max(
            0.0,
            _env_float(
                "AIC_IMG2POS_ANNOTATION_DEPTH_TIMEOUT_SEC",
                self.sync_wait_timeout_s,
            ),
        )
        self._depth_condition = threading.Condition()
        self._depth_buffers = {
            camera: deque(maxlen=32) for camera in ("left", "center", "right")
        }
        self._depth_subscriptions = []
        if self.auto_annotation_visibility:
            for camera in self._depth_buffers:
                subscription = parent_node.create_subscription(
                    Image,
                    f"/{camera}_camera/depth_image",
                    lambda message, camera=camera: self._on_depth_image(
                        camera, message
                    ),
                    qos_profile_sensor_data,
                )
                self._depth_subscriptions.append(subscription)
        self.capture_count = 0
        self.record = _env_bool("AIC_IMG2POS_RECORD", True)
        xy_limit = _env_float("AIC_PORT_COLLECT_XY_LIMIT_MM", 50.0) / 1000.0
        z_limit = _env_float("AIC_PORT_COLLECT_Z_LIMIT_MM", 50.0) / 1000.0
        roll_limit = _env_float("AIC_PORT_COLLECT_ROLL_LIMIT_RAD", 0.4363323129985824)
        pitch_limit = _env_float("AIC_PORT_COLLECT_PITCH_LIMIT_RAD", 0.4363323129985824)
        yaw_limit = _env_float("AIC_PORT_COLLECT_YAW_LIMIT_RAD", 0.6108652381980153)
        self.sample_ranges = {
            "x": _range_from_env("AIC_PORT_COLLECT_DX_MIN_MM", "AIC_PORT_COLLECT_DX_MAX_MM", -xy_limit, xy_limit, 0.001),
            "y": _range_from_env("AIC_PORT_COLLECT_DY_MIN_MM", "AIC_PORT_COLLECT_DY_MAX_MM", -xy_limit, xy_limit, 0.001),
            "z": _range_from_env("AIC_PORT_COLLECT_DZ_MIN_MM", "AIC_PORT_COLLECT_DZ_MAX_MM", 0.0, z_limit, 0.001),
            "roll": _range_from_env("AIC_PORT_COLLECT_ROLL_MIN_RAD", "AIC_PORT_COLLECT_ROLL_MAX_RAD", -roll_limit, roll_limit, 1.0),
            "pitch": _range_from_env("AIC_PORT_COLLECT_PITCH_MIN_RAD", "AIC_PORT_COLLECT_PITCH_MAX_RAD", -pitch_limit, pitch_limit, 1.0),
            "yaw": _range_from_env("AIC_PORT_COLLECT_YAW_MIN_RAD", "AIC_PORT_COLLECT_YAW_MAX_RAD", -yaw_limit, yaw_limit, 1.0),
        }
        self.rpy_norm_max = max(0.0, _env_float("AIC_PORT_COLLECT_RPY_NORM_MAX_RAD", 0.0))
        tiers = [
            value
            for value in _env_float_list("AIC_PORT_COLLECT_SAMPLING_TIERS_MM", "50,10,5,2")
            if value > 0.0
        ]
        self.sampling_tiers_m = [value / 1000.0 for value in tiers] or [xy_limit]
        weights = [
            value
            for value in _env_float_list("AIC_PORT_COLLECT_SAMPLING_TIER_WEIGHTS", "1,1,1,1")
            if value > 0.0
        ]
        self.sampling_tier_weights = (
            weights if len(weights) == len(self.sampling_tiers_m) else [1.0] * len(self.sampling_tiers_m)
        )
        self.samples = motion.build_samples(self)

        self._tool0_tcp = np.eye(4)
        self._tool0_tcp[2, 3] = _env_float("AIC_TOOL0_TO_TCP_Z", TOOL0_TCP_Z)
        self._tool0_optical = {
            camera: transform_matrix(translation, quaternion)
            for camera, (translation, quaternion) in TOOL0_OPTICAL.items()
        }
        dataset.write_manifest(self)

        try:
            signal.signal(signal.SIGTERM, self._on_sigterm)
        except ValueError:
            pass
        self.stop_file = Path(os.environ.get("AIC_STOP_FILE", "/tmp/aic_policy_stop"))
        threading.Thread(target=self._watch_stop_file, daemon=True).start()
        self.get_logger().info(
            f"[PortOffsetCollect] Ready: policy={self.collection_policy}, "
            f"steps={self.collect_steps}, "
            f"dataset={self.dataset_dir}, split={self.trial_split or 'hash'}, "
            f"depth_visibility={self.auto_annotation_visibility}"
        )

    def _on_depth_image(self, camera: str, message: Image) -> None:
        """camera별 최근 depth frame을 보관하고 대기 중인 capture를 깨운다."""
        with self._depth_condition:
            self._depth_buffers[camera].append(message)
            self._depth_condition.notify_all()

    def depth_image_at(self, camera: str, capture_stamp_ns: int) -> Image | None:
        """RGB capture stamp와 정확히 일치하는 depth frame을 제한 시간 동안 찾는다."""
        deadline = time.monotonic() + self.annotation_depth_timeout_s
        with self._depth_condition:
            while True:
                for message in reversed(self._depth_buffers[camera]):
                    if dataset._stamp_ns(message.header.stamp) == capture_stamp_ns:
                        return message
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._depth_condition.wait(remaining)

    def _watch_stop_file(self) -> None:
        """stop file이 생기면 policy 프로세스를 종료한다."""
        while True:
            if self.stop_file.exists():
                os._exit(0)
            time.sleep(0.5)

    def _on_sigterm(self, _signum, _frame) -> None:
        """SIGTERM을 정상 process 종료로 변환한다."""
        raise SystemExit(0)

    def log_text(self, message: str, color: str) -> str:
        """수집 상태 로그에 선택적 ANSI 색상을 적용한다."""
        if not self.color_log:
            return message
        return f"{COLORS['bold']}{COLORS[color]}{message}{COLORS['reset']}"

    def _wait_for_tf(self, target: str, source: str, timeout_s: float = 10.0) -> bool:
        """지정 TF가 제한 시간 안에 준비되는지 확인한다."""
        start = self.time_now()
        while self.time_now() - start < Duration(seconds=timeout_s):
            try:
                self._parent_node._tf_buffer.lookup_transform(target, source, Time())
                return True
            except TransformException:
                self.sleep_for(0.1)
        return False

    def lookup_transform(self, target: str, source: str):
        """최신 target 기준 source Transform을 반환한다."""
        return self._parent_node._tf_buffer.lookup_transform(target, source, Time()).transform

    def lookup_latest_stamped(self, target: str, source: str):
        """trial 고정 frame을 저장할 최신 stamped Transform을 반환한다."""
        return self._parent_node._tf_buffer.lookup_transform(target, source, Time())

    def lookup_transform_at(self, target: str, source: str, stamp):
        """camera 촬영 시각의 stamped Transform을 조회한다."""
        query_time = Time.from_msg(stamp)
        return self._parent_node._tf_buffer.lookup_transform(
            target,
            source,
            query_time,
            timeout=Duration(seconds=self.sync_wait_timeout_s),
        )

    def _port_frame(self, task: Task) -> str:
        """port entrance TF가 있으면 선택하고 기본 port link로 fallback한다."""
        port = f"task_board/{task.target_module_name}/{task.port_name}_link"
        entrance = f"{port}_entrance"
        return entrance if self._wait_for_tf("base_link", entrance, 2.0) else port

    def _annotation_port_frames(self, task: Task, fallback_frame: str) -> list[dict]:
        """task card mask에서 현재 scene에 생성된 port entrance frame을 나열한다."""
        match = re.search(r"(?:^|_)cards([01]+)(?:_|$)", str(task.id))
        connector = dataset._connector(task).lower()
        if match is None or connector not in {"sfp", "sc"}:
            rail = int(dataset._last_number(task.target_module_name, 0))
            port = (
                int(dataset._last_number(task.port_name, 0))
                if connector == "sfp"
                else 0
            )
            class_id, label = dataset.port_annotation_class(connector, rail, port)
            return [
                {
                    "class_id": class_id,
                    "label": label,
                    "port_type": connector,
                    "instance_id": str(task.port_name),
                    "frame_id": fallback_frame,
                }
            ]
        active_rails = [
            rail for rail, enabled in enumerate(reversed(match.group(1))) if enabled == "1"
        ]
        if connector == "sfp":
            ports = []
            for rail in active_rails:
                for port in range(dataset.SFP_PORT_COUNT):
                    class_id, label = dataset.port_annotation_class(
                        connector, rail, port
                    )
                    ports.append(
                        {
                            "class_id": class_id,
                            "label": label,
                            "port_type": connector,
                            "instance_id": f"nic_card_mount_{rail}/sfp_port_{port}",
                            "frame_id": (
                                f"task_board/nic_card_mount_{rail}/"
                                f"sfp_port_{port}_link_entrance"
                            ),
                        }
                    )
            return ports
        class_id, label = dataset.port_annotation_class(connector, 0)
        return [
            {
                "class_id": class_id,
                "label": label,
                "port_type": connector,
                "instance_id": f"sc_port_{rail}/sc_port_base",
                "frame_id": f"task_board/sc_port_{rail}/sc_port_base_link_entrance",
            }
            for rail in active_rails
        ]

    def _snapshot_annotation_ports(
        self, task: Task, fallback_frame: str
    ) -> list[dict]:
        """현재 scene의 port frame들을 trial 고정 base_link Transform으로 저장한다."""
        ports = self._annotation_port_frames(task, fallback_frame)
        snapshots = []
        for port in ports:
            stamped = self.lookup_latest_stamped("base_link", port["frame_id"])
            snapshots.append({**port, "transform": stamped.transform})
        return snapshots

    def _cable_tip_frame(self, task: Task) -> str:
        """task 정보에서 사용 가능한 cable tip frame을 선택한다."""
        candidates = [
            f"{task.cable_name}/{task.plug_name}_link",
            f"{task.cable_name}/{task.plug_name}_tip_link",
            f"{task.cable_name}/{task.plug_type}_tip_link",
        ]
        for frame in dict.fromkeys(candidates):
            if self._wait_for_tf("base_link", frame, 0.5):
                return frame
        return candidates[0]

    def _board_frame(self) -> str | None:
        """전체 보드 가시성 검사에 사용할 task board base frame을 찾는다."""
        for frame in ("task_board/task_board_base_link", "task_board_base_link"):
            if self._wait_for_tf("base_link", frame, 0.5):
                return frame
        return None

    def set_pose_target(self, move_robot, pose, stiffness=None, damping=None) -> int:
        """controller에 pose 명령을 보내고 발행 ROS 시각을 nanosecond로 반환한다."""
        stiffness = stiffness or motion.STIFFNESS
        damping = damping or motion.DAMPING
        stamp = self.get_clock().now().to_msg()
        update = MotionUpdate(
            header=Header(frame_id="base_link", stamp=stamp),
            pose=pose,
            target_stiffness=np.diag(stiffness).flatten(),
            target_damping=np.diag(damping).flatten(),
            feedforward_wrench_at_tip=Wrench(force=Vector3(), torque=Vector3()),
            wrench_feedback_gains_at_tip=[0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION
            ),
        )
        try:
            move_robot(motion_update=update)
        except Exception:
            pass
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _finish(self, episode_dir: Path, task: Task, counts: dict[str, int], status: str, detail: str = "") -> bool:
        """episode 수집 summary를 기록하고 engine에 완료를 반환한다."""
        summary = {
            "task_id": task.id,
            "success": False,
            "insertion_success": False,
            "task_completed_for_engine": True,
            "status": status,
            "detail": detail,
            "mode": "img2pos",
            "collection_policy": self.collection_policy,
            "lift_up_steps": counts["lift_up"],
            "approach_steps": counts["approach"],
            "collect_steps": counts["collect"],
            "collect_attempts": counts["attempts"],
        }
        summary_path = episode_dir / "episode_summary.json"
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.get_logger().info(
            f"DataCollect complete. status={status} collect_steps={counts['collect']}"
        )
        return True

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """lift-up, approach, collect를 실행하고 수집 결과를 확정한다."""
        send_feedback("data collect running")
        episode_name = time.strftime("%Y%m%d_%H%M%S") + f"_{task.id}"
        episode_dir = self.capture_root / episode_name
        episode_dir.mkdir(parents=True, exist_ok=True)
        counts = {"lift_up": 0, "approach": 0, "collect": 0, "attempts": 0}
        if not self.record:
            return self._finish(episode_dir, task, counts, "recording_disabled")

        port_frame = self._port_frame(task)
        cable_tip_frame = self._cable_tip_frame(task)
        if not self._wait_for_tf("base_link", port_frame) or not self._wait_for_tf(
            "base_link", cable_tip_frame
        ):
            return self._finish(
                episode_dir,
                task,
                counts,
                "tf_unavailable",
                f"port={port_frame}, plug={cable_tip_frame}",
            )
        try:
            port_snapshot = self.lookup_latest_stamped("base_link", port_frame)
        except TransformException as exc:
            return self._finish(episode_dir, task, counts, "port_tf_snapshot_failed", str(exc))

        annotation_ports = []
        if self.auto_annotate_ports:
            try:
                annotation_ports = self._snapshot_annotation_ports(task, port_frame)
            except TransformException as exc:
                return self._finish(
                    episode_dir,
                    task,
                    counts,
                    "annotation_port_tf_snapshot_failed",
                    str(exc),
                )

        board_snapshot = None
        if self.collection_policy == "board-view":
            board_frame = self._board_frame()
            if board_frame is None:
                return self._finish(
                    episode_dir, task, counts, "board_tf_unavailable"
                )
            try:
                board_snapshot = self.lookup_latest_stamped("base_link", board_frame)
            except TransformException as exc:
                return self._finish(
                    episode_dir, task, counts, "board_tf_snapshot_failed", str(exc)
                )

        context = {
            "task": task,
            "episode_name": episode_name,
            "counts": counts,
            "port_snapshot": port_snapshot,
            "board_snapshot": board_snapshot,
            "annotation_ports": annotation_ports,
            "cable_tip_frame": cable_tip_frame,
            "plug_offset": dataset.plug_reference_offset(task, cable_tip_frame),
            **motion.control_for(task),
        }
        stages = {
            "board-view": (("lift_up", motion.lift), ("collect", motion.collect)),
            "descent": (("lift_up", motion.lift), ("collect", motion.collect)),
            "near-port": (
                ("lift_up", motion.lift),
                ("approach", motion.approach),
                ("collect", motion.collect),
            ),
        }[self.collection_policy]
        for name, stage in stages:
            self.get_logger().info(f"[PortOffsetCollect] stage start: {name}")
            if not stage(self, context, get_observation, move_robot):
                return self._finish(episode_dir, task, counts, f"{name}_failed")
        self.sleep_for(0.5)
        return self._finish(episode_dir, task, counts, "ok")


# aic_model policy loader가 요구하는 심볼이다.
DataCollect = PortOffsetCollect
