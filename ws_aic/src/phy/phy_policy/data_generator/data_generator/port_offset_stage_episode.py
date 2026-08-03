from __future__ import annotations

"""PortOffsetCollect motion stage의 episode orchestration."""

import os
import time
from typing import Any

import numpy as np
from aic_model.policy import (
    GetObservationCallback,
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


def insert_cable(
    self,
    task: Task,
    get_observation: GetObservationCallback,
    move_robot: MoveRobotCallback,
    send_feedback: SendFeedbackCallback,
):
    """lift-up, approach, collect 단계를 실행하고 episode 결과를 확정한다."""
    self._task = task
    self._planner.reset()
    send_feedback("data collect running")

    episode_name = time.strftime("%Y%m%d_%H%M%S") + f"_{task.id}"
    episode_dir = self.capture_root / episode_name
    episode_dir.mkdir(parents=True, exist_ok=True)
    phase_step_counts = {"lift_up": 0, "approach": 0, "collect": 0}
    if not self._vision_offset_record_enabled:
        self.get_logger().warn(
            "[PortOffsetCollect] Vision offset recording disabled"
        )
        return self._finish_data_collection_episode(
            episode_dir=episode_dir,
            task=task,
            phase_step_counts=phase_step_counts,
            status="recording_disabled",
            detail="AIC_VISION_OFFSET_RECORD disabled",
        )

    port_frame = self._select_port_frame(task)
    cable_tip_frame = self._select_cable_tip_frame(task)
    if not self._wait_for_tf("base_link", port_frame) or not self._wait_for_tf(
        "base_link",
        cable_tip_frame,
    ):
        return self._finish_data_collection_episode(
            episode_dir=episode_dir,
            task=task,
            phase_step_counts=phase_step_counts,
            status="tf_unavailable",
            detail=(
                f"Missing required TF: port_frame={port_frame}, "
                f"cable_tip_frame={cable_tip_frame}"
            ),
        )
    self.get_logger().info(
        f"[PortOffsetCollect] SELECTED FRAMES: port_frame={port_frame}, cable_tip_frame={cable_tip_frame}"
    )
    try:
        port_tf_snapshot = self._lookup_latest_transform_stamped(
            "base_link",
            port_frame,
        )
    except TransformException as exc:
        return self._finish_data_collection_episode(
            episode_dir=episode_dir,
            task=task,
            phase_step_counts=phase_step_counts,
            status="port_tf_snapshot_failed",
            detail=str(exc),
        )
    self.get_logger().info(
        self._collect_log_text(
            "[PortOffsetCollect] Port TF snapshot ready: "
            f"frame={port_frame}, stamp="
            f"{port_tf_snapshot.header.stamp.sec}."
            f"{port_tf_snapshot.header.stamp.nanosec:09d}",
            "green",
        )
    )

    plug_reference_offset_local = self._plug_reference_offset_local(
        task,
        cable_tip_frame,
    )
    plug_reference_metadata = self._plug_reference_metadata(
        task,
        cable_tip_frame,
        plug_reference_offset_local,
    )
    self.get_logger().info(
        "[PortOffsetCollect] Plug reference point: "
        f"{plug_reference_metadata['point_name']} "
        f"frame={cable_tip_frame} "
        f"offset={plug_reference_metadata['local_offset_xyz_m']}"
    )
    control_ctx = self._configure_port_collect_control(task)
    ctx = {
        "task": task,
        "episode_name": episode_name,
        "episode_dir": episode_dir,
        "phase_step_counts": phase_step_counts,
        "port_frame": port_frame,
        "port_tf_snapshot": port_tf_snapshot,
        "cable_tip_frame": cable_tip_frame,
        "plug_reference_offset_local": plug_reference_offset_local,
        "plug_reference_metadata": plug_reference_metadata,
        "recording_started": False,
        "approach_reached_ground_truth_target": False,
    }
    ctx.update(control_ctx)

    stages = (
        ("lift_up", lambda: self._stage_lift_up(ctx, get_observation, move_robot)),
        ("approach", lambda: self._stage_approach(ctx, get_observation, move_robot)),
        ("collect", lambda: self._stage_collect(ctx, get_observation, move_robot)),
    )
    for stage_name, run_stage in stages:
        self.get_logger().info(f"[PortOffsetCollect] stage start: {stage_name}")
        if not run_stage():
            return self._finish_data_collection_episode(
                episode_dir=episode_dir,
                task=task,
                phase_step_counts=phase_step_counts,
                status=f"{stage_name}_failed",
            )

    self.sleep_for(0.5)
    return self._finish_data_collection_episode(
        episode_dir=episode_dir,
        task=task,
        phase_step_counts=phase_step_counts,
        status="ok",
    )
