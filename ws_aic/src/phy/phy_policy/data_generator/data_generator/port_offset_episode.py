from __future__ import annotations
"""PortOffsetCollect의 episode 종료와 dataset 업로드 처리."""

import json
import os
import signal
import sys
import threading
import time
import cv2
import numpy as np

from datetime import datetime
from pathlib import Path
from typing import Optional, Any
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose, Transform, Vector3, Wrench
from data_generator.lib.cheatcode import CheatCodePlanner
from data_generator.port_offset_config import (
    DAMPING_DEFAULT,
    SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME,
    STIFFNESS_DEFAULT,
    TOOL0_TO_OPTICAL,
    TOOL0_TO_TCP_Z,
)
from data_generator.port_offset_geometry import (
    _matrix_to_rpy_xyz,
    _matrix_from_pose,
    _matrix_from_translation_quat,
    _quat_to_matrix_xyzw,
)
from tf2_ros import TransformException

def _upload_vision_offset_dataset_to_hub(
    self,
    *,
    task: Task,
    status: str,
    collect_steps: int,
) -> dict[str, Any]:
    """수집 완료 후 vision-offset dataset 디렉터리를 Hugging Face dataset repo에 업로드한다."""
    if not getattr(self, "_rpy_push_to_hub", False):
        return {"enabled": False, "success": False, "reason": "disabled"}
    if status != "ok":
        return {"enabled": True, "success": False, "reason": f"skipped_status_{status}"}
    if collect_steps <= 0:
        return {"enabled": True, "success": False, "reason": "no_collect_samples"}
    upload_on_port_type = str(
        getattr(self, "_rpy_hf_upload_on_port_type", "") or ""
    ).strip().lower()
    task_port_type = str(getattr(task, "port_type", "") or "").strip().lower()
    if upload_on_port_type and task_port_type != upload_on_port_type:
        return {
            "enabled": True,
            "success": False,
            "reason": f"waiting_for_port_type_{upload_on_port_type}",
            "current_port_type": task_port_type,
        }

    repo_id = str(getattr(self, "_rpy_hf_repo_id", "") or "").strip()
    if not repo_id:
        reason = "AIC_VISION_OFFSET_REPO_ID is not set"
        self.get_logger().warn(f"[PortOffsetCollect] HF upload skipped: {reason}")
        return {"enabled": True, "success": False, "reason": reason}

    dataset_dir = Path(getattr(self, "_rpy_dataset_dir"))
    revision = str(getattr(self, "_rpy_hf_revision", "") or "main").strip() or "main"
    path_in_repo = str(getattr(self, "_rpy_hf_path_in_repo", "") or "").strip() or None
    private = bool(getattr(self, "_rpy_hf_private", True))
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        try:
            user_info = api.whoami()
        except Exception as exc:
            reason = (
                "Hugging Face authentication failed. Run `pixi run hf auth login` "
                "or set HF_TOKEN with write permission."
            )
            self.get_logger().error(f"[PortOffsetCollect] {reason} Details: {exc}")
            return {
                "enabled": True,
                "success": False,
                "repo_id": repo_id,
                "repo_type": "dataset",
                "revision": revision,
                "path_in_repo": path_in_repo or "",
                "reason": reason,
                "error": str(exc),
            }
        self.get_logger().info(
            "[PortOffsetCollect] Uploading vision-offset dataset to "
            f"https://huggingface.co/datasets/{repo_id}/tree/{revision} "
            f"as {user_info.get('name', 'unknown')}"
        )
        api.create_repo(
            repo_id=repo_id,
            repo_type="dataset",
            private=private,
            exist_ok=True,
        )
        if revision != "main":
            try:
                branches = [
                    branch.name
                    for branch in api.list_repo_refs(repo_id, repo_type="dataset").branches
                ]
                if revision not in branches:
                    api.create_branch(
                        repo_id=repo_id,
                        repo_type="dataset",
                        branch=revision,
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f"[PortOffsetCollect] HF branch preparation warning: {exc}"
                )
        if path_in_repo:
            self.get_logger().warn(
                "[PortOffsetCollect] AIC_VISION_OFFSET_HF_PATH_IN_REPO is ignored "
                "when using upload_large_folder"
            )
        api.upload_large_folder(
            repo_id=repo_id,
            repo_type="dataset",
            revision=revision,
            folder_path=str(dataset_dir),
            ignore_patterns=["*.tmp", "*.lock", "__pycache__/*", ".DS_Store"],
            private=private,
        )
        url = f"https://huggingface.co/datasets/{repo_id}/tree/{revision}"
        self.get_logger().info(f"[PortOffsetCollect] HF upload complete: {url}")
        return {
            "enabled": True,
            "success": True,
            "repo_id": repo_id,
            "repo_type": "dataset",
            "revision": revision,
            "path_in_repo": path_in_repo or "",
            "url": url,
            "upload_method": "upload_large_folder",
        }
    except Exception as exc:
        self.get_logger().error(f"[PortOffsetCollect] HF upload failed: {exc}")
        return {
            "enabled": True,
            "success": False,
            "repo_id": repo_id,
            "repo_type": "dataset",
            "revision": revision,
            "path_in_repo": path_in_repo or "",
            "reason": str(exc),
        }

# ── 메인 에피소드 수집 로직 ───────────────────────────────────────────────
def _finish_data_collection_episode(
    self,
    *,
    episode_dir: Path,
    task: Task,
    phase_step_counts: dict[str, int],
    status: str,
    detail: str = "",
) -> bool:
    """삽입 성공과 별개로 데이터 수집 task를 마무리하고 engine에는 완료를 알린다."""
    insertion_success = False
    summary = {
        "task_id": task.id,
        "success": insertion_success,
        "insertion_success": insertion_success,
        "task_completed_for_engine": True,
        "status": status,
        "detail": detail,
        "mode": "vision_offset",
        "lift_up_steps": int(phase_step_counts.get("lift_up", 0)),
        "approach_steps": int(phase_step_counts.get("approach", 0)),
        "collect_steps": int(phase_step_counts.get("collect", 0)),
        "collect_pattern": self.collect_pattern,
        "collect_start_radius": self.collect_start_radius,
        "collect_end_radius": self.collect_end_radius,
        "collect_turns": self.collect_turns,
        "collect_gaussian_sigma": self.collect_gaussian_sigma,
        "collect_gaussian_max_radius": self.collect_gaussian_max_radius,
    }
    self._write_episode_summary(episode_dir, summary)
    summary["hub_upload"] = self._upload_vision_offset_dataset_to_hub(
        task=task,
        status=status,
        collect_steps=int(phase_step_counts.get("collect", 0)),
    )
    self._write_episode_summary(episode_dir, summary)
    self.get_logger().info(
        f"DataCollect complete. status={status} "
        f"collect_steps={phase_step_counts.get('collect', 0)} "
        f"insertion_success={insertion_success} task_completed_for_engine=True"
    )
    return True

def _write_episode_summary(self, episode_dir: Path, summary: dict) -> None:
    """에피소드 단위 요약 정보를 episode_summary.json 파일로 저장한다."""
    (episode_dir / "episode_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
