from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Pose, Transform

from phy_policy.data_generator import dataset, motion


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(str(message))

    def error(self, message):
        self.messages.append(str(message))


class _Policy:
    def __init__(self):
        self.collection_policy = "board-view"
        self.collect_steps = 1
        self.max_attempts = 2
        self.samples = [{"name": "initial"}]
        self.step_sleep_s = 0.0
        self.base_z_offset = 0.020
        self.logger = _Logger()

    def get_logger(self):
        return self.logger

    def log_text(self, message, color):
        return message

    def sleep_for(self, seconds):
        return None

    def lookup_transform(self, target, source):
        return Transform()

    def lookup_transform_at(self, target, source, stamp):
        return SimpleNamespace(transform=Transform())

    def set_pose_target(self, move_robot, pose, stiffness, damping):
        return 1


def test_collect_skips_invisible_waypoint_and_uses_replacement(monkeypatch):
    policy = _Policy()
    visited = []
    observation = SimpleNamespace(
        center_image=SimpleNamespace(header=SimpleNamespace(stamp=object()))
    )
    settle = {
        "position_error_m": 0.0,
        "orientation_error_rad": 0.0,
        "tracking_position_error_m": 0.0,
        "tracking_orientation_error_rad": 0.0,
        "command_position_delta_m": 0.0,
        "command_orientation_delta_rad": 0.0,
        "wait_ns": 0,
    }

    def board_view_pose(current_policy, context, index):
        visited.append(current_policy.samples[index]["name"])
        return Pose(), {"port_axis": np.array([0.0, 0.0, 1.0])}, {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "tier_m": None,
            "distance_m": 0.8,
        }

    save_results = iter(
        [
            (False, f"{dataset.PORT_VISIBILITY_FAILURE_PREFIX} visible=[]"),
            (True, "saved"),
        ]
    )
    monkeypatch.setattr(motion, "_board_view_pose", board_view_pose)
    monkeypatch.setattr(motion, "_tcp_pose", lambda current: Pose())
    monkeypatch.setattr(motion, "_follow", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        motion,
        "wait_for_pose_convergence",
        lambda *args, **kwargs: (2, settle),
    )
    monkeypatch.setattr(
        motion,
        "build_samples",
        lambda current_policy: [{"name": "replacement"}],
    )
    monkeypatch.setattr(dataset, "shift_origin", lambda transform, offset: transform)
    monkeypatch.setattr(
        dataset,
        "wait_for_observation",
        lambda *args, **kwargs: (
            observation,
            {"sync_valid": True, "capture_stamp_ns": 2},
        ),
    )
    monkeypatch.setattr(
        dataset,
        "tf_sync",
        lambda current_policy, timestamps, transforms, static_sources: (
            True,
            timestamps,
        ),
    )
    monkeypatch.setattr(dataset, "target_xyz", lambda port_tf, plug_tf: [0.0, 0.0, 0.0])
    monkeypatch.setattr(
        motion,
        "_actual_sampling_offset",
        lambda *args, **kwargs: [0.0, 0.0, 0.0],
    )
    monkeypatch.setattr(dataset, "save_sample", lambda *args, **kwargs: next(save_results))

    context = {
        "counts": {"collect": 0, "attempts": 0},
        "port_snapshot": SimpleNamespace(transform=Transform()),
        "cable_tip_frame": "plug",
        "plug_offset": np.zeros(3),
        "collect_stiffness": [],
        "collect_damping": [],
        "episode_name": "episode",
        "task": object(),
    }

    completed = motion.collect(policy, context, lambda: observation, lambda **kwargs: None)

    assert completed
    assert visited == ["initial", "replacement"]
    assert context["counts"] == {"collect": 1, "attempts": 2}
    assert any("WAYPOINT SKIPPED" in message for message in policy.logger.messages)
    assert any("generated replacement waypoints" in message for message in policy.logger.messages)
