from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import Pose

from phy_data_collection.policy import motion
from phy_data_collection.policy.final_policy_vision import (
    PortVision,
    plane_normal,
    project_point,
    target_from_task,
    track_keypoints,
    triangulate_point,
)


def test_task_target_uses_port_type_rail_and_port():
    task = SimpleNamespace(
        port_type="sfp",
        target_module_name="nic_card_mount_4",
        port_name="sfp_port_1",
    )

    target = target_from_task(task)

    assert (target.port_type, target.rail_index, target.port_index) == ("SFP", 4, 1)
    assert target.class_name == "SFP_41"


@pytest.mark.parametrize(
    ("module", "port"),
    [("nic_card_mount_5", "sfp_port_0"), ("nic_card_mount_1", "sfp_port_2")],
)
def test_task_target_rejects_out_of_range_sfp(module, port):
    task = SimpleNamespace(
        port_type="sfp", target_module_name=module, port_name=port
    )

    with pytest.raises(ValueError):
        target_from_task(task)


def test_task_target_rejects_sc_until_dataset_is_available():
    task = SimpleNamespace(
        port_type="sc",
        target_module_name="sc_port_1",
        port_name="sc_port_base",
    )

    with pytest.raises(ValueError, match="supports SFP only"):
        target_from_task(task)


def test_dlt_reconstructs_base_link_point():
    intrinsic = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    camera_a_from_base = np.eye(4)
    camera_b_from_base = np.eye(4)
    camera_b_from_base[0, 3] = -0.10
    projection_a = intrinsic @ camera_a_from_base[:3]
    projection_b = intrinsic @ camera_b_from_base[:3]
    expected = np.array([0.02, 0.01, 1.0])

    actual = triangulate_point(
        projection_a,
        projection_b,
        project_point(projection_a, expected),
        project_point(projection_b, expected),
    )

    np.testing.assert_allclose(actual, expected, atol=1e-8)


def test_plane_normal_points_toward_camera_for_stand_off():
    corners = np.array(
        [
            [-0.01, -0.01, 1.0],
            [0.01, -0.01, 1.0],
            [0.01, 0.01, 1.0],
            [-0.01, 0.01, 1.0],
        ]
    )

    normal = plane_normal(corners, viewpoint=np.zeros(3))

    np.testing.assert_allclose(normal, [0.0, 0.0, -1.0], atol=1e-8)


def test_klt_tracks_previous_keypoints_into_current_frame():
    previous = np.zeros((120, 160), dtype=np.uint8)
    points = np.array([[40, 40], [80, 40], [80, 80], [40, 80]], dtype=np.float32)
    for x, y in points.astype(int):
        cv2.circle(previous, (x, y), 4, 255, -1)
        cv2.line(previous, (x - 6, y), (x + 6, y), 128, 1)
        cv2.line(previous, (x, y - 6), (x, y + 6), 128, 1)
    transform = np.float32([[1.0, 0.0, 5.0], [0.0, 1.0, 3.0]])
    current = cv2.warpAffine(previous, transform, (160, 120))

    tracked = track_keypoints(previous, current, points, forward_backward_max_px=1.0)

    assert tracked is not None
    current_points, forward_backward_rms = tracked
    np.testing.assert_allclose(current_points, points + [5.0, 3.0], atol=0.2)
    assert forward_backward_rms <= 1.0


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _Policy:
    def __init__(self):
        self.commands = []

    def get_logger(self):
        return _Logger()

    def set_pose_target(self, _move_robot, pose, *, stiffness, damping):
        self.commands.append((pose, stiffness, damping))

    def sleep_for(self, _seconds):
        pass


def test_follow_guard_stops_before_next_motion_command():
    policy = _Policy()

    completed = motion._follow(
        policy,
        lambda **_kwargs: None,
        Pose(),
        Pose(),
        5,
        0.0,
        "test",
        [],
        [],
        step_guard=lambda index, _pose: index < 2,
    )

    assert not completed
    assert len(policy.commands) == 2


class _FakeModel:
    names = {0: "SFP_41", 1: "SFP_40"}

    def predict(self, *, source, **_kwargs):
        boxes = [
            SimpleNamespace(cls=np.array([0]), conf=np.array([0.9])),
            SimpleNamespace(cls=np.array([1]), conf=np.array([0.99])),
        ]
        keypoints = np.array(
            [
                [[10, 10], [20, 10], [20, 20], [10, 20]],
                [[40, 10], [50, 10], [50, 20], [40, 20]],
            ],
            dtype=float,
        )
        result = SimpleNamespace(
            boxes=boxes,
            keypoints=SimpleNamespace(xy=keypoints, conf=np.ones((2, 4))),
            names=self.names,
        )
        return [result for _ in source]


def test_detection_never_switches_to_other_rail_or_port():
    target = target_from_task(
        SimpleNamespace(
            port_type="sfp",
            target_module_name="nic_card_mount_4",
            port_name="sfp_port_1",
        )
    )
    vision = PortVision(_Policy(), target, model=_FakeModel())
    images = {
        camera: np.zeros((60, 60, 3), dtype=np.uint8)
        for camera in ("left", "center", "right")
    }

    detections = vision._detect(images)

    assert all(len(camera_detections) == 1 for camera_detections in detections.values())
    assert all(
        camera_detections[0]["class_name"] == "SFP_41"
        for camera_detections in detections.values()
    )
