from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import Pose

from phy_policy.ros import motion
from phy_policy.ros.FinalPolicy import FinalPolicy
from phy_policy.ros.final_policy_vision import (
    PortEstimate,
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

    def warn(self, _message):
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


def _estimate_at(stamp_ns):
    return PortEstimate(
        "SFP_41",
        np.array([0.1, 0.2, 0.3]),
        np.array([0.0, 0.0, -1.0]),
        SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        ),
        stamp_ns,
        {},
        {},
        0.1,
    )


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

    def __init__(self):
        self.calls = 0

    def predict(self, *, source, **_kwargs):
        self.calls += 1
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


def test_detection_forwards_each_yolo_result_to_debug_callback():
    target = target_from_task(
        SimpleNamespace(
            port_type="sfp",
            target_module_name="nic_card_mount_4",
            port_name="sfp_port_1",
        )
    )
    published = []
    vision = PortVision(
        _Policy(),
        target,
        model=_FakeModel(),
        debug_image_callback=lambda camera, result, image, header: published.append(
            (camera, result, image.shape, header)
        ),
    )
    images = {
        camera: np.zeros((60, 60, 3), dtype=np.uint8)
        for camera in ("left", "center", "right")
    }
    headers = {camera: object() for camera in images}

    vision._detect(images, headers)

    assert [camera for camera, *_rest in published] == ["left", "center", "right"]
    assert all(shape == (60, 60, 3) for _camera, _result, shape, _header in published)
    assert all(header is headers[camera] for camera, _result, _shape, header in published)


def test_klt_tracking_does_not_run_yolo():
    target = target_from_task(
        SimpleNamespace(
            port_type="sfp",
            target_module_name="nic_card_mount_4",
            port_name="sfp_port_1",
        )
    )
    model = _FakeModel()
    vision = PortVision(_Policy(), target, model=model)
    previous_gray = np.zeros((120, 160), dtype=np.uint8)
    points = np.array([[40, 40], [80, 40], [80, 80], [40, 80]], dtype=np.float32)
    for x, y in points.astype(int):
        cv2.circle(previous_gray, (x, y), 4, 255, -1)
        cv2.line(previous_gray, (x - 6, y), (x + 6, y), 128, 1)
        cv2.line(previous_gray, (x, y - 6), (x, y + 6), 128, 1)
    current_gray = cv2.warpAffine(
        previous_gray,
        np.float32([[1.0, 0.0, 2.0], [0.0, 1.0, 1.0]]),
        (160, 120),
    )
    cameras = ("left", "center")
    previous_images = {
        camera: cv2.cvtColor(previous_gray, cv2.COLOR_GRAY2BGR) for camera in cameras
    }
    current_images = {
        camera: cv2.cvtColor(current_gray, cv2.COLOR_GRAY2BGR) for camera in cameras
    }
    detection = {
        "class_name": target.class_name,
        "confidence": 0.9,
        "keypoints": points,
        "uv": np.mean(points, axis=0),
    }
    previous = PortEstimate(
        target.class_name,
        np.array([60.0, 60.0, 1.0]),
        np.array([0.0, 0.0, -1.0]),
        SimpleNamespace(sec=1, nanosec=0),
        1_000_000_000,
        {camera: detection for camera in cameras},
        previous_images,
        0.1,
    )
    candidate = PortEstimate(
        target.class_name,
        previous.xyz.copy(),
        previous.normal.copy(),
        SimpleNamespace(sec=1, nanosec=1),
        1_000_000_001,
        previous.detections,
        current_images,
        0.1,
    )
    projection = np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
    )
    vision.track_reprojection_max_px = 10.0
    vision._projection_data = lambda _observation: {
        "stamp_ns": candidate.stamp_ns,
        "images": current_images,
        "projections": {camera: projection for camera in cameras},
    }
    vision._estimate_candidates = lambda _data, _detections: [candidate]

    tracked = vision.track(object(), previous)

    assert tracked is candidate
    assert model.calls == 0


def test_recovery_ignores_yolo_captured_before_robot_stopped():
    stopped_at = 10_000_000_000
    locked = _estimate_at(stopped_at - 1)
    candidates = iter(
        [
            _estimate_at(stopped_at),
            _estimate_at(stopped_at + 1),
            _estimate_at(stopped_at + 2),
        ]
    )
    calls = 0

    def poll_yolo(_wait):
        nonlocal calls
        calls += 1
        return True, next(candidates)

    policy = FinalPolicy.__new__(FinalPolicy)
    policy._parent_node = SimpleNamespace(get_logger=lambda: _Logger())
    policy.vision_retries = 3
    policy.vision_retry_s = 0.0
    policy.track_reacquire_hits = 2
    policy.target_lock_radius_m = 0.01
    policy._background_yolo_misses = 3
    policy._estimate = locked
    policy.sleep_for = lambda _seconds: None
    policy._publish_estimate = lambda *_args: None
    observation = SimpleNamespace(
        center_image=SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=10, nanosec=0)
            )
        )
    )

    recovered = policy._recover_target(locked, poll_yolo, lambda: observation, 0)

    assert recovered
    assert calls == 3
    assert policy._estimate.stamp_ns == stopped_at + 2
    assert policy._background_yolo_misses == 0


def test_background_yolo_reanchors_klt_at_current_frame():
    locked = _estimate_at(1)
    current_klt = _estimate_at(20)
    delayed_yolo = _estimate_at(10)
    reanchored = _estimate_at(30)
    references = []
    published = []

    def track(_observation, previous):
        references.append(previous)
        return reanchored

    policy = FinalPolicy.__new__(FinalPolicy)
    policy._estimate = current_klt
    policy._background_yolo_misses = 0
    policy.track_max_misses = 1
    policy.track_retry_s = 0.0
    policy.target_lock_radius_m = 0.01
    policy.sleep_for = lambda _seconds: None
    policy._publish_estimate = lambda estimate, label, _color: published.append(
        (estimate, label)
    )
    observation = SimpleNamespace(
        center_image=SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=0, nanosec=30)
            )
        )
    )

    allowed = policy._track_guard(
        SimpleNamespace(track=track),
        lambda: observation,
        locked,
        lambda _wait: (True, delayed_yolo),
        0,
    )

    assert allowed
    assert references == [delayed_yolo]
    assert policy._estimate is reanchored
    assert published == [(reanchored, "YOLO re-anchor waypoint=1")]
