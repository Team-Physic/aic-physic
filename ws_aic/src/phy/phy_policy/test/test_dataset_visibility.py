import json
from inspect import signature
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Transform

from phy_policy.data_generator import dataset
from phy_policy.ros.PortOffsetCollect import PortOffsetCollect


def test_points_projection_uses_full_image_bounds(monkeypatch):
    observation = SimpleNamespace(
        left_image=SimpleNamespace(width=100, height=100),
        left_camera_info=SimpleNamespace(
            k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ),
    )
    monkeypatch.setattr(dataset, "_base_to_camera", lambda *args: np.eye(4))

    inside = dataset._points_projection(
        SimpleNamespace(), observation, "left", [[1.0, 1.0, 1.0]]
    )
    outside = dataset._points_projection(
        SimpleNamespace(), observation, "left", [[-1.0, 1.0, 1.0]]
    )

    assert inside["visible"]
    assert inside["reason"] == "visible"
    assert not outside["visible"]
    assert outside["reason"] == "outside_image_bounds"
    assert outside["image_size_px"] == [100, 100]


def test_port_projection_rejects_large_bottom_connected_black_region(monkeypatch):
    image = np.full((100, 100, 3), 255, dtype=np.uint8)
    image[60:, 30:70] = 0
    monkeypatch.setattr(
        dataset,
        "_points_projection",
        lambda *args, **kwargs: {
            "visible": True,
            "points": [{"u_px": 50.0, "v_px": 80.0, "depth_m": 1.0}],
        },
    )
    monkeypatch.setattr(dataset, "image_to_bgr", lambda *args: image)

    result = dataset._port_projection(
        SimpleNamespace(), SimpleNamespace(left_image=object()), "left", Transform()
    )

    assert not result["visible"]
    assert result["reason"] == dataset.ROBOT_ARM_OCCLUSION_REASON


def test_port_projection_keeps_small_black_port_region(monkeypatch):
    image = np.full((100, 100, 3), 255, dtype=np.uint8)
    image[48:53, 48:53] = 0
    monkeypatch.setattr(
        dataset,
        "_points_projection",
        lambda *args, **kwargs: {
            "visible": True,
            "points": [{"u_px": 50.0, "v_px": 50.0, "depth_m": 1.0}],
        },
    )
    monkeypatch.setattr(dataset, "image_to_bgr", lambda *args: image)

    result = dataset._port_projection(
        SimpleNamespace(), SimpleNamespace(left_image=object()), "left", Transform()
    )

    assert result["visible"]
    assert "reason" not in result


def test_save_sample_requires_target_port_visibility(monkeypatch, tmp_path):
    calls = []

    def port_projection(policy, observation, camera, port_tf):
        calls.append(camera)
        return {"visible": camera == "left"}

    monkeypatch.setattr(dataset, "_port_projection", port_projection)
    policy = SimpleNamespace(min_visible_cameras=2, dataset_dir=tmp_path)

    saved, detail = dataset.save_sample(
        policy,
        episode_name="trial",
        task=object(),
        step_idx=0,
        observation=object(),
        port_tf=Transform(),
        timestamps={"sync_valid": True, "capture_stamp_ns": 1},
        label_xyz=[0.0, 0.0, 0.0],
        sample={},
        settle={},
    )

    assert "board_tf" not in signature(dataset.save_sample).parameters
    assert calls == ["left", "center", "right"]
    assert not saved
    assert "포트 가시성 부족" in detail
    assert dataset.is_port_visibility_failure(detail)


def test_save_sample_reports_visible_cameras_when_visibility_is_insufficient(
    monkeypatch, tmp_path
):
    def port_projection(policy, observation, camera, port_tf):
        if camera == "center":
            return {
                "visible": False,
                "reason": dataset.ROBOT_ARM_OCCLUSION_REASON,
                "points": [{"u_px": 50.0, "v_px": 80.0}],
                "image_size_px": [100, 100],
            }
        if camera == "right":
            return {
                "visible": False,
                "reason": "outside_image_bounds",
                "points": [{"u_px": 105.0, "v_px": 50.0}],
                "image_size_px": [100, 100],
            }
        return {
            "visible": True,
            "reason": "visible",
            "points": [{"u_px": 20.0, "v_px": 30.0}],
            "image_size_px": [100, 100],
        }

    monkeypatch.setattr(dataset, "_port_projection", port_projection)
    policy = SimpleNamespace(min_visible_cameras=2, dataset_dir=tmp_path)

    saved, detail = dataset.save_sample(
        policy,
        episode_name="trial",
        task=object(),
        step_idx=0,
        observation=object(),
        port_tf=Transform(),
        timestamps={"sync_valid": True, "capture_stamp_ns": 1},
        label_xyz=[0.0, 0.0, 0.0],
        sample={},
        settle={},
    )

    assert not saved
    assert "visible=['left']" in detail
    assert "robot_arm_occlusion=['center']" in detail
    assert "required=2" in detail
    assert "'left': {'visible': True, 'reason': 'visible'" in detail
    assert "'center': {'visible': False, 'reason': 'robot_arm_occlusion'" in detail
    assert "'right': {'visible': False, 'reason': 'outside_image_bounds'" in detail
    assert dataset.is_port_visibility_failure(detail)


def test_save_sample_accepts_one_occlusion_when_two_cameras_are_visible(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(
        dataset,
        "_port_projection",
        lambda policy, observation, camera, port_tf: {
            "visible": camera in {"left", "center"},
            **(
                {"reason": dataset.ROBOT_ARM_OCCLUSION_REASON}
                if camera == "right"
                else {}
            ),
        },
    )
    monkeypatch.setattr(
        dataset,
        "image_to_bgr",
        lambda policy, message, camera: np.zeros((2, 2, 3), dtype=np.uint8),
    )
    policy = SimpleNamespace(
        min_visible_cameras=2,
        dataset_dir=tmp_path,
        samples_path=tmp_path / "samples.jsonl",
        run_id="",
        trial_index=0,
        trial_split="train",
        val_ratio=0.0,
        test_ratio=0.0,
        collection_policy="near-port",
        capture_count=0,
    )
    observation = SimpleNamespace(
        left_image=object(),
        center_image=object(),
        right_image=object(),
    )

    saved, detail = dataset.save_sample(
        policy,
        episode_name="20260811_002131_portoffset_sfp_0000_cards00101_rail0_sfp_port_0",
        task=SimpleNamespace(
            id="portoffset_sfp_0000_cards00101_rail0_sfp_port_0",
            port_type="sfp",
            port_name="sfp_port_0",
            target_module_name="nic_card_mount_0",
        ),
        step_idx=0,
        observation=observation,
        port_tf=Transform(),
        timestamps={"sync_valid": True, "capture_stamp_ns": 1},
        label_xyz=[0.1, 0.2, 0.3],
        sample={
            "actual_xyz_m": [0.0, 0.0, 0.0],
            "tier_m": None,
            "actual_view_distance_m": 0.5,
        },
        settle={
            "position_error_m": 0.0,
            "orientation_error_rad": 0.0,
            "wait_ns": 0,
        },
    )

    record = json.loads(policy.samples_path.read_text(encoding="utf-8"))
    assert saved
    assert "visible=['left', 'center']" in detail
    assert "robot_arm_occlusion=['right']" in detail
    assert "'left': {'visible': True, 'reason': 'visible'}" in detail
    assert "'center': {'visible': True, 'reason': 'visible'}" in detail
    assert "'right': {'visible': False, 'reason': 'robot_arm_occlusion'}" in detail
    assert record["id"] == "20260811-002131_t0000_sfp-r0-p0_s000"
    assert record["images"]["center"] == (
        "images/train/center/trial_000/"
        "sfp_card_10100_rail0_port0_num001_center.jpg"
    )
    assert (tmp_path / record["images"]["center"]).is_file()
    assert set(record["images"]) == {"left", "center", "right"}
    assert record["target_xyz_m"] == [0.1, 0.2, 0.3]
    assert record["settle_orientation_error_rad"] == 0.0
    assert "settle_orientation_error_deg" not in record
    assert "seed" not in record
    assert "trials" not in record


def test_compact_capture_id_for_sc():
    capture_id = dataset._compact_capture_id(
        SimpleNamespace(trial_index=12),
        "20260811_003000_portoffset_sc_0012_cards10_rail1",
        SimpleNamespace(
            id="portoffset_sc_0012_cards10_rail1",
            port_type="sc",
            port_name="sc_port_base",
            target_module_name="sc_port_1",
        ),
        7,
    )

    assert capture_id == "20260811-003000_t0012_sc-r1_s007"


def test_image_relative_path_for_sc():
    path = dataset._image_relative_path(
        SimpleNamespace(trial_index=12),
        SimpleNamespace(
            id="portoffset_sc_0012_cards10_rail1",
            port_type="sc",
            port_name="sc_port_base",
            target_module_name="sc_port_1",
        ),
        7,
        "val",
        "left",
    )

    assert str(path) == (
        "images/val/left/trial_012/sc_card_01_rail1_num008_left.jpg"
    )


def test_card_mask_reverses_to_rail_index_order():
    task = SimpleNamespace(id="portoffset_sfp_0000_cards10100_rail4_sfp_port_0")

    assert dataset._card_mask(task) == "00101"


def test_port_outer_corners_apply_port_yaw():
    transform = Transform()
    transform.translation.x = 1.0
    transform.translation.y = 2.0
    transform.translation.z = 3.0
    transform.rotation.z = np.sin(np.pi / 4.0)
    transform.rotation.w = np.cos(np.pi / 4.0)

    corners = np.asarray(dataset._port_outer_corners_base("sfp", transform))
    width, height = dataset.PORT_OUTER_SIZE_M["sfp"]

    assert np.allclose(corners[0], [1.0 - height / 2.0, 2.0 - width / 2.0, 3.0])
    assert np.allclose(corners[2], [1.0 + height / 2.0, 2.0 + width / 2.0, 3.0])


def test_yolo_pose_row_contains_bbox_and_four_outer_keypoints():
    row = dataset._yolo_pose_row(
        0,
        {
            "image_size_px": [200, 100],
            "points": [
                {"u_px": 20.0, "v_px": 10.0},
                {"u_px": 100.0, "v_px": 10.0},
                {"u_px": 100.0, "v_px": 50.0},
                {"u_px": 20.0, "v_px": 50.0},
            ],
        },
    )
    values = row.split()

    assert len(values) == 17
    assert values[0] == "0"
    assert np.allclose([float(value) for value in values[1:5]], [0.3, 0.3, 0.4, 0.4])
    assert [int(values[index]) for index in (7, 10, 13, 16)] == [2, 2, 2, 2]


def test_save_sample_writes_yolo_pose_annotations(monkeypatch, tmp_path):
    monkeypatch.setattr(
        dataset,
        "_port_projection",
        lambda *args, **kwargs: {"visible": True},
    )
    monkeypatch.setattr(
        dataset,
        "_port_outer_projection",
        lambda *args, **kwargs: {
            "visible": True,
            "image_size_px": [100, 100],
            "points": [
                {"u_px": 10.0, "v_px": 20.0},
                {"u_px": 30.0, "v_px": 20.0},
                {"u_px": 30.0, "v_px": 40.0},
                {"u_px": 10.0, "v_px": 40.0},
            ],
        },
    )
    monkeypatch.setattr(
        dataset,
        "image_to_bgr",
        lambda *args, **kwargs: np.zeros((2, 2, 3), dtype=np.uint8),
    )
    policy = SimpleNamespace(
        min_visible_cameras=2,
        auto_annotate_ports=True,
        dataset_dir=tmp_path,
        samples_path=tmp_path / "samples.jsonl",
        run_id="",
        trial_index=0,
        trial_split="train",
        val_ratio=0.0,
        test_ratio=0.0,
        collection_policy="near-port",
        capture_count=0,
    )
    observation = SimpleNamespace(
        left_image=object(), center_image=object(), right_image=object()
    )
    task = SimpleNamespace(
        id="portoffset_sfp_0000_cards00001_rail0_sfp_port_0",
        port_type="sfp",
        port_name="sfp_port_0",
        target_module_name="nic_card_mount_0",
    )

    saved, _ = dataset.save_sample(
        policy,
        episode_name="20260811_120000_portoffset_sfp_0000_cards00001_rail0_sfp_port_0",
        task=task,
        step_idx=0,
        observation=observation,
        port_tf=Transform(),
        timestamps={"sync_valid": True, "capture_stamp_ns": 1},
        label_xyz=[0.0, 0.0, 0.0],
        sample={
            "actual_xyz_m": [0.0, 0.0, 0.0],
            "tier_m": None,
            "actual_view_distance_m": 0.5,
        },
        settle={
            "position_error_m": 0.0,
            "orientation_error_rad": 0.0,
            "wait_ns": 0,
        },
        annotation_ports=[
            {
                "class_id": 9,
                "label": "SFP_41",
                "port_type": "sfp",
                "transform": Transform(),
            }
        ],
    )

    record = json.loads(policy.samples_path.read_text(encoding="utf-8"))
    center_annotation = tmp_path / record["annotations"]["center"]
    assert saved
    assert center_annotation.is_file()
    assert len(center_annotation.read_text(encoding="utf-8").split()) == 17
    assert center_annotation.read_text(encoding="utf-8").startswith("9 ")
    assert record["annotation_format"] == "yolo_pose"
    assert record["annotation_object_counts"] == {"left": 1, "center": 1, "right": 1}
    assert record["annotation_labels"] == {
        "left": ["SFP_41"],
        "center": ["SFP_41"],
        "right": ["SFP_41"],
    }
    dataset._write_yolo_pose_config(policy)
    assert (tmp_path / "labels").is_symlink()
    assert str((tmp_path / "labels").readlink()) == "annotations"
    assert "kpt_shape: [4, 3]" in (tmp_path / "yolo_pose.yaml").read_text(
        encoding="utf-8"
    )
    assert "9: SFP_41" in (tmp_path / "yolo_pose.yaml").read_text(
        encoding="utf-8"
    )


def test_annotation_port_frames_follow_active_card_mask():
    task = SimpleNamespace(
        id="portoffset_sfp_0000_cards10100_rail4_sfp_port_0",
        port_type="sfp",
        port_name="sfp_port_0",
        target_module_name="nic_card_mount_4",
    )

    ports = PortOffsetCollect._annotation_port_frames(
        object(), task, "unused_fallback"
    )

    assert [port["instance_id"] for port in ports] == [
        "nic_card_mount_2/sfp_port_0",
        "nic_card_mount_2/sfp_port_1",
        "nic_card_mount_4/sfp_port_0",
        "nic_card_mount_4/sfp_port_1",
    ]
    assert [port["class_id"] for port in ports] == [4, 5, 8, 9]
    assert [port["label"] for port in ports] == [
        "SFP_20",
        "SFP_21",
        "SFP_40",
        "SFP_41",
    ]


def test_sfp_annotation_class_uses_zero_based_rail_and_port():
    assert dataset.port_annotation_class("sfp", 0, 0) == (0, "SFP_00")
    assert dataset.port_annotation_class("sfp", 4, 1) == (9, "SFP_41")
