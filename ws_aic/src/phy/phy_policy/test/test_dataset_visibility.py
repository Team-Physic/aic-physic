import json
from inspect import signature
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Transform

from phy_policy.data_generator import dataset


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


def test_save_sample_writes_all_cameras_after_visibility_gate(monkeypatch, tmp_path):
    monkeypatch.setattr(
        dataset,
        "_port_projection",
        lambda policy, observation, camera, port_tf: {
            "visible": camera in {"left", "center"}
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

    saved, _ = dataset.save_sample(
        policy,
        episode_name="trial",
        task=SimpleNamespace(port_type="sfp"),
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
    assert set(record["images"]) == {"left", "center", "right"}
    assert record["target_xyz_m"] == [0.1, 0.2, 0.3]
    assert "seed" not in record
    assert "trials" not in record
