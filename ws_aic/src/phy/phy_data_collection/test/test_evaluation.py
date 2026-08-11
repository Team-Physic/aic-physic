from __future__ import annotations

import json

from phy_data_collection.reporting.evaluation import summarize_dataset


def test_summary_separates_planned_and_actual_sampling_tiers(tmp_path) -> None:
    rows = [
        {
            "id": "legacy-near",
            "trial_id": "trial-1",
            "split": "train",
            "images": {"left": "l.jpg", "center": "c.jpg", "right": "r.jpg"},
            "connector": "sfp",
            "collection_policy": "near-port",
            "target_xyz_m": [0.0, 0.0, 0.02],
            "sampling_offset_xyz_m": [0.006, 0.0, 0.0],
            "sampling_tier_mm": 5.0,
        },
        {
            "id": "new-near",
            "trial_id": "trial-2",
            "split": "validation",
            "images": {"left": "l.jpg", "center": "c.jpg", "right": "r.jpg"},
            "connector": "sfp",
            "collection_policy": "near-port",
            "target_xyz_m": [0.0, 0.0, 0.02],
            "sampling_offset_xyz_m": [0.006, 0.0, 0.0],
            "sampling_tier_mm": 5.0,
            "planned_sampling_tier_mm": 5.0,
            "actual_sampling_tier_mm": 10.0,
        },
        {
            "id": "board-view",
            "trial_id": "trial-3",
            "split": "test",
            "images": {"left": "l.jpg", "center": "c.jpg", "right": "r.jpg"},
            "connector": "sfp",
            "collection_policy": "board-view",
            "target_xyz_m": [0.0, 0.0, 0.4],
            "sampling_offset_xyz_m": [0.0, 0.0, 0.4],
            "sampling_tier_mm": None,
            "planned_sampling_tier_mm": None,
            "actual_sampling_tier_mm": None,
        },
    ]
    (tmp_path / "samples.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )

    summary = summarize_dataset(tmp_path)

    assert summary["captures_by_planned_sampling_tier_mm"] == {
        "5.0": 2,
        "not_applicable": 1,
    }
    assert summary["captures_by_actual_sampling_tier_mm"] == {
        "10.0": 2,
        "not_applicable": 1,
    }
