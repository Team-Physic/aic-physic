import argparse
from pathlib import Path

import pytest

from phy_data_collection.portoffset_randomization import runtime
from phy_data_collection.portoffset_validation import (
    read_rosbag_record_clock,
    validate_trial,
)


def test_start_rosbag_records_with_simulation_clock(tmp_path, monkeypatch) -> None:
    captured = {}
    fake_process = object()

    def fake_start(cmd, **kwargs):
        captured["cmd"] = cmd
        captured["kwargs"] = kwargs
        return fake_process

    monkeypatch.setattr(runtime, "_start_logged_process", fake_start)
    args = argparse.Namespace(
        rosbag_topics=("/clock", "/center_camera/image"),
        color_log=False,
    )

    session = runtime.start_rosbag(
        args,
        output_dir=tmp_path / "trial",
        run_id="run-1",
    )

    assert session.proc is fake_process
    assert "--use-sim-time" in captured["cmd"]
    custom_data_index = captured["cmd"].index("--custom-data")
    assert captured["cmd"][custom_data_index + 1] == (
        "phy_event_timestamp_clock=ros_sim"
    )
    assert captured["cmd"][-2:] == ["/clock", "/center_camera/image"]


def test_read_rosbag_record_clock_from_metadata(tmp_path: Path) -> None:
    (tmp_path / "metadata.yaml").write_text(
        """
rosbag2_bagfile_information:
  custom_data:
    phy_event_timestamp_clock: ros_sim
""".lstrip(),
        encoding="utf-8",
    )

    assert read_rosbag_record_clock(tmp_path) == "ros_sim"


def test_read_rosbag_record_clock_is_unknown_for_legacy_bag(tmp_path: Path) -> None:
    (tmp_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n",
        encoding="utf-8",
    )

    assert read_rosbag_record_clock(tmp_path) == "unknown"


def test_validate_trial_rejects_invalid_delay_limit(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="finite and non-negative"):
        validate_trial(tmp_path, tmp_path, max_record_delay_ms=float("nan"))
