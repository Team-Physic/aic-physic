from phy_data_collection.camera_event_validation import (
    SIMULATION_CLOCK,
    SourceImageEvent,
    image_event_id,
    summarize_record_delays,
    validate_image_event,
)


def _event(
    *,
    stamp_ns: int = 1_000_000_000,
    record_stamp_ns: int = 1_012_000_000,
    jpeg: bytes = b"jpeg-frame",
) -> SourceImageEvent:
    return SourceImageEvent(
        topic="/center_camera/image",
        header_stamp_ns=stamp_ns,
        record_stamp_ns=record_stamp_ns,
        jpeg=jpeg,
    )


def test_matching_source_event_preserves_stamp_content_and_delay() -> None:
    result = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event()],
        record_clock=SIMULATION_CLOCK,
        max_record_delay_ns=20_000_000,
    )

    assert result["status"] == "PASS"
    assert result["event_id"] == image_event_id(
        "center", 1_000_000_000, b"jpeg-frame"
    )
    assert result["event_identity_matches"] is True
    assert result["source_stamp_matches_dataset"] is True
    assert result["jpeg_matches_source"] is True
    assert result["record_delay_ns"] == 12_000_000


def test_same_stamp_with_different_image_fails() -> None:
    result = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"different-frame",
        source_events=[_event()],
        record_clock=SIMULATION_CLOCK,
    )

    assert result["status"] == "FAIL"
    assert result["source_stamp_matches_dataset"] is True
    assert result["jpeg_matches_source"] is False
    assert "dataset JPEG content differs" in result["errors"][0]


def test_missing_source_stamp_fails() -> None:
    result = validate_image_event(
        camera="center",
        dataset_stamp_ns=2_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event()],
        record_clock=SIMULATION_CLOCK,
    )

    assert result["status"] == "FAIL"
    assert result["source_event_count"] == 0
    assert result["record_delay_ns"] is None


def test_conflicting_source_events_with_same_stamp_fail() -> None:
    result = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event(), _event(jpeg=b"conflicting-frame")],
        record_clock=SIMULATION_CLOCK,
    )

    assert result["status"] == "FAIL"
    assert result["source_event_count"] == 2
    assert "conflicting source image events" in result["errors"][0]


def test_unconfirmed_record_clock_does_not_report_delay() -> None:
    result = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event(record_stamp_ns=1_700_000_000_000_000_000)],
        record_clock="unknown",
    )

    assert result["status"] == "PASS"
    assert result["record_delay_ns"] is None
    assert result["warnings"] == []


def test_delay_threshold_and_negative_delay_fail() -> None:
    over_limit = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event(record_stamp_ns=1_021_000_000)],
        record_clock=SIMULATION_CLOCK,
        max_record_delay_ns=20_000_000,
    )
    negative = validate_image_event(
        camera="center",
        dataset_stamp_ns=1_000_000_000,
        dataset_jpeg=b"jpeg-frame",
        source_events=[_event(record_stamp_ns=999_000_000)],
        record_clock=SIMULATION_CLOCK,
    )

    assert over_limit["status"] == "FAIL"
    assert "exceeds tolerance" in over_limit["errors"][0]
    assert negative["status"] == "FAIL"
    assert "precedes" in negative["errors"][0]


def test_record_delay_summary_uses_matching_nonnegative_events_only() -> None:
    results = [
        {"event_identity_matches": True, "record_delay_ns": 10_000_000},
        {"event_identity_matches": True, "record_delay_ns": 20_000_000},
        {"event_identity_matches": False, "record_delay_ns": 999_000_000},
        {"event_identity_matches": True, "record_delay_ns": None},
        {"event_identity_matches": True, "record_delay_ns": -1},
    ]

    summary = summarize_record_delays(results)

    assert summary["available"] is True
    assert summary["sample_count"] == 2
    assert summary["mean_ms"] == 15.0
    assert summary["max_ms"] == 20.0
