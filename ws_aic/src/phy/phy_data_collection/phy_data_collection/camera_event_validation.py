"""Simulator camera source event와 저장 JPEG의 동일성을 검증한다."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Any, Iterable


SIMULATION_CLOCK = "ros_sim"


@dataclass(frozen=True)
class SourceImageEvent:
    """MCAP에 기록된 한 camera source image event."""

    topic: str
    header_stamp_ns: int
    record_stamp_ns: int
    jpeg: bytes

    @property
    def jpeg_sha256(self) -> str:
        """수집기와 같은 방식으로 encoding한 source JPEG fingerprint를 반환한다."""
        return hashlib.sha256(self.jpeg).hexdigest()


def image_event_id(camera: str, stamp_ns: int, jpeg: bytes) -> str:
    """camera, source stamp, image content로 재현 가능한 event ID를 만든다."""
    digest = hashlib.sha256()
    digest.update(camera.encode("utf-8"))
    digest.update(b"\0")
    digest.update(str(int(stamp_ns)).encode("ascii"))
    digest.update(b"\0")
    digest.update(jpeg)
    return digest.hexdigest()


def validate_image_event(
    *,
    camera: str,
    dataset_stamp_ns: int,
    dataset_jpeg: bytes,
    source_events: Iterable[SourceImageEvent],
    record_clock: str,
    max_record_delay_ns: int | None = None,
) -> dict[str, Any]:
    """source frame과 dataset frame이 같은 촬영 event인지 판정한다."""
    candidates = [
        event
        for event in source_events
        if event.header_stamp_ns == int(dataset_stamp_ns)
    ]
    matching = [event for event in candidates if event.jpeg == dataset_jpeg]
    errors: list[str] = []
    warnings: list[str] = []

    if not candidates:
        errors.append("source image event with dataset timestamp is missing")
    elif not matching:
        errors.append("dataset JPEG content differs from source image event")
    elif len(candidates) > 1:
        if len({event.jpeg for event in candidates}) > 1:
            errors.append("conflicting source image events share the timestamp")
        else:
            warnings.append(
                "multiple identical source image events share the timestamp"
            )

    source = (
        matching[0] if matching else candidates[0] if candidates else None
    )
    dataset_sha256 = (
        hashlib.sha256(dataset_jpeg).hexdigest() if dataset_jpeg else ""
    )
    dataset_event_id = (
        image_event_id(camera, dataset_stamp_ns, dataset_jpeg)
        if dataset_jpeg
        else ""
    )
    source_event_id = (
        image_event_id(camera, source.header_stamp_ns, source.jpeg)
        if source is not None
        else ""
    )
    event_identity_matches = bool(
        matching and source_event_id == dataset_event_id
    )
    record_delay_ns: int | None = None
    if source is not None and record_clock == SIMULATION_CLOCK:
        record_delay_ns = (
            int(source.record_stamp_ns) - int(source.header_stamp_ns)
        )
        if record_delay_ns < 0:
            errors.append(
                "MCAP record timestamp precedes source image timestamp"
            )
        elif (
            max_record_delay_ns is not None
            and record_delay_ns > int(max_record_delay_ns)
        ):
            errors.append("source-to-MCAP record delay exceeds tolerance")

    return {
        "event_id": dataset_event_id if event_identity_matches else "",
        "source_event_id": source_event_id,
        "dataset_event_id": dataset_event_id,
        "event_identity_matches": event_identity_matches,
        "status": "PASS" if not errors else "FAIL",
        "camera": camera,
        "source_topic": source.topic if source is not None else "",
        "source_event_count": len(candidates),
        "source_header_stamp_ns": (
            source.header_stamp_ns if source is not None else None
        ),
        "dataset_stamp_ns": int(dataset_stamp_ns),
        "source_stamp_matches_dataset": bool(
            source is not None and source.header_stamp_ns == int(dataset_stamp_ns)
        ),
        "source_jpeg_sha256": (
            source.jpeg_sha256 if source is not None else ""
        ),
        "dataset_jpeg_sha256": dataset_sha256,
        "jpeg_matches_source": bool(matching),
        "record_clock": record_clock,
        "record_stamp_ns": source.record_stamp_ns if source is not None else None,
        "record_delay_ns": record_delay_ns,
        "record_delay_ms": (
            record_delay_ns / 1_000_000.0 if record_delay_ns is not None else None
        ),
        "max_record_delay_ns": max_record_delay_ns,
        "max_record_delay_ms": (
            max_record_delay_ns / 1_000_000.0
            if max_record_delay_ns is not None
            else None
        ),
        "errors": errors,
        "warnings": warnings,
    }


def summarize_record_delays(events: Iterable[dict[str, Any]]) -> dict[str, Any]:
    """검증된 event들의 source-to-MCAP 기록 지연 분포를 요약한다."""
    values = sorted(
        int(event["record_delay_ns"])
        for event in events
        if event.get("record_delay_ns") is not None
        and event.get("event_identity_matches")
        and int(event["record_delay_ns"]) >= 0
    )
    if not values:
        return {"available": False, "sample_count": 0}

    def percentile(fraction: float) -> int:
        index = min(len(values) - 1, int((len(values) - 1) * fraction))
        return values[index]

    mean_ns = sum(values) / len(values)
    return {
        "available": True,
        "sample_count": len(values),
        "min_ns": values[0],
        "mean_ns": mean_ns,
        "p50_ns": percentile(0.50),
        "p95_ns": percentile(0.95),
        "p99_ns": percentile(0.99),
        "max_ns": values[-1],
        "min_ms": values[0] / 1_000_000.0,
        "mean_ms": mean_ns / 1_000_000.0,
        "p50_ms": percentile(0.50) / 1_000_000.0,
        "p95_ms": percentile(0.95) / 1_000_000.0,
        "p99_ms": percentile(0.99) / 1_000_000.0,
        "max_ms": values[-1] / 1_000_000.0,
    }
