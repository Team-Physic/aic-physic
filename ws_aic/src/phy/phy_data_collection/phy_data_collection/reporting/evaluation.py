"""img2pos dataset 분포와 모델 예측 품질을 mm 단위로 평가한다."""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    """빈 줄을 허용하며 JSONL 객체를 읽는다."""
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{line_number}: expected JSON object")
            rows.append(value)
    return rows


def _vector(row: dict[str, Any], field: str) -> tuple[float, float, float]:
    """지정 필드를 유한한 XYZ meter vector로 검증한다."""
    values = row.get(field)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{row.get('id', '<unknown>')}: invalid {field}")
    vector = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{row.get('id', '<unknown>')}: non-finite {field}")
    return vector


def _actual_sampling_tier_label(row: dict[str, Any]) -> str:
    """신규·legacy row의 촬영 시점 offset을 실제 near-port tier로 분류한다."""
    if row.get("collection_policy", "near-port") != "near-port":
        return "not_applicable"
    if "actual_sampling_tier_mm" in row:
        actual_tier_mm = row["actual_sampling_tier_mm"]
        return "out_of_range" if actual_tier_mm is None else str(actual_tier_mm)

    offset_field = (
        "sampling_offset_xyz_m"
        if "sampling_offset_xyz_m" in row
        else "target_xyz_m"
    )
    extent_mm = max(abs(value) for value in _vector(row, offset_field)) * 1000.0
    return next(
        (
            str(float(threshold_mm))
            for threshold_mm in (2, 5, 10, 50)
            if extent_mm <= threshold_mm
        ),
        "out_of_range",
    )


def _percentile(values: list[float], percentile: float) -> float:
    """선형 보간 percentile을 반환한다."""
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return ordered[low]
    fraction = position - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def summarize_dataset(dataset_dir: Path) -> dict[str, Any]:
    """capture 독립성을 보존해 split과 XYZ label 분포를 요약한다."""
    samples_path = dataset_dir / "samples.jsonl"
    rows = _read_jsonl(samples_path)
    captures: dict[str, dict[str, Any]] = {}
    trial_splits: dict[str, set[str]] = defaultdict(set)
    image_counts_by_split: Counter[str] = Counter()
    image_counts_by_camera: Counter[str] = Counter()
    for row in rows:
        capture_id = str(row["id"])
        if capture_id in captures:
            raise ValueError(f"duplicate capture row: {capture_id}")
        images = row.get("images")
        if not isinstance(images, dict) or not images:
            raise ValueError(f"{capture_id}: invalid images mapping")
        if set(images) != {"left", "center", "right"}:
            raise ValueError(f"{capture_id}: synchronized camera triplet is incomplete")
        captures[capture_id] = row
        image_counts_by_split[str(row["split"])] += len(images)
        image_counts_by_camera.update(str(camera) for camera in images)
        trial_splits[str(row["trial_id"])].add(str(row["split"]))

    labels = [_vector(row, "target_xyz_m") for row in captures.values()]
    near_port_captures = [
        row
        for row in captures.values()
        if row.get("collection_policy", "near-port") == "near-port"
    ]
    sampling_offsets = [
        _vector(
            row,
            "sampling_offset_xyz_m"
            if "sampling_offset_xyz_m" in row
            else "target_xyz_m",
        )
        for row in near_port_captures
    ]
    norms_mm = [math.sqrt(sum(value * value for value in label)) * 1000.0 for label in labels]
    axis_mm = [[label[index] * 1000.0 for label in labels] for index in range(3)]
    box_coverage = {
        str(threshold): {
            "captures": sum(
                all(abs(value * 1000.0) <= threshold for value in label)
                for label in sampling_offsets
            ),
            "ratio": (
                sum(
                    all(abs(value * 1000.0) <= threshold for value in label)
                    for label in sampling_offsets
                )
                / len(sampling_offsets)
                if sampling_offsets
                else 0.0
            ),
        }
        for threshold in (2, 5, 10, 50)
    }
    summary = {
        "capture_rows": len(rows),
        "images": sum(image_counts_by_camera.values()),
        "captures": len(captures),
        "trials": len(trial_splits),
        "trial_split_leaks": sum(len(splits) != 1 for splits in trial_splits.values()),
        "images_by_split": dict(image_counts_by_split),
        "captures_by_split": dict(Counter(str(row["split"]) for row in captures.values())),
        "images_by_camera": dict(image_counts_by_camera),
        "captures_by_connector": dict(
            Counter(str(row["connector"]) for row in captures.values())
        ),
        "captures_by_collection_policy": dict(
            Counter(str(row.get("collection_policy", "unknown")) for row in captures.values())
        ),
        "captures_by_sampling_tier_mm": dict(
            Counter(
                "not_applicable"
                if row.get("sampling_tier_mm") is None
                else str(row["sampling_tier_mm"])
                for row in captures.values()
            )
        ),
        "captures_by_planned_sampling_tier_mm": dict(
            Counter(
                "not_applicable"
                if row.get("planned_sampling_tier_mm", row.get("sampling_tier_mm"))
                is None
                else str(
                    row.get("planned_sampling_tier_mm", row.get("sampling_tier_mm"))
                )
                for row in captures.values()
            )
        ),
        "captures_by_actual_sampling_tier_mm": dict(
            Counter(
                _actual_sampling_tier_label(row)
                for row in captures.values()
            )
        ),
        "target_xyz_mm": {
            axis: {
                "min": min(values, default=0.0),
                "mean": sum(values) / len(values) if values else 0.0,
                "max": max(values, default=0.0),
            }
            for axis, values in zip(("x", "y", "z"), axis_mm, strict=True)
        },
        "near_port_sampling_offset_box_coverage_mm": box_coverage,
        "target_3d_norm_mm": {
            "p95": _percentile(norms_mm, 95.0),
        },
    }
    return summary


def _prediction_metrics(
    samples: list[dict[str, Any]],
    predictions: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    """XYZ MAE, 3D p95와 5mm 이내 비율을 계산한다."""
    errors: list[tuple[float, float, float]] = []
    norms: list[float] = []
    for sample in samples:
        prediction = predictions.get(str(sample["id"]))
        if prediction is None:
            continue
        target = _vector(sample, "target_xyz_m")
        predicted = _vector(prediction, "predicted_xyz_m")
        error = tuple((predicted[index] - target[index]) * 1000.0 for index in range(3))
        errors.append(error)
        norms.append(math.sqrt(sum(value * value for value in error)))
    count = len(errors)
    return {
        "total_samples": len(samples),
        "matched_samples": count,
        "prediction_coverage": count / len(samples) if samples else 0.0,
        "xyz_mae_mm": [
            sum(abs(error[index]) for error in errors) / count if count else 0.0
            for index in range(3)
        ],
        "error_3d_p95_mm": _percentile(norms, 95.0),
        "within_5mm_ratio": sum(value <= 5.0 for value in norms) / count if count else 0.0,
    }


def evaluate_predictions(
    dataset_dir: Path,
    predictions_path: Path,
    insertion_results_path: Path | None = None,
) -> dict[str, Any]:
    """전체 및 split별 예측 오차와 선택적 closed-loop 성공률을 반환한다."""
    samples = _read_jsonl(dataset_dir / "samples.jsonl")
    predictions = {str(row["id"]): row for row in _read_jsonl(predictions_path)}
    report = {
        "all": _prediction_metrics(samples, predictions),
        "by_split": {
            split: _prediction_metrics(
                [row for row in samples if row.get("split") == split], predictions
            )
            for split in ("train", "val", "test")
        },
    }
    if insertion_results_path is not None:
        results = _read_jsonl(insertion_results_path)
        invalid = [
            row.get("id", "<unknown>")
            for row in results
            if not isinstance(row.get("success"), bool)
        ]
        if invalid:
            raise ValueError(f"insertion result success must be boolean: {invalid[0]}")
        report["closed_loop"] = {
            "trials": len(results),
            "insertion_success_rate": (
                sum(row["success"] for row in results) / len(results)
                if results
                else 0.0
            ),
        }
    return report


def main() -> int:
    """dataset 요약 또는 prediction 평가 CLI를 실행한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--predictions", type=Path)
    parser.add_argument("--insertion-results", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"dataset": summarize_dataset(args.dataset_dir)}
    if args.predictions is not None:
        report["model"] = evaluate_predictions(
            args.dataset_dir, args.predictions, args.insertion_results
        )
    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
