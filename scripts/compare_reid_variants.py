#!/usr/bin/env python3
"""네 REMIND ReID 조건의 tracking_eval.json을 같은 채택 규칙으로 비교한다."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


VARIANTS = (
    "efficientnet",
    "dino_no_attn",
    "dino_keypoint_attn",
    "dino_hull_attn",
)


def wilson_interval(successes: int, attempts: int) -> tuple[float, float]:
    """Binomial 성공률의 95% Wilson confidence interval을 반환한다."""
    if attempts <= 0:
        return 0.0, 1.0
    z = 1.959963984540054
    p = successes / attempts
    denominator = 1.0 + z * z / attempts
    center = (p + z * z / (2.0 * attempts)) / denominator
    radius = z * math.sqrt(
        p * (1.0 - p) / attempts + z * z / (4.0 * attempts * attempts)
    ) / denominator
    return max(0.0, center - radius), min(1.0, center + radius)


def _percentile(rows: list[dict], key: str, percentile: float) -> float | None:
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    return float(np.percentile(values, percentile)) if values else None


def _frame_latency(rows: list[dict], percentile: float) -> float | None:
    """전체 frame loop 시간이 없으면 구형 pipeline 시간으로 fallback한다."""
    loop_latency = _percentile(rows, "loop_ms", percentile)
    return (
        loop_latency
        if loop_latency is not None
        else _percentile(rows, "pipeline_ms", percentile)
    )


def load_metrics(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    summary = payload.get("summary", {}) or {}
    identity = payload.get("collapsed_identity_metrics", {}) or {}
    attempts = int(summary.get("recovery_attempts_total", 0) or 0)
    successes = int(summary.get("recovery_success_reference_total", 0) or 0)
    foreign = int(summary.get("recovery_success_foreign_id_total", 0) or 0)
    low, high = wilson_interval(successes, attempts)
    per_frame = payload.get("per_frame", []) or []
    return {
        "path": str(path),
        "attempts": attempts,
        "successes": successes,
        "accuracy": successes / attempts if attempts else 0.0,
        "accuracy_ci95": [low, high],
        "foreign_rate": foreign / attempts if attempts else 0.0,
        "idsw": int(identity.get("idsw", summary.get("idsw", 0)) or 0),
        "frame_p50_ms": _frame_latency(per_frame, 50),
        "frame_p95_ms": _frame_latency(per_frame, 95),
        "peak_gpu_memory_bytes": summary.get(
            "mem_gpu_peak_allocated_bytes_max"
        ),
    }


def compare(metrics: dict[str, dict], perception_period_ms: float | None) -> dict:
    baseline = metrics["efficientnet"]
    for name, row in metrics.items():
        if name == "efficientnet":
            row["relative_accuracy_gain"] = 0.0
            row["decision"] = "baseline"
            continue
        base_accuracy = float(baseline["accuracy"])
        relative_gain = (
            (float(row["accuracy"]) - base_accuracy) / base_accuracy
            if base_accuracy > 0.0
            else float("inf") if float(row["accuracy"]) > 0.0 else 0.0
        )
        row["relative_accuracy_gain"] = relative_gain
        latency_ok = (
            perception_period_ms is None
            or row["frame_p95_ms"] is not None
            and float(row["frame_p95_ms"]) < perception_period_ms
        )
        foreign_ok = float(row["foreign_rate"]) <= float(baseline["foreign_rate"])
        idsw_ok = int(row["idsw"]) <= int(baseline["idsw"])
        intervals_separated = float(row["accuracy_ci95"][0]) > float(
            baseline["accuracy_ci95"][1]
        )
        if relative_gain < 0.05 or not foreign_ok or not idsw_ok or not latency_ok:
            decision = "reject"
        elif not intervals_separated:
            decision = "inconclusive"
        else:
            decision = "eligible"
        row["decision"] = decision
        row["checks"] = {
            "relative_accuracy_gain_at_least_5pct": relative_gain >= 0.05,
            "foreign_rate_not_worse": foreign_ok,
            "idsw_not_worse": idsw_ok,
            "frame_p95_below_period": latency_ok,
            "ci95_separated_from_efficientnet": intervals_separated,
        }
    eligible = [
        name
        for name, row in metrics.items()
        if row.get("decision") == "eligible"
    ]
    selected = min(
        eligible,
        key=lambda name: float(metrics[name]["frame_p95_ms"] or float("inf")),
        default=None,
    )
    return {
        "perception_period_ms": perception_period_ms,
        "variants": metrics,
        "selected": selected,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    for name in VARIANTS:
        parser.add_argument(f"--{name.replace('_', '-')}", type=Path, required=True)
    parser.add_argument("--perception-period-ms", type=float)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metrics = {
        name: load_metrics(getattr(args, name))
        for name in VARIANTS
    }
    result = compare(metrics, args.perception_period_ms)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
