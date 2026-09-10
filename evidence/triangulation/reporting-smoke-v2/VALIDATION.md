# Partial/final reporting validation

2026-09-10. Actual archived Gazebo capture: `../logging-check-v2`, job 268,
1 card, SFP_00, distance 0.45 m, angle 0 deg, light_scale 1.
This is reporting validation, not the remote 2,700-case accuracy evaluation.

## Reproduce

From repository root, choose a new output directory:

```bash
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/test_triangulation_protocol.py
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/test_triangulation_report.py
PYTHONDONTWRITEBYTECODE=1 ws_aic/src/.pixi/envs/default/bin/python3 \
  scripts/evaluate_triangulation_protocol.py \
  --capture evidence/triangulation/logging-check-v2 \
  --output /tmp/triangulation-report-reproduction --device cpu
# Repeat the evaluation command with --resume to reuse the saved case.
```

## Verified

- Existing protocol checks passed: layout balance, seed, geometry, visibility,
  capture resume completeness, denominator and persisted acquisition event log.
- Reporting tests passed: actual process SIGINT after first committed case;
  second case remains pending; resumed inference skips first case;
  second resume performs no inference; changed input triggers re-evaluation.
- Interrupt before model creation leaves a zero-completed partial report.
- Injected inference exception leaves a failed report and propagates the error.
- Missing evaluation manifest and unknown frozen source hash are rejected.
- Latest incomplete attempt supersedes stale complete.json.
- Randomized combination totals match selected jobs and planned unique layouts.
- Tests replace model inference with deterministic values; these are not accuracy evidence.
  Temporary case/report files are automatically removed by TemporaryDirectory.
- Actual CPU inference completed, then actual CLI --resume completed without case inference.
- Actual result: 1 eligible/evaluated case, 0 position estimates;
  `fewer_than_two_detections: 1`; YOLO median/P95/max are null, not zero.
- GT-corner calculation: 1 estimate, 1.7554167342883506e-13 mm error.
  This checks ideal projection arithmetic only, not YOLO accuracy.
- overview.png visually inspected: labels fit, missing YOLO estimates display N/A,
  GT calculation appears on its own axis. report.html embeds the image.

The previous frozen evaluator is permitted only through the recorded reporting upgrade;
its evaluate_case function is unchanged. Other frozen source hashes remain enforced.
No Gazebo acquisition, policy, portfolio, or unrelated evidence was changed.
