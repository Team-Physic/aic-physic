# Logging validation — 2026-09-10

- Final source: protocol-logging-check-v2/plan.json; source hashes verified after run.
- Command: run --renderer gpu --smoke --timeout-s 20.
- Actual GPU: NVIDIA GeForce GTX 1050, 4096 MiB; container GPU access confirmed.
- Job 268, layout_000, SFP_00, one card, light_scale=1.0, distance=0.45 m, angle=0 degrees.
- Result: captured=1; runner return code 0. protocol.log includes every case stage before completion.
- Unit checks: 100 layouts/2700 cases, geometry, visibility, resume, environment, log persistence,
  and actual case-loop failure logging with mocked robot all passed.
- Earlier logging-check-v1 is a development failure run (duplicate target keyword); fixed and covered by regression test.
- This is a logging smoke, not a full 2700-case accuracy validation or YOLO inference evaluation.

## Outstanding observations

- Existing gpu-gui-smoke/job_000/complete.json: 3 captured, 6 pose_not_reached.
- Existing gpu-full/job_000: interrupted, return code 130; not a completed evaluation.
- Cable spawn and shutdown errors appear in simulator logs, including transient spawn errors in the successful smoke.
  Their root causes are not resolved by this logging change. Return code 0 does not mean every observation succeeds.
