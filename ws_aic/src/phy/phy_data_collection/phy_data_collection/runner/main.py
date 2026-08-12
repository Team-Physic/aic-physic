#!/usr/bin/env python3
"""무작위 PortOffset trial을 준비하고 ROS 2/Gazebo lifecycle을 조율한다."""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
import random
import re
import secrets
import signal
import socket
import sys
import time
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from dataclasses import dataclass, field
from pathlib import Path
from queue import Empty
from typing import Any

from phy_data_collection.runner.cli import parse_args
from phy_data_collection.runner.constants import (
    COLLECTION_LOG_ROOT,
    CONFIG_DIR,
    DESCENT_MIN_CLEARANCE_MM,
    MIN_CLEARANCE_MM,
    POLICY_MODULE,
    REGISTRY_FILENAME,
    TRIAL_TIMEOUT_GRACE_S,
)
from phy_data_collection.reporting.evaluation import summarize_dataset
from phy_data_collection.runner.lifecycle import (
    OwnedProcessGroup,
    cleanup_stale_processes,
    process_groups_with_cmdline_marker,
    register_owned_group,
    register_owned_pgid,
    terminate_owned_group,
    write_group_registry,
)
from phy_data_collection.runner.runtime import (
    RosbagSession,
    dataset_dir,
    known_episode_summaries,
    rosbag_output_dir,
    start_gazebo,
    start_policy,
    start_rosbag,
    stop_policy,
    stop_rosbag,
    upload_dataset,
    wait_for_rosbag_start,
    wait_for_trial_summary,
    write_inputs,
)
from phy_data_collection.runner.scenario import make_trial_config
from phy_data_collection.runner.world import (
    log_trial_randomization,
    write_randomized_world,
)

CAPTURE_SAVED_RE = re.compile(r"\bsaved_count=(\d+)")


@dataclass
class RunContext:
    """한 수집 실행의 고유 경로와 현재 소유 중인 PGID를 보관한다."""

    args: argparse.Namespace
    run_id: str
    run_dir: Path
    stop_file: Path
    registry_path: Path
    active_groups: list[OwnedProcessGroup] = field(default_factory=list)
    progress_queue: Any | None = None
    worker_id: int | None = None


@dataclass
class WorkerProgress:
    """부모 프로세스가 표시할 worker별 trial 진행 상태."""

    worker_id: int
    state: str = "starting"
    trial_index: int | None = None
    trial_started_at: float | None = None
    completed: int = 0
    failed: int = 0
    captures: int = 0
    trial_captures: int = 0
    error: str = ""
    log_path: str = ""
    error_log_path: str = ""


@contextmanager
def _uninterruptible_cleanup():
    """cleanup 완료 전까지 추가 SIGINT가 teardown을 중단하지 못하게 한다."""
    previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous_handler)


@contextmanager
def _trial_output(args: argparse.Namespace, log_path: Path | None = None):
    """trial 상세 stdout/stderr를 로그 파일에 기록한다."""
    if getattr(args, "dry_run", False) or log_path is None:
        yield
        return
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_stream:
        with redirect_stdout(log_stream), redirect_stderr(log_stream):
            yield


def _run_log_dir(args: argparse.Namespace, parent_run_id: str) -> Path:
    """dataset version과 run ID로 격리된 상세 로그 디렉터리를 반환한다."""
    version = str(getattr(args, "dataset_version", "")).strip() or "unversioned"
    return COLLECTION_LOG_ROOT / version / parent_run_id


def _trial_log_path(
    args: argparse.Namespace,
    parent_run_id: str,
    worker_id: int,
    trial_index: int,
) -> Path:
    """worker와 global trial index가 드러나는 상세 로그 경로를 반환한다."""
    return (
        _run_log_dir(args, parent_run_id)
        / f"worker_{worker_id:02d}"
        / f"trial_{trial_index:04d}.log"
    )


def _create_run_context(
    args: argparse.Namespace,
    run_id: str | None = None,
    progress_queue: Any | None = None,
    worker_id: int | None = None,
) -> RunContext:
    """PID와 시각으로 충돌하지 않는 실행 ID 및 runtime 경로를 만든다."""
    run_id = run_id or f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"
    run_dir = CONFIG_DIR / run_id
    return RunContext(
        args=args,
        run_id=run_id,
        run_dir=run_dir,
        stop_file=run_dir / "policy_stop",
        registry_path=run_dir / REGISTRY_FILENAME,
        progress_queue=progress_queue,
        worker_id=worker_id,
    )


def _persist_groups(ctx: RunContext) -> None:
    """현재 살아 있는 소유 PGID 목록을 crash recovery registry에 반영한다."""
    write_group_registry(
        ctx.registry_path,
        run_id=ctx.run_id,
        stop_file=ctx.stop_file,
        groups=ctx.active_groups,
    )


def _remove_group_if_stopped(
    ctx: RunContext,
    group: OwnedProcessGroup | None,
    stopped: bool,
) -> None:
    """종료 검증에 성공한 그룹만 active registry 대상에서 제거한다."""
    if stopped and group in ctx.active_groups:
        ctx.active_groups.remove(group)


def _register_inner_simulator_groups(
    ctx: RunContext,
    config_path: Path,
) -> list[OwnedProcessGroup]:
    """config marker로 Distrobox 내부 ROS launch PGID를 찾아 registry에 추가한다."""
    marker = str(config_path)
    existing_pgids = {group.pgid for group in ctx.active_groups}
    added = False
    for pgid in sorted(process_groups_with_cmdline_marker(marker) - existing_pgids):
        try:
            group = register_owned_pgid(
                pgid,
                kind="simulator",
                run_id=ctx.run_id,
                marker=marker,
            )
        except ValueError as exc:
            print(f"[warn] inner simulator ownership check failed: {exc}")
            continue
        ctx.active_groups.append(group)
        added = True
    if added:
        _persist_groups(ctx)
    return [
        group
        for group in ctx.active_groups
        if group.kind == "simulator" and group.marker == marker
    ]


def _trial_paths(ctx: RunContext, index: int) -> tuple[Path, Path]:
    """trial별 engine config와 world의 고유 경로를 반환한다."""
    trial_dir = ctx.run_dir / f"trial_{index:04d}"
    return (
        trial_dir / "engine_config.yaml",
        trial_dir / "randomized_world.sdf",
    )


def _prepare_trial(
    ctx: RunContext,
    index: int,
    rng: random.Random,
) -> tuple[str, Path, Path | None, dict, dict]:
    """시나리오와 world를 랜덤화하고 trial 입력 파일을 기록한다."""
    config_path, requested_world_path = _trial_paths(ctx, index)
    config, scenario_params = make_trial_config(index, rng, ctx.args)
    task_id = next(iter(scenario_params))
    world_path, lighting = write_randomized_world(
        index,
        rng,
        ctx.args,
        requested_world_path,
    )
    scenario_params[task_id]["lighting"] = lighting
    scenario_params[task_id]["task_id"] = task_id
    scenario_params[task_id]["master_seed"] = ctx.args.seed
    scenario_params[task_id]["trial_seed"] = ctx.args.seed + index * 1_000_003
    write_inputs(config, config_path)
    log_trial_randomization(
        index=index,
        total=ctx.args.trials,
        task_id=task_id,
        scenario=scenario_params[task_id],
        lighting=lighting,
        args=ctx.args,
    )
    return (
        task_id,
        config_path,
        world_path,
        lighting,
        scenario_params[task_id],
    )


def _print_dry_run(
    config_path: Path,
    world_path: Path | None,
    lighting: dict,
) -> None:
    """프로세스를 실행하지 않고 생성된 config와 world metadata를 출력한다."""
    print(config_path.read_text(encoding="utf-8"))
    if world_path is not None:
        print(f"[dry-run] randomized world: {world_path}")
        print(json.dumps(lighting, indent=2, ensure_ascii=False))


def _run_trial(ctx: RunContext, index: int, rng: random.Random) -> None:
    """하나의 trial을 실행하고 소유한 policy, rosbag, simulator를 검증한다."""
    task_id, config_path, world_path, lighting, trial_metadata = _prepare_trial(
        ctx, index, rng
    )
    if ctx.args.dry_run:
        _print_dry_run(config_path, world_path, lighting)
        return

    simulator_wrapper_group: OwnedProcessGroup | None = None
    policy_group: OwnedProcessGroup | None = None
    rosbag_group: OwnedProcessGroup | None = None
    rosbag_session: RosbagSession | None = None
    trial_summary: dict | None = None
    policy_ok = True
    rosbag_ok = True
    simulator_ok = True
    interrupted = False
    previous_summaries = known_episode_summaries()
    timeout_s = (
        ctx.args.trial_timeout_s
        if ctx.args.trial_timeout_s is not None
        else float(ctx.args.time_limit_s) + TRIAL_TIMEOUT_GRACE_S
    )
    try:
        simulator_proc = start_gazebo(
            ctx.args,
            config_path,
            world_path,
            ctx.run_id,
        )
        simulator_wrapper_group = register_owned_group(
            simulator_proc,
            kind="simulator_wrapper",
            run_id=ctx.run_id,
            marker=str(config_path),
        )
        ctx.active_groups.append(simulator_wrapper_group)
        _persist_groups(ctx)

        wait_s = max(0.0, float(ctx.args.policy_start_wait_s))
        print(f"[wait] Gazebo/aic_engine head start: {wait_s:.1f}s")
        time.sleep(wait_s)
        inner_groups = _register_inner_simulator_groups(ctx, config_path)
        if not inner_groups:
            raise RuntimeError(
                "Distrobox inner simulator PGID discovery failed; "
                "refusing to start policy"
            )
        if ctx.args.record_rosbag:
            output_dir = rosbag_output_dir(
                ctx.args,
                run_id=ctx.run_id,
                index=index,
                task_id=task_id,
            )
            rosbag_session = start_rosbag(
                ctx.args,
                output_dir=output_dir,
                run_id=ctx.run_id,
            )
            rosbag_group = register_owned_group(
                rosbag_session.proc,
                kind="rosbag",
                run_id=ctx.run_id,
                marker=str(output_dir),
            )
            ctx.active_groups.append(rosbag_group)
            _persist_groups(ctx)
            if not wait_for_rosbag_start(rosbag_session, ctx.args):
                raise RuntimeError("rosbag recorder failed to start")
        line_callback = None
        if ctx.progress_queue is not None and ctx.worker_id is not None:
            line_callback = lambda line: _report_capture_progress(
                ctx.progress_queue,
                ctx.worker_id,
                index,
                line,
            )
        policy_proc = start_policy(
            ctx.args,
            stop_file=ctx.stop_file,
            run_id=ctx.run_id,
            trial_index=index,
            trial_split=_trial_split(ctx.args, index),
            trial_metadata=trial_metadata,
            line_callback=line_callback,
        )
        policy_group = register_owned_group(
            policy_proc,
            kind="policy",
            run_id=ctx.run_id,
            marker=str(ctx.stop_file),
        )
        ctx.active_groups.append(policy_group)
        _persist_groups(ctx)
        trial_summary = wait_for_trial_summary(
            task_id,
            previous_summaries,
            timeout_s,
            [simulator_proc, policy_proc],
        )
        if trial_summary is not None:
            post_summary_wait_s = max(
                0.0,
                float(ctx.args.post_summary_wait_s),
            )
            print(
                "[wait] AIC engine scoring/reset grace: "
                f"{post_summary_wait_s:.1f}s"
            )
            time.sleep(post_summary_wait_s)
    except KeyboardInterrupt:
        interrupted = True
        raise
    finally:
        with _uninterruptible_cleanup():
            if interrupted:
                print("[interrupt] fast teardown: additional Ctrl+C ignored")
            policy_ok = stop_policy(policy_group, ctx.stop_file, ctx.args)
            _remove_group_if_stopped(ctx, policy_group, policy_ok)
            rosbag_ok = stop_rosbag(rosbag_session, rosbag_group, ctx.args)
            _remove_group_if_stopped(ctx, rosbag_group, rosbag_ok)
            _persist_groups(ctx)
            _register_inner_simulator_groups(ctx, config_path)
            simulator_groups = [
                group
                for group in ctx.active_groups
                if group.marker == str(config_path)
                and group.kind in {"simulator", "simulator_wrapper"}
            ]
            for group in reversed(simulator_groups):
                group_ok = terminate_owned_group(
                    group,
                    ctx.args,
                    graceful_ros_shutdown=(
                        group.kind == "simulator" and not interrupted
                    ),
                )
                simulator_ok &= group_ok
                _remove_group_if_stopped(ctx, group, group_ok)
            _persist_groups(ctx)

    if not policy_ok or not rosbag_ok or not simulator_ok:
        raise RuntimeError("collector-owned PGID teardown verification failed")
    if trial_summary is None:
        raise RuntimeError(f"trial did not complete: task_id={task_id}")
    if str(trial_summary.get("status", "")) != "ok":
        raise RuntimeError(
            "trial collection failed: "
            f"task_id={task_id} status={trial_summary.get('status', 'missing')} "
            f"collect_steps={trial_summary.get('collect_steps', 0)} "
            f"detail={trial_summary.get('detail', '')}"
        )
    time.sleep(max(0.0, float(ctx.args.between_trial_wait_s)))


def _cleanup_interrupted_run(ctx: RunContext) -> None:
    """사용자 중단 시 현재 run이 소유한 PGID만 역순으로 종료한다."""
    with _uninterruptible_cleanup():
        policy_groups = [group for group in ctx.active_groups if group.kind == "policy"]
        for group in policy_groups:
            stopped = stop_policy(group, ctx.stop_file, ctx.args)
            _remove_group_if_stopped(ctx, group, stopped)
        for group in reversed(ctx.active_groups.copy()):
            stopped = terminate_owned_group(
                group,
                ctx.args,
                graceful_ros_shutdown=False,
            )
            _remove_group_if_stopped(ctx, group, stopped)
        _persist_groups(ctx)


def _parse_positive_csv(value: str, name: str) -> list[float]:
    """양수 CSV CLI 값을 검증한다."""
    try:
        values = [float(token.strip()) for token in value.split(",")]
    except ValueError as exc:
        raise ValueError(f"{name} must be a comma-separated number list") from exc
    if not values or any(not value > 0.0 for value in values):
        raise ValueError(f"{name} values must be positive")
    return values


def _port_is_available(port: int) -> bool:
    """worker Zenoh TCP port를 현재 host에서 bind할 수 있는지 확인한다."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def _prepare_args(args: argparse.Namespace) -> None:
    """대규모/병렬 수집 인자를 검증하고 안전한 기본 출력 version을 확정한다."""
    if args.trials < 1 or args.samples_per_trial < 1:
        raise ValueError("trials and samples-per-trial must be positive")
    if not hasattr(args, "seed"):
        args.seed = secrets.randbits(64)
    if args.workers < 1 or args.workers > args.trials:
        raise ValueError("workers must be between 1 and trials")
    if not 0.0 <= args.val_ratio < 1.0 or not 0.0 <= args.test_ratio < 1.0:
        raise ValueError("val-ratio and test-ratio must be in [0, 1)")
    if args.val_ratio + args.test_ratio >= 1.0:
        raise ValueError("val-ratio + test-ratio must be less than 1")
    tiers = _parse_positive_csv(args.sampling_tiers_mm, "sampling-tiers-mm")
    weights = _parse_positive_csv(args.sampling_tier_weights, "sampling-tier-weights")
    if len(tiers) != len(weights):
        raise ValueError("sampling tiers and weights must have the same length")
    if args.dz_min_mm < 0.0:
        raise ValueError("dz-min-mm must be non-negative to preserve port clearance")
    if args.base_z_offset_mm < DESCENT_MIN_CLEARANCE_MM:
        raise ValueError(
            "base-z-offset-mm must be at least "
            f"{DESCENT_MIN_CLEARANCE_MM:g}mm for descent clearance"
        )
    if args.near_port_base_z_offset_mm < MIN_CLEARANCE_MM:
        raise ValueError(
            "near-port-base-z-offset-mm must be at least "
            f"{MIN_CLEARANCE_MM:g}mm to prevent plug-port contact"
        )
    if args.near_port_min_capture_clearance_mm < MIN_CLEARANCE_MM:
        raise ValueError(
            "near-port-min-capture-clearance-mm must be at least "
            f"{MIN_CLEARANCE_MM:g}mm"
        )
    if args.board_distance_min_mm < args.base_z_offset_mm:
        raise ValueError("board-distance-min-mm must preserve base-z-offset-mm clearance")
    if args.board_distance_max_mm < args.board_distance_min_mm:
        raise ValueError("board-distance-max-mm must be at least board-distance-min-mm")
    if args.descent_start_distance_mm < args.base_z_offset_mm:
        raise ValueError("descent-start-distance-mm must preserve base-z-offset-mm clearance")
    if min(
        args.robot_joint_noise_rad,
        args.cable_rotation_noise_rad,
        args.port_roll_limit_rad,
        args.port_pitch_limit_rad,
        args.port_yaw_limit_rad,
        args.board_lateral_limit_mm,
        args.board_angle_limit_rad,
        args.descent_lateral_limit_mm,
        args.descent_angle_limit_rad,
        args.settle_orientation_tolerance_rad,
    ) < 0.0:
        raise ValueError("angle noise and policy limits must be non-negative")
    if args.cable_rotation_noise_rad > math.pi:
        raise ValueError("cable-rotation-noise-rad must not exceed pi")
    if not 1 <= args.min_visible_cameras <= 3:
        raise ValueError("min-visible-cameras must be in [1, 3]")
    if not args.policy.strip():
        args.policy = POLICY_MODULE
    if args.ros_domain_id_base < 0 or args.ros_domain_id_base + args.workers - 1 > 232:
        raise ValueError("worker ROS domain IDs must be in [0, 232]")
    if args.zenoh_port_base < 1024 or args.zenoh_port_base + args.workers - 1 > 65535:
        raise ValueError("worker Zenoh ports must be in [1024, 65535]")
    if args.settle_stable_observations < 1 or args.max_attempts < 1:
        raise ValueError("settle-stable-observations and max-attempts are invalid")
    if args.haptic_force_threshold_n <= 0.0 or args.haptic_contact_duration_s <= 0.0:
        raise ValueError(
            "haptic-force-threshold-n and haptic-contact-duration-s must be positive"
        )
    if args.cleanup_only:
        return
    if not args.dataset_version.strip():
        args.dataset_version = f"img2pos-{time.strftime('%Y%m%d-%H%M%S')}"
        print(f"[dataset] generated version: {args.dataset_version}")
    target = dataset_dir(args)
    if not args.dry_run and target.exists() and any(target.iterdir()) and not args.resume:
        raise ValueError(
            f"dataset already exists: {target}; use another --dataset-version or --resume"
        )
    if not args.dry_run:
        occupied = [
            port
            for port in range(args.zenoh_port_base, args.zenoh_port_base + args.workers)
            if not _port_is_available(port)
        ]
        if occupied:
            raise ValueError(f"Zenoh worker ports already in use: {occupied}")


def _split_assignments(args: argparse.Namespace) -> dict[int, str]:
    """정확한 개수의 trial을 seed 기반 train/val/test로 배정한다."""
    indices = list(range(args.trials))
    random.Random(args.seed ^ 0x5EED5EED).shuffle(indices)
    test_count = int(round(args.trials * args.test_ratio))
    val_count = int(round(args.trials * args.val_ratio))
    assignments = {index: "train" for index in indices}
    for index in indices[:test_count]:
        assignments[index] = "test"
    for index in indices[test_count : test_count + val_count]:
        assignments[index] = "val"
    return assignments


def _trial_split(args: argparse.Namespace, index: int) -> str:
    """현재 실행에서 미리 계산한 trial split을 반환한다."""
    return args.split_assignments[index]


def _emit_progress(
    progress_queue: Any | None,
    event: str,
    worker_id: int,
    **values: Any,
) -> None:
    """worker 상태를 부모 progress queue로 전달한다."""
    if progress_queue is None:
        return
    progress_queue.put(
        {
            "event": event,
            "worker_id": worker_id,
            "timestamp": time.monotonic(),
            **values,
        }
    )


def _report_capture_progress(
    progress_queue: Any,
    worker_id: int,
    trial_index: int,
    line: str,
) -> None:
    """policy의 capture 저장 로그를 worker progress event로 변환한다."""
    if "[PortOffsetCollect] CAPTURE SAVED:" not in line:
        return
    match = CAPTURE_SAVED_RE.search(line)
    if match is None:
        return
    _emit_progress(
        progress_queue,
        "capture_saved",
        worker_id,
        trial_index=trial_index,
        saved_count=int(match.group(1)),
    )


def _format_duration(seconds: float) -> str:
    """경과시간을 HH:MM:SS로 표시한다."""
    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def _apply_progress_event(
    workers: dict[int, WorkerProgress],
    event: dict[str, Any],
) -> None:
    """queue event를 부모가 보관하는 worker 상태에 반영한다."""
    worker = workers[int(event["worker_id"])]
    event_name = event["event"]
    if event_name == "worker_started":
        worker.state = "waiting"
    elif event_name == "trial_started":
        worker.state = "running"
        worker.trial_index = int(event["trial_index"])
        worker.trial_started_at = float(event["timestamp"])
        worker.trial_captures = 0
        worker.log_path = str(event.get("log_path", ""))
    elif event_name == "capture_saved":
        if int(event["trial_index"]) != worker.trial_index:
            return
        saved_count = int(event["saved_count"])
        if saved_count > worker.trial_captures:
            worker.captures += saved_count - worker.trial_captures
            worker.trial_captures = saved_count
    elif event_name in {"trial_completed", "trial_failed"}:
        if event_name == "trial_completed":
            worker.completed += 1
        else:
            worker.failed += 1
            worker.error = str(event.get("error", "trial failed"))
            worker.error_log_path = worker.log_path
        worker.state = "waiting"
        worker.trial_index = None
        worker.trial_started_at = None
    elif event_name == "worker_finished":
        worker.state = "finished"
    elif event_name == "worker_crashed":
        worker.state = "crashed"
        worker.error = str(event.get("error", "worker crashed"))
        worker.error_log_path = worker.log_path


def _progress_lines(
    workers: dict[int, WorkerProgress],
    total_trials: int,
    samples_per_trial: int,
    started_at: float,
    now: float,
) -> list[str]:
    """전체 capture 진행률과 worker별 현재 trial 상태를 짧은 행들로 만든다."""
    processed = sum(worker.completed + worker.failed for worker in workers.values())
    failures = sum(worker.failed for worker in workers.values())
    captures = sum(worker.captures for worker in workers.values())
    total_captures = total_trials * samples_per_trial
    elapsed = max(0.0, now - started_at)
    percent = captures / total_captures * 100.0 if total_captures else 100.0
    eta = "--:--:--"
    if captures:
        eta = _format_duration(elapsed / captures * (total_captures - captures))
    lines = [
        f"[progress] trials={processed}/{total_trials} "
        f"captures={captures}/{total_captures} ({percent:4.1f}%) failures={failures}",
        f"[time] elapsed={_format_duration(elapsed)} eta={eta}",
    ]
    for worker_id in sorted(workers):
        worker = workers[worker_id]
        if worker.state == "running" and worker.trial_started_at is not None:
            trial_elapsed = _format_duration(now - worker.trial_started_at)
            lines.append(
                f"[W{worker_id}] trial_{worker.trial_index:03d} "
                f"captures={worker.trial_captures}/{samples_per_trial} "
                f"elapsed={trial_elapsed}"
            )
        else:
            lines.append(
                f"[W{worker_id}] {worker.state} captures={worker.captures}"
            )
    return lines


def _show_progress(
    lines: list[str],
    *,
    terminal: bool,
    previous_line_count: int = 0,
) -> int:
    """TTY에서는 짧은 여러 행을 제자리 갱신하고 redirected output에는 그대로 기록한다."""
    if terminal:
        if previous_line_count:
            sys.stdout.write(f"\033[{previous_line_count}A")
        for line in lines:
            sys.stdout.write(f"\r\033[2K{line}\n")
        sys.stdout.flush()
    else:
        print("\n".join(lines), flush=True)
    return len(lines)


def _monitor_progress(
    processes: list[multiprocessing.Process],
    progress_queue: Any,
    workers: dict[int, WorkerProgress],
    total_trials: int,
    samples_per_trial: int,
    started_at: float,
) -> None:
    """child event를 모아 전체 및 worker별 진행상황을 주기적으로 표시한다."""
    terminal = sys.stdout.isatty()
    refresh_interval = 1.0 if terminal else 10.0
    last_refresh = 0.0
    rendered_line_count = 0
    while any(process.is_alive() for process in processes):
        try:
            event = progress_queue.get(timeout=0.2)
            _apply_progress_event(workers, event)
        except Empty:
            pass
        while True:
            try:
                _apply_progress_event(workers, progress_queue.get_nowait())
            except Empty:
                break
        now = time.monotonic()
        if now - last_refresh >= refresh_interval:
            rendered_line_count = _show_progress(
                _progress_lines(
                    workers,
                    total_trials,
                    samples_per_trial,
                    started_at,
                    now,
                ),
                terminal=terminal,
                previous_line_count=rendered_line_count,
            )
            last_refresh = now
    for process in processes:
        process.join()
    while True:
        try:
            _apply_progress_event(workers, progress_queue.get_nowait())
        except Empty:
            break
    for worker_id, process in enumerate(processes):
        worker = workers[worker_id]
        if process.exitcode not in {0, None} and worker.state != "crashed":
            worker.state = "failed"
            if not worker.error:
                worker.error = f"process exit code {process.exitcode}"
                worker.error_log_path = worker.log_path
    _show_progress(
        _progress_lines(
            workers,
            total_trials,
            samples_per_trial,
            started_at,
            time.monotonic(),
        ),
        terminal=terminal,
        previous_line_count=rendered_line_count,
    )
    for worker in workers.values():
        if worker.error:
            log_detail = (
                f" log={worker.error_log_path}" if worker.error_log_path else ""
            )
            print(f"[error] worker {worker.worker_id}: {worker.error}{log_detail}")


def _run_worker(
    args: argparse.Namespace,
    parent_run_id: str,
    worker_id: int,
    trial_indices: list[int],
    progress_queue: Any | None = None,
) -> int:
    """한 worker의 격리 설정으로 할당된 trial을 순차 실행한다."""
    args.worker_ros_domain_id = args.ros_domain_id_base + worker_id
    args.worker_zenoh_port = args.zenoh_port_base + worker_id
    args.worker_gz_partition = f"phy_{parent_run_id}_w{worker_id:02d}"
    run_id = f"{parent_run_id}-w{worker_id:02d}-{os.getpid()}"
    ctx = _create_run_context(
        args,
        run_id,
        progress_queue=progress_queue,
        worker_id=worker_id,
    )
    _emit_progress(
        progress_queue,
        "worker_started",
        worker_id,
    )
    failed_indices: list[int] = []
    try:
        for index in trial_indices:
            rng = random.Random(args.seed + index * 1_000_003)
            log_path = _trial_log_path(
                args,
                parent_run_id,
                worker_id,
                index,
            )
            _emit_progress(
                progress_queue,
                "trial_started",
                worker_id,
                trial_index=index,
                log_path=str(log_path),
            )
            try:
                with _trial_output(args, log_path):
                    _run_trial(ctx, index, rng)
            except RuntimeError as exc:
                failed_indices.append(index)
                _emit_progress(
                    progress_queue,
                    "trial_failed",
                    worker_id,
                    trial_index=index,
                    error=f"trial_{index:03d}: {exc}",
                )
                if progress_queue is None:
                    print(f"[error] trial {index} failed: {exc}; continuing worker")
            else:
                _emit_progress(
                    progress_queue,
                    "trial_completed",
                    worker_id,
                    trial_index=index,
                )
    except KeyboardInterrupt:
        with _trial_output(
            args,
            _run_log_dir(args, parent_run_id)
            / f"worker_{worker_id:02d}"
            / "interrupt_cleanup.log",
        ):
            _cleanup_interrupted_run(ctx)
        return 130
    finally:
        _persist_groups(ctx)
    _emit_progress(progress_queue, "worker_finished", worker_id)
    if failed_indices:
        if progress_queue is None:
            print(f"[error] worker {worker_id} failed trials: {failed_indices}")
        return 1
    return 0


def _worker_entry(
    args: argparse.Namespace,
    parent_run_id: str,
    worker_id: int,
    trial_indices: list[int],
    progress_queue: Any,
) -> None:
    """multiprocessing child의 exit code를 worker 결과와 일치시킨다."""
    try:
        result = _run_worker(
            args,
            parent_run_id,
            worker_id,
            trial_indices,
            progress_queue,
        )
    except Exception as exc:
        _emit_progress(
            progress_queue,
            "worker_crashed",
            worker_id,
            error=f"{type(exc).__name__}: {exc}",
        )
        result = 1
    raise SystemExit(result)


def _worker_trial_indices(total: int, workers: int, worker_id: int) -> list[int]:
    """global trial index를 worker에 중복 없이 round-robin 배정한다."""
    return list(range(worker_id, total, workers))


def _run_parallel(args: argparse.Namespace, parent_run_id: str) -> int:
    """trial을 worker별 round-robin 분배하고 모든 child 완료를 기다린다."""
    process_context = multiprocessing.get_context("spawn")
    progress_queue = process_context.Queue()
    processes: list[multiprocessing.Process] = []
    workers = {
        worker_id: WorkerProgress(worker_id=worker_id)
        for worker_id in range(args.workers)
    }
    started_at = time.monotonic()
    print(f"[logs] {_run_log_dir(args, parent_run_id)}")
    try:
        for worker_id in range(args.workers):
            indices = _worker_trial_indices(args.trials, args.workers, worker_id)
            process = process_context.Process(
                target=_worker_entry,
                args=(args, parent_run_id, worker_id, indices, progress_queue),
                name=f"phy-collector-{worker_id}",
            )
            process.start()
            processes.append(process)
            if worker_id + 1 < args.workers:
                time.sleep(max(0.0, args.worker_start_delay_s))
        _monitor_progress(
            processes,
            progress_queue,
            workers,
            args.trials,
            args.samples_per_trial,
            started_at,
        )
    except KeyboardInterrupt:
        print("\n[interrupt] stopping parallel collection workers...")
        for process in processes:
            if process.is_alive() and process.pid is not None:
                os.kill(process.pid, signal.SIGINT)
        for process in processes:
            process.join(timeout=30.0)
            if process.is_alive():
                process.terminate()
        return 130
    finally:
        progress_queue.close()
        progress_queue.join_thread()
    failed = [process for process in processes if process.exitcode != 0]
    if failed:
        print(
            "[error] collection workers failed: "
            + ", ".join(f"{process.name}={process.exitcode}" for process in failed)
        )
        return 1
    return 0


def _finalize_dataset(args: argparse.Namespace) -> int:
    """수집량·split 누수를 검사하고 summary 저장 후 선택적으로 한 번 업로드한다."""
    output_dir = dataset_dir(args)
    try:
        summary = summarize_dataset(output_dir)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"[error] dataset summary failed: {exc}")
        return 1
    summary["expected_captures"] = args.trials * args.samples_per_trial
    summary_path = output_dir / "collection_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(
        f"[summary] trials={summary['trials']} captures={summary['captures']} "
        f"images={summary['images']} output={output_dir}"
    )
    if summary["trial_split_leaks"]:
        print("[error] trial split leakage detected")
        return 1
    if summary["captures"] < summary["expected_captures"]:
        print(
            f"[error] captures below target: {summary['captures']}/"
            f"{summary['expected_captures']}"
        )
        return 1
    try:
        upload_dataset(args)
    except Exception as exc:
        print(f"[error] Hugging Face upload failed: {exc}")
        return 1
    return 0


def _write_metadata(args: argparse.Namespace) -> None:
    """현재 수집 실행의 seed와 총 trial 수를 별도 JSONL에 기록한다."""
    output_dir = dataset_dir(args)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "metadata.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps({"seed": args.seed, "trials": args.trials}, ensure_ascii=False)
            + "\n"
        )


def main() -> int:
    """CLI를 해석하고 독립 worker에서 randomized trial을 실행한다."""
    args = parse_args()
    try:
        _prepare_args(args)
    except ValueError as exc:
        print(f"[error] {exc}")
        return 2
    args.split_assignments = _split_assignments(args)
    if args.cleanup or args.cleanup_only:
        if not cleanup_stale_processes(args):
            print("[error] stale collector PGID cleanup verification failed")
            return 1
        if args.cleanup_only:
            return 0
    if not args.dry_run:
        _write_metadata(args)
    parent_run_id = f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"
    result = (
        _run_worker(args, parent_run_id, 0, list(range(args.trials)))
        if args.dry_run
        else _run_parallel(args, parent_run_id)
    )
    if result != 0 or args.dry_run:
        return result
    return _finalize_dataset(args)


if __name__ == "__main__":
    raise SystemExit(main())
