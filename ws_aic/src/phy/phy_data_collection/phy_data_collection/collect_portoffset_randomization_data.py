#!/usr/bin/env python3
"""무작위 PortOffset trial을 준비하고 ROS 2/Gazebo lifecycle을 조율한다."""

from __future__ import annotations

import argparse
import json
import os
import random
import signal
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path

from phy_data_collection.portoffset_randomization.cli import parse_args
from phy_data_collection.portoffset_randomization.constants import (
    CONFIG_DIR,
    REGISTRY_FILENAME,
    TRIAL_TIMEOUT_GRACE_S,
)
from phy_data_collection.portoffset_randomization.lifecycle import (
    OwnedProcessGroup,
    cleanup_stale_processes,
    process_groups_with_cmdline_marker,
    register_owned_group,
    register_owned_pgid,
    terminate_owned_group,
    write_group_registry,
)
from phy_data_collection.portoffset_randomization.runtime import (
    RosbagSession,
    known_episode_summaries,
    rosbag_output_dir,
    start_gazebo,
    start_policy,
    start_rosbag,
    stop_policy,
    stop_rosbag,
    wait_for_rosbag_start,
    wait_for_trial_summary,
    write_inputs,
)
from phy_data_collection.portoffset_randomization.scenario import make_trial_config
from phy_data_collection.portoffset_randomization.world import (
    log_trial_randomization,
    write_randomized_world,
)


@dataclass
class RunContext:
    """한 수집 실행의 고유 경로와 현재 소유 중인 PGID를 보관한다."""

    args: argparse.Namespace
    run_id: str
    run_dir: Path
    stop_file: Path
    registry_path: Path
    active_groups: list[OwnedProcessGroup] = field(default_factory=list)


@contextmanager
def _uninterruptible_cleanup():
    """cleanup 완료 전까지 추가 SIGINT가 teardown을 중단하지 못하게 한다."""
    previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def _create_run_context(args: argparse.Namespace) -> RunContext:
    """PID와 시각으로 충돌하지 않는 실행 ID 및 runtime 경로를 만든다."""
    run_id = f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"
    run_dir = CONFIG_DIR / run_id
    return RunContext(
        args=args,
        run_id=run_id,
        run_dir=run_dir,
        stop_file=run_dir / "policy_stop",
        registry_path=run_dir / REGISTRY_FILENAME,
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


def _trial_paths(ctx: RunContext, index: int) -> tuple[Path, Path, Path]:
    """trial별 engine config, scenario, world의 고유 경로를 반환한다."""
    trial_dir = ctx.run_dir / f"trial_{index:04d}"
    return (
        trial_dir / "engine_config.yaml",
        trial_dir / "scenario_params.json",
        trial_dir / "randomized_world.sdf",
    )


def _prepare_trial(
    ctx: RunContext,
    index: int,
    rng: random.Random,
) -> tuple[str, dict, Path, Path, Path | None, dict]:
    """시나리오와 world를 랜덤화하고 trial 입력 파일을 기록한다."""
    config_path, scenario_path, requested_world_path = _trial_paths(ctx, index)
    config, scenario_params = make_trial_config(index, rng, ctx.args)
    task_id = next(iter(scenario_params))
    world_path, lighting = write_randomized_world(
        index,
        rng,
        ctx.args,
        requested_world_path,
    )
    scenario_params[task_id]["lighting"] = lighting
    write_inputs(config, scenario_params, config_path, scenario_path)
    log_trial_randomization(
        index=index,
        total=ctx.args.trials,
        task_id=task_id,
        scenario=scenario_params[task_id],
        lighting=lighting,
        args=ctx.args,
    )
    return task_id, scenario_params, config_path, scenario_path, world_path, lighting


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
    (
        task_id,
        _,
        config_path,
        scenario_path,
        world_path,
        lighting,
    ) = _prepare_trial(ctx, index, rng)
    if ctx.args.dry_run:
        _print_dry_run(config_path, world_path, lighting)
        return

    simulator_wrapper_group: OwnedProcessGroup | None = None
    policy_group: OwnedProcessGroup | None = None
    rosbag_group: OwnedProcessGroup | None = None
    rosbag_session: RosbagSession | None = None
    trial_completed = False
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
        policy_proc = start_policy(
            ctx.args,
            scenario_params_path=scenario_path,
            stop_file=ctx.stop_file,
            run_id=ctx.run_id,
            trial_index=index,
            rosbag_path=(
                rosbag_session.output_dir if rosbag_session is not None else None
            ),
        )
        policy_group = register_owned_group(
            policy_proc,
            kind="policy",
            run_id=ctx.run_id,
            marker=str(scenario_path),
        )
        ctx.active_groups.append(policy_group)
        _persist_groups(ctx)
        trial_completed = wait_for_trial_summary(
            task_id,
            previous_summaries,
            timeout_s,
            [simulator_proc, policy_proc],
        )
        if trial_completed:
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
    if not trial_completed:
        raise RuntimeError(f"trial did not complete: task_id={task_id}")
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


def main() -> int:
    """CLI를 해석하고 모든 randomized trial을 순차 실행한다."""
    args = parse_args()
    if args.cleanup or args.cleanup_only:
        if not cleanup_stale_processes(args):
            print("[error] stale collector PGID cleanup verification failed")
            return 1
        if args.cleanup_only:
            return 0

    ctx = _create_run_context(args)
    rng = random.Random(args.seed)
    try:
        for index in range(args.trials):
            _run_trial(ctx, index, rng)
    except KeyboardInterrupt:
        print("\n[interrupt] cleaning collector-owned process groups...")
        _cleanup_interrupted_run(ctx)
        return 130
    except RuntimeError as exc:
        print(f"[error] {exc}")
        return 1
    finally:
        _persist_groups(ctx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
