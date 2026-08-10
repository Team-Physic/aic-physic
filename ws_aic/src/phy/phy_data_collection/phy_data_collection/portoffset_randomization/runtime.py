"""PortOffset policy와 Gazebo launch 실행 및 trial 완료 감시."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import yaml

from .constants import (
    ANSI_COLORS,
    DATA_GENERATOR_PACKAGE_ROOT,
    DATASET_ROOT,
    EPISODE_TRACKING_DIR,
    PIXI_WS,
    RUN_MARKER_ENV,
    WS_SRC,
)
from .lifecycle import (
    OwnedProcessGroup,
    terminate_owned_group,
    terminate_pgid,
    wait_group_exit,
)

ANSI_ESCAPE_RE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


@dataclass
class RosbagSession:
    """실행 중인 trial rosbag과 시작 확인 상태를 묶는다."""

    proc: subprocess.Popen[str]
    output_dir: Path
    ready: threading.Event


def _forward_process_output(
    proc: subprocess.Popen[str],
    line_callback: Callable[[str], None] | None = None,
) -> None:
    """callback에는 정제한 줄을 전달하고 화면에는 상태 ANSI 색상을 보존한다."""
    if proc.stdout is None:
        return
    for line in proc.stdout:
        clean_line = ANSI_ESCAPE_RE.sub("", line).rstrip("\r\n")
        if line_callback is not None:
            line_callback(clean_line)
        display_line = (
            clean_line
            if os.environ.get("NO_COLOR")
            else line.rstrip("\r\n")
        )
        print(display_line, flush=True)


def _start_logged_process(
    cmd: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    line_callback: Callable[[str], None] | None = None,
) -> subprocess.Popen[str]:
    """자식 출력을 pipe로 격리하고 단일 line-forwarding thread를 시작한다."""
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
        start_new_session=True,
    )
    threading.Thread(
        target=_forward_process_output,
        args=(proc, line_callback),
        name=f"output-{proc.pid}",
        daemon=True,
    ).start()
    return proc


def _rosbag_log(
    args: argparse.Namespace,
    text: str,
    color: str,
) -> None:
    """rosbag lifecycle 상태를 색상과 볼드로 출력한다."""
    if not args.color_log or os.environ.get("NO_COLOR"):
        print(text, flush=True)
        return
    prefix = ANSI_COLORS["bold"] + ANSI_COLORS.get(color, "")
    print(f"{prefix}{text}{ANSI_COLORS['reset']}", flush=True)


def rosbag_output_dir(
    args: argparse.Namespace,
    *,
    run_id: str,
    index: int,
    task_id: str,
) -> Path:
    """dataset version과 run/trial ID로 충돌 없는 rosbag 경로를 만든다."""
    root = Path(args.rosbag_output_dir).expanduser()
    if not root.is_absolute():
        root = WS_SRC / root
    version = args.dataset_version.strip() or "unversioned"
    return root / version / run_id / f"trial_{index:04d}_{task_id}"


def start_rosbag(
    args: argparse.Namespace,
    *,
    output_dir: Path,
    run_id: str,
) -> RosbagSession:
    """현재 trial 토픽을 MCAP으로 기록할 rosbag2 프로세스를 시작한다."""
    if output_dir.exists():
        raise RuntimeError(f"rosbag output already exists: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    ready = threading.Event()

    def mark_ready(line: str) -> None:
        if "Recording..." in line or "Listening for topics" in line:
            ready.set()

    env = os.environ.copy()
    env[RUN_MARKER_ENV] = run_id
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    env["ZENOH_CONFIG_OVERRIDE"] = "transport/shared_memory/enabled=false"
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    env["PIXI_COLOR"] = "never"
    env["PIXI_NO_PROGRESS"] = "true"
    cmd = [
        "pixi",
        "run",
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "--use-sim-time",
        "--custom-data",
        "phy_event_timestamp_clock=ros_sim",
        "-o",
        str(output_dir),
        "--topics",
        *args.rosbag_topics,
    ]
    _rosbag_log(args, f"[rosbag] STARTING: {output_dir}", "cyan")
    proc = _start_logged_process(
        cmd,
        cwd=WS_SRC,
        env=env,
        line_callback=mark_ready,
    )
    return RosbagSession(proc=proc, output_dir=output_dir, ready=ready)


def wait_for_rosbag_start(
    session: RosbagSession,
    args: argparse.Namespace,
) -> bool:
    """rosbag2의 Recording 로그 또는 조기 종료를 제한 시간 동안 감시한다."""
    deadline = time.monotonic() + max(0.0, args.rosbag_start_timeout_s)
    while time.monotonic() < deadline:
        ready = session.ready.wait(timeout=0.1)
        storage_open = any(session.output_dir.glob("*.mcap"))
        if ready or storage_open:
            _rosbag_log(
                args,
                f"[rosbag] RECORDING STARTED: {session.output_dir}",
                "green",
            )
            return True
        if session.proc.poll() is not None:
            _rosbag_log(
                args,
                f"[rosbag] START FAILED: exit={session.proc.returncode}",
                "yellow",
            )
            return False
    _rosbag_log(args, "[rosbag] START TIMEOUT", "yellow")
    return False


def validate_rosbag(output_dir: Path) -> tuple[bool, str]:
    """metadata, message 수, MCAP 양끝 magic을 검사해 정상 finalize를 판정한다."""
    metadata_path = output_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return False, "metadata.yaml missing"
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = metadata["rosbag2_bagfile_information"]
        message_count = int(info["message_count"])
    except (OSError, TypeError, ValueError, KeyError, yaml.YAMLError) as exc:
        return False, f"invalid metadata.yaml: {exc}"
    if message_count <= 0:
        return False, "message_count is zero"

    mcap_files = sorted(output_dir.glob("*.mcap"))
    if not mcap_files:
        return False, "MCAP file missing"
    magic = b"\x89MCAP0\r\n"
    total_bytes = 0
    for path in mcap_files:
        try:
            size = path.stat().st_size
            with path.open("rb") as stream:
                start_magic = stream.read(len(magic))
                stream.seek(-len(magic), os.SEEK_END)
                end_magic = stream.read(len(magic))
        except (OSError, ValueError) as exc:
            return False, f"cannot inspect {path.name}: {exc}"
        if start_magic != magic or end_magic != magic:
            return False, f"{path.name} was not finalized"
        total_bytes += size
    return True, f"messages={message_count}, size={total_bytes / (1024 * 1024):.1f} MiB"


def stop_rosbag(
    session: RosbagSession | None,
    group: OwnedProcessGroup | None,
    args: argparse.Namespace,
) -> bool:
    """rosbag에 SIGINT를 보내고 MCAP finalize 결과까지 검증한다."""
    if session is None or group is None:
        return True
    _rosbag_log(args, "[rosbag] FINALIZING...", "cyan")
    stopped = terminate_pgid(
        group.pgid,
        label="rosbag teardown",
        sigint_grace_s=args.rosbag_stop_grace_s,
        sigterm_grace_s=args.sim_cleanup_grace_s,
        sigkill_grace_s=args.sim_sigkill_grace_s,
    )
    if session.proc.poll() is None:
        try:
            session.proc.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            pass
    if not stopped:
        _rosbag_log(args, "[rosbag] FINALIZE FAILED: process remains", "yellow")
        return False
    valid, detail = validate_rosbag(session.output_dir)
    if not valid:
        _rosbag_log(args, f"[rosbag] FINALIZE FAILED: {detail}", "yellow")
        return False
    _rosbag_log(
        args,
        f"[rosbag] RECORDING COMPLETED: {session.output_dir} ({detail})",
        "green",
    )
    return True


def dataset_dir(args: argparse.Namespace) -> Path:
    """dataset version을 반영한 로컬 저장 디렉터리를 반환한다."""
    version = args.dataset_version.strip()
    return DATASET_ROOT / version if version else DATASET_ROOT


def _set_optional_env(
    env: dict[str, str],
    name: str,
    value: float | None,
) -> None:
    """선택 CLI 값이 제공된 경우에만 policy 환경변수로 전달한다."""
    if value is not None:
        env[name] = str(value)


def write_inputs(
    config: dict,
    scenario_params: dict,
    config_path: Path,
    scenario_params_path: Path,
) -> None:
    """trial별 engine YAML과 추적용 scenario JSON을 고유 경로에 저장한다."""
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    scenario_params_path.parent.mkdir(parents=True, exist_ok=True)
    scenario_params_path.write_text(
        json.dumps(scenario_params, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def _policy_environment(
    args: argparse.Namespace,
    *,
    scenario_params_path: Path,
    stop_file: Path,
    run_id: str,
    trial_index: int | None = None,
    rosbag_path: Path | None = None,
) -> dict[str, str]:
    """PortOffsetCollect가 사용할 ROS 2 및 데이터 수집 환경변수를 구성한다."""
    env = os.environ.copy()
    python_paths = [str(DATA_GENERATOR_PACKAGE_ROOT)]
    if env.get("PYTHONPATH"):
        python_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_paths)
    env["AIC_SCENARIO_PARAMS_FILE"] = str(scenario_params_path)
    env["AIC_CAPTURE_DIR"] = str(EPISODE_TRACKING_DIR)
    env["AIC_STOP_FILE"] = str(stop_file)
    env[RUN_MARKER_ENV] = run_id
    if trial_index is not None:
        env["AIC_PORTOFFSET_TRIAL_INDEX"] = str(trial_index)
    if rosbag_path is not None:
        env["AIC_PORTOFFSET_ROSBAG_PATH"] = str(rosbag_path.resolve())
    env["AIC_COLLECT_STEPS"] = str(args.samples_per_trial)
    env["AIC_RPY_DATASET_VERSION"] = args.dataset_version.strip()
    env["AIC_VISION_OFFSET_DATASET_DIR"] = str(dataset_dir(args))
    env["AIC_VISION_OFFSET_PUSH_TO_HUB"] = "true" if args.push_to_hub else "false"
    if args.vision_offset_repo_id:
        env["AIC_VISION_OFFSET_REPO_ID"] = args.vision_offset_repo_id
    if args.vision_offset_hf_revision:
        env["AIC_VISION_OFFSET_HF_REVISION"] = args.vision_offset_hf_revision
    if args.vision_offset_hf_path_in_repo:
        env["AIC_VISION_OFFSET_HF_PATH_IN_REPO"] = args.vision_offset_hf_path_in_repo
    env["AIC_VISION_OFFSET_UPLOAD_ON_PORT_TYPE"] = args.upload_on_port_type
    env["AIC_VISION_OFFSET_HF_PRIVATE"] = "true" if args.hf_private else "false"

    env["AIC_PORT_COLLECT_XY_LIMIT_MM"] = str(args.port_xy_limit_mm)
    env["AIC_PORT_COLLECT_Z_LIMIT_MM"] = str(args.port_z_limit_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DX_MIN_MM", args.dx_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DX_MAX_MM", args.dx_max_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DY_MIN_MM", args.dy_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DY_MAX_MM", args.dy_max_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DZ_MIN_MM", args.dz_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DZ_MAX_MM", args.dz_max_mm)
    env["AIC_PORT_COLLECT_ROLL_LIMIT_DEG"] = str(args.port_roll_limit_deg)
    env["AIC_PORT_COLLECT_PITCH_LIMIT_DEG"] = str(args.port_pitch_limit_deg)
    env["AIC_PORT_COLLECT_YAW_LIMIT_DEG"] = str(args.port_yaw_limit_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_ROLL_MIN_DEG", args.roll_min_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_ROLL_MAX_DEG", args.roll_max_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_PITCH_MIN_DEG", args.pitch_min_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_PITCH_MAX_DEG", args.pitch_max_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_YAW_MIN_DEG", args.yaw_min_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_YAW_MAX_DEG", args.yaw_max_deg)
    _set_optional_env(env, "AIC_PORT_COLLECT_RPY_NORM_MAX_RAD", args.rpy_norm_max_rad)
    _set_optional_env(
        env,
        "AIC_PORT_ACTUAL_RPY_NORM_MAX_RAD",
        args.actual_rpy_norm_max_rad,
    )

    env["AIC_RPY_MIN_VISIBLE_CAMERAS"] = str(args.min_visible_cameras)
    env["AIC_RPY_VISIBILITY_MARGIN_PX"] = str(args.visibility_margin_px)
    env["AIC_PORT_COLLECT_BASE_Z_OFFSET_M"] = str(args.base_z_offset_mm / 1000.0)
    env["AIC_COLLECT_SYNC_TOLERANCE_MS"] = str(args.sync_tolerance_ms)
    env["AIC_COLLECT_SYNC_WAIT_TIMEOUT_SEC"] = str(args.sync_wait_timeout_s)
    env["AIC_COLLECT_COLOR_LOG"] = "true" if args.color_log else "false"
    env["AIC_LEROBOT_REPO_ID"] = ""
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    env["ZENOH_CONFIG_OVERRIDE"] = "transport/shared_memory/enabled=false"
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    env["PIXI_COLOR"] = "never"
    env["PIXI_NO_PROGRESS"] = "true"
    return env


def start_policy(
    args: argparse.Namespace,
    *,
    scenario_params_path: Path,
    stop_file: Path,
    run_id: str,
    trial_index: int | None = None,
    rosbag_path: Path | None = None,
) -> subprocess.Popen:
    """PortOffsetCollect ROS 2 node를 독립 session/PGID로 실행한다."""
    env = _policy_environment(
        args,
        scenario_params_path=scenario_params_path,
        stop_file=stop_file,
        run_id=run_id,
        trial_index=trial_index,
        rosbag_path=rosbag_path,
    )
    try:
        stop_file.unlink()
    except FileNotFoundError:
        pass
    cmd = [
        "pixi",
        "run",
        "ros2",
        "run",
        "aic_model",
        "aic_model",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        f"policy:={args.policy}",
    ]
    print("[policy] " + shlex.join(cmd))
    return _start_logged_process(cmd, cwd=PIXI_WS, env=env)


def stop_policy(
    group: OwnedProcessGroup | None,
    stop_file: Path,
    args: argparse.Namespace,
) -> bool:
    """policy에 정상 stop을 요청하고 timeout 시 소유 PGID만 강제 종료한다."""
    if group is None:
        return True
    try:
        stop_file.write_text("stop\n", encoding="utf-8")
        if wait_group_exit(group.pgid, args.policy_stop_grace_s):
            print(f"[cleanup] policy graceful stop: PGID {group.pgid} 종료 확인")
            return True
        return terminate_owned_group(
            group,
            args,
            graceful_ros_shutdown=False,
        )
    finally:
        try:
            stop_file.unlink()
        except OSError:
            pass


def start_gazebo(
    args: argparse.Namespace,
    config_path: Path,
    world_path: Path | None,
    run_id: str,
) -> subprocess.Popen:
    """AIC Gazebo launch stack을 Distrobox의 독립 session/PGID로 실행한다."""
    launch_args = [
        "spawn_task_board:=false",
        "spawn_cable:=false",
        "ground_truth:=true",
        "start_aic_engine:=true",
        f"aic_engine_config_file:={config_path}",
    ]
    if world_path is not None:
        launch_args.append(f"world_file:={world_path}")
    if args.headless:
        launch_args.append("gazebo_gui:=false")
    if args.headless or not args.launch_rviz:
        launch_args.append("launch_rviz:=false")

    args_str = " ".join(shlex.quote(value) for value in launch_args)
    exports = [
        'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"',
        'export ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=false"',
        'export RCUTILS_COLORIZED_OUTPUT="0"',
        'export RCUTILS_LOGGING_BUFFERED_STREAM="1"',
        f"export {RUN_MARKER_ENV}={shlex.quote(run_id)}",
    ]
    inner = " && ".join([*exports, f"/entrypoint.sh {args_str}"])
    cmd = ["distrobox", "enter", args.distrobox, "--", "bash", "-lc", inner]
    print("[gazebo] " + shlex.join(cmd))
    return _start_logged_process(cmd, cwd=PIXI_WS)


def known_episode_summaries() -> set[Path]:
    """trial 시작 전에 이미 존재하던 episode summary 경로를 수집한다."""
    return set(EPISODE_TRACKING_DIR.glob("*/episode_summary.json"))


def _summary_matches_task(path: Path, task_id: str) -> bool:
    """episode summary가 현재 task ID에 해당하는지 검증한다."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return str(data.get("task_id", "")) == task_id


def wait_for_trial_summary(
    task_id: str,
    known_summaries: set[Path],
    timeout_s: float,
    watch_procs: list[subprocess.Popen | None],
) -> bool:
    """현재 task summary를 기다리며 policy와 simulator 조기 종료도 감시한다."""
    deadline = time.monotonic() + max(1.0, timeout_s)
    print(f"[wait] episode summary 대기: task_id={task_id}, timeout={timeout_s:.1f}s")
    while time.monotonic() < deadline:
        for summary_path in known_episode_summaries() - known_summaries:
            if _summary_matches_task(summary_path, task_id):
                print(f"[done] episode summary saved: {summary_path}")
                return True
        failed = [
            proc
            for proc in watch_procs
            if proc is not None and proc.poll() is not None
        ]
        if failed:
            print(
                "[warn] watched process exited before summary: "
                f"returncode={failed[0].returncode}"
            )
            return False
        time.sleep(1.0)
    print(f"[warn] timeout waiting for task summary: {task_id}")
    return False
