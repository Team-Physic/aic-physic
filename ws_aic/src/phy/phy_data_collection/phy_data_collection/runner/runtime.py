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
from urllib.parse import quote

import yaml

from .constants import (
    ANSI_COLORS,
    ANNOTATION_ROBOT_DESCRIPTION_PATH,
    BASE_ROS_GZ_BRIDGE_CONFIG_PATH,
    PACKAGE_ROOT,
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
DEPTH_CAMERAS = ("left", "center", "right")


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


def _peer_zenoh_override(port: int) -> str:
    """한 worker 전용 Zenoh router만 사용하도록 peer 설정을 반환한다."""
    return ";".join(
        [
            f'connect/endpoints=["tcp/localhost:{port}"]',
            "transport/shared_memory/enabled=false",
        ]
    )


def _apply_worker_isolation(env: dict[str, str], args: argparse.Namespace) -> None:
    """ROS, Gazebo와 Zenoh transport를 현재 worker 범위로 제한한다."""
    domain_id = int(getattr(args, "worker_ros_domain_id", args.ros_domain_id_base))
    zenoh_port = int(getattr(args, "worker_zenoh_port", args.zenoh_port_base))
    partition = str(getattr(args, "worker_gz_partition", f"phy_{domain_id}"))
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = partition
    env["IGN_PARTITION"] = partition
    env["ZENOH_CONFIG_OVERRIDE"] = _peer_zenoh_override(zenoh_port)


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
    _apply_worker_isolation(env, args)
    cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
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
    config_path: Path,
) -> None:
    """trial별 AIC engine YAML을 고유 경로에 저장한다."""
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _policy_environment(
    args: argparse.Namespace,
    *,
    stop_file: Path,
    run_id: str,
    trial_index: int | None = None,
    trial_split: str = "",
    trial_metadata: dict | None = None,
) -> dict[str, str]:
    """PortOffsetCollect가 사용할 ROS 2 및 데이터 수집 환경변수를 구성한다."""
    env = os.environ.copy()
    python_paths = [str(PACKAGE_ROOT)]
    if env.get("PYTHONPATH"):
        python_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_paths)
    env["AIC_CAPTURE_DIR"] = str(EPISODE_TRACKING_DIR)
    env["AIC_STOP_FILE"] = str(stop_file)
    env[RUN_MARKER_ENV] = run_id
    if trial_index is not None:
        env["AIC_PORTOFFSET_TRIAL_INDEX"] = str(trial_index)
        env["AIC_COLLECT_RANDOM_SEED"] = str(args.seed + trial_index * 1_000_003)
    if trial_split:
        env["AIC_IMG2POS_TRIAL_SPLIT"] = trial_split
    if trial_metadata is not None:
        env["AIC_IMG2POS_TRIAL_METADATA_JSON"] = json.dumps(
            trial_metadata,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    env["AIC_COLLECT_STEPS"] = str(args.samples_per_trial)
    env["AIC_IMG2POS_COLLECTION_POLICY"] = args.collection_policy
    env["AIC_IMG2POS_DATASET_VERSION"] = args.dataset_version.strip()
    env["AIC_IMG2POS_DATASET_DIR"] = str(dataset_dir(args))
    env["AIC_IMG2POS_VAL_RATIO"] = str(args.val_ratio)
    env["AIC_IMG2POS_TEST_RATIO"] = str(args.test_ratio)
    env["AIC_IMG2POS_AUTO_ANNOTATE_PORTS"] = (
        "true" if args.auto_annotate_ports else "false"
    )
    env["AIC_REID_BENCHMARK_LABELS"] = (
        "true" if args.reid_benchmark_labels else "false"
    )
    env["AIC_IMG2POS_DEPTH_VISIBILITY"] = (
        "true" if args.auto_annotate_ports else "false"
    )

    _set_optional_env(env, "AIC_PORT_COLLECT_DX_MIN_MM", args.dx_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DX_MAX_MM", args.dx_max_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DY_MIN_MM", args.dy_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DY_MAX_MM", args.dy_max_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DZ_MIN_MM", args.dz_min_mm)
    _set_optional_env(env, "AIC_PORT_COLLECT_DZ_MAX_MM", args.dz_max_mm)
    env["AIC_PORT_COLLECT_SAMPLING_TIERS_MM"] = args.sampling_tiers_mm
    env["AIC_PORT_COLLECT_SAMPLING_TIER_WEIGHTS"] = args.sampling_tier_weights
    env["AIC_PORT_COLLECT_ROLL_LIMIT_RAD"] = str(args.port_roll_limit_rad)
    env["AIC_PORT_COLLECT_PITCH_LIMIT_RAD"] = str(args.port_pitch_limit_rad)
    env["AIC_PORT_COLLECT_YAW_LIMIT_RAD"] = str(args.port_yaw_limit_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_ROLL_MIN_RAD", args.roll_min_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_ROLL_MAX_RAD", args.roll_max_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_PITCH_MIN_RAD", args.pitch_min_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_PITCH_MAX_RAD", args.pitch_max_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_YAW_MIN_RAD", args.yaw_min_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_YAW_MAX_RAD", args.yaw_max_rad)
    _set_optional_env(env, "AIC_PORT_COLLECT_RPY_NORM_MAX_RAD", args.rpy_norm_max_rad)
    env["AIC_IMG2POS_MIN_VISIBLE_CAMERAS"] = str(args.min_visible_cameras)
    base_z_offset_mm = (
        args.near_port_base_z_offset_mm
        if args.collection_policy == "near-port"
        else args.base_z_offset_mm
    )
    env["AIC_PORT_COLLECT_BASE_Z_OFFSET_M"] = str(base_z_offset_mm / 1000.0)
    env["AIC_NEAR_PORT_MIN_CAPTURE_CLEARANCE_M"] = str(
        args.near_port_min_capture_clearance_mm / 1000.0
    )
    env["AIC_BOARD_VIEW_DISTANCE_MIN_M"] = str(args.board_distance_min_mm / 1000.0)
    env["AIC_BOARD_VIEW_DISTANCE_MAX_M"] = str(args.board_distance_max_mm / 1000.0)
    env["AIC_BOARD_VIEW_LATERAL_LIMIT_M"] = str(args.board_lateral_limit_mm / 1000.0)
    env["AIC_BOARD_VIEW_ANGLE_LIMIT_RAD"] = str(args.board_angle_limit_rad)
    env["AIC_DESCENT_START_DISTANCE_M"] = str(args.descent_start_distance_mm / 1000.0)
    env["AIC_DESCENT_LATERAL_LIMIT_M"] = str(args.descent_lateral_limit_mm / 1000.0)
    env["AIC_DESCENT_ANGLE_LIMIT_RAD"] = str(args.descent_angle_limit_rad)
    env["AIC_COLLECT_SYNC_TOLERANCE_MS"] = str(args.sync_tolerance_ms)
    env["AIC_COLLECT_SYNC_WAIT_TIMEOUT_SEC"] = str(args.sync_wait_timeout_s)
    env["AIC_COLLECT_SETTLE_TIMEOUT_SEC"] = str(args.settle_timeout_s)
    env["AIC_COLLECT_SETTLE_POSITION_TOLERANCE_MM"] = str(
        args.settle_position_tolerance_mm
    )
    env["AIC_COLLECT_SETTLE_ORIENTATION_TOLERANCE_RAD"] = str(
        args.settle_orientation_tolerance_rad
    )
    env["AIC_COLLECT_SETTLE_STABLE_OBSERVATIONS"] = str(
        args.settle_stable_observations
    )
    env["AIC_COLLECT_SETTLE_POLL_SEC"] = str(args.settle_poll_s)
    env["AIC_COLLECT_MAX_ATTEMPTS"] = str(args.max_attempts)
    env["AIC_COLLECT_HAPTIC_GUARD"] = "true" if args.haptic_guard else "false"
    env["AIC_COLLECT_HAPTIC_FORCE_THRESHOLD_N"] = str(
        args.haptic_force_threshold_n
    )
    env["AIC_COLLECT_HAPTIC_CONTACT_DURATION_S"] = str(
        args.haptic_contact_duration_s
    )
    env["AIC_COLLECT_COLOR_LOG"] = "true" if args.color_log else "false"
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    _apply_worker_isolation(env, args)
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    return env


def start_policy(
    args: argparse.Namespace,
    *,
    stop_file: Path,
    run_id: str,
    trial_index: int | None = None,
    trial_split: str = "",
    trial_metadata: dict | None = None,
    line_callback: Callable[[str], None] | None = None,
) -> subprocess.Popen:
    """PortOffsetCollect ROS 2 node를 독립 session/PGID로 실행한다."""
    env = _policy_environment(
        args,
        stop_file=stop_file,
        run_id=run_id,
        trial_index=trial_index,
        trial_split=trial_split,
        trial_metadata=trial_metadata,
    )
    try:
        stop_file.unlink()
    except FileNotFoundError:
        pass
    cmd = [
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
    return _start_logged_process(
        cmd,
        cwd=PIXI_WS,
        env=env,
        line_callback=line_callback,
    )


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
    if args.auto_annotate_ports:
        bridge_config = _write_annotation_bridge_config(config_path)
        launch_args.extend(
            [
                f"description_file:={ANNOTATION_ROBOT_DESCRIPTION_PATH}",
                f"ros_gz_bridge_config_file:={bridge_config}",
            ]
        )

    launch_cmd = shlex.join(
        ["ros2", "launch", "aic_bringup", "aic_gz_bringup.launch.py", *launch_args]
    )
    domain_id = int(getattr(args, "worker_ros_domain_id", args.ros_domain_id_base))
    zenoh_port = int(getattr(args, "worker_zenoh_port", args.zenoh_port_base))
    partition = str(getattr(args, "worker_gz_partition", f"phy_{domain_id}"))
    router_override = ";".join(
        [
            'mode="router"',
            f'listen/endpoints=["tcp/[::]:{zenoh_port}"]',
            "connect/endpoints=[]",
            "routing/router/peers_failover_brokering=true",
            "transport/shared_memory/enabled=false",
        ]
    )
    exports = [
        ". /ws_aic/install/setup.bash",
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp",
        f"export ROS_DOMAIN_ID={domain_id}",
        f"export GZ_PARTITION={shlex.quote(partition)}",
        f"export IGN_PARTITION={shlex.quote(partition)}",
        "export RCUTILS_COLORIZED_OUTPUT=0",
        "export RCUTILS_LOGGING_BUFFERED_STREAM=1",
        f"export {RUN_MARKER_ENV}={shlex.quote(run_id)}",
        "export ZENOH_ROUTER_CONFIG_URI=/aic_zenoh_config.json5",
        f"export ZENOH_CONFIG_OVERRIDE={shlex.quote(router_override)}",
        "ros2 run rmw_zenoh_cpp rmw_zenohd &",
        "router_pid=$!",
        "cleanup_router() { kill -SIGINT \"$router_pid\" 2>/dev/null || true; wait \"$router_pid\" 2>/dev/null || true; }",
        "trap cleanup_router EXIT",
        f"export ZENOH_CONFIG_OVERRIDE={shlex.quote(_peer_zenoh_override(zenoh_port))}",
        launch_cmd,
    ]
    inner = "\n".join(["set -e", *exports])
    cmd = ["distrobox", "enter", args.distrobox, "--", "bash", "-lc", inner]
    print("[gazebo] " + shlex.join(cmd))
    return _start_logged_process(cmd, cwd=PIXI_WS)


def _write_annotation_bridge_config(config_path: Path) -> Path:
    """기존 bridge 설정에 수집용 depth image 세 topic을 추가한다."""
    entries = yaml.safe_load(
        BASE_ROS_GZ_BRIDGE_CONFIG_PATH.read_text(encoding="utf-8")
    )
    if not isinstance(entries, list):
        raise ValueError(
            f"invalid ROS-Gazebo bridge config: {BASE_ROS_GZ_BRIDGE_CONFIG_PATH}"
        )
    existing = {
        str(entry.get("ros_topic_name", entry.get("topic_name", "")))
        for entry in entries
        if isinstance(entry, dict)
    }
    for camera in DEPTH_CAMERAS:
        topic = f"/{camera}_camera/depth_image"
        if topic in existing:
            continue
        entries.append(
            {
                "ros_topic_name": topic,
                "gz_topic_name": topic,
                "ros_type_name": "sensor_msgs/msg/Image",
                "gz_type_name": "gz.msgs.Image",
                "direction": "GZ_TO_ROS",
                "lazy": True,
            }
        )
    output_path = config_path.with_name(f"{config_path.stem}_bridge.yaml")
    output_path.write_text(
        yaml.safe_dump(entries, sort_keys=False),
        encoding="utf-8",
    )
    return output_path


def _project_hf_token() -> str | None:
    """환경변수 또는 ws_aic/.env에서 Hugging Face token을 읽는다."""
    token = os.environ.get("HF_TOKEN", "").strip()
    if token:
        return token
    env_path = WS_SRC.parent / ".env"
    try:
        lines = env_path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    for line in lines:
        key, separator, value = line.partition("=")
        if separator and key.strip() == "HF_TOKEN":
            return value.strip().strip("\"'") or None
    return None


def upload_dataset(args: argparse.Namespace) -> str | None:
    """모든 trial이 끝난 dataset을 한 번만 지정 HF branch에 업로드한다."""
    if not args.push_to_hub:
        return None
    from huggingface_hub import HfApi

    token = _project_hf_token()
    api = HfApi(token=token)
    api.whoami()
    api.create_repo(
        repo_id=args.hf_repo_id,
        repo_type="dataset",
        private=args.hf_private,
        exist_ok=True,
    )
    revision = args.hf_revision.strip() or args.dataset_version
    if revision != "main":
        branches = [
            ref.name
            for ref in api.list_repo_refs(args.hf_repo_id, repo_type="dataset").branches
        ]
        if revision not in branches:
            api.create_branch(
                repo_id=args.hf_repo_id,
                repo_type="dataset",
                branch=revision,
            )
    api.upload_large_folder(
        repo_id=args.hf_repo_id,
        repo_type="dataset",
        revision=revision,
        folder_path=str(dataset_dir(args)),
        ignore_patterns=["*.tmp", "*.lock", "__pycache__/*", ".DS_Store"],
        private=args.hf_private,
    )
    url = (
        f"https://huggingface.co/datasets/{args.hf_repo_id}/tree/"
        f"{quote(revision, safe='')}"
    )
    print(f"[huggingface] upload complete: {url}")
    return url


def known_episode_summaries() -> set[Path]:
    """trial 시작 전에 이미 존재하던 episode summary 경로를 수집한다."""
    return set(EPISODE_TRACKING_DIR.glob("*/episode_summary.json"))


def _read_trial_summary(path: Path, task_id: str) -> dict | None:
    """현재 task의 유효한 episode summary를 읽는다."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or str(data.get("task_id", "")) != task_id:
        return None
    return data


def wait_for_trial_summary(
    task_id: str,
    known_summaries: set[Path],
    timeout_s: float,
    watch_procs: list[subprocess.Popen | None],
) -> dict | None:
    """현재 task summary를 기다리며 policy와 simulator 조기 종료도 감시한다."""
    deadline = time.monotonic() + max(1.0, timeout_s)
    print(f"[wait] episode summary 대기: task_id={task_id}, timeout={timeout_s:.1f}s")
    while time.monotonic() < deadline:
        for summary_path in known_episode_summaries() - known_summaries:
            summary = _read_trial_summary(summary_path, task_id)
            if summary is not None:
                print(
                    "[done] episode summary saved: "
                    f"{summary_path} status={summary.get('status', 'missing')}"
                )
                return summary
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
            return None
        time.sleep(1.0)
    print(f"[warn] timeout waiting for task summary: {task_id}")
    return None
