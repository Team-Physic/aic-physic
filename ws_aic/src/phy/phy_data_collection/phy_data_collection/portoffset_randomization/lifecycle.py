"""수집기가 소유한 ROS 2/Gazebo 프로세스 그룹의 생명주기 관리."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from .constants import (
    CONFIG_DIR,
    LEGACY_POLICY_STOP_FILE,
    REGISTRY_FILENAME,
    RUN_MARKER_ENV,
)


@dataclass
class OwnedProcessGroup:
    """수집기가 독점적으로 생성하고 종료할 수 있는 프로세스 그룹."""

    proc: subprocess.Popen | None
    pgid: int
    kind: str
    run_id: str
    marker: str

    @property
    def leader_pid(self) -> int:
        """registry에 기록할 대표 PID를 반환한다."""
        if self.proc is not None:
            return self.proc.pid
        members = process_group_members(self.pgid)
        return members[0] if members else self.pgid


def _process_state_and_pgid(pid: int) -> tuple[str, int] | None:
    """Linux proc stat에서 프로세스 상태와 PGID를 안전하게 읽는다."""
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
        fields = stat[stat.rfind(")") + 2:].split()
        return fields[0], int(fields[2])
    except (OSError, ValueError, IndexError):
        return None


def process_group_members(pgid: int) -> list[int]:
    """좀비를 제외하고 지정 PGID에 남아 있는 PID 목록을 반환한다."""
    members: list[int] = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return members
    for entry in entries:
        if not entry.name.isdigit():
            continue
        info = _process_state_and_pgid(int(entry.name))
        if info is not None and info[0] != "Z" and info[1] == pgid:
            members.append(int(entry.name))
    return sorted(members)


def _read_proc_bytes(pid: int, name: str) -> bytes:
    """권한 또는 종료 경쟁을 허용하며 proc 파일을 바이트로 읽는다."""
    try:
        return (Path("/proc") / str(pid) / name).read_bytes()
    except OSError:
        return b""


def _group_matches_marker(pgid: int, run_id: str, marker: str) -> bool:
    """PGID가 registry의 run ID 또는 고유 경로를 실제로 보유했는지 검증한다."""
    env_marker = f"{RUN_MARKER_ENV}={run_id}".encode()
    marker_bytes = marker.encode()
    for pid in process_group_members(pgid):
        if env_marker in _read_proc_bytes(pid, "environ"):
            return True
        if marker_bytes and marker_bytes in _read_proc_bytes(pid, "cmdline"):
            return True
    return False


def process_groups_with_cmdline_marker(marker: str) -> set[int]:
    """고유 config 경로를 명령행에 가진 비좀비 프로세스의 PGID를 찾는다."""
    marker_bytes = marker.encode()
    if not marker_bytes:
        return set()
    pgids: set[int] = set()
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return pgids
    for entry in entries:
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if marker_bytes not in _read_proc_bytes(pid, "cmdline"):
            continue
        info = _process_state_and_pgid(pid)
        if info is not None and info[0] != "Z" and info[1] != os.getpgrp():
            pgids.add(info[1])
    return pgids


def wait_group_exit(pgid: int, timeout_s: float) -> bool:
    """주어진 시간 동안 PGID의 모든 비좀비 프로세스가 끝나기를 기다린다."""
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        if not process_group_members(pgid):
            return True
        time.sleep(0.1)
    return not process_group_members(pgid)


def terminate_pgid(
    pgid: int,
    *,
    label: str,
    sigint_grace_s: float,
    sigterm_grace_s: float,
    sigkill_grace_s: float,
) -> bool:
    """소유 PGID에만 SIGINT, SIGTERM, SIGKILL을 단계적으로 적용한다."""
    signal_plan: list[tuple[signal.Signals, float]] = []
    if sigint_grace_s > 0.0:
        signal_plan.append((signal.SIGINT, sigint_grace_s))
    signal_plan.extend(
        [
            (signal.SIGTERM, sigterm_grace_s),
            (signal.SIGKILL, sigkill_grace_s),
        ]
    )
    for sig, grace_s in signal_plan:
        members = process_group_members(pgid)
        if not members:
            print(f"[cleanup] {label}: PGID {pgid} 종료 확인")
            return True
        print(f"[cleanup] {label}: {sig.name} -> PGID {pgid}, members={members}")
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        except PermissionError as exc:
            print(f"[error] {label}: PGID {pgid} signal permission denied: {exc}")
            return False
        if wait_group_exit(pgid, grace_s):
            print(f"[cleanup] {label}: PGID {pgid} 종료 확인")
            return True
    remaining = process_group_members(pgid)
    if remaining:
        print(f"[error] {label}: PGID {pgid} remnants={remaining}")
        return False
    return True


def register_owned_group(
    proc: subprocess.Popen,
    *,
    kind: str,
    run_id: str,
    marker: str,
) -> OwnedProcessGroup:
    """새 세션으로 실행된 프로세스의 PID와 PGID 및 소유 marker를 기록한다."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = proc.pid
    group = OwnedProcessGroup(proc, pgid, kind, run_id, marker)
    print(
        f"[lifecycle] registered {kind}: pid={proc.pid}, pgid={pgid}, "
        f"run_id={run_id}"
    )
    return group


def register_owned_pgid(
    pgid: int,
    *,
    kind: str,
    run_id: str,
    marker: str,
) -> OwnedProcessGroup:
    """Distrobox 내부에서 분리된 PGID를 소유 marker 검증 후 등록한다."""
    if not _group_matches_marker(pgid, run_id, marker):
        raise ValueError(f"PGID {pgid} does not match run ownership marker")
    group = OwnedProcessGroup(None, pgid, kind, run_id, marker)
    print(
        f"[lifecycle] registered {kind}: pid={group.leader_pid}, "
        f"pgid={pgid}, run_id={run_id}"
    )
    return group


def terminate_owned_group(
    group: OwnedProcessGroup | None,
    args: argparse.Namespace,
    *,
    graceful_ros_shutdown: bool,
) -> bool:
    """등록된 그룹을 종료하고 Popen 리더까지 회수한다."""
    if group is None:
        return True
    sigint_grace_s = 0.0
    if graceful_ros_shutdown:
        sigint_grace_s = (
            args.rosbag_stop_grace_s
            if group.kind == "rosbag"
            else args.sim_sigint_grace_s
        )
    ok = terminate_pgid(
        group.pgid,
        label=f"{group.kind} teardown",
        sigint_grace_s=sigint_grace_s,
        sigterm_grace_s=args.sim_cleanup_grace_s,
        sigkill_grace_s=args.sim_sigkill_grace_s,
    )
    if group.proc is not None and group.proc.poll() is None:
        try:
            group.proc.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            pass
    return ok


def write_group_registry(
    registry_path: Path,
    *,
    run_id: str,
    stop_file: Path,
    groups: list[OwnedProcessGroup],
) -> None:
    """살아 있는 소유 PGID만 원자적으로 registry 파일에 저장한다."""
    active = [group for group in groups if process_group_members(group.pgid)]
    if not active:
        try:
            registry_path.unlink()
        except FileNotFoundError:
            pass
        return
    payload = {
        "run_id": run_id,
        "stop_file": str(stop_file),
        "groups": [
            {
                "kind": group.kind,
                "pid": group.leader_pid,
                "pgid": group.pgid,
                "marker": group.marker,
            }
            for group in active
        ],
    }
    registry_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = registry_path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(registry_path)


def _load_registry(path: Path) -> dict | None:
    """손상된 registry를 허용하면서 JSON 객체만 반환한다."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def _legacy_collection_pgids() -> set[int]:
    """registry 도입 전 고유 config 경로를 사용한 수집 PGID를 찾는다."""
    pgids: set[int] = set()
    config_root = str(CONFIG_DIR).encode()
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return pgids
    for entry in entries:
        if not entry.name.isdigit():
            continue
        cmdline = _read_proc_bytes(int(entry.name), "cmdline")
        if config_root not in cmdline or b"aic_engine_config_file:=" not in cmdline:
            continue
        info = _process_state_and_pgid(int(entry.name))
        if info is not None and info[0] != "Z" and info[1] != os.getpgrp():
            pgids.add(info[1])
    return pgids


def _record_fields(record: object) -> tuple[int, str, str] | None:
    """registry 레코드를 검증해 PGID, marker, 종류로 변환한다."""
    if not isinstance(record, dict):
        return None
    try:
        return (
            int(record["pgid"]),
            str(record.get("marker", "")),
            str(record.get("kind", "owned")),
        )
    except (KeyError, TypeError, ValueError):
        return None


def cleanup_stale_processes(args: argparse.Namespace) -> bool:
    """run marker로 소유권이 입증된 이전 수집 PGID만 정리한다."""
    print("[cleanup] stale PortOffsetCollect process groups 정리 중...")
    success = True
    handled_pgids: set[int] = set()
    try:
        LEGACY_POLICY_STOP_FILE.write_text("stop\n", encoding="utf-8")
    except OSError:
        pass

    for registry_path in CONFIG_DIR.glob(f"*/{REGISTRY_FILENAME}"):
        registry = _load_registry(registry_path)
        if registry is None:
            continue
        run_id = str(registry.get("run_id", ""))
        stop_file = Path(str(registry.get("stop_file", "")))
        if CONFIG_DIR in stop_file.parents:
            try:
                stop_file.write_text("stop\n", encoding="utf-8")
            except OSError:
                pass
        records = [
            fields
            for record in registry.get("groups", [])
            if (fields := _record_fields(record)) is not None
        ]
        for pgid, marker, kind in records:
            if not process_group_members(pgid):
                continue
            if not _group_matches_marker(pgid, run_id, marker):
                print(
                    f"[warn] stale registry PGID {pgid} ownership mismatch; "
                    "PID reuse 가능성이 있어 종료하지 않음"
                )
                continue
            handled_pgids.add(pgid)
            if kind == "simulator":
                sigint_grace_s = args.sim_sigint_grace_s
            elif kind == "rosbag":
                sigint_grace_s = args.rosbag_stop_grace_s
            else:
                sigint_grace_s = 0.0
            success &= terminate_pgid(
                pgid,
                label=f"stale {kind}",
                sigint_grace_s=sigint_grace_s,
                sigterm_grace_s=args.sim_cleanup_grace_s,
                sigkill_grace_s=args.sim_sigkill_grace_s,
            )
        if not any(
            _group_matches_marker(pgid, run_id, marker)
            for pgid, marker, _ in records
        ):
            try:
                registry_path.unlink()
            except OSError:
                pass

    for pgid in sorted(_legacy_collection_pgids() - handled_pgids):
        success &= terminate_pgid(
            pgid,
            label="legacy simulator",
            sigint_grace_s=args.sim_sigint_grace_s,
            sigterm_grace_s=args.sim_cleanup_grace_s,
            sigkill_grace_s=args.sim_sigkill_grace_s,
        )

    try:
        LEGACY_POLICY_STOP_FILE.unlink()
    except OSError:
        pass
    remaining = _legacy_collection_pgids()
    if remaining:
        print(f"[error] collector-owned simulator PGIDs remain: {sorted(remaining)}")
        return False
    return success
