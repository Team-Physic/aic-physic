"""SFP/SC trial 구성과 시나리오 수준 uniform randomization."""

from __future__ import annotations

import argparse
import math
import random

from .constants import (
    BASE_ROBOT_HOME,
    LIMITS,
    SC_RAIL_COUNT,
    SFP_NIC_RAIL_COUNT,
    SFP_PORT_COUNT,
)


def _scoring_section() -> dict:
    """AIC engine이 기록해야 할 ROS 2 topic 목록을 구성한다."""
    topic_types = [
        ("/joint_states", "sensor_msgs/msg/JointState"),
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/scoring/tf", "tf2_msgs/msg/TFMessage"),
        ("/aic/gazebo/contacts/off_limit", "ros_gz_interfaces/msg/Contacts"),
        ("/fts_broadcaster/wrench", "geometry_msgs/msg/WrenchStamped"),
        (
            "/aic_controller/joint_commands",
            "aic_control_interfaces/msg/JointMotionUpdate",
        ),
        ("/aic_controller/pose_commands", "aic_control_interfaces/msg/MotionUpdate"),
        ("/scoring/insertion_event", "std_msgs/msg/String"),
        (
            "/aic_controller/controller_state",
            "aic_control_interfaces/msg/ControllerState",
        ),
    ]
    topics = [
        {"topic": {"name": name, "type": message_type}}
        for name, message_type in topic_types
    ]
    topics.insert(
        2,
        {
            "topic": {
                "name": "/tf_static",
                "type": "tf2_msgs/msg/TFMessage",
                "latched": True,
            }
        },
    )
    return {"topics": topics}


def _task_board_limits_section() -> dict:
    """AIC task board rail의 물리적 이동 한계를 반환한다."""
    return {
        "nic_rail": {
            "min_translation": LIMITS["nic_translation"][0],
            "max_translation": LIMITS["nic_translation"][1],
        },
        "sc_rail": {
            "min_translation": LIMITS["sc_translation"][0],
            "max_translation": LIMITS["sc_translation"][1],
        },
        "mount_rail": {"min_translation": -0.09425, "max_translation": 0.09425},
    }


def _robot_section(rng: random.Random, joint_noise_rad: float) -> dict:
    """각 home joint에 독립 uniform 각도 잡음을 더한다."""
    return {
        "home_joint_positions": {
            name: value + rng.uniform(-joint_noise_rad, joint_noise_rad)
            for name, value in BASE_ROBOT_HOME.items()
        }
    }


def _board_pose(rng: random.Random, port_type: str) -> dict:
    """포트 종류별 허용 범위에서 board x, y, yaw를 uniform 추출한다."""
    prefix = "sc" if port_type == "sc" else "sfp"
    return {
        "x": rng.uniform(*LIMITS[f"{prefix}_board_x"]),
        "y": rng.uniform(*LIMITS[f"{prefix}_board_y"]),
        "z": 1.14,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": rng.uniform(*LIMITS[f"{prefix}_board_yaw"]),
    }


def _entity_pose(translation: float = 0.0, yaw: float = 0.0) -> dict:
    """rail entity가 공유하는 translation/RPY pose를 생성한다."""
    return {
        "translation": translation,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": yaw,
    }


def _present_entity(name: str, translation: float = 0.0, yaw: float = 0.0) -> dict:
    """존재하는 rail entity의 공통 설정을 생성한다."""
    return {
        "entity_present": True,
        "entity_name": name,
        "entity_pose": _entity_pose(translation, yaw),
    }


def _active_rails(mask: int, rail_count: int) -> list[int]:
    """bitmask에서 활성 rail index를 낮은 번호 순서로 반환한다."""
    return [index for index in range(rail_count) if mask & (1 << index)]


def _nic_rails(poses: dict[int, tuple[float, float]]) -> dict:
    """bitmask에서 계산된 모든 NIC card pose를 rail 구성으로 변환한다."""
    rails = {
        f"nic_rail_{index}": {"entity_present": False}
        for index in range(SFP_NIC_RAIL_COUNT)
    }
    for rail, (translation, yaw) in poses.items():
        rails[f"nic_rail_{rail}"] = _present_entity(
            f"nic_card_{rail}", translation, yaw
        )
    return rails


def _background_sc_rails(rng: random.Random) -> dict:
    """SFP trial 배경 SC mount 위치를 uniform 추출한다."""
    return {
        "sc_rail_0": _present_entity(
            "sc_mount_0",
            rng.uniform(*LIMITS["sc_translation"]),
        ),
        "sc_rail_1": {"entity_present": False},
    }


def _sc_rails(poses: dict[int, float]) -> dict:
    """bitmask에서 계산된 모든 SC card pose를 rail 구성으로 변환한다."""
    rails = {
        f"sc_rail_{index}": {"entity_present": False}
        for index in range(SC_RAIL_COUNT)
    }
    for rail, translation in poses.items():
        rails[f"sc_rail_{rail}"] = _present_entity(
            f"sc_mount_{rail}", translation
        )
    return rails


def _mount_rails(port_type: str) -> dict:
    """포트 종류에 맞는 고정 mount rail 배치를 반환한다."""
    if port_type == "sc":
        return {
            "lc_mount_rail_0": {"entity_present": False},
            "sfp_mount_rail_0": _present_entity("sfp_mount_0"),
            "sc_mount_rail_0": _present_entity("sc_mount_2"),
            "lc_mount_rail_1": _present_entity("lc_mount_1"),
            "sfp_mount_rail_1": {"entity_present": False},
            "sc_mount_rail_1": {"entity_present": False},
        }
    return {
        "lc_mount_rail_0": _present_entity("lc_mount_0"),
        "sfp_mount_rail_0": _present_entity("sfp_mount_0"),
        "sc_mount_rail_0": _present_entity("sc_mount_0"),
        "lc_mount_rail_1": _present_entity("lc_mount_1"),
        "sfp_mount_rail_1": {"entity_present": False},
        "sc_mount_rail_1": {"entity_present": False},
    }


def _gripper_axis_offset(
    rng: random.Random,
    prefix: str,
    axis: str,
) -> float:
    """한 gripper 축의 기준값에 uniform 위치 잡음을 더한다."""
    return LIMITS[f"{prefix}_gripper_offset_{axis}"] + rng.uniform(
        *LIMITS["gripper_offset_noise"]
    )


def _gripper_offset(rng: random.Random, port_type: str) -> dict:
    """포트별 기준 gripper offset에 각 축 독립 uniform 잡음을 더한다."""
    prefix = "sc" if port_type == "sc" else "sfp"
    return {
        axis: _gripper_axis_offset(rng, prefix, axis)
        for axis in ("x", "y", "z")
    }


def _multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """xyzw quaternion 두 개를 곱한다."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """roll, pitch, yaw를 xyzw quaternion으로 변환한다."""
    sr, cr = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rpy_from_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """xyzw quaternion을 roll, pitch, yaw로 변환한다."""
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _sample_rotation_noise(
    rng: random.Random,
    max_angle_rad: float,
) -> tuple[float, float, float, float]:
    """SO(3) 구 내부에서 최대 회전각을 넘지 않는 quaternion을 추출한다."""
    if max_angle_rad <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    axis = [rng.normalvariate(0.0, 1.0) for _ in range(3)]
    norm = math.sqrt(sum(value * value for value in axis))
    while norm <= 1e-12:
        axis = [rng.normalvariate(0.0, 1.0) for _ in range(3)]
        norm = math.sqrt(sum(value * value for value in axis))
    angle = max_angle_rad * rng.random() ** (1.0 / 3.0)
    scale = math.sin(angle / 2.0) / norm
    return axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(angle / 2.0)


def _cable_rpy(
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[float, float, float]:
    """기준 cable 자세에 전체 회전각이 상한 이내인 local 잡음을 합성한다."""
    base = _quaternion_from_rpy(
        LIMITS["cable_roll"],
        LIMITS["cable_pitch"],
        LIMITS["cable_yaw"],
    )
    noise = _sample_rotation_noise(rng, args.cable_rotation_noise_rad)
    return _rpy_from_quaternion(_multiply_quaternions(base, noise))


def _cable_config(
    gripper_offset: dict,
    cable_rpy: tuple[float, float, float],
    cable_type: str,
) -> dict:
    """SFP와 SC가 공유하는 cable scene 설정을 생성한다."""
    roll, pitch, yaw = cable_rpy
    return {
        "pose": {
            "gripper_offset": gripper_offset,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        },
        "attach_cable_to_gripper": True,
        "cable_type": cable_type,
    }


def _task_config(
    *,
    cable_name: str,
    plug_type: str,
    plug_name: str,
    port_name: str,
    target_module_name: str,
    time_limit_s: int,
) -> dict:
    """포트 종류와 무관한 task 공통 필드를 조립한다."""
    return {
        "cable_type": "sfp_sc",
        "cable_name": cable_name,
        "plug_type": plug_type,
        "plug_name": plug_name,
        "port_type": plug_type,
        "port_name": port_name,
        "target_module_name": target_module_name,
        "time_limit": int(time_limit_s),
    }


def _scenario_metadata(
    *,
    trial_type: int,
    port_type: str,
    rail_idx: int,
    combination_mask: int,
    combination_bits: str,
    active_rails: list[int],
    board: dict,
    gripper_offset: dict,
    cable_rpy: tuple[float, float, float],
    nic_translation: float = 0.0,
    nic_yaw: float = 0.0,
    sc_translation: float = 0.0,
    sfp_port_idx: int = -1,
) -> dict:
    """dataset 추적에 저장할 공통 시나리오 metadata를 생성한다."""
    cable_roll, cable_pitch, cable_yaw = cable_rpy
    return {
        "trial_type": trial_type,
        "port_type": port_type,
        "rail_idx": rail_idx,
        "combination_mask": combination_mask,
        "combination_bits": combination_bits,
        "active_rails": active_rails,
        "board_x": board["x"],
        "board_y": board["y"],
        "board_yaw": board["yaw"],
        "gripper_offset_x": gripper_offset["x"],
        "gripper_offset_y": gripper_offset["y"],
        "gripper_offset_z": gripper_offset["z"],
        "nic_translation": nic_translation,
        "nic_yaw": nic_yaw,
        "sc_translation": sc_translation,
        "sfp_port_idx": sfp_port_idx,
        "cable_roll": cable_roll,
        "cable_pitch": cable_pitch,
        "cable_yaw": cable_yaw,
    }


def _make_sfp_trial(
    index: int,
    combination_mask: int,
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[str, dict, dict]:
    """지정 card 조합을 가진 SFP trial과 추적 metadata를 만든다."""
    active_rails = _active_rails(combination_mask, SFP_NIC_RAIL_COUNT)
    nic_rail = rng.choice(active_rails)
    port_index = rng.randrange(SFP_PORT_COUNT)
    port_name = f"sfp_port_{port_index}"
    combination_bits = f"{combination_mask:0{SFP_NIC_RAIL_COUNT}b}"
    task_id = (
        f"portoffset_sfp_{index:04d}_cards{combination_bits}_"
        f"rail{nic_rail}_{port_name}"
    )
    board = _board_pose(rng, "sfp")
    nic_poses = {
        rail: (
            rng.uniform(*LIMITS["nic_translation"]),
            rng.uniform(*LIMITS["nic_yaw"]),
        )
        for rail in active_rails
    }
    nic_translation, nic_yaw = nic_poses[nic_rail]
    gripper_offset = _gripper_offset(rng, "sfp")
    cable_rpy = _cable_rpy(rng, args)

    task_board = {"pose": board}
    task_board.update(_nic_rails(nic_poses))
    task_board.update(_background_sc_rails(rng))
    task_board.update(_mount_rails("sfp"))
    cable_name = "cable_0"
    trial = {
        "scene": {
            "task_board": task_board,
            "cables": {
                cable_name: _cable_config(
                    gripper_offset,
                    cable_rpy,
                    "sfp_sc_cable",
                )
            },
        },
        "tasks": {
            task_id: _task_config(
                cable_name=cable_name,
                plug_type="sfp",
                plug_name="sfp_tip",
                port_name=port_name,
                target_module_name=f"nic_card_mount_{nic_rail}",
                time_limit_s=args.time_limit_s,
            )
        },
    }
    metadata = _scenario_metadata(
        trial_type=0,
        port_type="sfp",
        rail_idx=nic_rail,
        combination_mask=combination_mask,
        combination_bits=combination_bits,
        active_rails=active_rails,
        board=board,
        gripper_offset=gripper_offset,
        cable_rpy=cable_rpy,
        nic_translation=nic_translation,
        nic_yaw=nic_yaw,
        sc_translation=task_board["sc_rail_0"]["entity_pose"]["translation"],
        sfp_port_idx=port_index,
    )
    return task_id, trial, {task_id: metadata}


def _make_sc_trial(
    index: int,
    combination_mask: int,
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[str, dict, dict]:
    """지정 card 조합을 가진 SC trial과 추적 metadata를 만든다."""
    active_rails = _active_rails(combination_mask, SC_RAIL_COUNT)
    sc_rail = rng.choice(active_rails)
    combination_bits = f"{combination_mask:0{SC_RAIL_COUNT}b}"
    task_id = f"portoffset_sc_{index:04d}_cards{combination_bits}_rail{sc_rail}"
    board = _board_pose(rng, "sc")
    sc_poses = {
        rail: rng.uniform(*LIMITS["sc_translation"])
        for rail in active_rails
    }
    sc_translation = sc_poses[sc_rail]
    gripper_offset = _gripper_offset(rng, "sc")
    cable_rpy = _cable_rpy(rng, args)

    task_board = {"pose": board}
    task_board.update(
        {f"nic_rail_{rail}": {"entity_present": False} for rail in range(SFP_NIC_RAIL_COUNT)}
    )
    task_board.update(_sc_rails(sc_poses))
    task_board.update(_mount_rails("sc"))
    cable_name = "cable_1"
    trial = {
        "scene": {
            "task_board": task_board,
            "cables": {
                cable_name: _cable_config(
                    gripper_offset,
                    cable_rpy,
                    "sfp_sc_cable_reversed",
                )
            },
        },
        "tasks": {
            task_id: _task_config(
                cable_name=cable_name,
                plug_type="sc",
                plug_name="sc_tip",
                port_name="sc_port_base",
                target_module_name=f"sc_port_{sc_rail}",
                time_limit_s=args.time_limit_s,
            )
        },
    }
    metadata = _scenario_metadata(
        trial_type=1,
        port_type="sc",
        rail_idx=sc_rail,
        combination_mask=combination_mask,
        combination_bits=combination_bits,
        active_rails=active_rails,
        board=board,
        gripper_offset=gripper_offset,
        cable_rpy=cable_rpy,
        sc_translation=sc_translation,
    )
    return task_id, trial, {task_id: metadata}


def make_trial_config(
    index: int,
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[dict, dict]:
    """선택 connector의 non-empty card 조합을 uniform 추출해 trial config를 만든다."""
    port_type = args.port_type
    rail_count = SFP_NIC_RAIL_COUNT if port_type == "sfp" else SC_RAIL_COUNT
    combination_mask = rng.randint(1, (1 << rail_count) - 1)
    if port_type == "sfp":
        task_id, trial, scenario_params = _make_sfp_trial(
            index, combination_mask, rng, args
        )
    else:
        task_id, trial, scenario_params = _make_sc_trial(
            index, combination_mask, rng, args
        )
    robot = _robot_section(rng, args.robot_joint_noise_rad)
    config = {
        "scoring": _scoring_section(),
        "task_board_limits": _task_board_limits_section(),
        "trials": {f"trial_{index:04d}_{port_type}": trial},
        "robot": robot,
    }
    scenario_params[task_id]["robot_home_joint_positions"] = robot[
        "home_joint_positions"
    ]
    return config, scenario_params
