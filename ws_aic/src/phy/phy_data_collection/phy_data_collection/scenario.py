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


def _robot_section(rng: random.Random, joint_noise_deg: float) -> dict:
    """각 home joint에 독립 uniform 각도 잡음을 더한다."""
    noise = math.radians(joint_noise_deg)
    return {
        "home_joint_positions": {
            name: value + rng.uniform(-noise, noise)
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


def _nic_rails(active_rail: int, translation: float, yaw: float) -> dict:
    """선택한 NIC rail 하나만 활성화한 rail 구성을 생성한다."""
    rails = {f"nic_rail_{index}": {"entity_present": False} for index in range(SFP_NIC_RAIL_COUNT)}
    rails[f"nic_rail_{active_rail}"] = _present_entity(
        f"nic_card_{active_rail}",
        translation,
        yaw,
    )
    return rails


def _nic_rail_for_trial(index: int, seed: int) -> int:
    """매 5개 SFP trial에서 모든 rail을 한 번씩 무작위 순서로 선택한다."""
    block, position = divmod(index, SFP_NIC_RAIL_COUNT)
    rails = list(range(SFP_NIC_RAIL_COUNT))
    random.Random(seed ^ (block * 0x9E3779B1)).shuffle(rails)
    return rails[position]


def _background_sc_rails(rng: random.Random) -> dict:
    """SFP trial 배경 SC mount 위치를 uniform 추출한다."""
    return {
        "sc_rail_0": _present_entity(
            "sc_mount_0",
            rng.uniform(*LIMITS["sc_translation"]),
        ),
        "sc_rail_1": {"entity_present": False},
    }


def _sc_rails(active_rail: int, translation: float) -> dict:
    """선택한 SC rail 하나만 활성화한 rail 구성을 생성한다."""
    rails = {f"sc_rail_{index}": {"entity_present": False} for index in range(SC_RAIL_COUNT)}
    rails[f"sc_rail_{active_rail}"] = _present_entity(
        f"sc_mount_{active_rail}",
        translation,
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


def _cable_rpy(
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[float, float, float]:
    """기준 cable RPY에 지정 각도 범위의 uniform 잡음을 더한다."""
    noise = math.radians(args.cable_rpy_noise_deg)
    return (
        LIMITS["cable_roll"] + rng.uniform(-noise, noise),
        LIMITS["cable_pitch"] + rng.uniform(-noise, noise),
        LIMITS["cable_yaw"] + rng.uniform(-noise, noise),
    )


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
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[str, dict, dict]:
    """하나의 SFP trial과 추적 metadata를 uniform randomization으로 만든다."""
    nic_rail = _nic_rail_for_trial(index, args.seed)
    port_index = rng.randrange(SFP_PORT_COUNT)
    port_name = f"sfp_port_{port_index}"
    task_id = f"portoffset_sfp_{index:04d}_rail{nic_rail}_{port_name}"
    board = _board_pose(rng, "sfp")
    nic_translation = rng.uniform(*LIMITS["nic_translation"])
    nic_yaw = rng.uniform(*LIMITS["nic_yaw"])
    gripper_offset = _gripper_offset(rng, "sfp")
    cable_rpy = _cable_rpy(rng, args)

    task_board = {"pose": board}
    task_board.update(_nic_rails(nic_rail, nic_translation, nic_yaw))
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
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[str, dict, dict]:
    """하나의 SC trial과 추적 metadata를 uniform randomization으로 만든다."""
    sc_rail = rng.randrange(SC_RAIL_COUNT)
    task_id = f"portoffset_sc_{index:04d}_rail{sc_rail}"
    board = _board_pose(rng, "sc")
    sc_translation = rng.uniform(*LIMITS["sc_translation"])
    gripper_offset = _gripper_offset(rng, "sc")
    cable_rpy = _cable_rpy(rng, args)

    task_board = {"pose": board}
    task_board.update(
        {f"nic_rail_{rail}": {"entity_present": False} for rail in range(SFP_NIC_RAIL_COUNT)}
    )
    task_board.update(_sc_rails(sc_rail, sc_translation))
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
        board=board,
        gripper_offset=gripper_offset,
        cable_rpy=cable_rpy,
        sc_translation=sc_translation,
    )
    return task_id, trial, {task_id: metadata}


def _enabled_port_types(args: argparse.Namespace) -> list[str]:
    """CLI 문자열에서 지원되는 포트 종류만 순서를 유지해 추출한다."""
    values = [token.strip().lower() for token in args.port_types.split(",")]
    port_types = [value for value in values if value in {"sfp", "sc"}]
    return port_types or ["sfp", "sc"]


def make_trial_config(
    index: int,
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[dict, dict]:
    """포트 순서를 선택하고 AIC engine config와 metadata를 완성한다."""
    port_types = _enabled_port_types(args)
    if args.port_order == "round_robin":
        port_type = port_types[index % len(port_types)]
    else:
        port_type = rng.choice(port_types)
    if port_type == "sc":
        task_id, trial, scenario_params = _make_sc_trial(index, rng, args)
    else:
        task_id, trial, scenario_params = _make_sfp_trial(index, rng, args)
    robot = _robot_section(rng, args.robot_joint_noise_deg)
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
