"""Gazebo world 조명 randomization과 trial 설정 로그 출력."""

from __future__ import annotations

import argparse
import math
import os
import random
import xml.etree.ElementTree as ET
from pathlib import Path

from .constants import ANSI_COLORS, BASE_ROBOT_HOME, WORLD_TEMPLATE_PATH


def _color(
    args: argparse.Namespace,
    text: str,
    color: str,
    *,
    bold: bool = True,
) -> str:
    """CLI 설정과 NO_COLOR 환경변수를 존중해 로그 색상을 적용한다."""
    if not args.color_log or os.environ.get("NO_COLOR"):
        return text
    prefix = ANSI_COLORS.get(color, "")
    if bold:
        prefix = ANSI_COLORS["bold"] + prefix
    return f"{prefix}{text}{ANSI_COLORS['reset']}"


def _mm(value_m: float) -> str:
    """미터 값을 부호가 포함된 밀리미터 문자열로 변환한다."""
    return f"{value_m * 1000.0:+.1f}mm"


def _deg(value_rad: float) -> str:
    """라디안 값을 부호가 포함된 각도 문자열로 변환한다."""
    return f"{math.degrees(value_rad):+.1f}deg"


def _vec_mm(values: tuple[float, ...] | list[float]) -> str:
    """3차원 미터 벡터를 밀리미터 문자열로 변환한다."""
    return f"({_mm(float(values[0]))}, {_mm(float(values[1]))}, {_mm(float(values[2]))})"


def _vec_deg(values: tuple[float, ...] | list[float]) -> str:
    """3차원 라디안 벡터를 각도 문자열로 변환한다."""
    return f"({_deg(float(values[0]))}, {_deg(float(values[1]))}, {_deg(float(values[2]))})"


def log_trial_randomization(
    *,
    index: int,
    total: int,
    task_id: str,
    scenario: dict,
    lighting: dict,
    args: argparse.Namespace,
) -> None:
    """trial에 적용한 board, cable, robot, 조명 randomization을 요약 출력한다."""
    port_type = str(scenario.get("port_type", ""))
    rail_idx = int(scenario.get("rail_idx", -1))
    board_xyz = (
        float(scenario.get("board_x", 0.0)),
        float(scenario.get("board_y", 0.0)),
        1.14,
    )
    gripper_xyz = (
        float(scenario.get("gripper_offset_x", 0.0)),
        float(scenario.get("gripper_offset_y", 0.0)),
        float(scenario.get("gripper_offset_z", 0.0)),
    )
    cable_rpy = (
        float(scenario.get("cable_roll", 0.0)),
        float(scenario.get("cable_pitch", 0.0)),
        float(scenario.get("cable_yaw", 0.0)),
    )
    robot_home = scenario.get("robot_home_joint_positions", {}) or {}
    joint_delta_deg = {
        name: math.degrees(float(robot_home.get(name, base)) - float(base))
        for name, base in BASE_ROBOT_HOME.items()
    }

    print(_color(args, f"\n=== Trial {index + 1}/{total}: {task_id} ===", "blue"))
    board_yaw = _deg(float(scenario.get("board_yaw", 0.0)))
    print(f"{_color(args, '[Task Board]', 'cyan')} xyz={_vec_mm(board_xyz)} yaw={board_yaw}")
    if port_type == "sc":
        port_detail = (
            f"type=SC cards={scenario.get('combination_bits', '')} "
            f"active_rails={scenario.get('active_rails', [])} "
            f"target=sc_port_{rail_idx}/sc_port_base "
            f"sc_translation={_mm(float(scenario.get('sc_translation', 0.0)))}"
        )
    else:
        port_detail = (
            f"type=SFP cards={scenario.get('combination_bits', '')} "
            f"active_rails={scenario.get('active_rails', [])} target_rail={rail_idx} "
            f"port=sfp_port_{int(scenario.get('sfp_port_idx', -1))} "
            f"nic_translation={_mm(float(scenario.get('nic_translation', 0.0)))} "
            f"nic_yaw={_deg(float(scenario.get('nic_yaw', 0.0)))} "
            "background_sc_translation="
            f"{_mm(float(scenario.get('sc_translation', 0.0)))}"
        )
    print(_color(args, "[Port]", "yellow") + " " + port_detail)
    joint_noise = ", ".join(
        f"{name}:{value:+.1f}" for name, value in joint_delta_deg.items()
    )
    cable_label = _color(args, "[Cable / Robot]", "magenta")
    print(
        f"{cable_label} gripper_offset={_vec_mm(gripper_xyz)} "
        f"cable_rpy={_vec_deg(cable_rpy)} joint_noise_deg={{{joint_noise}}}"
    )

    sim_parts = [
        f"headless={bool(args.headless)}",
        f"trials={args.trials}",
        f"samples={args.samples_per_trial}",
    ]
    if not lighting.get("enabled"):
        sim_parts.append("lighting=randomization_disabled")
        reason = lighting.get("reason")
        if reason:
            sim_parts.append(f"reason={reason}")
        print(_color(args, "[Simulator / Lighting]", "green") + " " + "; ".join(sim_parts))
        return

    sim_parts.append(f"world={lighting.get('world_file', '')}")
    sim_parts.append(f"distribution={lighting.get('distribution', '')}")
    if "ambient" in lighting:
        sim_parts.append(f"ambient={float(lighting['ambient']):.3f}")
    if "background" in lighting:
        sim_parts.append(f"background={float(lighting['background']):.3f}")
    print(_color(args, "[Simulator / Lighting]", "green") + " " + "; ".join(sim_parts))
    for name, info in (lighting.get("lights") or {}).items():
        rgb = info.get("diffuse_rgb") or (1.0, 1.0, 1.0)
        pose = info.get("pose") or []
        pose_xyz = _vec_mm(pose[:3]) if len(pose) >= 3 else "n/a"
        text = (
            f"{name}:intensity={float(info.get('intensity', 0.0)):.2f} "
            f"scale={float(info.get('scale', 0.0)):.2f} "
            f"rgb=({float(rgb[0]):.2f},{float(rgb[1]):.2f},{float(rgb[2]):.2f}) "
            f"xyz={pose_xyz}"
        )
        print("  " + _color(args, "light", "green", bold=False) + " " + text)


def _format_sdf_float(value: float) -> str:
    """SDF XML에 넣을 실수를 간결하고 재현 가능한 문자열로 만든다."""
    return f"{value:.6g}"


def _truncated_gaussian(
    rng: random.Random,
    minimum: float,
    maximum: float,
) -> float:
    """주어진 범위를 ±3σ로 해석한 truncated Gaussian 표본을 반환한다."""
    if minimum > maximum:
        raise ValueError(f"minimum {minimum} exceeds maximum {maximum}")
    if minimum == maximum:
        return minimum
    mean = (minimum + maximum) / 2.0
    sigma = (maximum - minimum) / 6.0
    while True:
        value = rng.gauss(mean, sigma)
        if minimum <= value <= maximum:
            return value


def _randomized_color(
    rng: random.Random,
    jitter: float,
) -> tuple[float, float, float]:
    """흰색 기준 RGB 각 채널에 독립 Gaussian jitter를 적용한다."""
    return tuple(
        max(0.0, min(1.0, 1.0 + _truncated_gaussian(rng, -jitter, jitter)))
        for _ in range(3)
    )


def write_randomized_world(
    index: int,
    rng: random.Random,
    args: argparse.Namespace,
    world_path: Path,
) -> tuple[Path | None, dict]:
    """template world의 배경과 조명을 Gaussian 랜덤화해 trial 파일로 저장한다."""
    if not args.randomize_lighting:
        return None, {"enabled": False}
    if not WORLD_TEMPLATE_PATH.exists():
        print(f"[warn] world template not found: {WORLD_TEMPLATE_PATH}")
        return None, {"enabled": False, "reason": "template_missing"}

    tree = ET.parse(WORLD_TEMPLATE_PATH)
    root = tree.getroot()
    metadata = {
        "enabled": True,
        "distribution": "truncated_gaussian",
        "trial_index": index,
        "world_file": str(world_path),
        "lights": {},
    }
    world = root.find("world")
    if world is None:
        raise RuntimeError(
            f"Invalid SDF world file: missing <world> in {WORLD_TEMPLATE_PATH}"
        )

    scene = world.find("scene")
    if scene is not None:
        ambient = scene.find("ambient")
        if ambient is not None:
            level = _truncated_gaussian(rng, args.ambient_min, args.ambient_max)
            ambient.text = " ".join(_format_sdf_float(level) for _ in range(3))
            metadata["ambient"] = level
        background = scene.find("background")
        if background is not None:
            level = _truncated_gaussian(rng, args.background_min, args.background_max)
            formatted = _format_sdf_float(level)
            background.text = f"{formatted} {formatted} {formatted} 1"
            metadata["background"] = level

    for light in world.findall("light"):
        name = light.attrib.get("name", "")
        intensity = light.find("intensity")
        base_intensity = (
            float(intensity.text.strip())
            if intensity is not None and intensity.text
            else 1.0
        )
        scale = _truncated_gaussian(
            rng,
            args.light_intensity_scale_min,
            args.light_intensity_scale_max,
        )
        new_intensity = max(0.0, base_intensity * scale)
        if intensity is not None:
            intensity.text = _format_sdf_float(new_intensity)

        color = _randomized_color(rng, args.light_color_jitter)
        diffuse = light.find("diffuse")
        if diffuse is not None:
            diffuse.text = " ".join(
                [*(_format_sdf_float(channel) for channel in color), "1"]
            )

        pose = light.find("pose")
        pose_values: list[float] = []
        if pose is not None and pose.text:
            pose_values = [float(token) for token in pose.text.split()]
            if len(pose_values) >= 3:
                pose_values[0] += _truncated_gaussian(
                    rng,
                    -args.light_pose_xy_jitter_m,
                    args.light_pose_xy_jitter_m,
                )
                pose_values[1] += _truncated_gaussian(
                    rng,
                    -args.light_pose_xy_jitter_m,
                    args.light_pose_xy_jitter_m,
                )
                z_jitter = _truncated_gaussian(
                    rng,
                    -args.light_pose_z_jitter_m,
                    args.light_pose_z_jitter_m,
                )
                pose_values[2] = max(0.5, pose_values[2] + z_jitter)
                pose.text = " ".join(_format_sdf_float(value) for value in pose_values)

        metadata["lights"][name] = {
            "base_intensity": base_intensity,
            "intensity": new_intensity,
            "scale": scale,
            "diffuse_rgb": color,
            "pose": pose_values,
        }

    world_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    return world_path, metadata
