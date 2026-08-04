#!/usr/bin/env python3
"""실제 시나리오 상수와 CLI 기본값으로 randomization 분포 그래프를 생성한다."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np

from phy_data_collection.portoffset_randomization.cli import build_parser
from phy_data_collection.portoffset_randomization.constants import (
    LIMITS,
    ROOT,
    SC_RAIL_COUNT,
    SFP_NIC_RAIL_COUNT,
    SFP_PORT_COUNT,
)

DEFAULT_OUTPUT = ROOT / "readme" / "photo" / "scenario_randomization_distributions.png"


def _collector_defaults() -> argparse.Namespace:
    """수집 runner parser에서 현재 기본값을 읽는다."""
    return build_parser().parse_args([])


def _parse_args() -> argparse.Namespace:
    """그래프 출력 및 시나리오 override 인자를 파싱한다."""
    defaults = _collector_defaults()
    parser = argparse.ArgumentParser(
        description="Plot PortOffsetCollect scenario randomization distributions."
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--dpi", type=int, default=170)
    parser.add_argument("--port-types", default=defaults.port_types)
    parser.add_argument(
        "--port-order",
        choices=("random", "round_robin"),
        default=defaults.port_order,
    )
    parser.add_argument(
        "--light-intensity-scale-min",
        type=float,
        default=defaults.light_intensity_scale_min,
    )
    parser.add_argument(
        "--light-intensity-scale-max",
        type=float,
        default=defaults.light_intensity_scale_max,
    )
    parser.add_argument(
        "--light-color-jitter",
        type=float,
        default=defaults.light_color_jitter,
    )
    parser.add_argument(
        "--light-pose-xy-jitter-m",
        type=float,
        default=defaults.light_pose_xy_jitter_m,
    )
    parser.add_argument(
        "--light-pose-z-jitter-m",
        type=float,
        default=defaults.light_pose_z_jitter_m,
    )
    parser.add_argument("--ambient-min", type=float, default=defaults.ambient_min)
    parser.add_argument("--ambient-max", type=float, default=defaults.ambient_max)
    parser.add_argument("--background-min", type=float, default=defaults.background_min)
    parser.add_argument("--background-max", type=float, default=defaults.background_max)
    return parser.parse_args()


def _enabled_port_types(value: str) -> list[str]:
    """CLI 문자열에서 그래프에 표시할 지원 포트 타입을 추출한다."""
    values = [token.strip().lower() for token in value.split(",")]
    enabled = list(dict.fromkeys(token for token in values if token in {"sfp", "sc"}))
    return enabled or ["sfp", "sc"]


def _gaussian_parameters(minimum: float, maximum: float) -> tuple[float, float]:
    """±3σ 경계에서 Gaussian 평균과 표준편차를 계산한다."""
    if minimum > maximum:
        raise ValueError(f"minimum {minimum} exceeds maximum {maximum}")
    return (minimum + maximum) / 2.0, (maximum - minimum) / 6.0


def _format_interval(values: tuple[float, float], unit: str = "") -> str:
    """그래프 설명에 사용할 범위 문자열을 만든다."""
    suffix = f" {unit}" if unit else ""
    return f"{values[0]:g} … {values[1]:g}{suffix}"


def _configure_style() -> dict[str, str]:
    """공통 Matplotlib 스타일과 색상 팔레트를 적용한다."""
    colors = {
        "navy": "#17233C",
        "blue": "#2F6BFF",
        "cyan": "#28B8D5",
        "orange": "#FF8A3D",
        "green": "#32A071",
        "grid": "#D9E1EC",
        "muted": "#5B667A",
    }
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titleweight": "bold",
            "axes.titlesize": 13,
            "axes.labelcolor": colors["navy"],
            "text.color": colors["navy"],
            "xtick.color": colors["muted"],
            "ytick.color": colors["muted"],
        }
    )
    return colors


def _geometry_text(port_types: list[str]) -> str:
    """활성 포트 타입에 대응하는 Uniform geometry 범위를 설명한다."""
    sections: list[str] = []
    if "sfp" in port_types:
        sections.append(
            "Task Board (SFP)\n"
            f"  X  {_format_interval(LIMITS['sfp_board_x'], 'm')}\n"
            f"  Y  {_format_interval(LIMITS['sfp_board_y'], 'm')}\n"
            f"  yaw {_format_interval(LIMITS['sfp_board_yaw'], 'rad')}\n\n"
            "SFP insertion port\n"
            f"  translation {_format_interval(LIMITS['nic_translation'], 'm')}\n"
            "  yaw "
            f"{_format_interval(tuple(math.degrees(value) for value in LIMITS['nic_yaw']), 'deg')}"
        )
    if "sc" in port_types:
        sections.append(
            "Task Board (SC)\n"
            f"  X  {_format_interval(LIMITS['sc_board_x'], 'm')}\n"
            f"  Y  {_format_interval(LIMITS['sc_board_y'], 'm')}\n"
            f"  yaw {_format_interval(LIMITS['sc_board_yaw'], 'rad')}\n\n"
            "SC insertion port\n"
            f"  translation {_format_interval(LIMITS['sc_translation'], 'm')}\n"
            "  local yaw 0; world yaw follows board"
        )
    return "\n\n".join(sections)


def _draw_geometry_panel(
    info_ax: plt.Axes,
    plot_ax: plt.Axes,
    port_types: list[str],
    colors: dict[str, str],
) -> None:
    """Task Board 설명과 Uniform 그래프를 서로 다른 axes에 그린다."""
    info_ax.axis("off")
    info_ax.add_patch(
        FancyBboxPatch(
            (0.0, 0.0),
            1.0,
            0.94,
            boxstyle="round,pad=0.008",
            transform=info_ax.transAxes,
            facecolor="#F2F6FF",
            edgecolor="#C9D8FF",
            linewidth=1.0,
            clip_on=False,
        )
    )
    info_ax.text(
        0.03,
        0.91,
        _geometry_text(port_types),
        transform=info_ax.transAxes,
        va="top",
        fontsize=8.8,
        linespacing=1.2,
    )

    x = np.linspace(0.0, 1.0, 400)
    plot_ax.fill_between(x, 0.0, 1.0, color=colors["blue"], alpha=0.18)
    plot_ax.plot(x, np.ones_like(x), color=colors["blue"], linewidth=3)
    plot_ax.vlines([0.0, 1.0], 0.0, 1.0, color=colors["blue"], linewidth=2)
    plot_ax.set_xlim(-0.04, 1.04)
    plot_ax.set_ylim(0.0, 1.18)
    plot_ax.set_xlabel("Normalized position within [minimum, maximum]")
    plot_ax.set_ylabel("Relative density")
    plot_ax.set_yticks([0.0, 1.0], ["0", "constant"])
    plot_ax.grid(axis="x", color=colors["grid"], linewidth=0.8)


def _selection_data(port_types: list[str]) -> tuple[list[str], list[float], list[str]]:
    """활성 포트 타입과 공유 상수로 categorical probability 막대를 만든다."""
    labels: list[str] = []
    probabilities: list[float] = []
    groups: list[str] = []
    type_probability = 1.0 / len(port_types)
    for port_type in port_types:
        labels.append(f"{port_type.upper()}")
        probabilities.append(type_probability)
        groups.append("type")
    if "sfp" in port_types:
        labels.extend(
            [
                f"SFP rail\n(0–{SFP_NIC_RAIL_COUNT - 1})",
                f"SFP port\n(0–{SFP_PORT_COUNT - 1})",
            ]
        )
        probabilities.extend([1.0 / SFP_NIC_RAIL_COUNT, 1.0 / SFP_PORT_COUNT])
        groups.extend(["sfp", "sfp"])
    if "sc" in port_types:
        labels.append(f"SC rail\n(0–{SC_RAIL_COUNT - 1})")
        probabilities.append(1.0 / SC_RAIL_COUNT)
        groups.append("sc")
    return labels, probabilities, groups


def _draw_selection_panel(
    ax: plt.Axes,
    port_types: list[str],
    port_order: str,
    colors: dict[str, str],
) -> None:
    """포트 타입과 rail/index의 이산 선택 확률을 그린다."""
    labels, probabilities, groups = _selection_data(port_types)
    palette = {"type": colors["cyan"], "sfp": colors["orange"], "sc": colors["green"]}
    positions = np.arange(len(labels))
    bars = ax.bar(
        positions,
        probabilities,
        color=[palette[group] for group in groups],
        width=0.68,
        edgecolor="white",
        linewidth=1.5,
    )
    for bar, value in zip(bars, probabilities):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            value + 0.025,
            f"{value:.2g}",
            ha="center",
            fontweight="bold",
        )
    ax.set_xticks(positions, labels)
    ax.tick_params(axis="x", labelsize=8.5)
    ax.set_ylim(0.0, max(0.66, max(probabilities) + 0.16))
    ax.set_ylabel("Selection probability")
    ax.grid(axis="y", color=colors["grid"], linewidth=0.8)
    ax.set_axisbelow(True)

def _lighting_rows(args: argparse.Namespace) -> list[tuple[str, float, float, str]]:
    """현재 CLI 값에서 조명별 μ, σ와 경계 문자열을 계산한다."""
    ambient = _gaussian_parameters(args.ambient_min, args.ambient_max)
    background = _gaussian_parameters(args.background_min, args.background_max)
    intensity = _gaussian_parameters(
        args.light_intensity_scale_min,
        args.light_intensity_scale_max,
    )
    return [
        ("ambient", *ambient, f"[{args.ambient_min:g}, {args.ambient_max:g}]"),
        (
            "background",
            *background,
            f"[{args.background_min:g}, {args.background_max:g}]",
        ),
        (
            "intensity",
            *intensity,
            f"[{args.light_intensity_scale_min:g}, {args.light_intensity_scale_max:g}]",
        ),
        (
            "RGB Δ",
            0.0,
            args.light_color_jitter / 3.0,
            f"[{-args.light_color_jitter:g}, {args.light_color_jitter:g}]",
        ),
        (
            "pose ΔX/Y",
            0.0,
            args.light_pose_xy_jitter_m / 3.0,
            f"[{-args.light_pose_xy_jitter_m:g}, {args.light_pose_xy_jitter_m:g}] m",
        ),
        (
            "pose ΔZ",
            0.0,
            args.light_pose_z_jitter_m / 3.0,
            f"[{-args.light_pose_z_jitter_m:g}, {args.light_pose_z_jitter_m:g}] m",
        ),
    ]


def _draw_lighting_panel(
    info_ax: plt.Axes,
    plot_ax: plt.Axes,
    args: argparse.Namespace,
    colors: dict[str, str],
) -> None:
    """조명 파라미터 설명과 Gaussian 그래프를 서로 다른 axes에 그린다."""
    lighting_text = "\n".join(
        f"{name:<12} μ {mean:<7.4g} σ {sigma:<7.4g} {bounds}"
        for name, mean, sigma, bounds in _lighting_rows(args)
    )
    info_ax.axis("off")
    info_ax.add_patch(
        FancyBboxPatch(
            (0.0, 0.0),
            1.0,
            0.90,
            boxstyle="round,pad=0.008",
            transform=info_ax.transAxes,
            facecolor="#FFF6EE",
            edgecolor="#FFD2B3",
            linewidth=1.0,
            clip_on=False,
        )
    )
    info_ax.text(
        0.03,
        0.85,
        lighting_text,
        transform=info_ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.5,
        linespacing=1.45,
    )

    z = np.linspace(-3.0, 3.0, 500)
    pdf = np.exp(-0.5 * z**2) / math.sqrt(2.0 * math.pi)
    plot_ax.fill_between(z, 0.0, pdf, color=colors["orange"], alpha=0.22)
    plot_ax.plot(z, pdf, color=colors["orange"], linewidth=3)
    plot_ax.axvline(
        0.0,
        color=colors["navy"],
        linewidth=1.4,
        linestyle="--",
        alpha=0.75,
    )
    plot_ax.axvline(-3.0, color=colors["muted"], linewidth=1.2, linestyle=":")
    plot_ax.axvline(3.0, color=colors["muted"], linewidth=1.2, linestyle=":")
    plot_ax.text(0.0, pdf.max() + 0.008, "μ", ha="center", fontweight="bold")
    plot_ax.text(-3.0, 0.012, "−3σ", ha="center", color=colors["muted"])
    plot_ax.text(3.0, 0.012, "+3σ", ha="center", color=colors["muted"])
    plot_ax.set_xlim(-3.35, 3.35)
    plot_ax.set_ylim(0.0, 0.44)
    plot_ax.set_xlabel("Standardized deviation from mean")
    plot_ax.set_ylabel("Probability density")
    plot_ax.grid(color=colors["grid"], linewidth=0.8)


def generate_graph(args: argparse.Namespace) -> Path:
    """현재 시나리오 파라미터로 PNG 분포 그래프를 생성한다."""
    colors = _configure_style()
    port_types = _enabled_port_types(args.port_types)
    figure = plt.figure(figsize=(16, 8.5), facecolor="#F7F9FC")
    grid = figure.add_gridspec(
        2,
        3,
        height_ratios=[0.06, 0.94],
        width_ratios=[1.15, 1.0, 1.3],
        hspace=0.04,
        wspace=0.23,
    )
    panel_titles = ("1. Task Board", "2. Target Port", "3. Lighting")
    for index, title in enumerate(panel_titles):
        panel_title_ax = figure.add_subplot(grid[0, index])
        panel_title_ax.axis("off")
        panel_title_ax.text(
            0.5,
            0.0,
            title,
            transform=panel_title_ax.transAxes,
            ha="center",
            va="bottom",
            fontsize=13,
            fontweight="bold",
        )
    geometry_grid = grid[1, 0].subgridspec(
        2,
        1,
        height_ratios=[0.52, 0.48],
        hspace=0.03,
    )
    geometry_info_ax = figure.add_subplot(geometry_grid[0], facecolor="white")
    geometry_plot_ax = figure.add_subplot(geometry_grid[1], facecolor="white")
    selection_ax = figure.add_subplot(grid[1, 1], facecolor="white")
    lighting_grid = grid[1, 2].subgridspec(
        2,
        1,
        height_ratios=[0.23, 0.77],
        hspace=0.03,
    )
    lighting_info_ax = figure.add_subplot(lighting_grid[0], facecolor="white")
    lighting_plot_ax = figure.add_subplot(lighting_grid[1], facecolor="white")
    axes = [geometry_plot_ax, selection_ax, lighting_plot_ax]
    _draw_geometry_panel(geometry_info_ax, geometry_plot_ax, port_types, colors)
    _draw_selection_panel(selection_ax, port_types, args.port_order, colors)
    _draw_lighting_panel(lighting_info_ax, lighting_plot_ax, args, colors)
    for ax in axes:
        for spine in ax.spines.values():
            spine.set_color("#DCE3ED")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(
        args.output,
        dpi=args.dpi,
        bbox_inches="tight",
        facecolor=figure.get_facecolor(),
    )
    plt.close(figure)
    return args.output


def main() -> None:
    """CLI entrypoint."""
    output = generate_graph(_parse_args())
    print(output)


if __name__ == "__main__":
    main()
