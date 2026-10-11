# YOLO + triangulation 성공 사례의 3D 오차 분포를 포트폴리오 그래프로 생성한다.
# 입력은 cases.csv의 error_mm와 summary.json의 평가 범위·요약값이다.
# 실패 사례는 오차 0으로 대체하지 않고 분포에서 제외하며, 성공 수를 함께 표시한다.
# 출력은 PNG, SVG, HTML이며 X축은 mm, Y축은 확률 밀도(1/mm)다.
# --cases/--summary는 입력 경로, --output은 출력 stem이다.
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def read_inputs(cases_path: Path, summary_path: Path) -> tuple[np.ndarray, dict]:
    """성공한 error_mm만 읽고 summary의 집계값과 일치하는지 검증한다."""
    values: list[float] = []
    with cases_path.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("error_mm", "").strip():
                values.append(float(row["error_mm"]))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    overall = summary["overall"]
    data = np.asarray(values, dtype=float)
    assert len(data) == overall["estimates"]
    assert math.isclose(float(np.median(data)), overall["median_mm"], rel_tol=1e-9)
    assert math.isclose(float(np.percentile(data, 95)), overall["p95_mm"], rel_tol=1e-9)
    assert math.isclose(float(data.max()), overall["max_mm"], rel_tol=1e-9)
    return data, summary


def kde_reflected(values: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """0 미만 오차가 생기지 않도록 반사 경계를 적용한 Gaussian KDE를 계산한다."""
    bandwidth = 1.06 * float(values.std(ddof=1)) * len(values) ** (-1 / 5)
    bandwidth = max(bandwidth, 0.01)
    kernel = np.exp(-0.5 * ((grid[:, None] - values[None, :]) / bandwidth) ** 2)
    reflected = np.exp(-0.5 * ((grid[:, None] + values[None, :]) / bandwidth) ** 2)
    return (kernel + reflected).mean(axis=1) / (bandwidth * math.sqrt(2 * math.pi))


def render(values: np.ndarray, summary: dict, output: Path) -> None:
    """실제 오차의 히스토그램·밀도 곡선과 median/p95 표시선을 저장한다."""
    font = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family": font.get_name(), "svg.fonttype": "path", "axes.unicode_minus": False})
    paper, ink, muted, orange, blue = "#ffffff", "#2d3142", "#4f5d75", "#eb6c36", "#2e5aa8"
    median = float(summary["overall"]["median_mm"])
    p95 = float(summary["overall"]["p95_mm"])
    maximum = float(summary["overall"]["max_mm"])
    display_max = 1.0  # 1.0 mm 이후 값은 전체 통계에는 유지하고 그래프에서만 잘라낸다.
    fig, ax = plt.subplots(figsize=(12, 7), facecolor=paper)
    fig.subplots_adjust(left=.12, right=.95, bottom=.22, top=.80)
    ax.set_facecolor(paper)
    bins = np.linspace(0, maximum + .05, 14)
    ax.hist(values, bins=bins, density=True, color="#cbd3df", edgecolor=muted, alpha=.85, label=f"오차 histogram (n={len(values)})")
    grid = np.linspace(0, maximum + .05, 500)
    ax.plot(grid, kde_reflected(values, grid), color=orange, linewidth=3, label="커널 밀도 추정")
    ax.axvline(median, color=blue, linewidth=2.2, linestyle="--", label=f"median {median:.3f} mm")
    ax.axvline(p95, color=orange, linewidth=2.2, linestyle=":", label=f"p95 {p95:.3f} mm")
    ax.axvspan(p95, min(maximum, display_max), color=orange, alpha=.08)
    ax.set_xlim(0, display_max)
    ax.set_xlabel("3D 위치 오차 (mm)", fontsize=13, labelpad=12, color=ink)
    ax.set_ylabel("확률 밀도 (1/mm)", fontsize=13, labelpad=12, color=ink)
    ax.grid(axis="y", color=ink, alpha=.10, linewidth=.8)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color(ink)
    ax.tick_params(colors=muted, labelsize=10)
    ax.legend(loc="upper right", frameon=False, fontsize=10)
    fig.text(.12, .915, "YOLO + Triangulation · 3D 위치 오차 분포", fontsize=23, weight="bold", color=ink)
    fig.text(.12, .865, f"성공한 추정 결과 {len(values)}개만 사용  |  median {median:.3f} mm  |  p95 {p95:.3f} mm", fontsize=12, color=muted)
    counts = summary["counts"]
    fig.text(.12, .095, f"평가 범위: {counts['selected_cases']} planned → {summary['overall']['eligible_cases']} eligible → {len(values)} estimates ({summary['overall']['estimate_rate']*100:.1f}%)", fontsize=10.5, color=muted)
    fig.text(.12, .055, "실패 사례는 분포에서 제외. 곡선은 167개 성공 오차의 분포 형태를 부드럽게 표시. p95: 95%가 이 값 이하.", fontsize=10, color=blue)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output.with_suffix(".png"), dpi=220, facecolor=paper)
    fig.savefig(output.with_suffix(".svg"), facecolor=paper)
    svg = output.with_suffix(".svg").read_text()
    svg = svg[svg.index("<svg"):]
    svg = svg.replace("<svg ", '<svg role="img" aria-labelledby="error-title error-desc" ', 1)
    pos = svg.index(">") + 1
    svg = svg[:pos] + '<title id="error-title">YOLO와 삼각측량 3D 위치 오차 분포</title><desc id="error-desc">성공한 추정 결과의 히스토그램과 커널 밀도 추정 곡선. median과 p95 세로선을 표시.</desc>' + svg[pos:]
    output.with_suffix(".svg").write_text(svg)
    output.with_suffix(".html").write_text("<!doctype html><html lang=\"ko\"><meta charset=\"utf-8\"><title>YOLO Triangulation Error Distribution</title><style>body{margin:0;background:#fff}svg{display:block;width:100%;height:auto}</style>" + svg + "</html>")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=Path, default=Path("/home/swlinux/Desktop/cases.csv"))
    parser.add_argument("--summary", type=Path, default=Path("/home/swlinux/Desktop/summary.json"))
    parser.add_argument("--output", type=Path, default=Path("evidence/portfolio/triangulation-error/triangulation-error-distribution"))
    args = parser.parse_args()
    values, summary = read_inputs(args.cases, args.summary)
    render(values, summary, args.output)
    print(args.output.with_suffix(".png"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
