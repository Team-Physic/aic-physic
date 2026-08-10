from __future__ import annotations

"""Small helpers and configuration readers for PortOffsetCollect."""

import os
import numpy as np
from pathlib import Path

def _env_bool(name: str, default: bool) -> bool:
    """환경변수의 일반적인 true/false 문자열을 bool로 변환한다."""
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", ""}

def _env_mm(name: str, default_mm: float) -> float:
    """밀리미터 환경변수를 읽어 미터 단위 float로 변환한다."""
    value = os.environ.get(name)
    if value is None:
        return default_mm / 1000.0
    try:
        return float(value) / 1000.0
    except ValueError:
        return default_mm / 1000.0

def _env_optional_mm(name: str) -> float | None:
    """선택적 밀리미터 환경변수를 미터로 변환하고 없으면 None을 반환한다."""
    value = os.environ.get(name)
    if value is None:
        return None
    try:
        return float(value) / 1000.0
    except ValueError:
        return None

def _env_mm_range(
    min_name: str,
    max_name: str,
    default_min_m: float,
    default_max_m: float,
) -> tuple[float, float]:
    """최소·최대 밀리미터 환경변수를 정렬된 미터 범위로 반환한다."""
    low = _env_optional_mm(min_name)
    high = _env_optional_mm(max_name)
    if low is None:
        low = default_min_m
    if high is None:
        high = default_max_m
    if low > high:
        low, high = high, low
    return low, high

def _env_deg(name: str, default_deg: float) -> float:
    """degree 환경변수를 읽어 radian 단위 float로 변환한다."""
    value = os.environ.get(name)
    if value is None:
        return np.deg2rad(default_deg)
    try:
        return np.deg2rad(float(value))
    except ValueError:
        return np.deg2rad(default_deg)

def _env_optional_deg(name: str) -> float | None:
    """선택적 degree 환경변수를 radian으로 변환하고 없으면 None을 반환한다."""
    value = os.environ.get(name)
    if value is None:
        return None
    try:
        return np.deg2rad(float(value))
    except ValueError:
        return None

def _env_deg_range(
    min_name: str,
    max_name: str,
    default_min_rad: float,
    default_max_rad: float,
) -> tuple[float, float]:
    """최소·최대 degree 환경변수를 정렬된 radian 범위로 반환한다."""
    low = _env_optional_deg(min_name)
    high = _env_optional_deg(max_name)
    if low is None:
        low = default_min_rad
    if high is None:
        high = default_max_rad
    if low > high:
        low, high = high, low
    return low, high

def _default_dataset_dir() -> Path:
    """dataset version을 반영한 기본 img2pos 데이터셋 경로를 반환한다."""
    base_dir = Path(__file__).resolve().parents[5] / "data" / "img2pos"
    version = os.environ.get("AIC_RPY_DATASET_VERSION", "").strip()
    return base_dir / version if version else base_dir
