from __future__ import annotations

"""Static configuration defaults for PortOffsetCollect."""

import numpy as np
import os


def _env_float(name: str, default: float) -> float:
    """환경변수를 float로 읽고 누락되거나 잘못되면 기본값을 반환한다."""
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    """환경변수를 int로 읽고 누락되거나 잘못되면 기본값을 반환한다."""
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


STIFFNESS_DEFAULT = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
DAMPING_DEFAULT = [40.0, 40.0, 40.0, 20.0, 20.0, 20.0]

COLLECT_BASE_Z_OFFSET_DEFAULT: float = 0.020
TOOL0_TO_TCP_Z: float = 0.1965
SFP_PLUG_REFERENCE_OFFSET_IN_CABLE_TIP_FRAME = np.array(
    [0.0, 0.0021125, 0.0],
    dtype=float,
)

TF_RECONSTRUCTION_POSITION_TOLERANCE_M: float = 1e-4  # 0.1 mm
TF_RECONSTRUCTION_ANGLE_TOLERANCE_RAD: float = 1e-3  # about 0.057 deg

INITIAL_LIFT_M: float = _env_float("AIC_DISTANCE_INITIAL_LIFT_M", 0.050)
INITIAL_LIFT_STEPS: int = _env_int("AIC_DISTANCE_INITIAL_LIFT_STEPS", 40)
INITIAL_LIFT_DT: float = _env_float("AIC_DISTANCE_INITIAL_LIFT_DT", 0.05)
INITIAL_LIFT_SETTLE_S: float = _env_float("AIC_DISTANCE_INITIAL_LIFT_SETTLE_S", 0.50)

APPROACH_TCP_OFFSET = np.array(
    [
        _env_float("AIC_APPROACH_TCP_OFFSET_X_M", 0.0),
        _env_float("AIC_APPROACH_TCP_OFFSET_Y_M", 0.015),
        _env_float("AIC_APPROACH_TCP_OFFSET_Z_M", 0.045),
    ],
    dtype=float,
)
APPROACH_VISION_RETRIES: int = _env_int("AIC_APPROACH_VISION_RETRIES", 20)
APPROACH_RETRY_DT: float = _env_float("AIC_APPROACH_RETRY_DT", 0.1)
APPROACH_NEAR_Z_OFFSET_M: float = _env_float("AIC_APPROACH_NEAR_Z_OFFSET_M", 0.020)
APPROACH_STEPS: int = _env_int("AIC_APPROACH_STEPS", 80)
APPROACH_DT: float = _env_float("AIC_APPROACH_DT", 0.05)
APPROACH_SETTLE_S: float = _env_float("AIC_APPROACH_SETTLE_S", 0.50)
APPROACH_STIFFNESS = [180.0, 180.0, 180.0, 45.0, 45.0, 45.0]
APPROACH_DAMPING = [75.0, 75.0, 75.0, 18.0, 18.0, 18.0]
APPROACH_NEAR_STIFFNESS = [140.0, 140.0, 140.0, 40.0, 40.0, 40.0]
APPROACH_NEAR_DAMPING = [65.0, 65.0, 65.0, 16.0, 16.0, 16.0]

TOOL0_TO_OPTICAL = {
    "left": (
        [-0.100516584, -0.058032593, -0.008935891],
        [-0.113039947, 0.065265728, -0.495722390, 0.858616135],
    ),
    "center": (
        [-0.000000001, -0.116079183, -0.008937891],
        [-0.130528330, 0.000001827, -0.000000288, 0.991444580],
    ),
    "right": (
        [0.100516583, -0.058032595, -0.008935891],
        [-0.113041775, -0.065262563, 0.495721890, 0.858616424],
    ),
}
