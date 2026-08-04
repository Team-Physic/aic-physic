"""Workspace path discovery shared by installed data-collection commands."""

from __future__ import annotations

import os
from pathlib import Path


def find_project_root() -> Path:
    """Return the repository root containing ``ws_aic/src/aic``."""
    override = os.environ.get("PHY_PROJECT_ROOT")
    if override:
        root = Path(override).expanduser().resolve()
        if (root / "ws_aic" / "src" / "aic").is_dir():
            return root
        raise RuntimeError(
            f"PHY_PROJECT_ROOT does not contain ws_aic/src/aic: {root}"
        )

    for candidate in Path(__file__).resolve().parents:
        if (candidate / "ws_aic" / "src" / "aic").is_dir():
            return candidate
    raise RuntimeError(
        "Could not locate the project root; set PHY_PROJECT_ROOT explicitly."
    )


PROJECT_ROOT = find_project_root()
