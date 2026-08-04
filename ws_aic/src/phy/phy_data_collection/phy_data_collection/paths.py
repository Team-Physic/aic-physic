"""Workspace path discovery shared by installed data-collection commands."""

from __future__ import annotations

import os
from pathlib import Path


def find_project_root() -> Path:
    """Return the repository root containing the vendored AIC Pixi workspace."""
    override = os.environ.get("PHY_PROJECT_ROOT")
    if override:
        root = Path(override).expanduser().resolve()
        workspace = root / "ws_aic" / "src"
        if (workspace / "pixi.toml").is_file() and (workspace / "aic").is_dir():
            return root
        raise RuntimeError(
            f"PHY_PROJECT_ROOT does not contain the AIC Pixi workspace: {root}"
        )

    for candidate in Path(__file__).resolve().parents:
        workspace = candidate / "ws_aic" / "src"
        if (workspace / "pixi.toml").is_file() and (workspace / "aic").is_dir():
            return candidate
    raise RuntimeError(
        "Could not locate the project root; set PHY_PROJECT_ROOT explicitly."
    )


PROJECT_ROOT = find_project_root()
