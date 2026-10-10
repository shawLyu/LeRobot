"""Resolve source and robot data paths independently of the working directory."""

import os
from pathlib import Path

WORKSPACE_ENV = "LEROBOT_ARM_WORKSPACE"


def source_root():
    """The source checkout or the runtime deployment containing this file."""
    return Path(__file__).resolve().parents[3]


def workspace_root():
    """Use the checkout by default, or one explicit existing robot workspace.

    A workspace supplies all resources; missing configuration never falls back
    to another robot's defaults. Select it before importing runtime modules.
    """
    configured = os.environ.get(WORKSPACE_ENV)
    root = Path(configured).expanduser().resolve() if configured else source_root()
    if not root.is_dir():
        raise FileNotFoundError(f"Robot workspace does not exist: {root}")
    return root


def resource_path(relative):
    """Find a required resource in the selected workspace, failing on missing data."""
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("resource paths must be relative to the workspace")
    path = workspace_root() / relative
    if not path.exists():
        raise FileNotFoundError(f"Missing robot resource: {path}")
    return path


def source_environment():
    """Make local subprocesses use this source tree without an editable installation."""
    environment = os.environ.copy()
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = str(source_root() / "src")
    if existing:
        environment["PYTHONPATH"] += os.pathsep + existing
    return environment
