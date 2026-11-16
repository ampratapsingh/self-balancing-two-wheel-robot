"""YAML configuration loading and validation."""
from __future__ import annotations
import logging
from pathlib import Path
from typing import Any
import yaml

logger = logging.getLogger(__name__)
REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML file. Accepts absolute or repo-relative paths."""
    p = Path(path)
    if not p.is_absolute():
        p = REPO_ROOT / p
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")
    with open(p) as f:
        data = yaml.safe_load(f)
    logger.debug("Loaded config: %s", p)
    return data


def load_robot_config(path: str | Path | None = None) -> dict[str, Any]:
    return load_yaml(path or REPO_ROOT / "config" / "robot_config.yaml")


def load_simulation_config(path: str | Path | None = None) -> dict[str, Any]:
    return load_yaml(path or REPO_ROOT / "config" / "simulation_config.yaml")


def load_pid_config(path: str | Path | None = None) -> dict[str, Any]:
    return load_yaml(path or REPO_ROOT / "config" / "pid_config.yaml")


def load_lqr_config(path: str | Path | None = None) -> dict[str, Any]:
    return load_yaml(path or REPO_ROOT / "config" / "lqr_config.yaml")


def load_experiment_config(experiment_id: str) -> dict[str, Any]:
    return load_yaml(REPO_ROOT / "config" / "experiments" / f"{experiment_id}.yaml")
