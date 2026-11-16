"""CSV and JSON I/O for experiment data."""
from __future__ import annotations

import json
import logging
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import pandas as pd

from balancing_robot.simulation import CSV_COLUMNS
from balancing_robot.config import REPO_ROOT

logger = logging.getLogger(__name__)


def _runtime_metadata() -> dict[str, str | int | None]:
    """Return lightweight provenance fields without requiring PyBullet to be installed."""
    try:
        completed = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
        )
        git_commit = completed.stdout.strip() if completed.returncode == 0 else None
    except OSError:
        git_commit = None

    try:
        import pybullet as p
        pybullet_version: str | int | None = getattr(p, "__version__", p.getAPIVersion())
    except ImportError:
        pybullet_version = None

    return {
        "git_commit": git_commit,
        "python_version": sys.version.split()[0],
        "pybullet_version": pybullet_version,
    }


def save_run(
    rows: list[dict],
    metadata: dict[str, Any],
    output_dir: str | Path,
    run_id: str,
) -> tuple[Path, Path]:
    """Save a simulation run's data and metadata.

    Args:
        rows: List of data row dicts (one per timestep).
        metadata: Metadata dict (config snapshot, seed, etc.).
        output_dir: Directory to save into.
        run_id: Unique run identifier (used for filenames).

    Returns:
        (csv_path, json_path) of saved files.
    """
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    csv_path = out / f"{run_id}.csv"
    json_path = out / f"{run_id}.json"

    df = pd.DataFrame(rows, columns=CSV_COLUMNS)
    df.to_csv(csv_path, index=False)

    # Preserve the caller's dictionary while recording the full provenance for this file.
    record_metadata = {
        **metadata,
        **_runtime_metadata(),
        "saved_at": datetime.now().astimezone().isoformat(),
        "csv_file": str(csv_path),
    }
    with open(json_path, "w") as f:
        json.dump(record_metadata, f, indent=2, default=str)

    logger.info("Saved run: %s", csv_path)
    return csv_path, json_path


def load_run(csv_path: str | Path) -> pd.DataFrame:
    """Load a run CSV into a DataFrame."""
    return pd.read_csv(csv_path)


def load_metadata(json_path: str | Path) -> dict[str, Any]:
    """Load run metadata from JSON."""
    with open(json_path) as f:
        return json.load(f)
