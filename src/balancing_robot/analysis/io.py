"""CSV and JSON I/O for experiment data."""
from __future__ import annotations

import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any

import pandas as pd

from balancing_robot.simulation import CSV_COLUMNS

logger = logging.getLogger(__name__)


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

    metadata["saved_at"] = datetime.utcnow().isoformat()
    metadata["csv_file"] = str(csv_path)
    with open(json_path, "w") as f:
        json.dump(metadata, f, indent=2, default=str)

    logger.info("Saved run: %s", csv_path)
    return csv_path, json_path


def load_run(csv_path: str | Path) -> pd.DataFrame:
    """Load a run CSV into a DataFrame."""
    return pd.read_csv(csv_path)


def load_metadata(json_path: str | Path) -> dict[str, Any]:
    """Load run metadata from JSON."""
    with open(json_path) as f:
        return json.load(f)
