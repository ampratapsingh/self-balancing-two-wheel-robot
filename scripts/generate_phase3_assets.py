#!/usr/bin/env python3
"""Generate reproducible Phase 3 result tables and report figures from saved CSVs."""
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "balancing_robot_matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from balancing_robot.analysis.metrics import compute_metrics
from balancing_robot.analysis.plots import (
    plot_position_drift,
    plot_tilt_sweep_map,
    plot_tilt_vs_time,
    plot_torque_vs_time,
)


METRIC_COLUMNS = [
    "experiment_id",
    "controller",
    "initial_tilt_deg",
    "success",
    "fall_time_s",
    "max_abs_tilt_deg",
    "settling_time_s",
    "ss_tilt_error_deg",
    "rms_tilt_rad",
    "peak_torque_Nm",
    "rms_torque_Nm",
    "final_x_m",
    "peak_x_m",
]


def _records(data_dir: Path, experiment_id: str) -> list[dict[str, Any]]:
    """Load each saved run and calculate the project's fixed metrics."""
    records: list[dict[str, Any]] = []
    for csv_path in sorted((data_dir / experiment_id).glob("*.csv")):
        frame = pd.read_csv(csv_path)
        metadata_path = csv_path.with_suffix(".json")
        metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        metrics = compute_metrics(frame, duration_s=metadata.get("duration_s"))
        row = {
            "experiment_id": experiment_id,
            "controller": str(frame["controller"].iloc[0]).upper(),
            "initial_tilt_deg": metadata.get("initial_tilt_deg"),
            **metrics,
            "csv_file": str(csv_path.relative_to(REPO_ROOT)),
            "frame": frame,
        }
        records.append(row)
    return records


def _table_frame(records: list[dict[str, Any]], columns: list[str] = METRIC_COLUMNS) -> pd.DataFrame:
    """Return a report-friendly table, excluding in-memory frames and source paths."""
    frame = pd.DataFrame(records)
    return frame.loc[:, columns].sort_values(["controller", "initial_tilt_deg"], na_position="first")


def _write_table(frame: pd.DataFrame, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(output_path, index=False, float_format="%.6f")


def _plot_peak_torque(sweep: list[dict[str, Any]], output_path: Path) -> None:
    fig, axis = plt.subplots(figsize=(10, 5))
    for controller, color in [("PID", "#1f77b4"), ("LQR", "#d62728")]:
        rows = sorted((r for r in sweep if r["controller"] == controller), key=lambda r: r["initial_tilt_deg"])
        axis.plot(
            [r["initial_tilt_deg"] for r in rows],
            [r["peak_torque_Nm"] for r in rows],
            "o-",
            color=color,
            label=controller,
        )
    axis.axhline(10.0, color="black", linestyle="--", linewidth=0.8, label="total torque limit")
    axis.set_xlabel("Initial tilt (degrees)")
    axis.set_ylabel("Peak total commanded torque (Nm)")
    axis.set_title("E06: Peak torque across initial-tilt sweep")
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def generate(results_dir: Path, tables_dir: Path, figures_dir: Path) -> None:
    """Generate all Phase 3 tables and the comparison figures used in the report."""
    all_records = {experiment: _records(results_dir / "data", experiment) for experiment in ("E01", "E02", "E03", "E04", "E05", "E06")}

    _write_table(_table_frame(all_records["E01"]), tables_dir / "E01_uncontrolled_metrics.csv")
    nominal = all_records["E02"] + all_records["E03"]
    _write_table(_table_frame(nominal), tables_dir / "E02_E03_nominal_comparison.csv")
    _write_table(_table_frame(all_records["E04"]), tables_dir / "E04_comparison_table.csv")

    disturbance_rows = []
    for record in all_records["E05"]:
        frame = record["frame"]
        after_impulse = frame.loc[frame["time_s"] >= 5.0, "tilt_deg"].abs()
        disturbance_rows.append({
            **record,
            "post_disturbance_max_abs_tilt_deg": float(after_impulse.max()),
            "final_abs_tilt_deg": float(abs(frame["tilt_deg"].iloc[-1])),
        })
    disturbance_columns = [
        "experiment_id", "controller", "initial_tilt_deg", "success", "fall_time_s",
        "max_abs_tilt_deg", "post_disturbance_max_abs_tilt_deg", "final_abs_tilt_deg",
        "rms_tilt_rad", "peak_torque_Nm", "final_x_m",
    ]
    _write_table(_table_frame(disturbance_rows, disturbance_columns), tables_dir / "E05_disturbance_metrics.csv")

    sweep_columns = [
        "experiment_id", "controller", "initial_tilt_deg", "success", "fall_time_s",
        "max_abs_tilt_deg", "rms_tilt_rad", "peak_torque_Nm", "final_x_m",
    ]
    _write_table(_table_frame(all_records["E06"], sweep_columns), tables_dir / "E06_tilt_sweep.csv")

    figures_dir.mkdir(parents=True, exist_ok=True)
    for experiment in ("E01", "E02", "E03"):
        record = all_records[experiment][0]
        label = record["controller"]
        plot_tilt_vs_time({label: record["frame"]}, figures_dir / f"{experiment}_tilt.png", title=f"{experiment}: Tilt response")
        plot_torque_vs_time({label: record["frame"]}, figures_dir / f"{experiment}_torque.png", title=f"{experiment}: Total commanded torque")

    comparison_at_10 = {r["controller"]: r["frame"] for r in all_records["E04"] if r["initial_tilt_deg"] == 10.0}
    plot_tilt_vs_time(comparison_at_10, figures_dir / "E04_tilt.png", title="E04: PID vs LQR from 10° initial tilt")
    plot_torque_vs_time(comparison_at_10, figures_dir / "E04_torque.png", title="E04: PID vs LQR torque from 10° initial tilt")
    plot_position_drift({r["controller"]: r["frame"] for r in nominal}, figures_dir / "E02_E03_position_drift.png", title="Nominal position drift: PID vs LQR")

    disturbance_dfs = {r["controller"]: r["frame"] for r in all_records["E05"]}
    plot_tilt_vs_time(disturbance_dfs, figures_dir / "E05_tilt.png", title="E05: Response to 10 N disturbance at 5 s")
    plot_torque_vs_time(disturbance_dfs, figures_dir / "E05_torque.png", title="E05: Torque response to 10 N disturbance")

    sweep_points = [
        {"controller": r["controller"].lower(), "initial_tilt_deg": r["initial_tilt_deg"], "success": r["success"]}
        for r in all_records["E06"]
    ]
    plot_tilt_sweep_map(sweep_points, figures_dir / "E06_tilt.png", title="E06: Initial-tilt success map")
    _plot_peak_torque(all_records["E06"], figures_dir / "E06_torque.png")

    (tables_dir / "README.md").write_text(
        "# Generated result tables\n\n"
        "Generated from the saved CSV evidence by `scripts/generate_phase3_assets.py`. "
        "Metric definitions are fixed in `src/balancing_robot/analysis/metrics.py`.\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=REPO_ROOT / "results")
    parser.add_argument("--tables-dir", type=Path, default=REPO_ROOT / "results" / "tables")
    parser.add_argument("--figures-dir", type=Path, default=REPO_ROOT / "report" / "figures")
    args = parser.parse_args()
    generate(args.results_dir, args.tables_dir, args.figures_dir)


if __name__ == "__main__":
    main()
