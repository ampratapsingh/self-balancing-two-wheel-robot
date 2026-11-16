#!/usr/bin/env python3
"""Regenerate all report figures from results/ CSVs."""
import argparse
import logging
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "balancing_robot_matplotlib"))

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description="Regenerate report figures from results/ CSVs")
    parser.add_argument("--results-dir", default=str(REPO_ROOT / "results"),
                        help="Directory containing experiment CSV results")
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "report" / "figures"),
                        help="Output directory for generated plots")
    parser.add_argument("--format", choices=["png", "pdf", "svg"], default="png",
                        help="Output figure format")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logger = logging.getLogger(__name__)

    results_dir = Path(args.results_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Only raw telemetry is valid input. Generated summary tables under
    # results/tables/ do not have a time series and must never replace runs.
    data_dir = results_dir / "data"
    csv_files = list(data_dir.rglob("*.csv")) if data_dir.is_dir() else []
    if not csv_files:
        logger.warning("No raw CSV files found in %s — run experiments first.", data_dir)
        sys.exit(0)

    logger.info("Found %d CSV file(s). Generating plots ...", len(csv_files))

    try:
        import pandas as pd
        from balancing_robot.analysis.plots import plot_tilt_vs_time, plot_torque_vs_time
        from balancing_robot.analysis.io import load_run
    except ImportError as e:
        logger.error("Analysis modules not yet implemented: %s", e)
        logger.error("Run after Phase 2 implementation is complete.")
        sys.exit(1)

    # Group by experiment_id
    runs: dict[str, dict[str, pd.DataFrame]] = {}
    for csv_path in sorted(csv_files):
        try:
            df = load_run(csv_path)
            exp_id = df["experiment_id"].iloc[0] if "experiment_id" in df.columns else csv_path.stem
            ctrl = df["controller"].iloc[0] if "controller" in df.columns else "unknown"
            runs.setdefault(exp_id, {})[ctrl] = df
        except Exception as exc:
            logger.warning("Skipping %s: %s", csv_path.name, exc)

    for exp_id, ctrl_dfs in runs.items():
        tilt_out = output_dir / f"{exp_id}_tilt.{args.format}"
        torque_out = output_dir / f"{exp_id}_torque.{args.format}"
        try:
            plot_tilt_vs_time(ctrl_dfs, str(tilt_out))
            plot_torque_vs_time(ctrl_dfs, str(torque_out))
            logger.info("Saved %s and %s", tilt_out.name, torque_out.name)
        except Exception as exc:
            logger.error("Failed to plot %s: %s", exp_id, exc)

    logger.info("Report assets written to %s", output_dir)


if __name__ == "__main__":
    main()
