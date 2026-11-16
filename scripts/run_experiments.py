#!/usr/bin/env python3
"""CLI entry point for running all E01-E06 experiments."""
import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description="Run all balancing robot experiments E01-E06")
    parser.add_argument("--all", action="store_true", help="Run all experiments E01-E06")
    parser.add_argument("--experiment", choices=["E01", "E02", "E03", "E04", "E05", "E06"],
                        help="Run a specific experiment by ID")
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "results"), help="Output directory")
    parser.add_argument("--headless", action="store_true", default=True, help="Run headless")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logger = logging.getLogger(__name__)

    if not args.all and not args.experiment:
        parser.print_help()
        sys.exit(1)

    experiment_ids = ["E01", "E02", "E03", "E04", "E05", "E06"] if args.all else [args.experiment]

    # Import experiment runners (implemented in Phase 2)
    try:
        from balancing_robot.experiments.baseline import run_uncontrolled, run_pid, run_lqr
        from balancing_robot.experiments.comparison import run_comparison
        from balancing_robot.experiments.disturbance import run_disturbance
        from balancing_robot.experiments.sweeps import run_tilt_sweep
    except ImportError as e:
        logger.error("Experiment modules not yet implemented: %s", e)
        logger.error("Run after Phase 2 implementation is complete.")
        sys.exit(1)

    runners = {
        "E01": run_uncontrolled,
        "E02": run_pid,
        "E03": run_lqr,
        "E04": run_comparison,
        "E05": run_disturbance,
        "E06": run_tilt_sweep,
    }

    out_dir = Path(args.output_dir)
    for eid in experiment_ids:
        logger.info("Running experiment %s ...", eid)
        try:
            runners[eid](output_dir=out_dir)
            logger.info("Experiment %s complete.", eid)
        except Exception as exc:
            logger.error("Experiment %s failed: %s", eid, exc)


if __name__ == "__main__":
    main()
