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

    # Load configs once
    from balancing_robot.config import (
        load_robot_config, load_simulation_config,
        load_pid_config, load_lqr_config, load_experiment_config,
    )
    robot_cfg = load_robot_config()
    sim_cfg = load_simulation_config()
    pid_cfg = load_pid_config()
    lqr_cfg = load_lqr_config()

    # Import experiment runners
    try:
        from balancing_robot.experiments.baseline import run_uncontrolled, run_pid, run_lqr
        from balancing_robot.experiments.comparison import run_comparison
        from balancing_robot.experiments.disturbance import run_disturbance
        from balancing_robot.experiments.sweeps import run_tilt_sweep
    except ImportError as e:
        logger.error("Experiment modules not yet implemented: %s", e)
        sys.exit(1)

    out_dir = Path(args.output_dir)

    exp_map = {
        "E01": lambda: run_uncontrolled(robot_cfg, sim_cfg, load_experiment_config("e01_uncontrolled"), out_dir),
        "E02": lambda: run_pid(robot_cfg, sim_cfg, pid_cfg, load_experiment_config("e02_pid_balancing"), out_dir),
        "E03": lambda: run_lqr(robot_cfg, sim_cfg, lqr_cfg, load_experiment_config("e03_lqr_balancing"), out_dir),
        "E04": lambda: run_comparison(robot_cfg, sim_cfg, pid_cfg, lqr_cfg,
                                       load_experiment_config("e04_comparison"), out_dir),
        "E05": lambda: run_disturbance(robot_cfg, sim_cfg, pid_cfg, lqr_cfg,
                                        load_experiment_config("e05_disturbance"), out_dir),
        "E06": lambda: run_tilt_sweep(robot_cfg, sim_cfg, pid_cfg, lqr_cfg,
                                       load_experiment_config("e06_tilt_sweep"), out_dir),
    }

    failed_experiments = []
    for eid in experiment_ids:
        logger.info("Running experiment %s ...", eid)
        try:
            result = exp_map[eid]()
            logger.info("Experiment %s complete: %s", eid, result)
        except Exception as exc:
            logger.error("Experiment %s failed: %s", eid, exc, exc_info=True)
            failed_experiments.append(eid)

    if failed_experiments:
        logger.error("Failed experiment(s): %s", ", ".join(failed_experiments))
        sys.exit(1)


if __name__ == "__main__":
    main()
