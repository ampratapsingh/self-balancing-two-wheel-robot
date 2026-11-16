#!/usr/bin/env python3
"""CLI entry point for PID vs LQR controller comparison (Experiment E04)."""
import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(
        description="Compare PID and LQR controllers across initial tilt angles (E04)"
    )
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "results"), help="Output directory")
    parser.add_argument("--duration", type=float, default=20.0, help="Duration per run (s)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logger = logging.getLogger(__name__)

    # Runs experiment E04: PID vs LQR at multiple initial tilts
    try:
        from balancing_robot.experiments.comparison import run_comparison
    except ImportError as e:
        logger.error("Experiment module not yet implemented: %s", e)
        logger.error("Run after Phase 2 implementation is complete.")
        sys.exit(1)

    logger.info("Running E04: PID vs LQR comparison ...")
    run_comparison(output_dir=Path(args.output_dir), duration_s=args.duration, seed=args.seed)
    logger.info("E04 complete. Results in %s", args.output_dir)


if __name__ == "__main__":
    main()
