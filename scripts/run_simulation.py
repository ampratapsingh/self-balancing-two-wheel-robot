#!/usr/bin/env python3
"""CLI entry point for running the simulation."""
import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

def main():
    parser = argparse.ArgumentParser(description="Run self-balancing robot simulation")
    parser.add_argument("--controller", choices=["pid", "lqr", "none"], default="lqr")
    parser.add_argument("--gui", action="store_true", help="Open PyBullet GUI")
    parser.add_argument("--headless", action="store_true", help="Run headless (default)")
    parser.add_argument("--duration", type=float, default=20.0, help="Simulation duration (s)")
    parser.add_argument("--initial-tilt-deg", type=float, default=3.0)
    parser.add_argument(
        "--config-dir",
        type=Path,
        default=REPO_ROOT / "config",
        help="Directory containing robot, simulation, PID, and LQR YAML files",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "results"))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    from balancing_robot.config import load_robot_config, load_simulation_config, load_pid_config, load_lqr_config
    config_dir = args.config_dir.resolve()
    robot_cfg = load_robot_config(config_dir / "robot_config.yaml")
    sim_cfg = load_simulation_config(config_dir / "simulation_config.yaml")

    controller = None
    controller_cfg = None
    if args.controller == "pid":
        from balancing_robot.controllers.pid import PIDController
        pid_cfg = load_pid_config(config_dir / "pid_config.yaml")
        controller = PIDController.from_config(pid_cfg)
        controller_cfg = pid_cfg
    elif args.controller == "lqr":
        from balancing_robot.dynamics import compute_model_params, compute_state_space
        from balancing_robot.controllers.lqr import LQRController
        lqr_cfg = load_lqr_config(config_dir / "lqr_config.yaml")
        params = compute_model_params(robot_cfg)
        A, B = compute_state_space(params)
        controller = LQRController.from_config(lqr_cfg, A, B)
        controller_cfg = lqr_cfg

    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    gui = args.gui and not args.headless
    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=controller,
        gui=gui,
        initial_tilt_deg=args.initial_tilt_deg,
        duration_s=args.duration,
        experiment_id="E_manual",
        run_id=f"manual_{args.controller}_seed{args.seed}",
        seed=args.seed,
        controller_cfg=controller_cfg,
    )
    result = sim.run()
    out_dir = Path(args.output_dir) / "data" / "manual"
    csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
    print(f"Done. Success={result.success}. Data saved to {csv_path}")

if __name__ == "__main__":
    main()
