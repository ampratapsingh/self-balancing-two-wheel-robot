"""Experiment E01: Uncontrolled fall. E02: PID balancing. E03: LQR balancing."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def run_uncontrolled(
    robot_cfg: dict[str, Any],
    sim_cfg: dict[str, Any],
    exp_cfg: dict[str, Any],
    output_dir: str | Path,
    gui: bool = False,
) -> dict:
    """Run E01: uncontrolled fall experiment."""
    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=None,
        gui=gui,
        initial_tilt_deg=float(exp_cfg.get("initial_tilt_deg", 2.0)),
        duration_s=float(exp_cfg.get("duration_s", 5.0)),
        experiment_id=exp_cfg.get("experiment_id", "E01"),
        run_id="E01_uncontrolled_seed0",
        seed=int(exp_cfg.get("seed", 42)),
    )
    result = sim.run()

    out_dir = Path(output_dir) / "data" / "E01"
    csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
    logger.info("E01 complete: fell=%s fall_time=%s", not result.success, result.fall_time)
    return {"csv": str(csv_path), "metadata": result.metadata}


def run_pid(
    robot_cfg: dict[str, Any],
    sim_cfg: dict[str, Any],
    pid_cfg: dict[str, Any],
    exp_cfg: dict[str, Any],
    output_dir: str | Path,
    gui: bool = False,
) -> dict:
    """Run E02: PID balancing experiment."""
    from balancing_robot.controllers.pid import PIDController
    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    controller = PIDController.from_config(pid_cfg)
    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=controller,
        gui=gui,
        initial_tilt_deg=float(exp_cfg.get("initial_tilt_deg", 3.0)),
        duration_s=float(exp_cfg.get("duration_s", 30.0)),
        experiment_id=exp_cfg.get("experiment_id", "E02"),
        run_id="E02_pid_seed42",
        seed=int(exp_cfg.get("seed", 42)),
    )
    result = sim.run()
    out_dir = Path(output_dir) / "data" / "E02"
    csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
    return {"csv": str(csv_path), "success": result.success, "metadata": result.metadata}


def run_lqr(
    robot_cfg: dict[str, Any],
    sim_cfg: dict[str, Any],
    lqr_cfg: dict[str, Any],
    exp_cfg: dict[str, Any],
    output_dir: str | Path,
    gui: bool = False,
) -> dict:
    """Run E03: LQR balancing experiment."""
    from balancing_robot.dynamics import compute_model_params, compute_state_space
    from balancing_robot.controllers.lqr import LQRController
    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)
    controller = LQRController.from_config(lqr_cfg, A, B)
    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=controller,
        gui=gui,
        initial_tilt_deg=float(exp_cfg.get("initial_tilt_deg", 3.0)),
        duration_s=float(exp_cfg.get("duration_s", 30.0)),
        experiment_id=exp_cfg.get("experiment_id", "E03"),
        run_id="E03_lqr_seed42",
        seed=int(exp_cfg.get("seed", 42)),
    )
    result = sim.run()
    out_dir = Path(output_dir) / "data" / "E03"
    csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
    return {"csv": str(csv_path), "success": result.success, "metadata": result.metadata}
