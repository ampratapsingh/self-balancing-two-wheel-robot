"""Experiment E05: Disturbance response — apply external impulse at t=5s."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def run_disturbance(
    robot_cfg: dict[str, Any],
    sim_cfg: dict[str, Any],
    pid_cfg: dict[str, Any],
    lqr_cfg: dict[str, Any],
    exp_cfg: dict[str, Any],
    output_dir: str | Path,
    gui: bool = False,
) -> list[dict]:
    """Run E05: disturbance response for PID and LQR controllers.

    Uses the built-in disturbance_cfg support in Simulation.
    """
    from balancing_robot.controllers.pid import PIDController
    from balancing_robot.controllers.lqr import LQRController
    from balancing_robot.dynamics import compute_model_params, compute_state_space
    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    out_dir = Path(output_dir) / "data" / "E05"
    duration_s = float(exp_cfg.get("duration_s", 20.0))
    seed = int(exp_cfg.get("seed", 42))
    initial_tilt_deg = float(exp_cfg.get("initial_tilt_deg", 3.0))
    controllers_to_run = exp_cfg.get("controllers", ["pid", "lqr"])

    # Disturbance config
    dist = exp_cfg.get("disturbance", None)
    disturbance_cfg = None
    if dist:
        disturbance_cfg = {
            "apply_at_s": float(dist.get("apply_at_s", 5.0)),
            "force_N": dist.get("force_N", [10.0, 0.0, 0.0]),
            "duration_s": float(dist.get("duration_s", 0.1)),
            "height_m": float(dist.get("height_m", 0.3)),
        }

    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)
    results = []
    for ctrl_name in controllers_to_run:
        if ctrl_name == "pid":
            controller = PIDController.from_config(pid_cfg)
            active_controller_cfg = pid_cfg
        else:
            controller = LQRController.from_config(lqr_cfg, A, B)
            active_controller_cfg = lqr_cfg

        run_id = f"E05_{ctrl_name}_dist_seed{seed}"
        sim = Simulation(
            sim_cfg=sim_cfg,
            robot_cfg=robot_cfg,
            controller=controller,
            gui=gui,
            initial_tilt_deg=initial_tilt_deg,
            duration_s=duration_s,
            experiment_id="E05",
            run_id=run_id,
            seed=seed,
            disturbance_cfg=disturbance_cfg,
            controller_cfg=active_controller_cfg,
        )
        result = sim.run()
        csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
        entry = {
            "controller": ctrl_name,
            "success": result.success,
            "fall_time": result.fall_time,
            "csv": str(csv_path),
        }
        logger.info("E05 %s disturbance: success=%s fall_time=%s",
                    ctrl_name, result.success, result.fall_time)
        results.append(entry)

    return results
