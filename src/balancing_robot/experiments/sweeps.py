"""Experiment E06: Initial tilt sweep — test max stabilizable tilt for PID and LQR."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def run_tilt_sweep(
    robot_cfg: dict[str, Any],
    sim_cfg: dict[str, Any],
    pid_cfg: dict[str, Any],
    lqr_cfg: dict[str, Any],
    exp_cfg: dict[str, Any],
    output_dir: str | Path,
    gui: bool = False,
) -> list[dict]:
    """Run E06: sweep initial tilt angles and record success/failure for PID and LQR."""
    from balancing_robot.controllers.pid import PIDController
    from balancing_robot.controllers.lqr import LQRController
    from balancing_robot.dynamics import compute_model_params, compute_state_space
    from balancing_robot.simulation import Simulation
    from balancing_robot.analysis.io import save_run

    out_dir = Path(output_dir) / "data" / "E06"
    duration_s = float(exp_cfg.get("duration_s", 15.0))
    seed = int(exp_cfg.get("seed", 42))
    tilt_range = exp_cfg.get("tilt_range_deg", [1, 2, 3, 5, 7, 10, 12, 15, 18, 20, 25, 30])
    controllers_to_run = exp_cfg.get("controllers", ["pid", "lqr"])

    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)

    results = []
    for ctrl_name in controllers_to_run:
        for tilt in tilt_range:
            if ctrl_name == "pid":
                controller = PIDController.from_config(pid_cfg)
            else:
                controller = LQRController.from_config(lqr_cfg, A, B)

            run_id = f"E06_{ctrl_name}_tilt{tilt}deg_seed{seed}"
            sim = Simulation(
                sim_cfg=sim_cfg,
                robot_cfg=robot_cfg,
                controller=controller,
                gui=gui,
                initial_tilt_deg=float(tilt),
                duration_s=duration_s,
                experiment_id="E06",
                run_id=run_id,
                seed=seed,
            )
            result = sim.run()
            csv_path, json_path = save_run(result.rows, result.metadata, out_dir, result.run_id)
            entry = {
                "controller": ctrl_name,
                "tilt_deg": tilt,
                "success": result.success,
                "fall_time": result.fall_time,
                "csv": str(csv_path),
            }
            logger.info("E06 %s tilt=%.0f°: success=%s fall_time=%s",
                        ctrl_name, tilt, result.success, result.fall_time)
            results.append(entry)

    return results
