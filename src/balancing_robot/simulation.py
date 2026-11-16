"""
Main simulation loop.

Ties together PhysicsEngine + Robot + Sensors + Controller into a runnable loop.
Supports both headless (experiment) and GUI (demo) modes.
Collects data row-by-row for CSV export.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from balancing_robot.physics import PhysicsEngine
from balancing_robot.robot import Robot
from balancing_robot.sensors import Sensors
from balancing_robot.controllers.base import BaseController

logger = logging.getLogger(__name__)

# CSV column names (matches the brief's data schema exactly)
CSV_COLUMNS = [
    "experiment_id", "controller", "run_id", "seed", "time_s",
    "tilt_rad", "tilt_deg", "tilt_rate_rad_s",
    "x_m", "x_vel_m_s",
    "left_wheel_vel_rad_s", "right_wheel_vel_rad_s",
    "left_torque_cmd_Nm", "right_torque_cmd_Nm",
    "left_torque_applied_Nm", "right_torque_applied_Nm",
    "disturbance_force_N", "status",
]


@dataclass
class SimulationResult:
    """Container for simulation run data."""
    experiment_id: str = "E00"
    controller_name: str = "none"
    run_id: str = "run_0"
    seed: int = 42
    rows: list[dict] = field(default_factory=list)
    success: bool = False
    fall_time: float | None = None
    metadata: dict = field(default_factory=dict)


class Simulation:
    """Main simulation loop.

    Args:
        sim_cfg: simulation_config.yaml dict.
        robot_cfg: robot_config.yaml dict.
        controller: A BaseController instance (or None for uncontrolled).
        gui: If True, open PyBullet GUI.
        initial_tilt_deg: Initial tilt at spawn (degrees).
        duration_s: Simulation duration in seconds.
        experiment_id: ID string for data labelling.
        run_id: Run identifier string.
        seed: Random seed.
        disturbance_cfg: Optional disturbance parameters dict.
    """

    def __init__(
        self,
        sim_cfg: dict[str, Any],
        robot_cfg: dict[str, Any],
        controller: BaseController | None,
        gui: bool = False,
        initial_tilt_deg: float = 3.0,
        duration_s: float = 20.0,
        experiment_id: str = "E00",
        run_id: str = "run_0",
        seed: int = 42,
        disturbance_cfg: dict | None = None,
        urdf_path: str | None = None,
    ) -> None:
        self.sim_cfg = sim_cfg
        self.robot_cfg = robot_cfg
        self.controller = controller
        self.gui = gui
        self.initial_tilt_deg = initial_tilt_deg
        self.duration_s = duration_s
        self.experiment_id = experiment_id
        self.run_id = run_id
        self.seed = seed
        self.disturbance_cfg = disturbance_cfg

        self.fall_threshold_rad = math.radians(
            float(sim_cfg.get("fall_threshold_deg", 45.0))
        )
        self.dt = float(sim_cfg["physics"]["timestep"])
        ctrl_hz = float(sim_cfg.get("controller_rate_hz", 240.0))
        self.ctrl_step_divisor = max(1, round(1.0 / (self.dt * ctrl_hz)))

        # Resolve URDF path
        if urdf_path is None:
            from balancing_robot.config import REPO_ROOT
            urdf_path = str(REPO_ROOT / robot_cfg.get("urdf_path", "assets/robot/balancing_robot.urdf"))

        self._urdf_path = urdf_path
        self._physics: PhysicsEngine | None = None
        self._robot: Robot | None = None
        self._sensors: Sensors | None = None

    def run(self) -> SimulationResult:
        """Execute the full simulation and return collected data."""
        result = SimulationResult(
            experiment_id=self.experiment_id,
            controller_name=self.controller.name if self.controller else "none",
            run_id=self.run_id,
            seed=self.seed,
        )

        self._physics = PhysicsEngine(self.sim_cfg, gui=self.gui)
        self._robot = Robot(
            self._urdf_path,
            self._physics.client,
            self.robot_cfg,
            self.initial_tilt_deg,
        )
        rng = np.random.default_rng(self.seed)
        self._sensors = Sensors(
            self._robot.robot_id,
            self._physics.client,
            rng=rng,
        )

        if self.controller:
            self.controller.reset()

        n_steps = int(self.duration_s / self.dt)
        fell = False
        tau_cmd = 0.0

        # GUI text overlay
        text_id = -1
        if self.gui:
            cam = self.sim_cfg.get("camera", {})
            self._physics.reset_camera(
                float(cam.get("distance", 1.5)),
                float(cam.get("yaw", 45.0)),
                float(cam.get("pitch", -20.0)),
            )

        disturbance_active = False
        disturbance_start_step = None
        disturbance_end_step = None
        disturbance_force = (0.0, 0.0, 0.0)
        if self.disturbance_cfg:
            dist_t = float(self.disturbance_cfg["apply_at_s"])
            dist_dur = float(self.disturbance_cfg.get("duration_s", 0.1))
            disturbance_start_step = int(dist_t / self.dt)
            disturbance_end_step = int((dist_t + dist_dur) / self.dt)
            f = self.disturbance_cfg["force_N"]
            disturbance_force = (float(f[0]), float(f[1]), float(f[2]))

        for step in range(n_steps):
            t = step * self.dt

            # Read sensor state
            state = self._sensors.get_state()
            theta = state["theta"]
            status = "BALANCING"

            # Check fall condition
            if abs(theta) > self.fall_threshold_rad:
                status = "FELL"
                fell = True
                result.fall_time = t
                # Record this step and break
                self._record_row(result, state, t, tau_cmd, 0.0, status)
                break

            # Controller update (at controller rate)
            if self.controller and (step % self.ctrl_step_divisor == 0):
                tau_cmd = self.controller.compute(state, self.dt * self.ctrl_step_divisor)

            # Apply torques
            tau_per_wheel_cmd = tau_cmd / 2.0 if tau_cmd != 0.0 else 0.0
            if self.controller:
                tau_l, tau_r = self._robot.apply_torque(tau_cmd)
            else:
                tau_l, tau_r = 0.0, 0.0

            # Apply disturbance force if scheduled
            dist_force_magnitude = 0.0
            if (disturbance_start_step is not None and
                    disturbance_start_step <= step < disturbance_end_step):
                height = float(self.disturbance_cfg.get("height_m", 0.3))
                pos_base, _ = __import__("pybullet").getBasePositionAndOrientation(
                    self._robot.robot_id, physicsClientId=self._physics.client
                )
                force_pos = (pos_base[0], pos_base[1], height)
                self._robot.apply_external_force(disturbance_force, force_pos)
                dist_force_magnitude = float(np.linalg.norm(disturbance_force))

            # Step physics
            self._physics.step()

            # Record data
            self._record_row(result, state, t, tau_l, dist_force_magnitude, status)

            # GUI camera follow
            if self.gui and step % 10 == 0:
                self._update_gui(state, tau_cmd, status, t)

        result.success = not fell and (result.fall_time is None)
        result.metadata = {
            "dt": self.dt,
            "duration_s": self.duration_s,
            "initial_tilt_deg": self.initial_tilt_deg,
            "controller": self.controller.name if self.controller else "none",
            "n_steps": n_steps,
            "fell": fell,
        }
        logger.info(
            "Simulation done: success=%s fall_time=%s controller=%s",
            result.success, result.fall_time, result.controller_name,
        )

        self._physics.disconnect()
        return result

    def _record_row(
        self,
        result: SimulationResult,
        state: dict,
        t: float,
        tau_per_wheel: float,
        dist_force: float,
        status: str,
    ) -> None:
        """Append one data row to result.rows."""
        wl, wr = self._robot.get_wheel_velocities()
        tl_applied, tr_applied = self._robot.get_applied_torques()
        result.rows.append({
            "experiment_id": self.experiment_id,
            "controller": result.controller_name,
            "run_id": self.run_id,
            "seed": self.seed,
            "time_s": round(t, 6),
            "tilt_rad": round(state["theta"], 8),
            "tilt_deg": round(math.degrees(state["theta"]), 6),
            "tilt_rate_rad_s": round(state["theta_dot"], 8),
            "x_m": round(state["x"], 8),
            "x_vel_m_s": round(state["x_dot"], 8),
            "left_wheel_vel_rad_s": round(wl, 8),
            "right_wheel_vel_rad_s": round(wr, 8),
            "left_torque_cmd_Nm": round(tau_per_wheel, 8),
            "right_torque_cmd_Nm": round(tau_per_wheel, 8),
            "left_torque_applied_Nm": round(tl_applied, 8),
            "right_torque_applied_Nm": round(tr_applied, 8),
            "disturbance_force_N": round(dist_force, 4),
            "status": status,
        })

    def _update_gui(self, state: dict, tau: float, status: str, t: float) -> None:
        """Update camera and overlay text in GUI mode."""
        import pybullet as p
        # Follow robot position
        pos_base, _ = p.getBasePositionAndOrientation(
            self._robot.robot_id, physicsClientId=self._physics.client
        )
        cam = self.sim_cfg.get("camera", {})
        p.resetDebugVisualizerCamera(
            cameraDistance=float(cam.get("distance", 1.5)),
            cameraYaw=float(cam.get("yaw", 45.0)),
            cameraPitch=float(cam.get("pitch", -20.0)),
            cameraTargetPosition=[pos_base[0], pos_base[1], 0.3],
            physicsClientId=self._physics.client,
        )
