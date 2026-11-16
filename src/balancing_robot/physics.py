"""
Physics engine initialisation and management.

Wraps PyBullet physics setup:
- Connects to DIRECT (headless) or GUI mode
- Sets gravity, timestep, solver parameters
- Loads the ground plane
- Provides deterministic step-advance
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

GROUND_PLANE_URDF = "plane.urdf"


class PhysicsEngine:
    """Manages a single PyBullet physics world.

    Args:
        sim_cfg: Loaded simulation_config.yaml dict.
        gui: If True, open the GUI window. If False, use DIRECT (headless).
    """

    def __init__(self, sim_cfg: dict[str, Any], gui: bool = False) -> None:
        import pybullet as p
        import pybullet_data

        self.cfg = sim_cfg
        self.gui = gui
        self._client: int = -1

        mode = p.GUI if gui else p.DIRECT
        self._client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client)

        # Gravity
        g = float(sim_cfg["physics"]["gravity"])  # negative value = downward
        p.setGravity(0, 0, g, physicsClientId=self._client)

        # Timestep and solver
        phys = sim_cfg["physics"]
        dt = float(phys["timestep"])
        p.setPhysicsEngineParameter(
            fixedTimeStep=dt,
            numSolverIterations=int(phys.get("num_solver_iterations", 50)),
            numSubSteps=int(phys.get("num_sub_steps", 1)),
            physicsClientId=self._client,
        )

        # Disable real-time simulation (use manual stepping)
        p.setRealTimeSimulation(0, physicsClientId=self._client)

        # Load ground plane
        self._plane_id = p.loadURDF(
            GROUND_PLANE_URDF, physicsClientId=self._client
        )

        self._dt = dt
        logger.info(
            "PhysicsEngine: mode=%s dt=%.6f client=%d",
            "GUI" if gui else "DIRECT", dt, self._client,
        )

    @property
    def client(self) -> int:
        return self._client

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def plane_id(self) -> int:
        return self._plane_id

    def step(self) -> None:
        """Advance the simulation by one physics timestep."""
        import pybullet as p
        p.stepSimulation(physicsClientId=self._client)

    def reset_camera(self, distance: float, yaw: float, pitch: float,
                     target: tuple = (0, 0, 0.3)) -> None:
        """Reset the debug camera (GUI mode only)."""
        if not self.gui:
            return
        import pybullet as p
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target,
            physicsClientId=self._client,
        )

    def disconnect(self) -> None:
        """Disconnect from physics server."""
        import pybullet as p
        try:
            p.disconnect(physicsClientId=self._client)
        except Exception:
            pass

    def __del__(self):
        self.disconnect()
