"""
Robot interface: loads the URDF, identifies joints by name, applies torques.

Joint name convention (must match URDF):
  left_wheel_joint  → index identified at load time
  right_wheel_joint → index identified at load time

Torque split:
  tau_left = tau_right = tau_total / 2.0
  (symmetric straight-line balancing)

Motor setup:
  Before switching to TORQUE_CONTROL, velocity motors are DISABLED by calling:
  setJointMotorControl2(VELOCITY_CONTROL, targetVelocity=0, force=0)
  This is mandatory for free-spinning wheels under torque control.
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"


class Robot:
    """Wraps the PyBullet robot body for the balancing robot.

    Args:
        urdf_path: Absolute path to the URDF file.
        physics_client: PyBullet client ID.
        robot_cfg: Loaded robot_config.yaml dict.
        initial_tilt_deg: Initial forward tilt at spawn (degrees).
    """

    def __init__(
        self,
        urdf_path: str | Path,
        physics_client: int,
        robot_cfg: dict[str, Any],
        initial_tilt_deg: float = 3.0,
    ) -> None:
        import pybullet as p

        self.client = physics_client
        self.cfg = robot_cfg
        self._urdf_path = str(urdf_path)
        self._robot_id: int = -1
        self._joint_map: dict[str, int] = {}
        self._left_idx: int = -1
        self._right_idx: int = -1
        self._max_torque_per_wheel = float(robot_cfg.get("max_torque_per_wheel", 5.0))

        self._spawn(initial_tilt_deg)

    def _spawn(self, initial_tilt_deg: float) -> None:
        """Load URDF and configure joints."""
        import pybullet as p

        wheel_r = float(self.cfg["wheel"]["radius"])
        spawn_z = wheel_r  # wheels touch ground

        # Convert initial tilt to quaternion (rotation about Y axis)
        theta0 = math.radians(initial_tilt_deg)
        # q = (sin(θ/2)*0, sin(θ/2)*1, sin(θ/2)*0, cos(θ/2)) — Y-axis rotation
        qy = math.sin(theta0 / 2.0)
        qw = math.cos(theta0 / 2.0)
        spawn_quat = (0.0, qy, 0.0, qw)

        self._robot_id = p.loadURDF(
            self._urdf_path,
            basePosition=[0.0, 0.0, spawn_z],
            baseOrientation=spawn_quat,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client,
        )

        # Build joint name → index map
        num_joints = p.getNumJoints(self._robot_id, physicsClientId=self.client)
        for i in range(num_joints):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self.client)
            joint_name = info[1].decode() if isinstance(info[1], bytes) else info[1]
            self._joint_map[joint_name] = i

        # Identify wheel joint indices by name
        if LEFT_WHEEL_JOINT not in self._joint_map:
            raise RuntimeError(
                f"Joint '{LEFT_WHEEL_JOINT}' not found in URDF. "
                f"Available joints: {list(self._joint_map.keys())}"
            )
        if RIGHT_WHEEL_JOINT not in self._joint_map:
            raise RuntimeError(
                f"Joint '{RIGHT_WHEEL_JOINT}' not found in URDF. "
                f"Available joints: {list(self._joint_map.keys())}"
            )

        self._left_idx = self._joint_map[LEFT_WHEEL_JOINT]
        self._right_idx = self._joint_map[RIGHT_WHEEL_JOINT]

        # CRITICAL: Disable default velocity motors before torque control
        # Without this, PyBullet's default joint motors resist motion
        for idx in [self._left_idx, self._right_idx]:
            p.setJointMotorControl2(
                self._robot_id, idx,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0,
                physicsClientId=self.client,
            )

        # Set wheel contact friction
        fric = self.cfg.get("wheel_friction", {})
        lat = float(fric.get("lateral", 0.8))
        roll = float(fric.get("rolling", 0.001))
        spin = float(fric.get("spinning", 0.001))
        for idx in [self._left_idx, self._right_idx]:
            p.changeDynamics(
                self._robot_id, idx,
                lateralFriction=lat,
                rollingFriction=roll,
                spinningFriction=spin,
                physicsClientId=self.client,
            )

        logger.info(
            "Robot spawned: id=%d left_idx=%d right_idx=%d tilt=%.1f°",
            self._robot_id, self._left_idx, self._right_idx, initial_tilt_deg,
        )

    @property
    def robot_id(self) -> int:
        return self._robot_id

    @property
    def left_joint_idx(self) -> int:
        return self._left_idx

    @property
    def right_joint_idx(self) -> int:
        return self._right_idx

    def apply_torque(self, tau_total: float) -> tuple[float, float]:
        """Apply torque to both wheels.

        Args:
            tau_total: Total torque (Nm). Split equally between wheels.
                Positive = forward motion.

        Returns:
            (tau_left, tau_right) actually commanded (after per-wheel saturation).
        """
        import pybullet as p

        tau_per_wheel = tau_total / 2.0
        # Saturate per-wheel torque
        tau_per_wheel = max(
            -self._max_torque_per_wheel,
            min(self._max_torque_per_wheel, tau_per_wheel),
        )

        for idx in [self._left_idx, self._right_idx]:
            p.setJointMotorControl2(
                self._robot_id, idx,
                controlMode=p.TORQUE_CONTROL,
                force=tau_per_wheel,
                physicsClientId=self.client,
            )

        return tau_per_wheel, tau_per_wheel

    def get_wheel_velocities(self) -> tuple[float, float]:
        """Return (left_wheel_vel, right_wheel_vel) in rad/s."""
        import pybullet as p
        ls_l = p.getJointState(self._robot_id, self._left_idx, physicsClientId=self.client)
        ls_r = p.getJointState(self._robot_id, self._right_idx, physicsClientId=self.client)
        return float(ls_l[1]), float(ls_r[1])

    def get_applied_torques(self) -> tuple[float, float]:
        """Return measured joint torques from getJointState (reaction forces).
        
        Note: PyBullet getJointState[3] is the applied torque in TORQUE_CONTROL.
        """
        import pybullet as p
        ls_l = p.getJointState(self._robot_id, self._left_idx, physicsClientId=self.client)
        ls_r = p.getJointState(self._robot_id, self._right_idx, physicsClientId=self.client)
        return float(ls_l[3]), float(ls_r[3])

    def reset(self, initial_tilt_deg: float = 3.0) -> None:
        """Remove old body and respawn."""
        import pybullet as p
        p.removeBody(self._robot_id, physicsClientId=self.client)
        self._joint_map.clear()
        self._spawn(initial_tilt_deg)

    def apply_external_force(
        self,
        force_world: tuple[float, float, float],
        pos_world: tuple[float, float, float],
    ) -> None:
        """Apply an external force to the chassis (for disturbance experiments).

        Args:
            force_world: Force vector in world frame (N).
            pos_world: Position in world frame where force is applied.
        """
        import pybullet as p
        p.applyExternalForce(
            self._robot_id,
            -1,  # base link
            forceObj=force_world,
            posObj=pos_world,
            flags=p.WORLD_FRAME,
            physicsClientId=self.client,
        )
