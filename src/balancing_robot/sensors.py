"""
Sensor module: extracts robot state from PyBullet simulation.

State vector: [x, x_dot, theta, theta_dot]
  x       : chassis horizontal position (m), positive forward (+X world)
  x_dot   : chassis horizontal velocity (m/s)
  theta   : pitch angle (rad), 0 = upright, >0 = forward lean
  theta_dot: pitch angular velocity (rad/s)

Tilt extraction: derived from base orientation quaternion.
  The pitch angle theta is the signed angle of the chassis Z-axis (body up)
  relative to world Z, projected onto the world XZ plane.
  This avoids gimbal-lock issues with Euler angle decomposition.

  Method:
    q = quaternion of base link (from getBasePositionAndOrientation)
    rot_matrix = matrix_from_quat(q)   # 3x3
    # chassis "up" direction in world frame = third column of rot matrix
    chassis_up_world = rot_matrix[:, 2]
    # project onto world XZ plane (forward-vertical plane):
    # theta = atan2(chassis_up_world[0], chassis_up_world[2])
    # = angle of chassis up-axis from world Z, in the XZ plane
    # = forward tilt angle (positive when leaning forward)
    theta = atan2(chassis_up_world[0], chassis_up_world[2])

  For the pitch rate:
    omega_world = angular velocity of base (from getBaseVelocity()[1])
    # Express in body frame: omega_body = rot_matrix.T @ omega_world
    # Pitch rate = component about Y-axis in world frame
    theta_dot = omega_world[1]   # Y-component of world angular velocity = pitch rate
    # Note: for small tilts, body-frame and world-frame Y are nearly identical.
    # For larger tilts, using body-frame Y-component is more accurate.
    # We use world-frame Y for simplicity; document this choice.

Optional features (off by default):
  - Gaussian noise on tilt and tilt rate
  - One-step delay buffer
  - Low-pass filter on tilt rate

"""
from __future__ import annotations

import logging
from collections import deque
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


class Sensors:
    """Extracts and optionally corrupts state estimates from PyBullet.

    Args:
        robot_id: PyBullet body ID of the robot.
        physics_client: PyBullet physics client ID.
        noise_std_theta: Gaussian std dev for tilt noise (rad). 0 = disabled.
        noise_std_theta_dot: Gaussian std dev for tilt-rate noise (rad/s). 0 = disabled.
        delay_steps: Number of physics steps to delay sensor readings. 0 = no delay.
        lpf_alpha: Low-pass filter coefficient for tilt rate.
            filtered = alpha * prev + (1-alpha) * raw.
            1.0 = no filtering (pass through raw).
        rng: Optional numpy random generator for deterministic noise.
    """

    def __init__(
        self,
        robot_id: int,
        physics_client: int,
        noise_std_theta: float = 0.0,
        noise_std_theta_dot: float = 0.0,
        delay_steps: int = 0,
        lpf_alpha: float = 1.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.client = physics_client
        self.noise_std_theta = noise_std_theta
        self.noise_std_theta_dot = noise_std_theta_dot
        self.delay_steps = delay_steps
        self.lpf_alpha = lpf_alpha
        self.rng = rng or np.random.default_rng()

        self._delay_buffer: deque = deque()
        self._lpf_state: float = 0.0
        self._first_read: bool = True

    def reset(self) -> None:
        """Reset delay buffer and filter state."""
        self._delay_buffer.clear()
        self._lpf_state = 0.0
        self._first_read = True

    def get_state(self) -> dict[str, float]:
        """Read the current robot state from PyBullet.

        Returns:
            dict with keys: x, x_dot, theta, theta_dot
                x       (m)     : horizontal position
                x_dot   (m/s)   : horizontal velocity
                theta   (rad)   : pitch angle (0=upright, >0=forward)
                theta_dot (rad/s): pitch angular velocity
        """
        import pybullet as p

        # --- Position and orientation ---
        pos, quat = p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.client
        )
        # --- Velocity ---
        lin_vel, ang_vel = p.getBaseVelocity(
            self.robot_id, physicsClientId=self.client
        )

        x = float(pos[0])
        x_dot = float(lin_vel[0])

        # --- Tilt extraction from quaternion ---
        theta, theta_dot = self._extract_tilt(quat, ang_vel)

        # --- Apply noise ---
        if self.noise_std_theta > 0.0:
            theta += float(self.rng.normal(0.0, self.noise_std_theta))
        if self.noise_std_theta_dot > 0.0:
            raw_theta_dot = theta_dot + float(
                self.rng.normal(0.0, self.noise_std_theta_dot)
            )
        else:
            raw_theta_dot = theta_dot

        # --- Low-pass filter on tilt rate ---
        if self._first_read:
            self._lpf_state = raw_theta_dot
            self._first_read = False
        self._lpf_state = self.lpf_alpha * self._lpf_state + (1.0 - self.lpf_alpha) * raw_theta_dot
        theta_dot_filtered = self._lpf_state

        raw_state = {
            "x": x,
            "x_dot": x_dot,
            "theta": theta,
            "theta_dot": theta_dot_filtered,
        }

        # --- Delay buffer ---
        if self.delay_steps == 0:
            return raw_state

        self._delay_buffer.append(raw_state)
        if len(self._delay_buffer) <= self.delay_steps:
            # Not enough history yet: return zeros
            return {"x": 0.0, "x_dot": 0.0, "theta": 0.0, "theta_dot": 0.0}
        return self._delay_buffer.popleft()

    @staticmethod
    def _extract_tilt(
        quat: tuple[float, float, float, float],
        ang_vel_world: tuple[float, float, float],
    ) -> tuple[float, float]:
        """Extract tilt angle and rate from quaternion and angular velocity.

        The quaternion convention in PyBullet is (x, y, z, w).

        Theta is the signed angle of the chassis Z-axis (body "up") from
        world Z, measured in the world XZ (forward-vertical) plane.

        Args:
            quat: (x, y, z, w) quaternion of the base link.
            ang_vel_world: (wx, wy, wz) angular velocity in world frame (rad/s).

        Returns:
            (theta, theta_dot): pitch angle (rad) and pitch rate (rad/s).
        """
        # Build rotation matrix from quaternion (PyBullet convention: x,y,z,w)
        qx, qy, qz, qw = quat
        # Rotation matrix columns are the body axes expressed in world frame
        # Third column = body Z axis = "up" direction of chassis in world
        # Formula: R[:, 2] = [2(qx*qz + qy*qw), 2(qy*qz - qx*qw), 1 - 2(qx² + qy²)]
        up_x = 2.0 * (qx * qz + qy * qw)
        up_y = 2.0 * (qy * qz - qx * qw)
        up_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        # Theta = angle of chassis "up" from world Z, in the XZ plane
        # atan2(up_x, up_z): positive when up_x > 0 i.e. chassis tilts forward
        theta = float(np.arctan2(up_x, up_z))

        # Pitch rate: Y-component of world angular velocity
        # (For small tilts, world-Y ≈ body-Y pitch axis)
        theta_dot = float(ang_vel_world[1])

        return theta, theta_dot

    @staticmethod
    def state_to_array(state: dict[str, float]) -> np.ndarray:
        """Convert state dict to numpy array [x, x_dot, theta, theta_dot]."""
        return np.array([state["x"], state["x_dot"], state["theta"], state["theta_dot"]])

    @staticmethod
    def tilt_from_quat_known(quat: tuple, expected_theta: float, tol: float = 0.01) -> bool:
        """Validate that tilt extraction gives expected result for a known pose.

        Args:
            quat: (x,y,z,w) quaternion.
            expected_theta: Expected tilt in radians.
            tol: Tolerance in radians.

        Returns:
            True if extracted theta matches expected within tolerance.
        """
        theta, _ = Sensors._extract_tilt(quat, (0.0, 0.0, 0.0))
        return abs(theta - expected_theta) < tol
