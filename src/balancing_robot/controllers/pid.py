"""
PID controller for tilt-based balancing.

Convention:
  - error = theta_ref - theta  (default theta_ref = 0)
  - Positive error (forward lean, theta > 0) → positive torque output
    (drives wheels forward to bring chassis back upright)
  - Uses derivative-on-measurement (not on error) to avoid derivative kick
    on setpoint changes
  - Anti-windup: conditional integration (integral clamp)
  - Output saturated to [-max_torque_total, +max_torque_total]
"""
from __future__ import annotations

import logging
from typing import Any

from balancing_robot.controllers.base import BaseController

logger = logging.getLogger(__name__)


class PIDController(BaseController):
    """Tilt-based PID balancing controller.

    Args:
        Kp: Proportional gain.
        Ki: Integral gain.
        Kd: Derivative gain.
        theta_ref: Reference (setpoint) tilt angle (rad). Default 0 (upright).
        integral_clamp: Maximum absolute value of the integral term.
        derivative_filter_alpha: IIR filter coefficient for derivative.
            filtered = alpha * prev + (1-alpha) * raw_derivative.
            1.0 = no filtering.
        max_torque_total: Output saturation limit (Nm, total both wheels).
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        theta_ref: float = 0.0,
        integral_clamp: float = 10.0,
        derivative_filter_alpha: float = 0.7,
        max_torque_total: float = 10.0,
    ) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.theta_ref = theta_ref
        self.integral_clamp = integral_clamp
        self.derivative_filter_alpha = derivative_filter_alpha
        self.max_torque_total = max_torque_total

        # Internal state
        self._integral: float = 0.0
        self._prev_theta: float | None = None
        self._deriv_filtered: float = 0.0

    @property
    def name(self) -> str:
        return "PID"

    def reset(self) -> None:
        """Reset integrator and derivative filter."""
        self._integral = 0.0
        self._prev_theta = None
        self._deriv_filtered = 0.0

    def compute(self, state: dict, dt: float) -> float:
        """Compute total torque command.

        Args:
            state: dict with at least "theta" (rad) and optionally "theta_dot".
            dt: Timestep in seconds (must be > 0).

        Returns:
            Total torque (Nm). Positive = forward (wheels drive forward).
        """
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        theta = float(state["theta"])
        # error > 0 when leaning forward → positive torque → corrects forward lean
        error = theta - self.theta_ref

        # Proportional term
        p_term = self.Kp * error

        # Integral term with conditional anti-windup (clamping)
        self._integral += error * dt
        self._integral = max(-self.integral_clamp, min(self.integral_clamp, self._integral))
        i_term = self.Ki * self._integral

        # Derivative on measurement (not on error) to avoid derivative kick
        # d(theta)/dt ≈ (theta - prev_theta) / dt
        if self._prev_theta is None:
            raw_deriv = 0.0
        else:
            raw_deriv = (theta - self._prev_theta) / dt
        self._prev_theta = theta

        # Apply IIR low-pass filter to derivative
        # alpha=1.0 means no filtering (pass through raw); alpha=0.0 means freeze.
        # filtered = (1-alpha)*prev + alpha*raw
        alpha = self.derivative_filter_alpha
        self._deriv_filtered = (1.0 - alpha) * self._deriv_filtered + alpha * raw_deriv

        # Derivative term: derivative-on-measurement.
        # When theta increasing (leaning forward), deriv > 0 → positive D → more corrective torque
        d_term = self.Kd * self._deriv_filtered

        output = p_term + i_term + d_term

        # Saturate output
        output = max(-self.max_torque_total, min(self.max_torque_total, output))

        logger.debug(
            "PID: theta=%.4f err=%.4f P=%.3f I=%.3f D=%.3f out=%.3f",
            theta, error, p_term, i_term, d_term, output,
        )
        return output

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "PIDController":
        """Instantiate from a loaded pid_config.yaml dict."""
        return cls(
            Kp=float(cfg["Kp"]),
            Ki=float(cfg["Ki"]),
            Kd=float(cfg["Kd"]),
            theta_ref=float(cfg.get("theta_ref", 0.0)),
            integral_clamp=float(cfg.get("integral_clamp", 10.0)),
            derivative_filter_alpha=float(cfg.get("derivative_filter_alpha", 0.7)),
            max_torque_total=float(cfg.get("max_torque_total", 10.0)),
        )
