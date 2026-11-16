"""
LQR (Linear Quadratic Regulator) controller.

State: [x, x_dot, theta, theta_dot]^T
Input: u = -K @ x_state
Cost: J = integral(x^T Q x + u^T R u) dt

Gain K is computed by solving the continuous algebraic Riccati equation (CARE)
using scipy.linalg.solve_continuous_are.

Stability: closed-loop eigenvalues of (A - B*K) must all have negative real parts.
"""
from __future__ import annotations

import logging
from typing import Any

import numpy as np
from scipy.linalg import solve_continuous_are

from balancing_robot.controllers.base import BaseController

logger = logging.getLogger(__name__)


class LQRController(BaseController):
    """LQR balancing controller.

    Args:
        A: (4,4) state matrix of the linearised system.
        B: (4,1) input matrix.
        Q: (4,4) state cost matrix.
        R: (1,1) control cost matrix.
        max_torque_total: Output saturation limit (Nm).
    """

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        max_torque_total: float = 10.0,
    ) -> None:
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.max_torque_total = max_torque_total
        self._K: np.ndarray | None = None

        # Compute gain on init
        self._compute_gain()

    def _compute_gain(self) -> None:
        """Solve CARE and compute LQR gain K = R^{-1} B^T P."""
        try:
            P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        except Exception as e:
            raise RuntimeError(f"CARE solution failed: {e}") from e

        R_inv = np.linalg.inv(self.R)
        self._K = R_inv @ self.B.T @ P
        logger.info("LQR gain K = %s", self._K)

        # Verify closed-loop stability
        A_cl = self.A - self.B @ self._K
        eigs = np.linalg.eigvals(A_cl)
        unstable = [e for e in eigs if e.real >= 0]
        if unstable:
            logger.warning(
                "LQR closed-loop has unstable eigenvalues: %s", unstable
            )
        else:
            logger.info("LQR closed-loop eigenvalues all stable: %s", eigs)

    @property
    def K(self) -> np.ndarray:
        """The computed LQR gain matrix (1×4)."""
        assert self._K is not None
        return self._K

    @property
    def name(self) -> str:
        return "LQR"

    def reset(self) -> None:
        """LQR has no internal integrator state; resets are no-ops."""
        pass

    def compute(self, state: dict, dt: float) -> float:
        """Compute total torque command.

        Args:
            state: dict with keys x, x_dot, theta, theta_dot.
            dt: Timestep (unused in LQR, included for interface compatibility).

        Returns:
            Total torque (Nm). Positive = forward.
        """
        x_vec = np.array([
            state["x"],
            state["x_dot"],
            state["theta"],
            state["theta_dot"],
        ])

        u = float((-self._K @ x_vec).item())
        u = max(-self.max_torque_total, min(self.max_torque_total, u))

        logger.debug(
            "LQR: state=%s K=%s u=%.3f", x_vec, self._K, u
        )
        return u

    @classmethod
    def from_config(
        cls,
        cfg: dict[str, Any],
        A: np.ndarray,
        B: np.ndarray,
    ) -> "LQRController":
        """Instantiate from a loaded lqr_config.yaml dict.

        Args:
            cfg: Loaded lqr_config.yaml dict.
            A: State matrix from dynamics.py.
            B: Input matrix from dynamics.py.
        """
        Q_cfg = cfg["Q"]
        Q = np.diag([
            float(Q_cfg["q_x"]),
            float(Q_cfg["q_xdot"]),
            float(Q_cfg["q_theta"]),
            float(Q_cfg["q_thetadot"]),
        ])
        R = np.array([[float(cfg["R"])]])
        max_torque = float(cfg.get("max_torque_total", 10.0))
        return cls(A=A, B=B, Q=Q, R=R, max_torque_total=max_torque)

    def closed_loop_eigenvalues(self) -> np.ndarray:
        """Return eigenvalues of the closed-loop system (A - B K)."""
        A_cl = self.A - self.B @ self._K
        return np.linalg.eigvals(A_cl)

    def is_stable(self) -> bool:
        """Return True if all closed-loop eigenvalues have negative real parts."""
        eigs = self.closed_loop_eigenvalues()
        return all(e.real < 0 for e in eigs)
