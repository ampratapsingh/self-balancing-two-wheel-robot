"""
State-space dynamics model for the self-balancing two-wheel robot.

Derives the linearised A, B matrices from the Lagrangian equations of motion.
State: x_state = [x, x_dot, theta, theta_dot]^T
Input: u = tau_total (Nm, total torque both wheels, positive = forward)

Convention:
  - theta = 0: upright
  - theta > 0: forward lean
  - positive u produces forward acceleration (+x)

Reference: Lagrangian derivation in docs/mathematical_model.md
"""
from __future__ import annotations

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def compute_model_params(robot_cfg: dict[str, Any]) -> dict[str, float]:
    """Extract and derive all physical parameters needed for the linear model.

    Args:
        robot_cfg: Loaded robot_config.yaml dict.

    Returns:
        dict of named parameters (SI units).
    """
    r = float(robot_cfg["wheel"]["radius"])
    w_width = float(robot_cfg["wheel"]["width"])
    m_w = float(robot_cfg["wheel"]["mass"])
    m_c = float(robot_cfg["chassis"]["mass"])
    H = float(robot_cfg["chassis"]["height"])
    W = float(robot_cfg["chassis"]["width"])
    D = float(robot_cfg["chassis"]["depth"])
    g = float(robot_cfg["gravity"])

    # CoM of chassis above wheel axle (geometric centre of box)
    l = H / 2.0

    # Chassis pitch inertia about its own CoM (rotation about Y)
    # I_c_yy = m/12 * (W^2 + D^2)  [Y is pitch axis; transverse dims are W and D]
    I_c = m_c / 12.0 * (W**2 + D**2)

    # Wheel spin inertia (about Y axis = spin axis)
    I_w = 0.5 * m_w * r**2

    # Total translating mass
    M_total = m_c + 2.0 * m_w

    # Effective mass including wheel rotational inertia (rolling constraint: x_dot = r * phi_dot)
    M_eff = M_total + 2.0 * I_w / r**2

    # Effective pendulum inertia about axle (parallel-axis theorem)
    I_pend = I_c + m_c * l**2

    return {
        "r": r,
        "m_w": m_w,
        "m_c": m_c,
        "l": l,
        "g": g,
        "I_c": I_c,
        "I_w": I_w,
        "M_total": M_total,
        "M_eff": M_eff,
        "I_pend": I_pend,
        "W": W,
        "D": D,
        "H": H,
    }


def compute_state_space(params: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    """Compute the linearised A and B matrices.

    Linearisation point: theta=0, theta_dot=0, x_dot=0 (upright equilibrium).

    The 2x2 inertia matrix for [x_ddot, theta_ddot]:
        M2 = [[M_eff,        m_c*l],
              [m_c*l,        I_pend]]

    Generalised force vector from input u = tau_total:
        G = [1/r, -1]^T

    Gravity-driven forcing (linearised, sinθ ≈ θ):
        f_gravity = [0, m_c*g*l] acts on [ẍ, θ̈] via
        F2 = [[0, 0], [0, m_c*g*l]]
        (contributes to A matrix via M2^{-1} F2)

    Full state: [x, x_dot, theta, theta_dot]^T

    Returns:
        A: (4,4) state matrix
        B: (4,1) input matrix
    """
    r = params["r"]
    m_c = params["m_c"]
    l = params["l"]
    g = params["g"]
    M_eff = params["M_eff"]
    I_pend = params["I_pend"]

    # 2x2 inertia matrix for [x_ddot, theta_ddot]
    M2 = np.array([
        [M_eff,   m_c * l],
        [m_c * l, I_pend],
    ])

    # Gravity forcing matrix (only theta_ddot has gravity term)
    F2 = np.array([
        [0.0, 0.0],
        [0.0, m_c * g * l],
    ])

    # Input gain vector [1/r, -1] (torque-to-force mapping)
    G2 = np.array([[1.0 / r], [-1.0]])

    M2_inv = np.linalg.inv(M2)

    # Acceleration = M2_inv @ (F2 @ [x, theta]^T + G2 * u)
    # So: [x_ddot, theta_ddot] = M2_inv @ F2 @ [x, theta]^T + M2_inv @ G2 * u
    acc_from_state = M2_inv @ F2  # 2x2: maps [x, theta] to [x_ddot, theta_ddot]
    acc_from_u = M2_inv @ G2      # 2x1: maps u to [x_ddot, theta_ddot]

    # Build 4x4 A matrix: state = [x, x_dot, theta, theta_dot]
    # x_dot depends on x_dot (index 1 -> index 0 derivative)
    # theta_dot depends on theta_dot (index 3 -> index 2 derivative)
    # x_ddot = acc_from_state[0,0]*x + acc_from_state[0,1]*theta
    # theta_ddot = acc_from_state[1,0]*x + acc_from_state[1,1]*theta
    A = np.zeros((4, 4))
    # d/dt [x] = x_dot
    A[0, 1] = 1.0
    # d/dt [x_dot] = x_ddot = acc_from_state[0,0]*x + acc_from_state[0,1]*theta
    A[1, 0] = acc_from_state[0, 0]
    A[1, 2] = acc_from_state[0, 1]
    # d/dt [theta] = theta_dot
    A[2, 3] = 1.0
    # d/dt [theta_dot] = theta_ddot = acc_from_state[1,0]*x + acc_from_state[1,1]*theta
    A[3, 0] = acc_from_state[1, 0]
    A[3, 2] = acc_from_state[1, 1]

    # Build 4x1 B matrix
    B = np.zeros((4, 1))
    B[1, 0] = acc_from_u[0, 0]
    B[3, 0] = acc_from_u[1, 0]

    return A, B


def compute_eigenvalues(A: np.ndarray) -> np.ndarray:
    """Return eigenvalues of A. One positive real eigenvalue confirms instability."""
    return np.linalg.eigvals(A)


def compute_controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Compute the controllability matrix [B, AB, A²B, A³B]."""
    n = A.shape[0]
    cols = [B]
    for _ in range(n - 1):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def is_controllable(A: np.ndarray, B: np.ndarray, tol: float = 1e-10) -> bool:
    """Return True if the (A, B) pair is controllable (full-rank controllability matrix)."""
    C_mat = compute_controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(C_mat, tol=tol)
    return rank == A.shape[0]


def simulate_linear_model(
    A: np.ndarray,
    B: np.ndarray,
    x0: np.ndarray,
    K: np.ndarray | None,
    dt: float,
    n_steps: int,
    max_torque: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Simulate the linear model using zero-order hold (Euler integration).

    Args:
        A: (4,4) state matrix.
        B: (4,1) input matrix.
        x0: (4,) initial state vector.
        K: (1,4) LQR gain matrix or None for open-loop.
        dt: timestep in seconds.
        n_steps: number of steps.
        max_torque: torque saturation limit (Nm).

    Returns:
        states: (n_steps+1, 4) array of states.
        torques: (n_steps, 1) array of applied torques.
    """
    states = np.zeros((n_steps + 1, 4))
    torques = np.zeros((n_steps, 1))
    states[0] = x0

    for i in range(n_steps):
        x = states[i]
        if K is not None:
            # K @ x has shape (1,); extract its scalar explicitly for NumPy 2.x.
            u = float((-K @ x).item())
            u = np.clip(u, -max_torque, max_torque)
        else:
            u = 0.0
        torques[i, 0] = u
        x_dot = A @ x + B.flatten() * u
        states[i + 1] = x + dt * x_dot

    return states, torques


def print_model_summary(robot_cfg: dict[str, Any]) -> None:
    """Print a full summary of the dynamics model to stdout."""
    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)
    eigs = compute_eigenvalues(A)
    ctrl = is_controllable(A, B)
    C_mat = compute_controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(C_mat)

    print("=" * 60)
    print("LINEARISED STATE-SPACE MODEL SUMMARY")
    print("=" * 60)
    print("\nPhysical parameters:")
    for k, v in params.items():
        print(f"  {k:12s} = {v:.6f}")
    print(f"\nA matrix (4x4):\n{A}")
    print(f"\nB matrix (4x1):\n{B}")
    print(f"\nEigenvalues of A: {eigs}")
    print(f"  → Unstable poles (Re>0): {[e for e in eigs if e.real > 1e-6]}")
    print(f"\nControllability matrix rank: {rank} / {A.shape[0]}")
    print(f"System is controllable: {ctrl}")
    print("=" * 60)
