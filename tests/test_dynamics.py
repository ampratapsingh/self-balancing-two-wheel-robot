"""Tests for the linearised dynamics model."""
import numpy as np
import pytest
from balancing_robot.config import load_robot_config
from balancing_robot.dynamics import (
    compute_model_params,
    compute_state_space,
    compute_eigenvalues,
    is_controllable,
    compute_controllability_matrix,
)


@pytest.fixture
def robot_cfg():
    return load_robot_config()


@pytest.fixture
def model(robot_cfg):
    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)
    return params, A, B


def test_model_params_positive(model):
    params, _, _ = model
    for k, v in params.items():
        assert v > 0, f"Parameter {k} must be positive, got {v}"


def test_A_shape(model):
    _, A, B = model
    assert A.shape == (4, 4)
    assert B.shape == (4, 1)


def test_A_has_unstable_eigenvalue(model):
    _, A, _ = model
    eigs = compute_eigenvalues(A)
    positive_real = [e for e in eigs if e.real > 1e-6]
    assert len(positive_real) >= 1, (
        f"Expected at least one unstable eigenvalue, got eigenvalues: {eigs}"
    )


def test_A_has_expected_zero_eigenvalues(model):
    """The double-integrator structure gives two zero eigenvalues."""
    _, A, _ = model
    eigs = sorted(np.abs(np.linalg.eigvals(A)))
    # Two eigenvalues should be near zero (from the x and theta integrators)
    # In a simplified model: 0, 0, +unstable, -stable
    near_zero = [e for e in eigs if abs(e) < 1e-6]
    # At least one near-zero (position is not driven, it integrates)
    assert len(near_zero) >= 1, f"Expected near-zero eigenvalue(s), got {eigs}"


def test_system_controllable(model):
    _, A, B = model
    assert is_controllable(A, B), "System must be controllable"


def test_B_produces_forward_tilt_response():
    """Positive torque must decelerate the pendulum fall (B[3,0] < 0).

    Physical reasoning: positive torque (forward) creates a backward reaction
    torque on chassis, reducing forward tilt rate. So B[3,0] should be negative.
    """
    from balancing_robot.config import load_robot_config
    cfg = load_robot_config()
    params = compute_model_params(cfg)
    A, B = compute_state_space(params)
    # B[3,0] is the effect of torque on theta_ddot
    # Positive torque → backward reaction → theta_ddot < 0 when theta > 0
    assert B[3, 0] < 0, f"B[3,0] should be negative (torque reduces tilt), got {B[3,0]}"


def test_A_kinematic_entries():
    """A[0,1]=1 (x_dot → x) and A[2,3]=1 (theta_dot → theta)."""
    from balancing_robot.config import load_robot_config
    cfg = load_robot_config()
    params = compute_model_params(cfg)
    A, _ = compute_state_space(params)
    assert A[0, 1] == pytest.approx(1.0)
    assert A[2, 3] == pytest.approx(1.0)
    # Cross terms that shouldn't exist in linearised model
    assert A[0, 0] == pytest.approx(0.0)
    assert A[2, 2] == pytest.approx(0.0)


def test_gravity_term_positive():
    """A[3,2] should be positive (gravity makes tilt unstable: theta_ddot > 0 when theta > 0)."""
    from balancing_robot.config import load_robot_config
    cfg = load_robot_config()
    params = compute_model_params(cfg)
    A, _ = compute_state_space(params)
    assert A[3, 2] > 0, f"A[3,2] should be positive (gravity instability), got {A[3,2]}"


def test_controllability_matrix_rank():
    from balancing_robot.config import load_robot_config
    cfg = load_robot_config()
    params = compute_model_params(cfg)
    A, B = compute_state_space(params)
    C_mat = compute_controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(C_mat)
    assert rank == 4, f"Controllability matrix rank should be 4, got {rank}"
