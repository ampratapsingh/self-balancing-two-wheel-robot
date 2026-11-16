"""Tests for LQR controller."""
import numpy as np
import pytest
from balancing_robot.config import load_robot_config, load_lqr_config
from balancing_robot.dynamics import compute_model_params, compute_state_space
from balancing_robot.controllers.lqr import LQRController


@pytest.fixture
def model():
    cfg = load_robot_config()
    params = compute_model_params(cfg)
    A, B = compute_state_space(params)
    return A, B


@pytest.fixture
def lqr(model):
    A, B = model
    cfg = load_lqr_config()
    return LQRController.from_config(cfg, A, B)


def test_lqr_gain_shape(lqr):
    assert lqr.K.shape == (1, 4)


def test_lqr_closed_loop_stable(lqr):
    """All closed-loop eigenvalues must have negative real parts."""
    assert lqr.is_stable(), (
        f"LQR closed-loop is NOT stable. Eigenvalues: {lqr.closed_loop_eigenvalues()}"
    )


def test_lqr_zero_state_zero_output(lqr):
    state = {"x": 0.0, "x_dot": 0.0, "theta": 0.0, "theta_dot": 0.0}
    out = lqr.compute(state, dt=1/240)
    assert out == pytest.approx(0.0, abs=1e-6)


def test_lqr_forward_tilt_produces_output(lqr):
    """Non-zero tilt should produce non-zero torque."""
    state = {"x": 0.0, "x_dot": 0.0, "theta": 0.1, "theta_dot": 0.0}
    out = lqr.compute(state, dt=1/240)
    assert abs(out) > 0.01


def test_lqr_saturation(model):
    A, B = model
    cfg = load_lqr_config()
    lqr = LQRController.from_config(cfg, A, B)
    # Large tilt should saturate
    state = {"x": 100.0, "x_dot": 50.0, "theta": 2.0, "theta_dot": 10.0}
    out = lqr.compute(state, dt=1/240)
    assert abs(out) <= lqr.max_torque_total + 1e-9


def test_lqr_reset_is_noop(lqr):
    """Reset should not raise and LQR should still work."""
    lqr.reset()
    state = {"x": 0.0, "x_dot": 0.0, "theta": 0.05, "theta_dot": 0.0}
    out = lqr.compute(state, dt=1/240)
    assert isinstance(out, float)


def test_lqr_k_sign_convention(lqr):
    """K[0,2] (tilt gain) should be negative so that u = -K*x gives positive
    torque when theta > 0 (forward lean) and negative torque when theta < 0.
    u = -K @ [x, xdot, theta, thetadot]
    When theta > 0 → need u > 0 → -K[0,2]*theta > 0 → K[0,2] < 0
    """
    # K[0,2] should be negative (tilt gain creates restoring torque)
    assert lqr.K[0, 2] < 0, (
        f"K[0,2] should be negative (tilt restoring), got {lqr.K[0, 2]}"
    )
