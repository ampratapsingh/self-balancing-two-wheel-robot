"""Tests for PID controller."""
import numpy as np
import pytest
from balancing_robot.controllers.pid import PIDController


@pytest.fixture
def pid():
    return PIDController(Kp=35.0, Ki=0.5, Kd=2.5,
                         theta_ref=0.0, integral_clamp=10.0,
                         derivative_filter_alpha=1.0,  # no filter for predictable tests
                         max_torque_total=10.0)


def test_pid_zero_tilt_zero_output(pid):
    """Zero tilt → zero output (no accumulated integral)."""
    state = {"theta": 0.0, "x": 0.0, "x_dot": 0.0, "theta_dot": 0.0}
    out = pid.compute(state, dt=0.01)
    assert out == pytest.approx(0.0, abs=1e-6)


def test_pid_proportional_response(pid):
    """Positive tilt → nonzero output with magnitude proportional to Kp."""
    state = {"theta": 0.1, "x": 0.0, "x_dot": 0.0, "theta_dot": 0.0}
    out = pid.compute(state, dt=0.01)
    # error = theta_ref - theta = 0 - 0.1 = -0.1
    # P = Kp * error = 35 * (-0.1) = -3.5
    # First step: derivative = 0, integral ≈ 0 (dt*error very small)
    # output ≈ -3.5 (saturated to -3.5, within [-10, 10])
    assert abs(out) == pytest.approx(abs(pid.Kp * 0.1), rel=0.01)


def test_pid_saturation():
    """Output must not exceed max_torque_total."""
    pid = PIDController(Kp=1000.0, Ki=0.0, Kd=0.0,
                        max_torque_total=5.0, derivative_filter_alpha=1.0)
    state = {"theta": 1.0, "x": 0.0, "x_dot": 0.0, "theta_dot": 0.0}
    out = pid.compute(state, dt=0.01)
    assert abs(out) <= 5.0 + 1e-9


def test_pid_integral_clamp():
    """Integral should not grow beyond clamp."""
    pid = PIDController(Kp=0.0, Ki=1.0, Kd=0.0,
                        integral_clamp=2.0, max_torque_total=100.0,
                        derivative_filter_alpha=1.0)
    state = {"theta": -1.0}  # error = +1 (since theta_ref=0)
    for _ in range(1000):
        pid.compute(state, dt=0.01)
    # Integral should be clamped at 2.0
    assert abs(pid._integral) <= 2.0 + 1e-9


def test_pid_reset():
    """Reset clears integral and derivative state."""
    pid = PIDController(Kp=1.0, Ki=1.0, Kd=1.0,
                        max_torque_total=100.0, derivative_filter_alpha=1.0)
    state = {"theta": 0.5}
    for _ in range(10):
        pid.compute(state, dt=0.01)
    pid.reset()
    assert pid._integral == pytest.approx(0.0)
    assert pid._prev_theta is None


def test_pid_derivative_on_measurement():
    """Derivative term should use current minus previous theta, not error."""
    pid = PIDController(Kp=0.0, Ki=0.0, Kd=1.0,
                        max_torque_total=100.0, derivative_filter_alpha=1.0)
    # First call: no previous measurement, deriv=0
    out1 = pid.compute({"theta": 0.5}, dt=0.1)
    assert out1 == pytest.approx(0.0, abs=1e-9)  # no derivative on first call
    # Second call: theta increased by 0.1, so raw_deriv = 0.1/0.1 = 1.0
    # d_term = -Kd * filtered_deriv = -1.0 * 1.0 = -1.0
    out2 = pid.compute({"theta": 0.6}, dt=0.1)
    # With no filter (alpha=1.0): filtered = raw_deriv = 1.0
    # d_term = -1.0 * 1.0 = -1.0
    assert out2 == pytest.approx(-1.0, abs=1e-6)


def test_pid_dt_zero_raises():
    pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.0, max_torque_total=10.0)
    with pytest.raises(ValueError):
        pid.compute({"theta": 0.1}, dt=0.0)
