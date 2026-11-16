"""Tests for the Sensors module (tilt extraction logic only - no PyBullet needed)."""
import numpy as np
import pytest
from balancing_robot.sensors import Sensors
import math


def quat_from_pitch(theta_rad: float):
    """Create quaternion for a pure pitch rotation by theta about Y axis.

    PyBullet convention: (x, y, z, w).
    Rotation by theta about Y: q = (0, sin(theta/2), 0, cos(theta/2))
    """
    return (0.0, math.sin(theta_rad / 2), 0.0, math.cos(theta_rad / 2))


def test_tilt_extraction_upright():
    """Upright pose (zero rotation) should give theta = 0."""
    quat = (0.0, 0.0, 0.0, 1.0)  # identity quaternion
    theta, _ = Sensors._extract_tilt(quat, (0.0, 0.0, 0.0))
    assert theta == pytest.approx(0.0, abs=1e-6)


def test_tilt_extraction_forward_lean():
    """Forward lean of 10° should give theta ≈ +10°."""
    angle_deg = 10.0
    theta_rad = math.radians(angle_deg)
    quat = quat_from_pitch(theta_rad)
    theta, _ = Sensors._extract_tilt(quat, (0.0, 0.0, 0.0))
    assert theta == pytest.approx(theta_rad, abs=1e-4)


def test_tilt_extraction_backward_lean():
    """Backward lean of 5° should give theta ≈ -5°."""
    angle_deg = -5.0
    theta_rad = math.radians(angle_deg)
    quat = quat_from_pitch(theta_rad)
    theta, _ = Sensors._extract_tilt(quat, (0.0, 0.0, 0.0))
    assert theta == pytest.approx(theta_rad, abs=1e-4)


def test_tilt_rate_from_angular_velocity():
    """Y-component of angular velocity should map to theta_dot."""
    quat = (0.0, 0.0, 0.0, 1.0)  # upright
    omega_y = 0.5  # rad/s pitch rate
    _, theta_dot = Sensors._extract_tilt(quat, (0.0, omega_y, 0.0))
    assert theta_dot == pytest.approx(omega_y, abs=1e-6)


def test_state_to_array():
    state = {"x": 1.0, "x_dot": 0.5, "theta": 0.1, "theta_dot": 0.2}
    arr = Sensors.state_to_array(state)
    assert arr.shape == (4,)
    assert arr[0] == pytest.approx(1.0)
    assert arr[2] == pytest.approx(0.1)


def test_known_pose_validation():
    theta_rad = math.radians(15.0)
    quat = quat_from_pitch(theta_rad)
    result = Sensors.tilt_from_quat_known(quat, theta_rad, tol=0.01)
    assert result is True


def test_tilt_extraction_large_angle():
    """Test at 30° — still within physical range."""
    angle_deg = 30.0
    theta_rad = math.radians(angle_deg)
    quat = quat_from_pitch(theta_rad)
    theta, _ = Sensors._extract_tilt(quat, (0.0, 0.0, 0.0))
    assert theta == pytest.approx(theta_rad, abs=1e-4)
