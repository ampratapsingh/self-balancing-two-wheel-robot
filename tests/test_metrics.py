"""Tests for the metrics module."""
import math
import numpy as np
import pandas as pd
import pytest
from balancing_robot.analysis.metrics import compute_metrics, SETTLING_THRESHOLD_RAD


def make_df(time, tilt_rad, controller="pid", status=None):
    n = len(time)
    if status is None:
        status = ["BALANCING"] * n
    df = pd.DataFrame({
        "experiment_id": ["E_test"] * n,
        "controller": [controller] * n,
        "run_id": ["test"] * n,
        "seed": [42] * n,
        "time_s": time,
        "tilt_rad": tilt_rad,
        "tilt_deg": [math.degrees(t) for t in tilt_rad],
        "tilt_rate_rad_s": [0.0] * n,
        "x_m": [0.0] * n,
        "x_vel_m_s": [0.0] * n,
        "left_wheel_vel_rad_s": [0.0] * n,
        "right_wheel_vel_rad_s": [0.0] * n,
        "left_torque_cmd_Nm": [1.0] * n,
        "right_torque_cmd_Nm": [1.0] * n,
        "left_torque_applied_Nm": [0.9] * n,
        "right_torque_applied_Nm": [0.9] * n,
        "disturbance_force_N": [0.0] * n,
        "status": status,
    })
    return df


def test_metrics_perfect_balance():
    """Perfect balance: all zeros → success, zero metrics."""
    time = np.linspace(0, 10, 2400)
    tilt = np.zeros_like(time)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["max_abs_tilt_rad"] == pytest.approx(0.0, abs=1e-6)
    assert m["rms_tilt_rad"] == pytest.approx(0.0, abs=1e-6)
    assert m["settling_time_s"] == pytest.approx(0.0, abs=1e-6)


def test_metrics_failed_run():
    """Run with FELL status at end."""
    time = [0.0, 0.1, 0.2]
    tilt = [0.05, 0.3, 0.9]
    status = ["BALANCING", "BALANCING", "FELL"]
    df = make_df(time, tilt, status=status)
    m = compute_metrics(df)
    assert m["success"] is False
    assert m["fall_time_s"] == pytest.approx(0.2, abs=1e-6)


def test_metrics_rms_tilt():
    """RMS of a constant tilt."""
    time = np.linspace(0, 5, 100)
    tilt = np.full_like(time, 0.1)  # constant 0.1 rad
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["rms_tilt_rad"] == pytest.approx(0.1, abs=1e-4)


def test_metrics_settling_time():
    """Settling time: tilt drops below threshold at known time."""
    time = np.linspace(0, 10, 1000)
    # Tilt starts at 0.1, decays to 0 after t=5s
    tilt = np.where(time < 5.0, 0.1, 0.0)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["settling_time_s"] is not None
    assert m["settling_time_s"] <= 5.1  # settled around t=5


def test_metrics_empty_df():
    """Empty DataFrame should return failed metrics without error."""
    df = pd.DataFrame()
    m = compute_metrics(df)
    assert m["success"] is False
