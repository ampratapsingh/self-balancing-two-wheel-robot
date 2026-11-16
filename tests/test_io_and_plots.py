"""Tests for analysis I/O and plot generation."""
import math
import numpy as np
import pandas as pd
import pytest
import json
from pathlib import Path
import tempfile
from balancing_robot.analysis.io import save_run, load_run, load_metadata
from balancing_robot.analysis.plots import plot_tilt_vs_time, plot_torque_vs_time
from balancing_robot.simulation import CSV_COLUMNS


def make_sample_rows(n=100):
    rows = []
    for i in range(n):
        t = i * 0.01
        rows.append({
            "experiment_id": "E_test",
            "controller": "pid",
            "run_id": "test_run",
            "seed": 42,
            "time_s": t,
            "tilt_rad": 0.05 * math.exp(-0.5 * t),
            "tilt_deg": math.degrees(0.05 * math.exp(-0.5 * t)),
            "tilt_rate_rad_s": -0.025 * math.exp(-0.5 * t),
            "x_m": 0.0,
            "x_vel_m_s": 0.0,
            "left_wheel_vel_rad_s": 0.0,
            "right_wheel_vel_rad_s": 0.0,
            "left_torque_cmd_Nm": 1.0,
            "right_torque_cmd_Nm": 1.0,
            "left_torque_applied_Nm": 0.9,
            "right_torque_applied_Nm": 0.9,
            "disturbance_force_N": 0.0,
            "status": "BALANCING",
        })
    return rows


def test_save_and_load_run():
    rows = make_sample_rows()
    metadata = {"controller": "pid", "duration_s": 1.0, "dt": 0.01}
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path, json_path = save_run(rows, metadata, tmpdir, "test_run")
        assert csv_path.exists()
        assert json_path.exists()
        assert csv_path.stat().st_size > 0
        df = load_run(csv_path)
        assert len(df) == 100
        assert "tilt_rad" in df.columns
        meta = load_metadata(json_path)
        assert meta["controller"] == "pid"


def test_csv_has_expected_columns():
    rows = make_sample_rows()
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path, _ = save_run(rows, {}, tmpdir, "col_test")
        df = load_run(csv_path)
        for col in CSV_COLUMNS:
            assert col in df.columns, f"Missing column: {col}"


def test_plot_tilt_vs_time():
    rows = make_sample_rows()
    df = pd.DataFrame(rows)
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "tilt.png"
        plot_tilt_vs_time({"PID": df}, str(out))
        assert out.exists()
        assert out.stat().st_size > 0


def test_plot_torque_vs_time():
    rows = make_sample_rows()
    df = pd.DataFrame(rows)
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "torque.png"
        plot_torque_vs_time({"PID": df}, str(out))
        assert out.exists()
        assert out.stat().st_size > 0
