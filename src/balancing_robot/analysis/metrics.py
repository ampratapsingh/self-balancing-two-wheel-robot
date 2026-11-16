"""
Quantitative performance metrics for controller evaluation.

All metric definitions are fixed here and must not be changed between experiments.
Reference: docs/methodology.md

State convention:
  tilt_rad: pitch angle (rad), 0 = upright
  time_s: timestamps (s)
  torque: total torque commanded (Nm)
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
import pandas as pd


# Fall threshold (must match simulation.py)
FALL_THRESHOLD_RAD = math.radians(45.0)

# Settling band: |tilt| < SETTLING_THRESHOLD_RAD for all future time
SETTLING_THRESHOLD_RAD = math.radians(0.5)  # ±0.5° absolute


def compute_metrics(df: pd.DataFrame, duration_s: float | None = None) -> dict[str, Any]:
    """Compute all performance metrics from a simulation CSV.

    Args:
        df: DataFrame with columns matching CSV_COLUMNS from simulation.py.
        duration_s: Total simulation duration. Defaults to max time_s.

    Returns:
        dict of metric name → value.
    """
    if df.empty:
        return _failed_metrics()

    if duration_s is None:
        duration_s = float(df["time_s"].max())

    tilt = df["tilt_rad"].values
    time = df["time_s"].values
    status = df["status"].values

    # Success flag
    fell = any(s == "FELL" for s in status)
    fall_time = float(df[df["status"] == "FELL"]["time_s"].min()) if fell else None
    success = not fell

    # 1. Maximum |tilt|
    max_abs_tilt = float(np.max(np.abs(tilt)))

    # 2. Settling time: first t after which |theta(t')| < SETTLING_THRESHOLD for all t' >= t
    settling_time = _compute_settling_time(time, tilt, SETTLING_THRESHOLD_RAD)

    # 3. Steady-state tilt error: mean |tilt| over last 5 seconds
    ss_window = 5.0
    ss_mask = time >= (time[-1] - ss_window)
    ss_tilt_error = float(np.mean(np.abs(tilt[ss_mask]))) if ss_mask.any() else float("nan")

    # 4. Overshoot: for a regulation response, max |tilt| beyond initial tilt
    if len(tilt) > 1:
        initial_tilt = float(np.abs(tilt[0]))
        overshoot = max(0.0, max_abs_tilt - initial_tilt)
    else:
        overshoot = 0.0

    # 5. RMS tilt error
    rms_tilt = float(np.sqrt(np.mean(tilt**2)))

    # 6. Wheel angular velocities
    if "left_wheel_vel_rad_s" in df.columns:
        wheel_vels = np.abs(df[["left_wheel_vel_rad_s", "right_wheel_vel_rad_s"]].values)
        peak_wheel_vel = float(np.max(wheel_vels))
        rms_wheel_vel = float(np.sqrt(np.mean(wheel_vels**2)))
    else:
        peak_wheel_vel = float("nan")
        rms_wheel_vel = float("nan")

    # 7. Horizontal displacement
    if "x_m" in df.columns:
        x = df["x_m"].values
        final_x = float(x[-1])
        peak_x = float(np.max(np.abs(x)))
    else:
        final_x = float("nan")
        peak_x = float("nan")

    if "x_vel_m_s" in df.columns:
        final_speed = float(np.abs(df["x_vel_m_s"].values[-1]))
    else:
        final_speed = float("nan")

    # 8. Torque metrics
    if "left_torque_cmd_Nm" in df.columns:
        # Total commanded torque = left + right
        torque_total = (df["left_torque_cmd_Nm"] + df["right_torque_cmd_Nm"]).values
        peak_torque = float(np.max(np.abs(torque_total)))
        rms_torque = float(np.sqrt(np.mean(torque_total**2)))
        # Fraction of time saturated (at max torque - 0.01 tolerance)
        # We don't have max_torque here; approximate as peak > 9.5 out of 10
        sat_fraction = float(np.mean(np.abs(torque_total) >= 9.5))
    else:
        peak_torque = float("nan")
        rms_torque = float("nan")
        sat_fraction = float("nan")

    return {
        "success": success,
        "fall_time_s": fall_time,
        "max_abs_tilt_rad": max_abs_tilt,
        "max_abs_tilt_deg": math.degrees(max_abs_tilt),
        "settling_time_s": settling_time,
        "ss_tilt_error_rad": ss_tilt_error,
        "ss_tilt_error_deg": math.degrees(ss_tilt_error) if not math.isnan(ss_tilt_error) else float("nan"),
        "overshoot_rad": overshoot,
        "overshoot_deg": math.degrees(overshoot),
        "rms_tilt_rad": rms_tilt,
        "peak_wheel_vel_rad_s": peak_wheel_vel,
        "rms_wheel_vel_rad_s": rms_wheel_vel,
        "final_x_m": final_x,
        "peak_x_m": peak_x,
        "final_speed_m_s": final_speed,
        "peak_torque_Nm": peak_torque,
        "rms_torque_Nm": rms_torque,
        "saturation_fraction": sat_fraction,
    }


def _compute_settling_time(
    time: np.ndarray,
    tilt: np.ndarray,
    threshold: float,
) -> float | None:
    """Find the first time index after which |tilt| stays within threshold.

    Returns the settling time in seconds, or None if never settled.
    """
    for i in range(len(tilt)):
        if np.all(np.abs(tilt[i:]) <= threshold):
            return float(time[i])
    return None


def _failed_metrics() -> dict[str, Any]:
    """Return a metrics dict for a run that produced no data."""
    return {
        "success": False,
        "fall_time_s": 0.0,
        "max_abs_tilt_rad": float("nan"),
        "max_abs_tilt_deg": float("nan"),
        "settling_time_s": None,
        "ss_tilt_error_rad": float("nan"),
        "ss_tilt_error_deg": float("nan"),
        "overshoot_rad": float("nan"),
        "overshoot_deg": float("nan"),
        "rms_tilt_rad": float("nan"),
        "peak_wheel_vel_rad_s": float("nan"),
        "rms_wheel_vel_rad_s": float("nan"),
        "final_x_m": float("nan"),
        "peak_x_m": float("nan"),
        "final_speed_m_s": float("nan"),
        "peak_torque_Nm": float("nan"),
        "rms_torque_Nm": float("nan"),
        "saturation_fraction": float("nan"),
    }
