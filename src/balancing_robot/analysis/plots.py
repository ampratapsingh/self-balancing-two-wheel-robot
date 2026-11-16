"""
Plotting utilities for experiment results.

All plots read from saved CSV/JSON files only.
Consistent colours per controller: PID=blue, LQR=red, none=grey.
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

logger = logging.getLogger(__name__)

CONTROLLER_COLORS = {
    "pid": "#1f77b4",   # blue
    "lqr": "#d62728",   # red
    "none": "#7f7f7f",  # grey
    "PID": "#1f77b4",
    "LQR": "#d62728",
}
FIGURE_DPI = 300


def _get_color(controller: str) -> str:
    return CONTROLLER_COLORS.get(controller, "#2ca02c")


def plot_tilt_vs_time(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Tilt vs Time",
    xlabel: str = "Time (s)",
    ylabel: str = "Tilt (degrees)",
) -> None:
    """Plot tilt angle vs time for one or more runs.

    Args:
        dfs: dict of label → DataFrame (each with time_s, tilt_deg columns).
        output_path: Where to save the PNG.
        title, xlabel, ylabel: Figure labels.
    """
    fig, ax = plt.subplots(figsize=(10, 5))
    for label, df in dfs.items():
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        ax.plot(df["time_s"], df["tilt_deg"], label=label, color=color, linewidth=1.5)
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save(fig, output_path)


def plot_torque_vs_time(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Torque vs Time",
) -> None:
    """Plot total commanded torque vs time."""
    fig, ax = plt.subplots(figsize=(10, 5))
    for label, df in dfs.items():
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        torque_total = df["left_torque_cmd_Nm"] + df["right_torque_cmd_Nm"]
        ax.plot(df["time_s"], torque_total, label=label, color=color, linewidth=1.0)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Total Torque (Nm)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save(fig, output_path)


def plot_position_drift(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Horizontal Position Drift vs Time",
) -> None:
    """Plot chassis x position vs time."""
    fig, ax = plt.subplots(figsize=(10, 5))
    for label, df in dfs.items():
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        ax.plot(df["time_s"], df["x_m"], label=label, color=color, linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Horizontal Position (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save(fig, output_path)


def plot_tilt_sweep_map(
    results: list[dict],
    output_path: str | Path,
    title: str = "Initial Tilt Success Map",
) -> None:
    """Plot success/fail map vs initial tilt angle.

    Args:
        results: List of dicts with keys: controller, initial_tilt_deg, success.
        output_path: Where to save.
    """
    fig, ax = plt.subplots(figsize=(10, 5))
    controllers = sorted(set(r["controller"] for r in results))
    for ctrl in controllers:
        ctrl_results = [r for r in results if r["controller"] == ctrl]
        tilts = [r["initial_tilt_deg"] for r in ctrl_results]
        successes = [1 if r["success"] else 0 for r in ctrl_results]
        color = _get_color(ctrl)
        ax.plot(tilts, successes, "o-", label=ctrl.upper(), color=color, linewidth=2, markersize=8)
    ax.set_xlabel("Initial Tilt Angle (degrees)")
    ax.set_ylabel("Success (1=balanced, 0=fell)")
    ax.set_title(title)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Fell", "Balanced"])
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save(fig, output_path)


def _save(fig: plt.Figure, path: str | Path) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(p, dpi=FIGURE_DPI, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved plot: %s", p)
