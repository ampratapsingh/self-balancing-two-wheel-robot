"""Tests for config loading."""
import pytest
from balancing_robot.config import (
    load_robot_config, load_simulation_config,
    load_pid_config, load_lqr_config
)


def test_robot_config_loads():
    cfg = load_robot_config()
    assert "wheel" in cfg
    assert "chassis" in cfg
    assert cfg["wheel"]["radius"] == pytest.approx(0.08)
    assert cfg["chassis"]["mass"] == pytest.approx(2.0)


def test_simulation_config_loads():
    cfg = load_simulation_config()
    assert "physics" in cfg
    assert cfg["physics"]["timestep"] == pytest.approx(1/240, rel=1e-3)


def test_pid_config_loads():
    cfg = load_pid_config()
    assert "Kp" in cfg
    assert cfg["Kp"] > 0


def test_lqr_config_loads():
    cfg = load_lqr_config()
    assert "Q" in cfg
    assert "R" in cfg
