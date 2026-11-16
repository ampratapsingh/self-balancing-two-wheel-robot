"""Pytest configuration and shared fixtures."""
import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def repo_root():
    return REPO_ROOT


@pytest.fixture
def robot_config():
    from balancing_robot.config import load_robot_config
    return load_robot_config()


@pytest.fixture
def sim_config():
    from balancing_robot.config import load_simulation_config
    return load_simulation_config()
