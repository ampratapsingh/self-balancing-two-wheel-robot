"""Integration test: full simulation balance run — marked slow."""
import pytest


@pytest.mark.slow
def test_simulation_importable():
    """simulation.py stub must be importable."""
    import balancing_robot.simulation  # noqa: F401


@pytest.mark.slow
def test_lqr_balances_for_30s():
    """LQR controller should keep robot upright for 30s from 3° initial tilt.

    This test requires PyBullet and all Phase 2 modules. Skipped until Phase 2.
    """
    pytest.importorskip("pybullet", reason="pybullet not installed")

    from balancing_robot.config import load_robot_config, load_simulation_config, load_lqr_config
    from balancing_robot.dynamics import compute_model_params, compute_state_space
    from balancing_robot.controllers.lqr import LQRController
    from balancing_robot.simulation import Simulation

    robot_cfg = load_robot_config()
    sim_cfg = load_simulation_config()
    lqr_cfg = load_lqr_config()

    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)
    controller = LQRController.from_config(lqr_cfg, A, B)

    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=controller,
        gui=False,
        initial_tilt_deg=3.0,
        duration_s=30.0,
        experiment_id="E_integration",
        run_id="integration_lqr",
        seed=42,
    )
    result = sim.run()
    assert result.success, f"Robot fell at t={result.fall_time_s:.2f}s"


@pytest.mark.slow
def test_pid_balances_for_30s():
    """PID controller should keep robot upright for 30s from 3° initial tilt."""
    pytest.importorskip("pybullet", reason="pybullet not installed")

    from balancing_robot.config import load_robot_config, load_simulation_config, load_pid_config
    from balancing_robot.controllers.pid import PIDController
    from balancing_robot.simulation import Simulation

    robot_cfg = load_robot_config()
    sim_cfg = load_simulation_config()
    pid_cfg = load_pid_config()

    controller = PIDController.from_config(pid_cfg)

    sim = Simulation(
        sim_cfg=sim_cfg,
        robot_cfg=robot_cfg,
        controller=controller,
        gui=False,
        initial_tilt_deg=3.0,
        duration_s=30.0,
        experiment_id="E_integration",
        run_id="integration_pid",
        seed=42,
    )
    result = sim.run()
    assert result.success, f"Robot fell at t={result.fall_time_s:.2f}s"
