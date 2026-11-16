"""Placeholder tests for physics module — full implementation in Phase 2."""
import pytest


def test_physics_module_importable():
    """The physics interface remains importable without creating a world."""
    import balancing_robot.physics  # noqa: F401


@pytest.mark.slow
def test_physics_engine_steps_and_exposes_world(sim_config):
    """A DIRECT world is configured, stepped, and safely disconnected."""
    pytest.importorskip("pybullet", reason="pybullet not installed")
    from balancing_robot.physics import PhysicsEngine

    physics = PhysicsEngine(sim_config, gui=False)
    try:
        assert physics.client >= 0
        assert physics.plane_id >= 0
        assert physics.dt == pytest.approx(sim_config["physics"]["timestep"])
        physics.step()
    finally:
        physics.disconnect()
