"""Placeholder tests for robot module — full implementation in Phase 2."""
from pathlib import Path

import pytest


def test_robot_module_importable():
    """The robot interface remains importable without creating a body."""
    import balancing_robot.robot  # noqa: F401


@pytest.mark.slow
def test_robot_spawns_and_saturates_wheel_torque(repo_root, robot_config, sim_config):
    """The URDF wheel joints are found and total torque is split safely."""
    pytest.importorskip("pybullet", reason="pybullet not installed")
    from balancing_robot.physics import PhysicsEngine
    from balancing_robot.robot import Robot

    physics = PhysicsEngine(sim_config, gui=False)
    try:
        robot = Robot(
            Path(repo_root) / robot_config["urdf_path"],
            physics.client,
            robot_config,
            initial_tilt_deg=3.0,
        )
        assert robot.robot_id >= 0
        assert robot.left_joint_idx >= 0
        assert robot.right_joint_idx >= 0
        assert robot.left_joint_idx != robot.right_joint_idx

        import pybullet as p
        masses = [
            p.getDynamicsInfo(robot.robot_id, link_index, physicsClientId=physics.client)[0]
            for link_index in (-1, 0, robot.left_joint_idx, robot.right_joint_idx)
        ]
        assert masses[1] == pytest.approx(0.0)  # visual-only front marker
        assert sum(masses) == pytest.approx(2.5)

        left, right = robot.apply_torque(100.0)
        assert left == pytest.approx(robot_config["max_torque_per_wheel"])
        assert right == pytest.approx(robot_config["max_torque_per_wheel"])
        physics.step()
        assert len(robot.get_wheel_velocities()) == 2

        robot.reset(initial_tilt_deg=5.0)
        assert robot.left_joint_idx >= 0
        assert robot.right_joint_idx >= 0
    finally:
        physics.disconnect()
