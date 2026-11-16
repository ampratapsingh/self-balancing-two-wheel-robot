#!/usr/bin/env python3
"""Compute and print URDF inertia values from robot_config.yaml.

This script reads the robot config and computes analytical inertias,
so URDF values can be regenerated if parameters change.

Usage:
    python scripts/generate_urdf_inertias.py
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    from balancing_robot.config import load_robot_config
    cfg = load_robot_config()

    r = cfg["wheel"]["radius"]       # m
    w = cfg["wheel"]["width"]        # m
    mw = cfg["wheel"]["mass"]        # kg
    H = cfg["chassis"]["height"]     # m
    W = cfg["chassis"]["width"]      # m
    D = cfg["chassis"]["depth"]      # m
    mc = cfg["chassis"]["mass"]      # kg

    # Chassis inertia (box, uniform density)
    # I_xx = m/12 * (H^2 + D^2)  [rotation about X axis, H and D are transverse]
    Ic_xx = mc / 12 * (H**2 + D**2)
    # I_yy = m/12 * (W^2 + D^2)  [rotation about Y, pitch axis]
    Ic_yy = mc / 12 * (W**2 + D**2)
    # I_zz = m/12 * (W^2 + H^2)  [rotation about Z, yaw axis]
    Ic_zz = mc / 12 * (W**2 + H**2)

    # Wheel inertia (solid cylinder, spin axis = Y)
    # I_yy (spin) = 1/2 * m * r^2
    Iw_yy = 0.5 * mw * r**2
    # I_xx = I_zz (perpendicular) = m/12 * (3r^2 + L^2) where L = width
    Iw_perp = mw / 12 * (3 * r**2 + w**2)

    # CoM height above axle
    l = H / 2  # chassis CoM above axle (geometric centre)
    spawn_z = r  # wheel radius = base_link z at spawn

    print("=" * 60)
    print("INERTIA GENERATION REPORT")
    print("=" * 60)
    print(f"\nRobot parameters:")
    print(f"  Wheel radius r = {r} m")
    print(f"  Wheel width  w = {w} m")
    print(f"  Wheel mass  mw = {mw} kg")
    print(f"  Chassis H x W x D = {H} x {W} x {D} m")
    print(f"  Chassis mass mc = {mc} kg")

    print(f"\nChassis inertia (box, CoM at geometric centre):")
    print(f"  Ixx = {Ic_xx:.6f} kg*m^2  [about X, (H^2+D^2)/12*m]")
    print(f"  Iyy = {Ic_yy:.6f} kg*m^2  [about Y, (W^2+D^2)/12*m]  <- pitch axis")
    print(f"  Izz = {Ic_zz:.6f} kg*m^2  [about Z, (W^2+H^2)/12*m]")

    print(f"\nWheel inertia (solid cylinder, spin axis = Y):")
    print(f"  Iyy = {Iw_yy:.6f} kg*m^2  [spin axis, 0.5*mw*r^2]")
    print(f"  Ixx = Izz = {Iw_perp:.6f} kg*m^2  [perpendicular, mw/12*(3r^2+w^2)]")

    print(f"\nGeometry:")
    print(f"  Chassis CoM above axle l = {l:.4f} m")
    print(f"  Base spawn z = {spawn_z:.4f} m (= wheel radius)")
    print(f"  Chassis top z ~= {spawn_z + H:.4f} m from ground")

    total_mass = mc + 2 * mw
    com_z = (mc * (spawn_z + l) + 2 * mw * spawn_z) / total_mass
    print(f"\nTotal mass: {total_mass:.4f} kg")
    print(f"System CoM height above ground: {com_z:.4f} m")

    print("\n" + "=" * 60)
    print("URDF inertia values to use:")
    print("=" * 60)
    print(f"""
<link name="base_link">
  <inertial>
    <origin xyz="0 0 {l:.4f}" rpy="0 0 0"/>
    <mass value="{mc}"/>
    <inertia ixx="{Ic_xx:.6f}" ixy="0" ixz="0"
             iyy="{Ic_yy:.6f}" iyz="0"
             izz="{Ic_zz:.6f}"/>
  </inertial>
</link>

<link name="left_wheel_link">  <!-- same for right_wheel_link -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="{mw}"/>
    <inertia ixx="{Iw_perp:.6f}" ixy="0" ixz="0"
             iyy="{Iw_yy:.6f}" iyz="0"
             izz="{Iw_perp:.6f}"/>
  </inertial>
</link>
""")


if __name__ == "__main__":
    main()
