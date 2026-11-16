#!/usr/bin/env python3
"""Inspect URDF: print links, joints, axes, masses, inertias, total CoM.

Usage:
    python scripts/inspect_urdf.py [--urdf path/to/file.urdf]

Output is also saved to docs/urdf_inspection.txt.
"""
import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description="Inspect robot URDF via PyBullet")
    parser.add_argument("--urdf", default=str(REPO_ROOT / "assets" / "robot" / "balancing_robot.urdf"))
    parser.add_argument("--output", default=str(REPO_ROOT / "docs" / "urdf_inspection.txt"))
    args = parser.parse_args()

    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print("ERROR: pybullet not installed. Run: pip install pybullet")
        sys.exit(1)

    lines = []

    def log(msg=""):
        print(msg)
        lines.append(str(msg))

    # Start headless PyBullet
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)

    robot_id = p.loadURDF(args.urdf, basePosition=[0, 0, 0.08],
                          flags=p.URDF_USE_INERTIA_FROM_FILE,
                          physicsClientId=client)

    num_joints = p.getNumJoints(robot_id, physicsClientId=client)
    num_links = num_joints  # PyBullet: num links = num joints (base_link is -1)

    log("=" * 70)
    log("URDF INSPECTION REPORT")
    log(f"File: {args.urdf}")
    log("=" * 70)

    # Base link
    base_info = p.getDynamicsInfo(robot_id, -1, physicsClientId=client)
    log(f"\nBASE LINK (index -1): base_link")
    log(f"  mass          = {base_info[0]:.6f} kg")
    log(f"  local CoM     = {base_info[3]}")
    log(f"  inertia diag  = {base_info[2]}")  # (ixx, iyy, izz)
    log(f"  local CoM quat= {base_info[4]}")

    log(f"\nJOINTS AND LINKS ({num_joints} joints):")
    joint_name_to_index = {}
    for i in range(num_joints):
        ji = p.getJointInfo(robot_id, i, physicsClientId=client)
        jtype_map = {
            p.JOINT_REVOLUTE: "REVOLUTE",
            p.JOINT_CONTINUOUS: "CONTINUOUS",
            p.JOINT_PRISMATIC: "PRISMATIC",
            p.JOINT_FIXED: "FIXED",
            p.JOINT_SPHERICAL: "SPHERICAL",
            p.JOINT_PLANAR: "PLANAR",
        }
        jtype = jtype_map.get(ji[2], f"UNKNOWN({ji[2]})")
        axis = ji[13]
        parent_name = ji[12] if ji[12] else "world"
        link_name = ji[12]  # ji[12] is the link name
        # Actually ji[1] is joint name, ji[12] is parent link frame name
        joint_name = ji[1].decode() if isinstance(ji[1], bytes) else ji[1]
        link_name_child = ji[12].decode() if isinstance(ji[12], bytes) else ji[12]
        # Note: PyBullet getJointInfo returns:
        # [0]=index, [1]=jointName, [2]=jointType, [3]=qIndex, [4]=uIndex,
        # [5]=flags, [6]=jointDamping, [7]=jointFriction, [8]=lowerLimit,
        # [9]=upperLimit, [10]=maxForce, [11]=maxVelocity, [12]=linkName,
        # [13]=jointAxis, [14]=parentFramePos, [15]=parentFrameOrn, [16]=parentIndex

        link_name_str = ji[12].decode() if isinstance(ji[12], bytes) else str(ji[12])
        joint_name_str = ji[1].decode() if isinstance(ji[1], bytes) else str(ji[1])
        joint_name_to_index[joint_name_str] = i

        log(f"\n  Joint {i}: {joint_name_str}")
        log(f"    type          = {jtype}")
        log(f"    link (child)  = {link_name_str}")
        log(f"    axis          = {ji[13]}")
        log(f"    parent frame pos = {ji[14]}")
        log(f"    damping       = {ji[6]:.6f}")
        log(f"    friction      = {ji[7]:.6f}")

        dinfo = p.getDynamicsInfo(robot_id, i, physicsClientId=client)
        log(f"    mass          = {dinfo[0]:.6f} kg")
        log(f"    inertia diag  = ixx={dinfo[2][0]:.6e}, iyy={dinfo[2][1]:.6e}, izz={dinfo[2][2]:.6e}")
        log(f"    local CoM     = {dinfo[3]}")

    # Total CoM
    total_mass = 0.0
    weighted_pos = [0.0, 0.0, 0.0]
    # Reset and step to get positions
    p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.08], [0, 0, 0, 1], physicsClientId=client)
    pos_b, _ = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
    base_mass = p.getDynamicsInfo(robot_id, -1, physicsClientId=client)[0]
    total_mass += base_mass
    for k in range(3):
        weighted_pos[k] += base_mass * pos_b[k]
    for i in range(num_joints):
        ls = p.getLinkState(robot_id, i, physicsClientId=client)
        link_pos = ls[0]  # world position of link CoM
        link_mass = p.getDynamicsInfo(robot_id, i, physicsClientId=client)[0]
        total_mass += link_mass
        for k in range(3):
            weighted_pos[k] += link_mass * link_pos[k]
    com = [wp / total_mass for wp in weighted_pos]
    log(f"\nTOTAL MASS: {total_mass:.4f} kg")
    log(f"TOTAL CoM (world frame, robot upright): {com}")
    log(f"\nJoint name → index mapping: {joint_name_to_index}")
    log("\n" + "=" * 70)
    log("END OF INSPECTION")
    log("=" * 70)

    p.disconnect(physicsClientId=client)

    # Save to file
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(lines))
    print(f"\nSaved to {out_path}")


if __name__ == "__main__":
    main()
