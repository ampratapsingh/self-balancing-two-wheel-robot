# Robot Asset: Coordinate Frames and Conventions

## Coordinate Frame

- **World frame**: X-axis points **forward** (direction of travel), Y-axis points **left**, Z-axis points **up**.
- All positions and velocities are expressed in world frame unless stated otherwise.

## Tilt Convention

- **θ (theta)**: The robot's tilt angle measured from the vertical (Z-axis).
- **θ > 0**: Robot tilts **forward** (in the +X direction).
- **θ < 0**: Robot tilts **backward** (in the -X direction).
- **θ = 0**: Robot is perfectly upright (balanced).

## Geometry

| Parameter             | Value    | Notes                             |
|-----------------------|----------|-----------------------------------|
| Wheel radius          | 0.08 m   | Used for spawn height computation |
| Wheel spawn height    | 0.08 m   | Center of wheel at ground + radius |
| CoM height above axle | 0.225 m  | Center of chassis above wheel axle |
| Total CoM height      | ~0.305 m | Approx. wheel center + CoM offset |

## Joint Axis

- Wheel joints rotate about the **Y-axis** (left-right axis).
- Left wheel and right wheel are symmetric about the robot's sagittal plane.

## Torque Sign Convention

- **Positive torque** applied to both wheels → robot moves **forward** (+X direction).
- **Negative torque** applied to both wheels → robot moves **backward** (-X direction).
- Controllers output a **total torque** value which is split equally between left and right wheels.

## URDF Notes

- The URDF file `balancing_robot.urdf` is generated/placed here by the URDF generation script (Phase 1).
- Link names follow the convention: `base_link`, `left_wheel`, `right_wheel`.
- Joint names: `left_wheel_joint`, `right_wheel_joint`.
