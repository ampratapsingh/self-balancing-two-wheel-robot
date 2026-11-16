# Novelty and Contribution

## Overview

This project goes beyond a standard textbook PID inverted-pendulum demonstration.
The contributions listed here are **claimed only where supported by experimental data**.
Each contribution references the specific result, figure, or table that supports it.

> **NOTE:** The claims below are limited to the saved simulation evidence. Verify all reference
> entries marked `[VERIFY]` in the literature review before final submission.

---

## Contribution 1: Systematic PID vs LQR Comparison Under Identical Conditions

**What was done:**
Both PID (tilt-based, three-term) and LQR (full-state, optimal) controllers were implemented from scratch, tested under exactly identical conditions (same robot, same initial conditions, same timestep, same disturbances, same metrics), and compared quantitatively.

**How it differs from basic implementation:**
Most textbook examples demonstrate one controller. This project implements both, proves they both work (or documents where they fail), and quantifies the trade-offs with nine performance metrics.

**How it was tested:**
Experiments E02 (PID nominal), E03 (LQR nominal), E04 (head-to-head comparison at multiple initial tilts).

**Supporting results:**
- Table: `results/tables/E04_comparison_table.csv`
- Figure: `report/figures/E04_tilt.png` (matched 10° comparison)
- Result: both PID and LQR complete the 15° run; PID settles faster while LQR has lower final displacement.

**Limitations:**
- Comparison is in simulation only; real hardware would show additional differences due to sensor noise, motor dynamics, and actuation delays.
- PID gains were manually tuned; LQR gains are optimal for the linear model but may not be optimal for the nonlinear PyBullet model.

---

## Contribution 2: Robustness vs Initial Tilt Angle (Region-of-Attraction Map)

**What was done:**
A systematic sweep of initial tilt angles (1° to 30°) was run for both PID and LQR controllers (Experiment E06). For each angle, the simulation was run to completion and classified as balanced or fell.

**How it differs from basic implementation:**
Standard demonstrations use a single small initial tilt. This contribution maps the region of attraction (the set of initial conditions from which the robot recovers) for each controller.

**How it was tested:**
Experiment E06: 12 tilt angles × 2 controllers = 24 runs.

**Supporting results:**
- Figure: `report/figures/E06_tilt.png`
- Table: `results/tables/E06_tilt_sweep.csv`
- Result: both PID and LQR succeed at every tested angle through 30°; their upper robustness limits remain unmeasured.

**Limitations:**
- 2D sweep only (no initial velocity or position offset).
- PyBullet simulation only; nonlinear dynamics mean the true region of attraction may differ from the linear model's prediction.

---

## Contribution 3: Disturbance Rejection Analysis

**What was done:**
An external force impulse was applied to the chassis mid-run (Experiment E05). The maximum tilt deviation, recovery time, and whether the robot eventually stabilised were recorded for both controllers.

**How it differs from basic implementation:**
Most demos balance from rest. This experiment tests robustness to unexpected external forces — an important practical requirement for real robots operating in environments with people or uneven terrain.

**How it was tested:**
Experiment E05: 10 N forward impulse at t=5s, 0.1s duration, applied at height 0.3m. Both controllers.

**Supporting results:**
- Figure: `report/figures/E05_tilt.png`
- Table: `results/tables/E05_disturbance_metrics.csv`
- Result: both controllers remain upright; their measured tilt and position trade-offs are reported in the table.

**Limitations:**
- Single disturbance magnitude tested in the primary experiment (10 N). The optional E07 sweeps magnitudes.
- Force is applied at a fixed height; real disturbances have varying application points.

---

## Future Work: Linear-Model vs PyBullet Mismatch Analysis

**Status:** Not claimed as a completed contribution. The project uses the linear model to design
LQR and E01 confirms physical open-loop falling, but it does not yet contain a saved quantitative
linear-versus-PyBullet overlay. Add that experiment before claiming a mismatch measurement.

**Limitations:**
- Linear model is valid only for small angles; mismatch grows with tilt amplitude.
- PyBullet contact model introduces non-ideal effects (cylinder contact, joint damping) not captured by the Lagrangian.

---

## Contribution 5: Position Drift Analysis and Controller Comparison

**What was done:**
The tilt-only PID controller's position drift was quantified and contrasted with the LQR controller's inherent position regulation (from the x and ẋ states in the LQR cost function).

**How it differs from basic implementation:**
Tilt-only PID drift is a known limitation, but most projects do not measure it quantitatively. This project reports final displacement, peak displacement, and final speed for both controllers under identical conditions.

**How it was tested:**
Experiments E02 and E03 (30s runs): `x_m` column in CSVs.

**Supporting results:**
- Figure: `report/figures/E02_E03_position_drift.png`
- Metrics: `results/tables/E02_E03_nominal_comparison.csv`
- Result: nominal PID final position is 0.018531 m versus 0.008544 m for LQR.

**Limitations:**
- LQR position regulation is linear and may not hold for large deviations.
- Neither controller includes explicit yaw/steering control; lateral drift is not measured.

---

## What is NOT Claimed as Novelty

- The basic PID implementation alone (standard textbook material).
- PyBullet simulation of a robot (widely used).
- The Lagrangian derivation (standard control textbook content).
- Ziegler–Nichols-style tuning (standard method).

---

## Contribution Template (for final report)

For each claimed contribution, the report Section 9 must state:
1. **What was done** (1–2 sentences)
2. **How it differs from baseline** (1 sentence)
3. **How it was tested** (experiment ID, conditions)
4. **Supporting evidence** (Figure/Table number and brief description)
5. **Limitations** (honest statement of what was not covered)
