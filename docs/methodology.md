# Methodology

## 1. Metric Definitions

All metrics are computed by `src/balancing_robot/analysis/metrics.py` from saved CSV data.
Definitions are fixed and must not change between experiments.

### 1.1 State Variables
| Variable | Symbol | Units | Definition |
|----------|--------|-------|------------|
| Tilt angle | θ | rad / degrees in plots | Pitch angle; θ=0 upright, θ>0 forward lean |
| Tilt rate | θ̇ | rad/s | d(θ)/dt |
| Position | x | m | Chassis horizontal position (+X = forward) |
| Velocity | ẋ | m/s | Chassis horizontal velocity |
| Wheel velocity | φ̇_L, φ̇_R | rad/s | Individual wheel angular velocities |
| Total torque | τ | Nm | τ_L + τ_R (sum of both wheel torques) |

### 1.2 Settling Time
**Definition:** The first time t_s such that |θ(t)| ≤ 0.5° for all t ≥ t_s.
- Threshold: ±0.5° absolute (≈ ±0.00873 rad)
- If the robot never settles to this band, settling time is reported as `None`

**Rationale:** 0.5° is a tight practical threshold that distinguishes genuine regulation from sustained oscillation.

### 1.3 Steady-State Error
**Definition:** Mean |θ| over the last 5 seconds of a successful run.
- If the run is shorter than 5 seconds, uses all available data.
- Reported in both rad and degrees.

### 1.4 Overshoot
**Definition:** `max(0, max|θ| − |θ(0)|)` where θ(0) is the initial tilt.
- For a regulation response from initial tilt θ_0: overshoot = how much the tilt exceeds the starting value in the opposite direction.
- Reported in degrees.

### 1.5 RMS Tilt Error
**Definition:** `√(mean(θ²))` over the entire run duration.
- Lower is better. Penalises both large deviations and slow settling.

### 1.6 Success Flag
**Definition:** A run is "successful" if:
1. The robot maintains |θ| < 45° (fall threshold) for the entire duration, AND
2. No chassis–ground contact occurs (not explicitly tracked; fall threshold proxy).

Runs that "fail" are still reported with time-to-fall.

---

## 2. Tuning Protocol (PID)

### Step 1: Proportional-only (Ki=0, Kd=0)
- Increase Kp from 0 until the robot oscillates or marginally balances.
- Record the critical gain K_cr and oscillation period T_cr.
- These inform initial Kd selection (Kd ≈ K_cr * T_cr / 8π per Ziegler–Nichols).

### Step 2: Add Derivative
- With Ki=0, increase Kd until oscillations are damped.
- Derivative filter alpha (0.7 default) reduces noise amplification.

### Step 3: Add Integral (if needed)
- Only if a persistent steady-state offset exists in a successful Kp+Kd run.
- Keep Ki small (≤ 1.0 Nm·s/rad). Watch for integral windup.

### Step 4: Refinement
- Run a grid search over `[Kp, Kd]` with Ki fixed.
- Score: ITAE (Integral of Time-weighted Absolute Error) = ∫t|θ(t)|dt over 20s.
- Lower score = better. Save results to `results/tables/pid_tuning.csv`.

### Step 5: Validation
- Test on initial conditions **different from those used for tuning**.
- Report both tuning-condition and validation-condition performance.

---

## 3. Q/R Selection Protocol (LQR)

### Starting point
- Q = diag(q_x, q_ẋ, q_θ, q_θ̇) with strong tilt penalty: q_θ >> q_x.
- Default: Q = diag(1, 1, 100, 10), R = 0.1.

### Sweep
- Vary q_θ in {10, 50, 100, 500, 1000} and R in {0.01, 0.1, 1.0, 10.0}.
- For each combination: compute K, check closed-loop eigenvalues, run 10s simulation, record metrics.
- Save to `results/tables/lqr_qr_sweep.csv`.

### Selection criterion
- Choose Q/R that gives: (1) all eigenvalues stable, (2) lowest ITAE, (3) acceptable torque usage (peak < max_torque × 0.8).

---

## 4. Experiment Protocols

### E01: Uncontrolled Fall
- Initial tilt: 2° forward
- Controller: none (zero torque)
- Duration: 5 s (robot falls long before this)
- Record: fall time, fall trajectory
- Compare with linear-model prediction: `t_fall ≈ 1/λ_unstable * arccosh(45°/2°)` (approximate)

### E02: PID Balancing (nominal)
- Initial tilt: 3°
- Duration: 30 s
- Seed: 42
- Noise: off
- Record: all CSV columns

### E03: LQR Balancing (nominal)
- Same conditions as E02 (identical for fair comparison)

### E04: PID vs LQR Comparison
- Initial tilts: [2°, 5°, 10°, 15°]
- Duration: 20 s each
- Both controllers, same seed
- Produce: overlay plots, metrics table

### E05: Disturbance Response
- Initial tilt: 3° (balanced first for ≥5s)
- Disturbance: 10 N impulse, forward direction, applied at t=5s, duration 0.1s, height 0.3m
- Both controllers
- Measure: max tilt deviation, recovery time, final stability

### E06: Initial Tilt Sweep
- Tilts: 1°, 2°, 3°, 5°, 7°, 10°, 12°, 15°, 18°, 20°, 25°, 30°
- Both controllers
- Duration: 15s per run
- Output: success/fail map

---

## 5. Reproducibility Requirements

- All runs generated from the current code write a metadata JSON alongside the CSV containing:
  - `git_commit`: current HEAD commit hash
  - `python_version`: Python version string
  - `pybullet_version`: PyBullet version
  - `seed`: random seed used
  - `config_snapshot`: copy of all relevant config values
  - `saved_at`: timezone-aware ISO-8601 timestamp

The Phase 2 CSVs remain the original evidence set. Their already-saved JSON files predate
the provenance update and therefore contain only the original run fields; rerun an experiment
to produce an expanded metadata record without altering those historical outputs.

- Determinism: given the same seed and config, two runs must produce identical CSV files.
  - Verified by running the same experiment twice and diffing the CSVs.
  - `setRealTimeSimulation(0)` and manual stepping ensures determinism.
  - PyBullet solver is seeded implicitly through fixed timestep and no random elements.

---

## 6. Comparison Fairness

When comparing PID vs LQR, all of the following must be identical:
- Robot physical parameters (same URDF)
- Gravity, timestep, solver settings
- Initial tilt angle
- Simulation duration
- Disturbance timing, magnitude, direction
- Controller update rate (240 Hz for both)
- Sensor noise settings (off for primary comparison)
- Random seed
- Metric definitions

Any deviation from these conditions must be explicitly documented.
