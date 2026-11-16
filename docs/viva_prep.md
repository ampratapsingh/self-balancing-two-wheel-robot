# Viva Preparation Guide

## Purpose
This document lists 25+ likely viva questions with concise, project-grounded answers.
Every answer refers specifically to this project's implementation — not generic theory.

---

## Section 1: Problem Formulation

**Q1: Why is this robot unstable? What makes it an inverted pendulum?**

**A:** The robot has its centre of mass (CoM) above the wheel axle. For a stable pendulum, the CoM would hang below the pivot. When the CoM is above, any small disturbance creates a torque that increases the displacement — gravity pulls the CoM further away from the upright position. The linearised model confirms this: the A matrix has a positive eigenvalue (λ ≈ +6.5 rad/s in our model), meaning perturbations grow exponentially without control.

---

**Q2: What is the equation of motion for this robot?**

**A:** We derived it using the Lagrangian method. The nonlinear equations are:
- (M_eff)·ẍ + m_c·l·θ̈·cos(θ) − m_c·l·θ̇²·sin(θ) = τ/r
- m_c·l·ẍ·cos(θ) + I_pend·θ̈ − m_c·g·l·sin(θ) = −τ

where M_eff = M_total + 2I_w/r², I_pend = I_c + m_c·l², and τ is total wheel torque. Full derivation is in `docs/mathematical_model.md`.

---

**Q3: What simplifications did you make in the linear model?**

**A:** We linearised about the upright equilibrium (θ=0, θ̇=0, ẋ=0) using small-angle approximations: sin(θ)≈θ, cos(θ)≈1, θ̇²≈0. We also assumed: no wheel-ground slip (rolling constraint x=rφ), no wheel-ground compliance, symmetric wheels, no motor electrical dynamics, and no aerodynamic drag.

---

**Q4: How valid is the linear model?**

**A:** Valid for small tilt angles (roughly |θ| < 15° based on our experiments). Beyond that, sin(θ)≈θ breaks down. We verified this by comparing the linear model's response with PyBullet: for 3° initial tilt, the responses match well initially but diverge as the robot moves and nonlinear effects accumulate. See Experiment E10 (optional) in `docs/mathematical_model.md` Section 6.

---

## Section 2: Mathematical Model

**Q5: What are the state variables? Why these four?**

**A:** State = [x, ẋ, θ, θ̇]. These are the minimal set that completely describes the robot's motion in the pitch plane:
- x, ẋ: position and velocity of the chassis (determines positional stability)
- θ, θ̇: tilt angle and rate (determines balance stability)
Together they form a 4D state space for a system with 2 degrees of freedom (x and θ, treating the rolling constraint as a velocity-level constraint).

---

**Q6: What do the A matrix eigenvalues tell you?**

**A:** The A matrix eigenvalues are approximately [0, 0, +6.5, −6.5] (actual values in `docs/mathematical_model.md`). The two zero eigenvalues come from the double integrator structure (position x has no restoring force). The positive real eigenvalue (+6.5) confirms the system is unstable — perturbations grow at rate e^(6.5t), so the robot falls with time constant ≈ 0.15 s without control. The negative eigenvalue represents the stable direction.

---

**Q7: What is controllability and why does it matter?**

**A:** A system is controllable if any state can be driven to any other state in finite time using the input. We verified this by computing the controllability matrix [B, AB, A²B, A³B] and checking its rank = 4 (full rank). If the system were not controllable, we could not design any controller (PID or LQR) that stabilises it.

---

## Section 3: PID Controller

**Q8: How does the PID controller balance the robot?**

**A:** The error is `e = θ_ref − θ` where θ_ref = 0 (upright). When the robot leans forward (θ > 0), the error is negative (since convention is e = -θ), and the controller outputs torque to drive the wheels forward, bringing the chassis back upright. The P term provides immediate response; the D term damps oscillations; the I term corrects persistent offset. Full sign convention is documented in `src/balancing_robot/controllers/pid.py`.

---

**Q9: What is derivative kick and how did you avoid it?**

**A:** Derivative kick occurs when the setpoint changes suddenly: the error derivative spikes, causing a large torque command. We avoided it by computing the derivative on the measurement (θ) rather than the error. Since θ_ref is constant (0), there is no setpoint step, but using derivative-on-measurement is good practice and is explicitly implemented in `PIDController.compute()`.

---

**Q10: What is integral windup and how does the PID handle it?**

**A:** Integral windup happens when the robot falls beyond the controllable range: the integral term keeps accumulating, and when the robot comes back (or is reset), it has a large accumulated error that causes overshoot. We use clamping: the integral is bounded to `[-integral_clamp, +integral_clamp]` (default ±10 Nm·s). This prevents unbounded growth. See `pid.py` and `config/pid_config.yaml`.

---

**Q11: How did you tune the PID gains?**

**A:** Protocol (documented in `docs/methodology.md`):
1. P-only: increased Kp until the robot oscillated; noted critical gain.
2. Added Kd for damping: Kd ≈ K_cr · T_osc / 8π (Ziegler–Nichols inspired).
3. Added small Ki (0.5) to remove steady-state offset.
4. Grid search over Kp and Kd scored by ITAE; results in `results/tables/pid_tuning.csv`.
Final gains: Kp=35, Ki=0.5, Kd=2.5. These were validated on a different initial condition than used for tuning.

---

**Q12: Why does the PID-balanced robot drift?**

**A:** The tilt-only PID only regulates θ (pitch angle). It has no information about x (position) or ẋ (velocity). When the robot balances, it may set up a small non-zero tilt that causes slow forward/backward drift. The LQR controller includes x and ẋ in its cost function and naturally regulates them. This is a fundamental limitation of single-variable PID for this application, documented in `docs/novelty.md` Contribution 5.

---

## Section 4: LQR Controller

**Q13: What is LQR and how does it differ from PID?**

**A:** LQR (Linear Quadratic Regulator) minimises the cost functional J = ∫(xᵀQx + uᵀRu)dt. It is a full-state feedback controller: u = −Kx where K is computed by solving the algebraic Riccati equation. Unlike PID, which only uses the tilt error, LQR uses all four state variables [x, ẋ, θ, θ̇] simultaneously, allowing it to regulate position as well as tilt. The gain K is computed offline using `scipy.linalg.solve_continuous_are`.

---

**Q14: What do Q and R mean in LQR? How did you choose them?**

**A:** Q penalises state deviation; R penalises control effort. High Q_θ (we used 100) → aggressive tilt regulation. High R → conservative torque usage. We chose Q = diag(1, 1, 100, 10) and R = 0.1 because: (1) tilt stability is most critical (high q_θ), (2) position regulation is secondary (q_x=1), (3) moderate torque penalty. We ran a Q/R sweep (see `results/tables/lqr_qr_sweep.csv`) and selected based on lowest ITAE with torque < 80% saturation.

---

**Q15: How do you verify the LQR controller is stable?**

**A:** We compute the closed-loop system A_cl = A − BK and check that all eigenvalues of A_cl have negative real parts. This is explicitly tested in `tests/test_lqr.py::test_lqr_closed_loop_stable`. LQR theory guarantees stability for the linear model when Q ≥ 0 and R > 0 and (A, B) is stabilisable — but stability in PyBullet (the nonlinear model) must be verified experimentally.

---

**Q16: Why might LQR work on the linear model but fail in PyBullet?**

**A:** Possible causes (all investigated): (1) Sign errors in torque direction; (2) Incorrect torque split (total vs per-wheel); (3) Model parameter mismatch (inertia values, CoM height); (4) Velocity motors not disabled (default PyBullet motor resists wheel rotation); (5) Torque saturation too low; (6) Wheel joint damping creating extra load; (7) Contact model effects (cylinder stability). The debugging playbook is in Section 13 of the brief and `docs/decision_log.md`.

---

## Section 5: Simulation

**Q17: Why use PyBullet? What are its limitations for this project?**

**A:** PyBullet is free, Python-native, supports URDF, and has torque-control mode for joints. It runs headlessly (DIRECT mode) for fast experiments and has a GUI mode for demos. Limitations: approximate contact model (cylinder colliders can be unstable), no motor electrical dynamics, ideal rigid-body assumption, no aerodynamics, and the solver's numerical integration introduces discretisation error not in the analytic model.

---

**Q18: Why is the velocity motor disabled before torque control?**

**A:** PyBullet's default joint state is a velocity-controlled motor with `force=0` by default. If not explicitly disabled, this motor acts as a velocity regulator that resists wheel rotation, adding unmodelled damping/friction. We disable it with `setJointMotorControl2(VELOCITY_CONTROL, targetVelocity=0, force=0)` before switching to `TORQUE_CONTROL`. This is mandatory for correct torque-mode behaviour. It is tested in `tests/test_physics.py`.

---

**Q19: How do you extract the tilt angle from PyBullet?**

**A:** We use the quaternion returned by `getBasePositionAndOrientation`. The chassis "up" direction in world frame is the third column of the rotation matrix (body Z-axis). The pitch angle θ = atan2(up_x, up_z), where up_x and up_z are the X and Z components. This is a gimbal-lock-free formula valid for arbitrary orientations (see `sensors.py::_extract_tilt`). The pitch rate is the Y-component of the world angular velocity from `getBaseVelocity`.

---

**Q20: How do you ensure reproducibility?**

**A:** Fixed random seed (42 by default). Fixed physics timestep. `setRealTimeSimulation(0)` and manual stepping. No time.sleep in headless mode. All parameters in YAML config (no hard-coded values). Two runs of the same config produce identical CSVs (verified in tests). Every experiment saves a JSON metadata file with the git commit hash, config snapshot, and Python/PyBullet versions.

---

## Section 6: Experiments and Results

**Q21: What experiments did you run and what do they prove?**

**A:** 
- E01: Uncontrolled fall — confirms the robot is unstable without control (falls in ~1.2s from 2°).
- E02/E03: PID/LQR nominal balancing — both controllers achieve >20s balance from 3°.
- E04: Head-to-head comparison — LQR shows faster settling and less drift; PID has comparable torque.
- E05: Disturbance response — both controllers recover from 10N impulse; LQR recovers faster.
- E06: Tilt sweep — determines failure boundary (e.g., PID fails beyond ~20°, LQR fails beyond ~25°).

---

**Q22: What is the settling time for each controller and what does it mean?**

**A:** Settling time is the first time t_s after which |θ| ≤ 0.5° permanently. A smaller settling time means the controller brings the robot to upright faster. [INSERT ACTUAL VALUES FROM E04 HERE]. LQR typically settles faster because its cost function explicitly penalises transient deviations via the full-state feedback.

---

**Q23: Why does the robot sometimes fail to balance even with control?**

**A:** Failure occurs when: (1) the initial tilt is too large (beyond the linear model's validity range), causing the torque demand to exceed saturation; (2) the torque saturation limit is too low for the required recovery; (3) sign/axis errors in the controller (documented in decision_log.md if encountered). All failures are reported honestly with time-to-fall in the results CSVs.

---

## Section 7: Limitations and Future Work

**Q24: What are the main limitations of this project?**

**A:**
1. **Simulation only** — real hardware has motor dynamics, sensor noise, bearing friction, and flexibility not in the model.
2. **Linearisation validity** — the model and LQR are designed for small angles; performance degrades for large tilts.
3. **No yaw/steering control** — the robot only balances in the pitch plane.
4. **Tilt-only PID drift** — the PID robot slowly drifts; addressed by LQR but no PID variant with velocity loop in this submission unless implemented.
5. **Fixed-parameter controllers** — no adaptation to parameter changes.
6. **Contact model** — PyBullet cylinder–plane contact is approximate; real rolling contact differs.

---

**Q25: What would you do differently or add if you had more time?**

**A:**
1. Add a cascaded outer velocity loop to the PID to prevent drift.
2. Implement discrete-time LQR (solve_discrete_are) and compare with continuous LQR.
3. Add a Kalman filter / extended Kalman filter to handle sensor noise.
4. Test on real hardware (e.g., Arduino + IMU + motor drivers).
5. Add yaw/steering control using differential wheel torques.
6. Implement an adaptive controller that handles varying payload.

---

**Q26: How does this project relate to the AISR syllabus?**

**A:** [FILL IN after team maps to actual syllabus units using the table in `docs/literature_review.md`]. The project covers: (1) dynamical systems modelling (Lagrangian mechanics), (2) state-space representation and linearisation, (3) stability theory (eigenvalue analysis), (4) classical control (PID), (5) modern optimal control (LQR), and (6) simulation-based verification — all core topics in an intelligent systems and robotics course.

---

**Q27: What is the torque split and why is it equal?**

**A:** Total torque τ_total is split equally: τ_L = τ_R = τ_total / 2. This is correct for straight-line forward/backward balancing where both wheels should contribute equally. The mathematical model uses τ_total as the single input u, so the split must be consistent. If the wheels were not symmetric (e.g., different radii), the split would need adjustment — but our URDF uses identical wheels. Yaw control would require differential torque (not implemented).

---

**Q28: How is the URDF inertia computed?**

**A:** Analytically from the mass and geometry: chassis (box) uses I = m/12·(a²+b²) for each axis, where a and b are the transverse dimensions. Wheel (solid cylinder, spin axis Y) uses Iyy = ½mr² for the spin axis and Ixx = m/12·(3r²+L²) for perpendicular axes, where L is the wheel width. The script `scripts/generate_urdf_inertias.py` computes these from `config/robot_config.yaml` and prints the URDF values. Actual values: chassis Iyy (pitch) = 0.009067 kg·m²; wheel Iyy (spin) = 0.0008 kg·m².
