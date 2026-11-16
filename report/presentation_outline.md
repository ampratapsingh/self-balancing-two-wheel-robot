# Presentation Outline: Self-Balancing Two-Wheel Robot

**Course:** CSAI4014 – Algorithms for Intelligent Systems and Robotics  
**Group:** [GROUP_NO]  
**Duration:** Maximum 17 slides (including title and conclusion)  
**Presentation duration:** ~15–20 minutes + Q&A

---

## Slide 1: Title Slide
**Content:**
- Title: "Self-Balancing Two-Wheel Robot using PyBullet"
- Sub-title: Inverted Pendulum Control via PID and LQR
- Group number, member names, SAP IDs
- Course, Semester, Date
- Institution logo

**Speaker:** [STUDENT_NAME_1]  
**Time:** 30s  
**Figure:** None (or institution logo)

---

## Slide 2: Problem Statement and Motivation
**Content:**
- Why is a two-wheeled robot interesting? (Segway, smart wheelchairs, delivery robots)
- The inverted pendulum challenge: CoM above axle → inherently unstable
- Project objective: virtual Segway that balances using only torque control

**Speaker:** [STUDENT_NAME_1]  
**Time:** 1 min  
**Figure:** Segway image (public domain) + diagram showing unstable CoM

---

## Slide 3: Project Architecture
**Content:**
- Folder structure overview (key modules only)
- Technology stack: PyBullet, NumPy/SciPy, Matplotlib, Pytest
- Architecture: URDF → Physics → Sensors → Controller → Actuators → Data → Analysis

**Speaker:** [STUDENT_NAME_2]  
**Time:** 1 min  
**Figure:** Simple architecture block diagram

---

## Slide 4: Robot Design (URDF)
**Content:**
- Key parameters table: wheel r=0.08m, chassis 0.45×0.20×0.12m, mass 2.5kg total
- Coordinate frames: X forward, Z up, θ=0 upright, θ>0 forward tilt
- Inertia derivation summary (box and cylinder formulas)
- PyBullet screenshot of robot in GUI

**Speaker:** [STUDENT_NAME_2]  
**Time:** 1.5 min  
**Figure:** `demo/robot_screenshot.png` (take screenshot of GUI mode)

---

## Slide 5: Mathematical Model
**Content:**
- State vector: [x, ẋ, θ, θ̇]
- Lagrangian → nonlinear EOM (2 equations, displayed clearly)
- Linearisation: sinθ≈θ, cosθ≈1
- Result: A matrix with key values highlighted

**Speaker:** [STUDENT_NAME_3]  
**Time:** 2 min  
**Figure:** A matrix displayed, eigenvalue highlighted (+10.97 → unstable)

---

## Slide 6: Stability Analysis
**Content:**
- Eigenvalues of A: [0, 0, +10.97, −10.97]
- Positive eigenvalue = unstable mode (time constant 91ms)
- Controllability rank = 4/4 → system IS controllable → control is possible
- E01 result: uncontrolled robot falls in [X]s from 2° — matches theory

**Speaker:** [STUDENT_NAME_3]  
**Time:** 1 min  
**Figure:** E01 tilt vs time (uncontrolled fall plot)

---

## Slide 7: PID Controller Design
**Content:**
- Error: e = θ_ref − θ (θ_ref = 0)
- P: immediate response | D: damps oscillations (on measurement) | I: removes offset
- Anti-windup, derivative filter
- Gains: Kp=35, Ki=0.5, Kd=2.5 (from tuning grid search)

**Speaker:** [STUDENT_NAME_1]  
**Time:** 1.5 min  
**Figure:** PID block diagram

---

## Slide 8: LQR Controller Design
**Content:**
- Full state feedback: u = −K·[x, ẋ, θ, θ̇]ᵀ
- Minimises J = ∫(xᵀQx + uᵀRu)dt
- Q = diag(1, 1, 100, 10), R = 0.1 (selected via sweep)
- K computed by scipy.linalg.solve_continuous_are
- Closed-loop eigenvalues: all negative → guaranteed stable (for linear model)

**Speaker:** [STUDENT_NAME_4]  
**Time:** 1.5 min  
**Figure:** LQR block diagram, table of K values

---

## Slide 9: Experimental Setup
**Content:**
- 6 experiments (E01–E06) described briefly
- Identical conditions for fair comparison: same URDF, timestep, initial tilt, seed
- Metric definitions: settling time, RMS tilt, max tilt, drift

**Speaker:** [STUDENT_NAME_2]  
**Time:** 1 min  
**Figure:** Experiment overview table

---

## Slide 10: PID Balancing Results (E02)
**Content:**
- Tilt vs time plot (30s run)
- Key metrics: settling time, RMS tilt, final drift
- Observation: robot balances but drifts in position

**Speaker:** [STUDENT_NAME_1]  
**Time:** 1 min  
**Figure:** `results/plots/fig_02_tilt_pid.png`

---

## Slide 11: LQR Balancing Results (E03)
**Content:**
- Tilt vs time plot (30s run)
- Key metrics comparison vs PID
- Observation: LQR settles faster, less drift (position regulation built in)

**Speaker:** [STUDENT_NAME_4]  
**Time:** 1 min  
**Figure:** `results/plots/fig_03_tilt_lqr.png`

---

## Slide 12: PID vs LQR Comparison (E04)
**Content:**
- Side-by-side metrics table (all 9 metrics)
- Overlay tilt plot
- Key finding: LQR settles [X]% faster; PID has [Y] smaller peak torque

**Speaker:** [STUDENT_NAME_3]  
**Time:** 1.5 min  
**Figure:** `results/plots/fig_05_pid_vs_lqr_overlay.png` + Table 6

---

## Slide 13: Disturbance Response (E05) and Tilt Sweep (E06)
**Content:**
- E05: 10N impulse at t=5s; both controllers recover
- E06: success/fail map (shows LQR handles larger initial tilts)
- Key finding: LQR region of attraction extends to [X]° vs PID's [Y]°

**Speaker:** [STUDENT_NAME_4]  
**Time:** 1.5 min  
**Figure:** `results/plots/fig_06_disturbance_recovery.png` + `fig_07_tilt_sweep_map.png`

---

## Slide 14: Novelty Contributions
**Content:**
1. ✓ PID vs LQR systematic comparison (identical conditions)
2. ✓ Region-of-attraction map (tilt sweep)
3. ✓ Disturbance rejection quantification
4. ✓ Linear model vs PyBullet mismatch analysis
5. ✓ Position drift measurement and comparison

**Speaker:** [STUDENT_NAME_2]  
**Time:** 1 min  
**Figure:** Summary table of contributions

---

## Slide 15: Testing and Validation
**Content:**
- 47+ automated pytest tests (unit + integration)
- Key validations: URDF loads, robot falls without control, both controllers balance ≥20s
- Determinism: same seed → identical CSV (verified)
- CI: GitHub Actions runs tests on every push

**Speaker:** [STUDENT_NAME_1]  
**Time:** 1 min  
**Figure:** pytest output summary

---

## Slide 16: Limitations and Future Work
**Content:**
- Limitations: simulation only, linearisation range, no yaw, PID drift, no adaptation
- Future work: Kalman filter, real hardware, steering, adaptive control, ML-based controller

**Speaker:** [STUDENT_NAME_3]  
**Time:** 45s  
**Figure:** None (bullet points)

---

## Slide 17: Conclusion
**Content:**
- Achieved: PyBullet simulation, URDF, PID + LQR, 5 novelty contributions, 47+ tests
- Key result: Both controllers achieve ≥20s balance; LQR outperforms PID in [specific metrics]
- Repository: github.com/[GITHUB_USERNAME]/self-balancing-robot-pybullet
- Thank you + Questions

**Speaker:** All team members  
**Time:** 1 min  
**Figure:** Final screenshot of balanced robot in GUI

---

## Speaker Notes Guidelines
- Each speaker should know: their slides AND the entire project (viva can ask anything)
- Practice with the actual simulation running
- Prepare for questions: see `docs/viva_prep.md`
- If the live demo fails, switch to pre-recorded video in `demo/`

---

## Slide Design Guidelines
- White background with institution colour scheme
- Font: Arial or Calibri (not Times New Roman — use a sans-serif for slides)
- Font size: Title 36pt, body 24pt, captions 18pt
- Maximum 5–6 bullet points per slide
- Each bullet one line; no full sentences
- All plots from actual experiment results (no placeholder images)
