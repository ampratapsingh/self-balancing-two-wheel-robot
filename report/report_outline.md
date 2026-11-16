# Report Outline: Self-Balancing Two-Wheel Robot (Inverted Pendulum)

**Course:** CSAI4014 – Algorithms for Intelligent Systems and Robotics  
**Programme:** B.Tech CSE (AI & ML), Semester VII  
**Group:** [GROUP_NO]  
**Date:** [SUBMISSION_DATE]  

**Target:** 25–35 pages (body text, excluding front matter)  
**Format:** A4 portrait, Times New Roman 12pt, 1.5 line spacing, justified, 3cm left margin, 2.5cm other margins, single-line page border on every page.

---

## Front Matter (Roman Numerals i–vii)

| Section | Pages | Content |
|---------|-------|---------|
| Title Page | i | Title, group, course, date, institution, supervisor [FACULTY_NAME] |
| Certificate | ii | Official certificate (template from institution) |
| Declaration | iii | "We declare that this work is our own..." [all signatures] |
| Acknowledgement | iv | Acknowledgements to [FACULTY_NAME], department, etc. |
| Abstract | v | 150–200 words: problem, methods (PID + LQR), key results, conclusion |
| Index (Table of Contents) | vi | Auto-generated from section headings |
| List of Figures | vi–vii | All figures with numbers, captions, and page numbers |
| List of Tables | vii | All tables with numbers, captions, and page numbers |

---

## Chapter 1: Introduction (2–3 pages)

**Target: 2.5 pages**

1.1 Motivation and background  
1.2 Problem statement: the inverted pendulum challenge  
1.3 Project objectives (numbered list)  
1.4 Scope and organisation of the report  

**Key figure:** Photo/screenshot of the PyBullet GUI showing the robot upright (use `demo/` screenshot)

---

## Chapter 2: Literature Review (3–4 pages)

**Target: 3.5 pages**

2.1 Inverted pendulum control — historical context  
2.2 Self-balancing robots: JOE [3] and related work  
2.3 PID control for balancing systems [2, 6]  
2.4 LQR and optimal control [5, 8]  
2.5 PyBullet and simulation-based design [4]  
2.6 Summary and research gap  

**All 8+ references must be cited at least once across Chapters 2–9.**

---

## Chapter 3: Problem Formulation and Syllabus Mapping (1–2 pages)

**Target: 1.5 pages**

3.1 System description (physical setup)  
3.2 Control objective (maintain upright balance)  
3.3 Assumptions and constraints  
3.4 Syllabus mapping table: [SYLLABUS_UNIT] ↔ [PROJECT_COMPONENT]  

---

## Chapter 4: Mathematical Modelling (4–5 pages)

**Target: 4.5 pages**

4.1 Coordinate frame and variable definitions  
4.2 Lagrangian derivation  
  - 4.2.1 Kinetic energy (chassis, wheels, rolling constraint)  
  - 4.2.2 Potential energy  
  - 4.2.3 Generalised forces (torque input)  
  - 4.2.4 Nonlinear equations of motion  
4.3 Linearisation about upright equilibrium  
  - 4.3.1 Small-angle approximation  
  - 4.3.2 Linear state-space form A, B, C, D  
4.4 Numerical model (A/B from config parameters)  
4.5 Eigenvalue analysis  
  - 4.5.1 Open-loop eigenvalues (confirm instability)  
  - 4.5.2 Controllability  
4.6 Model limitations and mismatch analysis  

**Key table:** Parameter table (mass, geometry, inertias, CoM height)  
**Key figure:** Block diagram of the linearised system

---

## Chapter 5: Robot Design and Simulation Setup (2–3 pages)

**Target: 2.5 pages**

5.1 Physical robot structure (URDF)  
  - 5.1.1 Links (chassis, wheels, marker)  
  - 5.1.2 Joint types and axes  
  - 5.1.3 Inertia derivation  
5.2 Contact model and friction settings  
5.3 Physics engine configuration (timestep, solver)  
5.4 Coordinate frames and sign conventions  
5.5 Sensor model (tilt extraction from quaternion)  

**Key figure:** PyBullet screenshot — GUI with robot visible, forward marker visible  
**Key table:** URDF parameters table

---

## Chapter 6: Controller Design (4–5 pages)

**Target: 4.5 pages**

6.1 Control architecture overview  
  - 6.1.1 Common interface (compute, reset, torque split)  
  - 6.1.2 Controller rate and zero-order hold  

6.2 PID Controller  
  - 6.2.1 Control law and sign convention  
  - 6.2.2 Anti-windup (integral clamping)  
  - 6.2.3 Derivative on measurement  
  - 6.2.4 Tuning protocol and results  

6.3 LQR Controller  
  - 6.3.1 Optimal control formulation  
  - 6.3.2 Riccati equation solution  
  - 6.3.3 Closed-loop stability verification  
  - 6.3.4 Q/R selection and sweep results  

6.4 Comparison of approaches (theoretical)  

**Key table:** PID gains (from pid_tuning.csv), LQR gains K and Q/R  
**Key figure:** Block diagram of each controller

---

## Chapter 7: Experimental Methodology (1.5–2 pages)

**Target: 1.5 pages**

7.1 Experimental design overview (reference docs/methodology.md)  
7.2 Metric definitions (all 9 metrics — reference metrics.py)  
7.3 Fairness conditions for controller comparison  
7.4 Reproducibility measures  
7.5 Data collection and CSV schema  

---

## Chapter 8: Results and Discussion (5–6 pages)

**Target: 5.5 pages**

8.1 E01: Uncontrolled fall (fall time vs linear model prediction)  
8.2 E02: PID nominal balancing  
8.3 E03: LQR nominal balancing  
8.4 E04: PID vs LQR comparison  
  - 8.4.1 Metrics table (all 9 metrics for each controller)  
  - 8.4.2 Tilt vs time overlay plot  
  - 8.4.3 Torque vs time overlay  
  - 8.4.4 Position drift comparison  
8.5 E05: Disturbance response  
8.6 E06: Initial tilt sweep / region of attraction  
8.7 Discussion of failures (honest reporting)  

**All results from actual saved CSVs. All plots from results/plots/. No fabricated numbers.**

---

## Chapter 9: Novelty and Contribution (1.5–2 pages)

**Target: 1.5 pages**

9.1 Introduction (what constitutes novelty beyond textbook)  
9.2 Contribution 1: PID vs LQR comparison [→ Table E04, Fig 5]  
9.3 Contribution 2: Region-of-attraction map [→ Fig 7, Table E06]  
9.4 Contribution 3: Disturbance rejection analysis [→ Fig 6, Table E05]  
9.5 Contribution 4: Linear model vs PyBullet mismatch analysis [→ docs/mathematical_model.md §6]  
9.6 Contribution 5: Position drift quantification [→ Fig 8]  
9.7 Summary table of contributions  

---

## Chapter 10: Testing and Validation (1–1.5 pages)

**Target: 1 page**

10.1 Test suite overview (pytest, 47+ tests)  
10.2 Unit tests (config, dynamics, sensors, controllers, metrics)  
10.3 Integration test (balance for ≥20s)  
10.4 Test report summary (from docs/test_report.md)  
10.5 Determinism validation  

---

## Chapter 11: Limitations (1 page)

**Target: 1 page**

11.1 Simulation vs reality gap  
11.2 Linearisation validity range  
11.3 2D (pitch-plane) only — no yaw/steering  
11.4 PID drift (tilt-only regulation)  
11.5 Fixed-parameter controllers (no adaptation)  
11.6 Contact model approximations  
11.7 Motor model simplifications  

---

## Chapter 12: Conclusion and Future Work (1–1.5 pages)

**Target: 1 page**

12.1 Summary of achievements (against objectives from §1.3)  
12.2 Key findings (which controller is better and under what conditions)  
12.3 Future work (Kalman filter, steering, real hardware, adaptive control)  

---

## References (1 page)

≥8 IEEE-format references, all cited in text. See docs/literature_review.md.

---

## Appendices

| Appendix | Content | Approximate Pages |
|----------|---------|-------------------|
| A | Full robot_config.yaml and simulation_config.yaml | 1 |
| B | Key code excerpts (pid.py, lqr.py, dynamics.py, sensors.py) | 2–3 |
| C | Individual contribution records (from docs/contribution_template.md) | 1 |
| D | Peer assessment summary | 0.5 |
| E | URDF inspection output (docs/urdf_inspection.txt) | 1 |

---

## Page Count Summary

| Section | Target Pages |
|---------|-------------|
| Front matter | 7 |
| Chapters 1–12 | 27–35 |
| References | 1 |
| Appendices | 5–6 |
| **Total** | **~40–49** |

> NOTE: Adjust chapter lengths to stay within 25–35 pages for the body (Chapters 1–12 + References). Front matter and appendices are additional.

---

## Figure Numbering

| Figure # | Description | Source |
|----------|-------------|--------|
| Fig 1 | PyBullet GUI screenshot — robot balanced | demo/ |
| Fig 2 | Robot structure diagram (URDF links) | draw manually |
| Fig 3 | Block diagram: linearised state-space model | draw |
| Fig 4 | PID block diagram | draw |
| Fig 5 | PID vs LQR tilt overlay | results/plots/fig_05_pid_vs_lqr_overlay.png |
| Fig 6 | Disturbance recovery comparison | results/plots/fig_06_disturbance_recovery.png |
| Fig 7 | Initial tilt success map | results/plots/fig_07_tilt_sweep_map.png |
| Fig 8 | Position drift vs time | results/plots/fig_08_position_drift.png |
| Fig 9 | Torque vs time (PID + LQR) | results/plots/fig_04_torque_vs_time.png |
| Fig 10 | PID gain sweep (tuning) | results/plots/pid_tuning.png |
| Fig 11 | LQR Q/R sweep | results/plots/lqr_qr_sweep.png |

---

## Table Numbering

| Table # | Description | Source |
|---------|-------------|--------|
| Table 1 | Robot physical parameters | config/robot_config.yaml |
| Table 2 | Inertia values | scripts/generate_urdf_inertias.py |
| Table 3 | Physics engine settings | config/simulation_config.yaml |
| Table 4 | PID gains and tuning results | results/tables/pid_tuning.csv |
| Table 5 | LQR Q/R sweep | results/tables/lqr_qr_sweep.csv |
| Table 6 | Metrics comparison (E04) | results/tables/E04_comparison.csv |
| Table 7 | Disturbance metrics (E05) | results/tables/E05_disturbance.csv |
| Table 8 | Tilt sweep results (E06) | results/tables/E06_tilt_sweep.csv |
| Table 9 | Syllabus mapping | docs/literature_review.md |
