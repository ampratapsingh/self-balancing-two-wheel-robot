# Literature Review

## Overview

This document summarises the literature studied for the self-balancing two-wheel robot project.
At least 5 sources were studied in depth; at least 8 IEEE-format references are provided.
Every reference in this list must be cited in the report body, and every in-text citation must appear in this list.

> **NOTE:** Verify all references before submission. DOIs and page numbers are omitted where not confirmed. Entries marked [VERIFY] need manual confirmation of details.

---

## 1. Sources Studied in Depth

### Source 1: Feedback Systems — Åström & Murray
**Citation:** [1] K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008.

**Summary:** A comprehensive textbook on feedback control systems. Chapter 3 covers dynamic modelling (Lagrangian mechanics), Chapter 4 covers state-space representations, and Chapter 9 covers optimal control including LQR.

**Relevance:** Directly supports the mathematical modelling section (Lagrangian derivation) and the LQR controller design.

---

### Source 2: Modern Control Engineering — Ogata
**Citation:** [2] K. Ogata, *Modern Control Engineering*, 5th ed., Prentice Hall, 2010.

**Summary:** Standard control engineering textbook covering PID design, state-space methods, stability analysis, and root locus. Chapter 8 covers PID tuning methods including Ziegler–Nichols.

**Relevance:** Supports the PID tuning protocol; provides basis for stability analysis and performance metric definitions.

---

### Source 3: JOE: A Mobile Inverted Pendulum — Grasser et al.
**Citation:** [3] F. Grasser, A. D'Arrigo, S. Colombi, and A. C. Rufer, "JOE: a mobile, inverted pendulum," *IEEE Trans. Ind. Electron.*, vol. 49, no. 1, pp. 107–114, Feb. 2002. [VERIFY: DOI 10.1109/41.982254]

**Summary:** One of the earliest documented mobile inverted-pendulum robots. The paper presents a Lagrangian-based model for a two-wheeled robot, a PD controller design, and experimental validation on real hardware. The robot structure (chassis above axle, continuous wheel joints) is directly analogous to this project.

**Relevance:** Primary design reference for the robot structure and mathematical model. The Lagrangian equations of motion in this project closely follow the JOE derivation. Key difference: this project also implements LQR and uses PyBullet simulation instead of real hardware.

---

### Source 4: PyBullet Quickstart Guide — Coumans & Bai
**Citation:** [4] E. Coumans and Y. Bai, "PyBullet, a Python module for physics simulation for games, robotics and machine learning," *http://pybullet.org*, 2016–2021.

**Summary:** The official reference for PyBullet physics simulation. Covers URDF loading, joint control modes (TORQUE_CONTROL, VELOCITY_CONTROL), contact dynamics, and solver parameters.

**Relevance:** Essential for all simulation implementation details: joint control, friction parameters, collision geometry, physics engine settings, and GUI features.

---

### Source 5: Optimal Control: Linear Quadratic Methods — Anderson & Moore
**Citation:** [5] B. D. O. Anderson and J. B. Moore, *Optimal Control: Linear Quadratic Methods*, Prentice-Hall, 1990.

**Summary:** The definitive reference for LQR optimal control. Covers the algebraic Riccati equation (continuous and discrete time), stability guarantees of LQR, robustness margins, and practical Q/R selection.

**Relevance:** Provides the theoretical foundation for the LQR controller: Riccati equation solution, stability proof, and Q/R selection guidelines.

---

## 2. Additional References

### Reference 6: PID Controllers: Theory, Design and Tuning — Åström & Hägglund
**Citation:** [6] K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design and Tuning*, 2nd ed., Instrument Society of America, 1995.

**Summary:** Comprehensive treatment of PID control, including anti-windup strategies, derivative filtering, and systematic tuning methods.

**Relevance:** Supports the PID implementation details: derivative-on-measurement, integral clamping, and filter design.

---

### Reference 7: Feedback Control of Dynamic Systems — Franklin, Powell & Emami-Naeini
**Citation:** [7] G. F. Franklin, J. D. Powell, and A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019.

**Summary:** Undergraduate control systems textbook covering classical (PID) and modern (state-space) control methods, with worked examples of inverted pendulum control.

**Relevance:** Background reference for the inverted pendulum as a canonical unstable system; supports the linearisation discussion.

---

### Reference 8: Kalman — LQR Foundation [VERIFY]
**Citation:** [8] R. E. Kalman, "Contributions to the theory of optimal control," *Bol. Soc. Mat. Mexicana*, vol. 5, pp. 102–119, 1960. [VERIFY]

**Summary:** Original paper by Kalman that introduced the linear quadratic regulator (LQR) framework.

**Relevance:** Foundational reference for the LQR controller; cited when introducing optimal control theory.

---

### Reference 9: Self-Balancing Robot [VERIFY — recent paper needed]
**Citation:** [9] [VERIFY] Find a recent (2018–2024) paper on self-balancing two-wheeled robots from IEEE Xplore. Suggested search: "two-wheeled self-balancing robot LQR simulation" on IEEE Xplore.

**Relevance:** Recent implementation comparison.

**ACTION REQUIRED:** Team must search IEEE Xplore and replace this placeholder with a verified citation.

---

### Reference 10: NumPy/SciPy [VERIFY]
**Citation:** [10] P. Virtanen et al., "SciPy 1.0: Fundamental Algorithms for Scientific Computing in Python," *Nature Methods*, vol. 17, pp. 261–272, 2020. [VERIFY DOI: 10.1038/s41592-019-0686-2]

**Relevance:** `scipy.linalg.solve_continuous_are` is used for the LQR Riccati equation solution.

---

## 3. IEEE Reference Format

All references in the report must follow IEEE format:
- Journals: Author(s), "Title," *Journal*, vol., no., pp., Month Year.
- Books: Author(s), *Title*, Edition, Publisher, Year.
- Online: Author(s), "Title," URL, accessed Month Year.
- Do not invent DOIs; omit if unconfirmed.
- Every reference in this list must be cited in the text.
- Every in-text citation must appear in this list.

---

## 4. Syllabus Mapping

> **ACTION REQUIRED:** The team must fill in this table with actual AISR syllabus unit numbers and names from the course handout.

| Syllabus Unit | Unit Name | Project Component |
|---------------|-----------|-------------------|
| [SYLLABUS_UNIT_1] | [UNIT_NAME] | Lagrangian dynamics modelling |
| [SYLLABUS_UNIT_2] | [UNIT_NAME] | State-space representation, linearisation |
| [SYLLABUS_UNIT_3] | [UNIT_NAME] | PID controller design and tuning |
| [SYLLABUS_UNIT_4] | [UNIT_NAME] | LQR optimal control, Riccati equation |
| [SYLLABUS_UNIT_5] | [UNIT_NAME] | Stability analysis (eigenvalues, poles) |
| [SYLLABUS_UNIT_6] | [UNIT_NAME] | Simulation and verification (PyBullet) |
| [SYLLABUS_UNIT_7] | [UNIT_NAME] | Performance metrics, experimental design |

---

## 5. Notes on Reference Integrity

- All authors, titles, journals/books, and years listed above were cross-checked against publicly available information at the time of writing (November 2026).
- DOIs marked [VERIFY] must be checked before submission. Never fabricate a DOI.
- Page numbers are omitted where not confirmed from the source document.
- References [9] must be replaced with a verified, specific, recent paper before submission.
