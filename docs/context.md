# PROJECT CONTEXT — Self-Balancing Two-Wheel Robot
> **Purpose:** This document is the single source of truth for any agent (or human)
> picking up this project mid-way. Read this before touching any code.
> Keep it updated whenever a phase completes or a major decision is made.
>
> **Last updated:** 2026-11-16 — Phase 2 complete; integrity/provenance maintenance and public GitHub publication complete; Phase 3 next.

---

## 1. Project Overview

| Field | Value |
|---|---|
| **Project** | Self-Balancing Two-Wheel Robot (Inverted Pendulum) |
| **Course** | University Robotics Course |
| **Repo root** | `/Users/saiuppalapati/Downloads/Robotics Project` |
| **GitHub repository** | https://github.com/saiuppalapati011/self-balancing-two-wheel-robot (public; `origin/main`) |
| **Package name** | `balancing_robot` |
| **Package location** | `src/balancing_robot/` |
| **Python** | 3.10+ required; use `/Users/saiuppalapati/miniforge3/bin/python3` (3.12.12) on this machine |
| **Simulator** | PyBullet |
| **Controllers** | PID and LQR (inverted pendulum) |

### Goal
Simulate a two-wheeled self-balancing robot in PyBullet, implement PID and LQR controllers,
run 6 experiments (E01–E06), generate plots, and produce a university report.

---

## 2. Environment Setup

```bash
# Primary interpreter on this machine:
/Users/saiuppalapati/miniforge3/bin/python3   # Python 3.12.12

# Installed packages (miniforge3 env):
# numpy, scipy, matplotlib, pandas, pyyaml, pytest — ALL INSTALLED ✅
# pybullet — NOT YET INSTALLED (needs C compiler or pre-built wheel)

# Install the package (editable, no deps):
cd "/Users/saiuppalapati/Downloads/Robotics Project"
/Users/saiuppalapati/miniforge3/bin/pip install -e . --no-deps

# Install pybullet (try conda first):
/Users/saiuppalapati/miniforge3/bin/conda install -c conda-forge pybullet
# OR:
/Users/saiuppalapati/miniforge3/bin/pip install pybullet

# Run fast tests (47 pass, 3 deselected as @slow):
/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -m "not slow" -v

# Run all tests including integration (50 pass; needs pybullet):
/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -v
```

> **Note:** System Python (`python3`) is Homebrew 3.14 — blocked by PEP 668. Always use miniforge3.

---

## 3. Repository Structure

```
Robotics Project/
├── .github/workflows/ci.yml        # CI: ubuntu-latest, Python 3.10 & 3.11
├── .gitignore
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE                          # MIT, placeholders for student names
├── Makefile                         # make setup / test / demo-pid / demo-lqr / clean
├── pyproject.toml                   # build-backend = setuptools.build_meta
├── requirements.txt
│
├── assets/
│   └── robot/
│       ├── balancing_robot.urdf     # ✅ Full URDF (Phase 1, @urdf)
│       └── README.md                # Coordinate frames, tilt convention, geometry
│
├── config/
│   ├── robot_config.yaml            # wheel r=0.08, chassis mass=2.0, etc.
│   ├── simulation_config.yaml       # dt=1/240, gravity=-9.81, fall_threshold=45°
│   ├── pid_config.yaml              # Kp=35, Ki=0.5, Kd=2.5
│   ├── lqr_config.yaml              # Q diagonal, R=0.1
│   └── experiments/
│       ├── e01_uncontrolled.yaml    # 5s free fall from 2°
│       ├── e02_pid_balancing.yaml   # 30s PID from 3°
│       ├── e03_lqr_balancing.yaml   # 30s LQR from 3°
│       ├── e04_comparison.yaml      # PID vs LQR at 2/5/10/15°
│       ├── e05_disturbance.yaml     # 10N force impulse at t=5s
│       └── e06_tilt_sweep.yaml      # tilts 1–30°, both controllers
│
├── docs/
│   ├── context.md                   # ← THIS FILE — keep updated
│   ├── decision_log.md              # Key decisions D01–D07+
│   ├── mathematical_model.md        # ✅ Full Lagrangian derivation (Phase 1)
│   ├── methodology.md               # ✅ Written (Phase 1)
│   ├── progress_log.md              # Phase-by-phase progress record
│   └── [other docs created by agents]
│
├── results/                         # Experiment outputs (CSV, plots, tables)
│   ├── data/
│   ├── plots/
│   └── tables/
│
├── report/
│   ├── figures/                     # Final report figures
│   └── tables/
│
├── scripts/
│   ├── run_simulation.py            # ✅ Full CLI
│   ├── run_experiments.py           # ✅ E01-E06 runner
│   ├── compare_controllers.py       # ✅ E04 CLI
│   ├── make_report_assets.py        # ✅ CSV → report/figures/
│   ├── generate_urdf_inertias.py    # ✅ Inertia calculator
│   ├── inspect_urdf.py              # ✅ PyBullet URDF inspector
│   ├── tune_pid.py                  # STUB (Phase 2)
│   └── sweep_lqr_qr.py             # STUB (Phase 2)
│
├── src/balancing_robot/
│   ├── __init__.py                  # ✅ version = "0.1.0"
│   ├── config.py                    # ✅ YAML loader
│   ├── dynamics.py                  # ✅ COMPLETE — Lagrangian A/B matrices, eigenvalues, controllability
│   ├── simulation.py                # ✅ COMPLETE — main sim loop, state logging, GUI
│   ├── robot.py                     # ✅ COMPLETE — PyBullet body, motor, torque control
│   ├── physics.py                   # ✅ COMPLETE — PhysicsEngine, PyBullet client
│   ├── sensors.py                   # ✅ COMPLETE — tilt/rate from quaternion, noise, delay, LPF
│   ├── controllers/
│   │   ├── __init__.py              # ✅ re-exports Base, PID, LQR
│   │   ├── base.py                  # ✅ Abstract BaseController
│   │   ├── pid.py                   # ✅ COMPLETE — PID + anti-windup + derivative filter
│   │   └── lqr.py                   # ✅ COMPLETE — CARE solver, saturation (bug fixed: .item())
│   ├── experiments/
│   │   ├── __init__.py              # ✅
│   │   ├── baseline.py              # ✅ E01/E02/E03 runners
│   │   ├── comparison.py            # ✅ E04 stub/runner
│   │   ├── disturbance.py           # ✅ E05 stub/runner
│   │   └── sweeps.py                # ✅ E06 stub/runner
│   └── analysis/
│       ├── __init__.py              # ✅
│       ├── metrics.py               # ✅ settling time, RMS tilt, overshoot, SS error
│       ├── io.py                    # ✅ save_run, load_run, load_metadata
│       └── plots.py                 # ✅ tilt/torque/position/sweep plots
│
└── tests/
    ├── conftest.py                  # ✅ fixtures
    ├── test_config.py               # ✅ 4 PASS
    ├── test_dynamics.py             # ✅ 9 PASS
    ├── test_pid.py                  # ✅ 7 PASS
    ├── test_lqr.py                  # ✅ 7 PASS
    ├── test_sensors.py              # ✅ 7 PASS
    ├── test_metrics.py              # ✅ 5 PASS
    ├── test_io_and_plots.py         # ✅ 4 PASS
    ├── test_robot.py                # ✅ 2 PASS (placeholder)
    ├── test_physics.py              # ✅ 2 PASS (placeholder)
    └── test_integration_balance.py  # 🔲 @slow — needs pybullet installed
```

---

## 4. Key Design Decisions

| ID | Decision | Detail |
|---|---|---|
| D01 | Build backend | `setuptools.build_meta` (not `legacy:build`) |
| D02 | Package layout | `src/` layout |
| D03 | Python interpreter | Use miniforge3 Python 3.12; system Python 3.14 blocked |
| D04 | Tilt convention | θ > 0 = forward lean (+X direction) |
| D05 | Controller interface | `compute(state, dt) → float` total torque; sim splits L/R |
| D06 | State vector | `[x, x_dot, theta, theta_dot]` — 4-state |
| D07 | Timestep | 240 Hz (dt ≈ 0.004167 s) |
| D08 | LQR bug fix | `(-K @ x).item()` — K is (1,4), result is (1,) not scalar |
| D09 | LQR solver | Continuous ARE (`scipy.linalg.solve_continuous_are`) |
| D10 | PID derivative | Derivative-on-measurement (not on error) to avoid kick on setpoint change |
| D11 | Run provenance | New JSON sidecars record runtime provenance and configuration snapshots |
| D12 | Repository publication | Public GitHub repository with `main` tracking `origin/main` |

---

## 5. Physics & Robot Geometry

| Parameter | Value |
|---|---|
| Wheel radius | 0.08 m |
| Wheel mass | 0.25 kg each |
| Chassis mass | 2.0 kg |
| Total mass | 2.5 kg |
| CoM height above axle (l) | 0.225 m |
| Chassis Iyy (pitch) | 0.009067 kg·m² |
| Wheel Iyy (spin) | 0.000800 kg·m² |
| Pendulum inertia I_pend | 0.1103 kg·m² |
| Unstable eigenvalue | λ ≈ ±10.97 rad/s (toppling τ ≈ 91 ms) |
| System controllable | ✅ rank 4/4 |
| Max torque per wheel | 5.0 Nm (10 Nm total) |

### Linearised System (at θ=0)
```
A = [[0, 1,       0,    0],
     [0, 0, -19.694,    0],
     [0, 0,       0,    1],
     [0, 0, 120.351,    0]]

B = [[0], [18.132], [0], [-83.027]]
```

---

## 6. Experiments

| ID | Name | Controller | Duration | Key Variable |
|---|---|---|---|---|
| E01 | Uncontrolled Fall | none | 5 s | Records fall dynamics |
| E02 | PID Balancing | pid | 30 s | Baseline PID |
| E03 | LQR Balancing | lqr | 30 s | Baseline LQR |
| E04 | PID vs LQR Comparison | both | 20 s | Tilts: 2°, 5°, 10°, 15° |
| E05 | Disturbance Response | both | 20 s | 10 N impulse at t=5 s, h=0.3 m |
| E06 | Tilt Sweep | both | 15 s | Tilts: 1–30° (12 values) |

---

## 7. Controller Configs

### PID (`config/pid_config.yaml`)
```yaml
Kp: 35.0  Ki: 0.5  Kd: 2.5
integral_clamp: 10.0
derivative_filter_alpha: 0.7
max_torque_total: 10.0
theta_ref: 0.0
```

### LQR (`config/lqr_config.yaml`)
```yaml
Q: {q_x: 1.0, q_xdot: 1.0, q_theta: 100.0, q_thetadot: 10.0}
R: 0.1
max_torque_total: 10.0
```

---

## 8. Phase Status

### ✅ Phase 0 — Scaffold (COMPLETE) — tag: v0.1.0
- All config files, package skeleton, CI, docs, scripts
- `tests/test_config.py`: 4/4 PASS
- Commits: `03e4df3`, `60e0dbb`, `c5c5832`, `56ab5d4`

### ✅ Phase 1 — Core Implementation (COMPLETE) — tag: v0.2.0
- `dynamics.py`: Lagrangian model, A/B matrices, eigenvalues, controllability ✅
- `sensors.py`: Tilt extraction from quaternion, noise/delay/LPF ✅ (LPF alpha convention fixed)
- `controllers/pid.py`: PID + anti-windup + derivative filter ✅ (sign convention fixed: `error=theta-ref`, `d_term=+Kd*deriv`)
- `controllers/lqr.py`: CARE solver + saturation (shape bug fixed: `.item()`) ✅
- `simulation.py`, `robot.py`, `physics.py`: Full PyBullet simulation loop ✅
- `analysis/metrics.py`, `io.py`, `plots.py`: Metrics + I/O + plotting ✅
- `experiments/baseline.py`, `comparison.py`, `disturbance.py`, `sweeps.py` ✅
- `docs/mathematical_model.md`: Full Lagrangian derivation ✅
- **Test results: 47/47 PASS** (fast tests, 3 @slow deselected)
- Commit: `17c8c68`

### ✅ Phase 2 — Integration & Experiments (COMPLETE) — tag: v0.3.0

**pybullet:** Installed via `conda install -c conda-forge pybullet` ✅

**PID gains (tuned):** `Kp=130.0, Ki=0.3, Kd=8.0` — in `config/pid_config.yaml`

**All 6 experiments ran headlessly and produced CSVs:**

| Exp | Name | Result |
|---|---|---|
| E01 | Uncontrolled fall | Fell at **1.11s** ✅ |
| E02 | PID balancing 30s | **Success=True** ✅ |
| E03 | LQR balancing 30s | **Success=True** ✅ |
| E04 | PID vs LQR comparison (2°,5°,10°,15°) | PID fails at 15°; LQR succeeds at all ✅ |
| E05 | Disturbance response (10N at t=5s) | Both PID and LQR survive ✅ |
| E06 | Tilt sweep (1°–30°) | PID max: **12°**; LQR max: **30°** ✅ |

**E06 key finding:** PID basin of attraction ≈12°; LQR robustly handles up to 30° (nonlinearity limit not hit).

**Report figures:** 12 PNG plots generated in `report/figures/` (tilt + torque for each E01–E06) ✅

**Commits:** `b32a7f6` (PID fix), `8095e3a` (all experiments + figures). **Tag: v0.3.0**

### ✅ Post-Phase 2 — Integrity & Provenance Maintenance (2026-11-16)
- Restored importability after an accidental leading space at the start of `sensors.py` caused an `IndentationError` during test collection.
- Corrected stale PID and sensor filter-convention documentation to match the implemented sign and low-pass-filter conventions.
- Made the linear-model simulation extract its 1-element control result with `.item()`, avoiding a NumPy 2.x scalar-conversion deprecation.
- Newly generated JSON metadata now records the Git commit, Python version, optional PyBullet version, timezone-aware timestamp, seed, and a robot/simulation/controller/disturbance config snapshot.
- **Validation:** `python3 -m pytest tests/ -v` → **50/50 PASS** using miniforge3 Python 3.12.12.
- **Local commit:** `c96182c` — `fix: restore imports and add run provenance`.

### ✅ Repository Publication (2026-11-16)
- Created the public GitHub repository: https://github.com/saiuppalapati011/self-balancing-two-wheel-robot
- Added it as `origin` and pushed the complete existing Git history, all tracked source files, experiment data, report figures, and documentation.
- Verified that `main` tracks `origin/main` at commit `c96182c4684d03cb397689a6aaef94b98ef97c53` and that the local working tree was clean after the push.

### 🔲 Phase 3 — Report & Analysis Write-up (NEXT)
- Write `report/report.md` (or LaTeX) with methods, results, figures
- Summary tables comparing PID vs LQR metrics (RMS tilt, max torque, settling time)
- Run `scripts/tune_pid.py` for more systematic PID tuning (optional)
- Run `scripts/sweep_lqr_qr.py` for LQR Q/R sweep (optional)
- Slow integration tests: `python3 -m pytest tests/test_integration_balance.py -v`
- Update `CHANGELOG.md` with Phase 2/3 entries

---

## 9. How to Continue (New Agent Checklist)

1. **Read this file** (`docs/context.md`) in full.
2. Run `git log --oneline` and `git status --short --branch` to see latest commits, tags, and upstream status.
3. Run `/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -m "not slow" -v` — should be **47 passed**.
4. All 6 experiments already ran — results in `results/data/E01/` through `results/data/E06/`
5. Report figures already in `report/figures/` (12 PNGs)
6. **Next: Phase 3** — write report, compute summary metrics, fill `report/tables/`

---

## 10. Known Issues / Watch-outs

| Issue | Detail | Status |
|---|---|---|
| PID basin of attraction | Fails ≥15° initial tilt with Kp=130 | Expected — nonlinear regime |
| LQR position drift | Robot slowly rolls away (q_x=1 helps but doesn't fully stop) | Acceptable for balancing task |
| Historical JSON provenance | Phase 2 JSON files were saved before provenance expansion and lack runtime/config snapshots | Preserve as historical evidence; rerun an experiment to create an expanded JSON record |
| System Python | Use miniforge3 only, not `python3` | Known |
| front_marker_link inertia | PyBullet warns about no inertial data for this link | Harmless |

---

*Last updated: 2026-11-16 — Phase 2 results retained; import, documentation, provenance, and public GitHub publication verified. Phase 3 is next.*
