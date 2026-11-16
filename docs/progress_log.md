# Progress Log

## Phase 0 — Repository Scaffold (2026-11-16) ✅

### Completed
- Initialized git repository with `git init`; initial commit `03e4df3`
- Created all project configuration files:
  - `pyproject.toml` (setuptools build, Python ≥3.10)
  - `requirements.txt`, `Makefile`, `.gitignore`
- Created CI workflow: `.github/workflows/ci.yml` (ubuntu-latest, Python 3.10 & 3.11)
- Created full YAML config suite:
  - `config/robot_config.yaml`, `simulation_config.yaml`, `pid_config.yaml`, `lqr_config.yaml`
  - `config/experiments/e01_uncontrolled.yaml` through `e06_tilt_sweep.yaml`
- Created package skeleton under `src/balancing_robot/`:
  - `__init__.py`, `config.py` (YAML loader with repo-relative path resolution)
  - `controllers/base.py` (abstract `BaseController`)
  - `controllers/__init__.py` (re-exports PID, LQR, Base)
  - Stub modules: `simulation.py`, `robot.py`, `physics.py`, `sensors.py`, `dynamics.py`
  - `experiments/__init__.py`, `analysis/__init__.py`
- Created `tests/conftest.py` with shared fixtures (`repo_root`, `robot_config`, `sim_config`)
- Created `tests/test_config.py` — 4 config-loading tests, all pass ✅
- Created `assets/robot/README.md` (coordinate frames, tilt convention, geometry)
- Tagged release `v0.1.0`

### Note
`pyproject.toml` backend changed from `setuptools.backends.legacy:build` to `setuptools.build_meta`
for compatibility with the installed pip/setuptools version on this machine (setuptools 80.9.0).
Both are equivalent; the former is an alias for the latter introduced in setuptools 68.

---

## Phase 1 — Core Implementation (2026-11-16) ✅

### Completed
- **dynamics.py** — Lagrangian EOM → linearised A/B matrices, eigenvalues (λ≈±10.97), controllability rank=4
- **sensors.py** — Tilt from quaternion (atan2 method), tilt rate, optional noise/delay/LPF
- **controllers/pid.py** — PID with derivative-on-measurement, anti-windup integral clamp, IIR filter
- **controllers/lqr.py** — Continuous CARE solver, stability check, saturation. Bug fixed: `.item()` on (1,4)@(4,) result
- **simulation.py** — Full PyBullet simulation loop with state logging, GUI camera tracking, fall detection
- **robot.py** — Robot URDF loader, motor management, torque control
- **physics.py** — PhysicsEngine wrapping PyBullet client
- **analysis/metrics.py** — settling time, RMS tilt, overshoot, steady-state error, torque saturation fraction
- **analysis/io.py** — save_run (CSV+JSON), load_run, load_metadata
- **analysis/plots.py** — tilt vs time, torque vs time, position drift, tilt sweep heatmap
- **experiments/baseline.py** — E01 (uncontrolled), E02 (PID), E03 (LQR)
- **experiments/comparison.py**, **disturbance.py**, **sweeps.py** — E04/E05/E06 runners
- **docs/mathematical_model.md** — Full 450+ line Lagrangian derivation
- **Test results: 47/47 PASS** (fast), 3 @slow deselected (need pybullet)
- Tagged **v0.2.0**

### Bug Fixed
- `lqr.py` line 108: `float(-self._K @ x_vec)` → `float((-self._K @ x_vec).item())`
  K is (1,4), result of K@x is (1,) not scalar — `float()` on ndarray fails in NumPy

---

## Phase 1 — Controllers, Dynamics, Simulation Core (In Progress — SUPERSEDED)

Agents `@dynamics`, `@controllers`, `@simcore` are building:
- `dynamics.py` + mathematical model docs
- Full `pid.py`, `lqr.py`, `sensors.py`
- `simulation.py`, `robot.py`, `physics.py`, analysis modules, experiment runners

Also added (scaffold agent):
- `scripts/run_simulation.py` — full CLI entry point
- `scripts/run_experiments.py` — E01-E06 runner
- `scripts/compare_controllers.py` — E04 comparison CLI
- `scripts/make_report_assets.py` — plot regeneration from results/ CSVs
- `scripts/tune_pid.py`, `scripts/sweep_lqr_qr.py` — stubs for Phase 2
- `docs/progress_log.md`, `docs/decision_log.md`
- `tests/test_robot.py`, `tests/test_physics.py` — placeholder stubs
- `tests/test_integration_balance.py` — @pytest.mark.slow integration tests
