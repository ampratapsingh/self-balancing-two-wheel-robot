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


---

## Phase 2 — Physics Validation & All Experiments (2026-11-16) ✅

### Completed
- **pybullet installed** via `conda install -c conda-forge pybullet` (Oct 2025 build) ✅
- **Physics validations:**
  - V1: Chassis mass = 2.000 kg ✅
  - V2: Uncontrolled fall in 1.11s from 2° ✅
  - V3: Wheel velocities build under applied torque ✅
- **Sensor LPF bug fixed:** Alpha convention was inverted (`alpha*prev+(1-alpha)*raw` → fixed to `(1-alpha)*prev+alpha*raw`)
- **PID sign convention fixed:** `error = theta - theta_ref` (not `theta_ref - theta`); `d_term = +Kd * deriv`
- **PID gains tuned:** `Kp=130.0, Ki=0.3, Kd=8.0` via grid sweep (31/32 combos succeed)
- **LQR bug fixed:** `.item()` on 1D result to get scalar
- **47/47 fast tests PASS** (after fixing test_pid sign expectation)
- **All 6 experiments ran headlessly:**
  - E01: Uncontrolled fall at 1.11s
  - E02: PID balances for 30s — Success=True
  - E03: LQR balances for 30s — Success=True
  - E04: PID vs LQR at 2°,5°,10°,15° — LQR wins at 15°
  - E05: Both survive 10N disturbance impulse at t=5s
  - E06: PID max tilt = 12°; LQR max tilt = 30°
- **12 report figures generated** in `report/figures/` (tilt + torque for E01–E06)
- **Tagged v0.3.0**

### Key Commits
- `1ade2cf` — sensor LPF fix
- `b32a7f6` — PID sign + gains + test fix
- `8095e3a` — experiments E04/E05/E06 implemented + all results + figures

### Findings
- LQR significantly outperforms PID in terms of robustness (handles 30° vs 12° for PID)
- Both controllers handle the 10N disturbance impulse successfully
- Nonlinear effects limit PID performance but are well within the linearisation valid range for LQR

---

## Post-Phase 2 — Integrity & Provenance Maintenance (2026-11-16) ✅

### Completed
- Fixed an accidental leading whitespace character before the `sensors.py` module docstring. The character caused `IndentationError` during import and blocked the fast test suite at collection.
- Updated the PID and sensor documentation to reflect the implemented `error = theta - theta_ref` sign convention and `(1-alpha)*previous + alpha*raw` filter convention.
- Updated the linear-model control scalar conversion to use `.item()` for NumPy 2.x compatibility.
- Expanded metadata written for future simulation runs: Git commit, Python version, optional PyBullet version, timezone-aware save time, seed, and config snapshot.
- Added coverage that validates the new metadata fields.

### Validation
- `/Users/saiuppalapati/miniforge3/bin/python3 -m compileall -q src scripts` ✅
- `/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -m "not slow" -q` → **47 passed, 3 deselected** ✅
- `/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -v` → **50 passed** ✅

### Note
Existing Phase 2 CSVs and JSON sidecars were deliberately retained unchanged as historical evidence. JSON produced by new runs has the expanded provenance fields.

---

## Repository Publication (2026-11-16) ✅

### Completed
- Created the public repository https://github.com/saiuppalapati011/self-balancing-two-wheel-robot.
- Configured it as `origin` and pushed the complete local Git history and all tracked project files.
- Published commit `c96182c` (`fix: restore imports and add run provenance`) to `main`.

### Verification
- `main` tracks `origin/main`.
- Remote `main` resolved to `c96182c4684d03cb397689a6aaef94b98ef97c53` immediately after the push.

---

## Phase 3 — Report and Analysis Assets (2026-11-16) 🟡

### Completed
- Identified and fixed a failed-run logging edge case: the final fall row stored total torque as a per-wheel value, doubling reported peak torque for failed runs.
- Reran E04 and E06 after the fix. Failed PID rows now report the configured 10 Nm total torque limit correctly.
- Added `scripts/generate_phase3_assets.py`, which derives report tables and comparison figures from the saved CSV evidence.
- Generated `E01_uncontrolled_metrics.csv`, `E02_E03_nominal_comparison.csv`, `E04_comparison_table.csv`, `E05_disturbance_metrics.csv`, and `E06_tilt_sweep.csv` under `results/tables/`.
- Added `report/report.md`, an evidence-based report draft that cites the generated assets and explicitly flags remaining submission placeholders.
- Regenerated report figures, including PID/LQR position drift, E04 matched comparison, E05 disturbance response, and the E06 success map.
- Corrected the tilt-sweep plotting helper to sort numerical tilt values before joining points; this prevents lexicographic filename order from producing a misleading E06 map.

### Evidence-based findings
- PID succeeds through 12° initial tilt and fails at 15° and above; LQR succeeds at every tested tilt through 30°.
- Both controllers remain upright in the nominal 30 s run, but neither meets the strict ±0.5° settling definition for the full remaining duration.
- PID has lower nominal RMS tilt but large position drift; the selected LQR limits drift but has persistent tilt offset in PyBullet.
- Both remain upright after the 10 N disturbance, but their tilt and position trade-offs differ. See `results/tables/E05_disturbance_metrics.csv`.

### Validation
- `/Users/saiuppalapati/miniforge3/bin/python3 -m pytest tests/ -v` → **50 passed** ✅
- Regenerated table/figure assets from saved results and checked the expected five tables, 24 E06 records, and corrected 10 Nm failed-run peak torque ✅

### Remaining for formal submission
- Replace report front-matter and syllabus placeholders.
- Verify bibliography entries marked `[VERIFY]` and replace the placeholder recent-paper reference.
- Produce and visually inspect the final formatted DOCX/PDF against `report/format_checklist.md`.
