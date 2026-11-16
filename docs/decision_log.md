# Decision Log

Records key architectural and implementation decisions made during the project.

---

## 2026-11-16 — Phase 0

### D01: Build backend: `setuptools.build_meta` (not `setuptools.backends.legacy:build`)
**Decision:** Use `setuptools.build_meta` as the pyproject.toml build backend.  
**Rationale:** `setuptools.backends.legacy:build` is a valid alias introduced in setuptools 68, but
the vendored `pyproject_hooks` inside the installed pip could not resolve it. `setuptools.build_meta`
is universally supported and functionally identical.  
**Impact:** No runtime difference. CI matrix (ubuntu-latest) uses latest pip, so either works there.

### D02: Package layout — `src/` layout
**Decision:** Place the `balancing_robot` package under `src/balancing_robot/`.  
**Rationale:** Prevents accidental imports of the uninstalled package during development; forces
`pip install -e .` before testing. Follows modern Python packaging best practice.

### D03: Python interpreter — miniforge3 Python 3.12
**Decision:** Use `/Users/saiuppalapati/miniforge3/bin/python3` (Python 3.12.12) as the
primary development interpreter on this machine.  
**Rationale:** System Python 3.14 (Homebrew) lacks `setuptools` and blocks `pip install` via PEP 668.
Miniforge3 has a complete conda environment including `numpy`, `scipy`, `matplotlib`, `pandas`,
`pyyaml`, `pytest`.  
**Impact:** Makefile `PYTHON := python3` will resolve to system Python; on this machine, use
`/Users/saiuppalapati/miniforge3/bin/python3` or activate the conda env first.

### D04: Tilt angle convention — θ > 0 = forward lean
**Decision:** Positive θ means the robot leans forward (+X direction).  
**Rationale:** Consistent with the standard inverted pendulum formulation where positive tilt
produces positive forward acceleration under a PD/LQR controller.  
**Reference:** `assets/robot/README.md`

### D05: Controller interface — single `compute(state, dt) → float`
**Decision:** All controllers implement `BaseController.compute(state, dt) → float` returning
*total* torque (Nm), which the simulation splits equally to left and right wheels.  
**Rationale:** Keeps controllers decoupled from hardware/URDF specifics; differential steering
not needed for balancing-only control in Phase 1.

### D06: State vector — `[x, x_dot, theta, theta_dot]`
**Decision:** Four-state inverted pendulum model.  
**Rationale:** Minimal state for balancing; x and x_dot enable position regulation in later phases.
Yaw is ignored in Phase 1 (straight-line motion assumed).

### D07: Timestep — 240 Hz (dt = 1/240 ≈ 0.004167 s)
**Decision:** Physics and controller run at 240 Hz.  
**Rationale:** Matches PyBullet's default and is fast enough for the dynamics without excessive
CPU cost. Settable in `simulation_config.yaml`.

### D11: Persist reproducibility provenance with each newly generated run
**Decision:** Add the Git commit, Python version, optional PyBullet version, timezone-aware save
time, seed, and a configuration snapshot to every newly saved JSON sidecar.
**Rationale:** The methodology requires enough provenance to identify the exact code and
configuration that produced a run. The original Phase 2 JSONs are retained unchanged as the
historical evidence set; rerunning an experiment creates a record with the expanded fields.
**Impact:** CSV schemas and previously generated experiment files are unchanged.
