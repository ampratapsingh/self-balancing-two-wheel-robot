# Team Handoff Guide

## Project state

This repository contains the verified PyBullet implementation for P09,
Self-Balancing Two-Wheel Robot, Group G01, Batch B1. The code, experiment
evidence, plots, report draft, demo script, viva preparation, and faculty
format notes are committed on `main`.

The active evidence set uses the corrected 2.5 kg URDF. Earlier 3.5 kg output
was invalid because PyBullet assigned the visual marker an unintended 1 kg
mass. Do not reuse the old claim that PID fails at 15 degrees. The regenerated
results show that both controllers remain upright through the largest tested
30 degree initial tilt.

## First ten minutes

1. Clone the repository and check that `main` is current.
2. Use Python 3.10 or newer. On the original machine use:

   ```bash
   export PYTHON=/Users/saiuppalapati/miniforge3/bin/python3
   ```

3. Install the package and dependencies:

   ```bash
   $PYTHON -m pip install -e .
   $PYTHON -m pip install pytest pytest-timeout
   ```

4. Run the complete verification suite:

   ```bash
   make PYTHON=$PYTHON test-all
   ```

   Expected result: `50 passed`.

5. Regenerate report assets from the committed raw evidence:

   ```bash
   make PYTHON=$PYTHON plots
   ```

6. Confirm the working tree is clean before beginning a new task:

   ```bash
   git status --short --branch
   ```

## Safe commands and evidence policy

| Goal | Command | Effect |
|---|---|---|
| Validate all code | `make PYTHON=$PYTHON test-all` | Read-only verification except normal cache files |
| Regenerate report tables and figures | `make PYTHON=$PYTHON plots` | Rebuilds generated artifacts from existing CSV evidence |
| Run a short demonstration | `make PYTHON=$PYTHON headless-lqr` | Adds/overwrites only a manual run below `results/data/manual/` |
| Run all experiments | `make PYTHON=$PYTHON experiments` | Overwrites the official E01–E06 raw evidence |

Do not run all experiments merely to check that the project works. Use the
test suite for that. Rerun E01–E06 only after an intentional model/controller
change, then regenerate assets, update the report conclusions, record the
reason in `docs/progress_log.md`, and commit raw data, tables, and figures
together.

## Technical map

| Area | Primary files |
|---|---|
| Robot and physics | `assets/robot/balancing_robot.urdf`, `src/balancing_robot/robot.py`, `src/balancing_robot/physics.py` |
| Dynamics and controllers | `src/balancing_robot/dynamics.py`, `src/balancing_robot/controllers/` |
| Simulation loop and sensors | `src/balancing_robot/simulation.py`, `src/balancing_robot/sensors.py` |
| Experiments | `src/balancing_robot/experiments/`, `config/experiments/` |
| Metrics, I/O, and plots | `src/balancing_robot/analysis/`, `scripts/generate_phase3_assets.py` |
| Tests | `tests/` |
| Current report evidence | `results/data/`, `results/tables/`, `report/figures/`, `report/report.md` |

## Current verified facts

- The physical URDF mass is 2.5 kg. `scripts/inspect_urdf.py` confirms this.
- PID gains are Kp 130, Ki 0.3, and Kd 8.0.
- LQR uses Q = diag(1, 1, 100, 10), R = 0.1, and a 10 Nm total torque limit.
- All 50 tests pass with the verified Miniforge Python 3.12.12 environment.
- E01 falls from 2 degrees after about 0.542 s without control.
- Both controllers complete the 30 s nominal balancing run and each tested
  initial tilt through 30 degrees.
- PID has lower nominal tilt error and faster settling. LQR has lower final
  position displacement, but reaches the configured 10 Nm torque limit.

## Faculty submission work

The faculty instructions and sample report are the authority for formatting.
They require a 25–35 page A4 report with a border on every page, Times New
Roman 12 pt body text, 1.5 line spacing, front matter, figure/table lists,
IEEE references, and mandatory Novelty and Contribution sections.

Use `report/report.md` only as the technical-content source. Before submitting,
rebuild it in the supplied faculty DOCX format and complete these group-owned
items with real information:

- cover-page member details and signatures;
- writing allocation and technical-contribution table;
- each member's co-learning reflection;
- meeting log and peer-assessment sheets;
- verified recent reference entries;
- GitHub and unlisted YouTube video links;
- 17-slide maximum presentation PDF and 18–22 minute team recording.

## Before committing

1. Run `make PYTHON=$PYTHON test-all`.
2. Run `make PYTHON=$PYTHON plots` if code or raw experiment data changed.
3. Check `git diff --check` and review `git status`.
4. Update `docs/context.md`, `docs/progress_log.md`, and `CHANGELOG.md` for a
   material technical or evidence change.
5. Commit source, tests, raw data, tables, figures, and report claims together
   when they refer to the same experiment set.
