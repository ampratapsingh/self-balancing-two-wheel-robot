# Self-Balancing Two-Wheel Robot

A PyBullet simulation of a two-wheeled inverted-pendulum robot. The project
implements and compares a tilt-based PID controller with a full-state LQR
controller, then saves raw telemetry, metrics, tables, and report figures for
six repeatable experiments.

## Start here

The simulation is functional and verified: compilation succeeds, all 50 tests
pass, the public command-line interfaces load, report assets regenerate from
saved CSV evidence, and the PyBullet URDF inspection confirms the intended
2.5 kg total robot mass.

Before making changes, read these documents in order:

1. [`docs/team_handoff.md`](docs/team_handoff.md) — concise runbook for the next team member.
2. [`docs/context.md`](docs/context.md) — architecture, configurations, evidence, and decisions.
3. [`docs/progress_log.md`](docs/progress_log.md) — completed work and remaining faculty deliverables.

The faculty submission is P09, Group G01, Batch B1. The technical project is
complete; the remaining work is the faculty-formatted report, slides, team
video, and group-specific appendices.

## Requirements

- Python 3.10 or newer
- PyBullet 3.2.5 or newer
- NumPy, SciPy, pandas, Matplotlib, and PyYAML

Install in a virtual environment or Conda environment:

```bash
python -m pip install -e .
python -m pip install pytest pytest-timeout
```

On the original development machine, the verified interpreter is
`/Users/saiuppalapati/miniforge3/bin/python3` (Python 3.12.12).

## Install and verify

```bash
make PYTHON=/path/to/python test-all
```

This runs all 50 tests, including PyBullet integration tests. To run the
faster subset, use `make PYTHON=/path/to/python test` (45 passed and 5 slow
tests deselected).

## Run the simulator

```bash
# Headless 20-second LQR demonstration
make PYTHON=/path/to/python headless-lqr

# Headless PID demonstration
make PYTHON=/path/to/python headless-pid

# GUI demonstration (requires a desktop session)
make PYTHON=/path/to/python demo-lqr
```

The direct CLI supports an alternate directory of controller/configuration
YAML files:

```bash
python scripts/run_simulation.py --controller lqr --headless --config-dir config
```

## Use the saved evidence safely

```bash
# Regenerate tables and figures from the committed raw evidence. This is safe.
python scripts/generate_phase3_assets.py

# Run E01–E06 and overwrite raw data below results/data/.
# Do this only when intentionally producing a new, documented evidence set.
python scripts/run_experiments.py --all

# Run only the PID-versus-LQR comparison (E04)
python scripts/compare_controllers.py --duration 20 --seed 42

# Regenerate the report’s evidence tables and figures from raw CSVs
make PYTHON=/path/to/python plots
```

Experiment definitions are in `config/experiments/`; raw evidence is in
`results/data/`; generated tables are in `results/tables/`; and the report
draft is `report/report.md`.

Do not reuse the pre-correction claims that PID failed from 15° initial tilt.
The current 2.5 kg URDF evidence is authoritative: both controllers succeed
through the largest tested 30° initial tilt.

## Faculty deliverables still owned by the group

- A 25–35 page report rebuilt in the supplied faculty sample format.
- Contribution and writing-allocation tables based on actual team work.
- One 5–8 line co-learning reflection per member, meeting log, and peer sheets.
- A maximum-17-slide presentation PDF and an 18–22 minute unlisted team video.
- Verified bibliography entries and the final report PDF for submission.

The repository deliberately does not invent these team-specific details. See
the handoff guide for the exact files and completion checklist.

## Results at a glance

- Without control, the robot falls from a 2° tilt in about 0.542 s.
- Both controllers balance from the 3° nominal case for 30 s.
- PID has lower nominal tilt error and faster settling; LQR has lower final
  position displacement in the nominal and disturbance tests.
- Both controllers succeed at every tested initial tilt through 30°; their
  robustness limits have not yet been measured.

These findings are bounded by the simulated robot, chosen gains, and specified
test conditions. See `report/report.md` and the generated result tables for
the precise measurements and limitations.
