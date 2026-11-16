# Self-Balancing Two-Wheel Robot

A PyBullet simulation of a two-wheeled inverted-pendulum robot. The project
implements and compares a tilt-based PID controller with a full-state LQR
controller, then saves raw telemetry, metrics, tables, and report figures for
six repeatable experiments.

## Current status

The implementation is functionally complete and its full test suite covers
configuration, modelling, controllers, sensors, simulation, PyBullet physics,
robot torque control, analysis, and end-to-end 30-second balance runs.

The remaining coursework tasks are editorial: replace the report front matter
and verified bibliography placeholders, then format and inspect the final
DOCX/PDF. See `docs/context.md` for the complete handoff state.

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

## Verify the project

```bash
make PYTHON=/path/to/python test-all
```

This runs all 50 tests, including PyBullet integration tests. To run only
fast tests, use `make PYTHON=/path/to/python test`.

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

## Reproduce experiments and report assets

```bash
# Run E01–E06 and write raw data below results/data/
python scripts/run_experiments.py --all

# Run only the PID-versus-LQR comparison (E04)
python scripts/compare_controllers.py --duration 20 --seed 42

# Regenerate the report’s evidence tables and figures from saved raw CSVs
python scripts/generate_phase3_assets.py

# Regenerate generic time-series report figures from raw telemetry only
make PYTHON=/path/to/python plots
```

Experiment definitions are in `config/experiments/`; raw evidence is in
`results/data/`; generated tables are in `results/tables/`; and the report
draft is `report/report.md`.

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
