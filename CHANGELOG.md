# Changelog

All notable changes to this project are documented here.

## [Unreleased]

### Fixed
- Restored `sensors.py` importability after an indentation error blocked test collection.
- Corrected stale PID/sensor convention documentation and NumPy scalar conversion in the linear-model simulator.
- Corrected failed-run torque logging and regenerated E04/E06 evidence so peak total torque is not doubled in the final fall row.

### Added
- Provenance and configuration snapshots to JSON metadata written by newly generated runs.
- Public project repository: https://github.com/saiuppalapati011/self-balancing-two-wheel-robot
- Reproducible Phase 3 table/figure generator and an evidence-based Markdown report draft.

## [0.1.0] - 2026-11-16
### Added
- Initial repository scaffold
- Project configuration files
- CI workflow skeleton
- Package structure (src/balancing_robot/)
