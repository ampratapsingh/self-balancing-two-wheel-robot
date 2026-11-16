# Contributing

## Branch conventions
- `main`: always runnable, tests-green
- `feature/<name>`: new features
- `experiment/<name>`: experiment results
- `docs/<name>`: documentation

## Commit conventions (Conventional Commits)
- `feat:` new features
- `fix:` bug fixes
- `test:` tests
- `docs:` documentation
- `refactor:` refactoring
- `chore:` maintenance
- `exp:` experiment results

## Workflow
1. Branch from main
2. Develop and test locally
3. Run make test before pushing
4. Push branch; merge after tests pass

## Code style
- Python 3.10+, type hints on public functions
- Docstrings on public functions/classes
- 100 character line length
- snake_case for files/modules/functions/variables
