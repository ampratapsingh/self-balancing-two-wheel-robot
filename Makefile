.PHONY: setup test experiments plots demo-pid demo-lqr lint clean

PYTHON := python3
SCRIPTS := scripts

setup:
	$(PYTHON) -m pip install -e .
	$(PYTHON) -m pip install pytest pytest-timeout

test:
	$(PYTHON) -m pytest tests/ -m "not slow" -v

test-all:
	$(PYTHON) -m pytest tests/ -v

experiments:
	$(PYTHON) $(SCRIPTS)/run_experiments.py --all

plots:
	$(PYTHON) $(SCRIPTS)/make_report_assets.py

demo-pid:
	$(PYTHON) $(SCRIPTS)/run_simulation.py --controller pid --gui

demo-lqr:
	$(PYTHON) $(SCRIPTS)/run_simulation.py --controller lqr --gui

headless-pid:
	$(PYTHON) $(SCRIPTS)/run_simulation.py --controller pid --headless --duration 20

headless-lqr:
	$(PYTHON) $(SCRIPTS)/run_simulation.py --controller lqr --headless --duration 20

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null; true
	find . -name "*.pyc" -delete 2>/dev/null; true
