# Demo Script: Live Simulation Demonstration

**Duration:** Approximately 5 minutes  
**Audience:** Examiners and classmates  
**Presenter:** [STUDENT_NAME] (designated demo presenter)

---

## Interpreter

Set `PYTHON` to the interpreter that has the project dependencies installed. On the verified
development machine this is `/Users/saiuppalapati/miniforge3/bin/python3`:

```bash
export PYTHON=/Users/saiuppalapati/miniforge3/bin/python3
```

## Pre-Demo Checklist (do this 10 minutes before)

- [ ] Python environment active: `source venv/bin/activate`
- [ ] All dependencies installed: `pip install -e .`
- [ ] GUI mode works: test with `python scripts/run_simulation.py --controller lqr --gui --duration 5`
- [ ] Screen resolution suitable: 1920×1080 preferred
- [ ] PyBullet GUI visible (not minimised behind other windows)
- [ ] Backup video ready: `demo/backup_recording.mp4` (see backup plan below)
- [ ] Close all unnecessary applications to free GPU/CPU

---

## Step-by-Step Live Demo (5 Minutes)

### Step 1: Show the Repository (30 seconds)
```bash
cd /Users/saiuppalapati/Downloads/Robotics\ Project
git log --oneline -5
```
*Say:* "This is our Git repository at [commit hash]. All simulation code, tests, and results are version-controlled. The project has [N] commits."

---

### Step 2: Show the URDF Robot Loads (30 seconds)
```bash
$PYTHON scripts/inspect_urdf.py
```
*Say:* "This script uses PyBullet to load our URDF and print all link names, joint axes, inertias, and total CoM. The complete robot has the intended 2.5 kg mass and a centre of mass 0.215 m above ground, so it is an inverted pendulum."

---

### Step 3: Uncontrolled Fall Demo (45 seconds)
```bash
$PYTHON scripts/run_simulation.py --controller none --gui --duration 3 --initial-tilt-deg 2
```
*Say:* "With zero torque, the robot falls in approximately 0.54 seconds from just 2° tilt. This confirms the system is unstable, matching our eigenvalue analysis — unstable pole at +10.97 rad/s."

*Wait for robot to fall. Show the tilt angle increasing rapidly in the terminal or GUI text.*

---

### Step 4: PID Balancing Demo (1 minute)
```bash
$PYTHON scripts/run_simulation.py --controller pid --gui --duration 20 --initial-tilt-deg 3
```
*Say:* "Now with the PID controller active. Watch how the wheels drive forward and backward to maintain balance. The yellow marker on the front face shows the direction."

*Let it run for ~15 seconds. Point out:*
- "The robot is balancing — tilt stays near zero"
- "PID regulates tilt directly; the saved comparison plots quantify its position displacement separately"

---

### Step 5: LQR Balancing Demo (1 minute)
```bash
$PYTHON scripts/run_simulation.py --controller lqr --gui --duration 20 --initial-tilt-deg 3
```
*Say:* "Now with LQR. This controller uses all four state variables: position, velocity, tilt, and tilt rate. In our recorded tests it gives lower final position displacement, while PID settles faster in tilt."

*Point out the difference in settling behaviour.*

---

### Step 6: Show Experiment Results (1 minute)
```bash
$PYTHON scripts/compare_controllers.py
```
*OR: Show pre-generated plots:*
```bash
open report/figures/E04_tilt.png
open report/figures/E06_tilt.png
```
*Say:* "These plots were generated from actual simulation data. The left shows PID versus LQR under identical conditions. The right shows our tilt sweep: both controllers completed every tested initial tilt through 30°, so their upper robustness limits still need a larger sweep."

---

### Step 7: Show Test Suite (30 seconds)
```bash
$PYTHON -m pytest tests/ -m "not slow" -v --tb=short 2>&1 | tail -20
```
*Say:* "Our suite has 50 automated checks covering the model, controllers, sensors, URDF/physics interface, metrics, and full balance runs. The fast subset currently has 45 passing checks; all 50 pass in the complete suite."

---

## Backup Plan (If GUI Fails)

If the PyBullet GUI does not open (e.g., display issues on the exam machine):

### Option A: Run headless and show CSV
```bash
$PYTHON scripts/run_simulation.py --controller lqr --headless --duration 20 --initial-tilt-deg 3
open results/data/manual/manual_lqr_seed42.csv  # view the data
```

### Option B: Show pre-recorded video
```bash
open demo/backup_recording.mp4
```
The backup recording (`demo/backup_recording.mp4`) shows:
1. Uncontrolled fall from 2°
2. PID balancing for 20s (noting tilt regulation)
3. LQR balancing for 20s (noting full-state position regulation)
4. Both controllers under a 10N disturbance (E05)

**To create the backup recording** (do this before the demo):
- Use QuickTime Player (macOS) or OBS Studio to record the screen
- Run the full GUI demo sequence
- Save as MP4 at 1080p
- Do NOT commit this file to git (it's in .gitignore); keep it in `demo/` locally

### Option C: Show the plots only
If nothing else works, open the plots in `results/plots/` and walk through the experimental results using those images.

---

## Disturbance Demo (Optional, if time permits)
```bash
python3 scripts/run_simulation.py --controller lqr --gui --duration 15 --initial-tilt-deg 3
# While running, press 'D' key to apply disturbance impulse
```
*Note:* Keyboard disturbance requires the GUI keyboard handler to be implemented. If not implemented, use the E05 experiment directly:
```bash
python3 -c "
import sys; sys.path.insert(0, 'src')
from balancing_robot.experiments.disturbance import run_disturbance
# ... (run E05)
"
```

---

## Talking Points During Demo

| Moment | Key Point to Make |
|--------|------------------|
| Robot loads | "URDF defines all physical parameters — mass, inertia, friction" |
| Robot falls | "Positive eigenvalue = exponential instability" |
| PID starts | "Error = θ − θ_ref; positive tilt → positive torque" |
| PID position changes | "Tilt-only control does not explicitly regulate position; the recorded comparison quantifies the effect." |
| LQR starts | "Full-state: regulates x and θ simultaneously" |
| LQR position response | "Full-state feedback penalises position and velocity; compare its displacement and settling metrics with PID." |
| Plots shown | "Numbers match the report exactly — same CSV files" |
| Tests run | "Automated tests verified before every commit" |

---

## Q&A Preparation

After the demo, examiners may ask questions. See `docs/viva_prep.md` for 28 prepared questions and answers. Key topics:
- Why is the robot unstable? (positive eigenvalue)
- What is the difference between PID and LQR?
- What does Q and R mean?
- Why does PID drift?
- How was the tilt angle extracted from PyBullet?
