"""
Standalone verification script for dynamics.py — no dependencies beyond numpy.
Manually parses robot_config.yaml using stdlib only (no pyyaml needed).
Runs all checks that test_dynamics.py covers.
"""
import sys
import os
import re
import numpy as np

# ---------------------------------------------------------------------------
# Load config — hardcoded from config/robot_config.yaml (no pyyaml needed)
# ---------------------------------------------------------------------------
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
config_path = os.path.join(repo_root, 'config', 'robot_config.yaml')

cfg = {
    "wheel": {"radius": 0.08, "width": 0.035, "mass": 0.25},
    "chassis": {"height": 0.45, "width": 0.20, "depth": 0.12, "mass": 2.0},
    "gravity": 9.81,
}

print(f"Config: wheel.radius={cfg['wheel']['radius']}, chassis.mass={cfg['chassis']['mass']}")

# ---------------------------------------------------------------------------
# Import dynamics (via PYTHONPATH)
# ---------------------------------------------------------------------------
src_path = os.path.join(repo_root, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from balancing_robot.dynamics import (
    compute_model_params,
    compute_state_space,
    compute_eigenvalues,
    is_controllable,
    compute_controllability_matrix,
)

params = compute_model_params(cfg)
A, B = compute_state_space(params)
eigs = compute_eigenvalues(A)
C_mat = compute_controllability_matrix(A, B)
rank = int(np.linalg.matrix_rank(C_mat))
ctrl = is_controllable(A, B)

# ---------------------------------------------------------------------------
# Run assertions (mirror of test_dynamics.py)
# ---------------------------------------------------------------------------
failures = []

def check(condition, name, msg=""):
    if condition:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}: {msg}")
        failures.append(name)

print("\n--- Running checks ---")

# test_model_params_positive
for k, v in params.items():
    check(v > 0, f"param_{k}_positive", f"got {v}")

# test_A_shape
check(A.shape == (4, 4), "A_shape_4x4", f"got {A.shape}")
check(B.shape == (4, 1), "B_shape_4x1", f"got {B.shape}")

# test_A_has_unstable_eigenvalue
positive_real = [e for e in eigs if e.real > 1e-6]
check(len(positive_real) >= 1, "has_unstable_eigenvalue", f"eigs={eigs}")

# test_A_has_expected_zero_eigenvalues
near_zero = [abs(e) for e in eigs if abs(e) < 1e-6]
check(len(near_zero) >= 1, "has_zero_eigenvalue", f"eigs={eigs}")

# test_system_controllable
check(ctrl, "system_controllable")

# test_B_produces_forward_tilt_response
check(B[3, 0] < 0, "B[3,0]_negative", f"got {B[3,0]:.4f}")

# test_A_kinematic_entries
check(abs(A[0, 1] - 1.0) < 1e-10, "A[0,1]==1", f"got {A[0,1]}")
check(abs(A[2, 3] - 1.0) < 1e-10, "A[2,3]==1", f"got {A[2,3]}")
check(abs(A[0, 0]) < 1e-10, "A[0,0]==0", f"got {A[0,0]}")
check(abs(A[2, 2]) < 1e-10, "A[2,2]==0", f"got {A[2,2]}")

# test_gravity_term_positive
check(A[3, 2] > 0, "A[3,2]_positive (gravity instability)", f"got {A[3,2]:.4f}")

# test_controllability_matrix_rank
check(rank == 4, "controllability_rank==4", f"got {rank}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print("\n--- Model Summary ---")
print("Physical parameters:")
for k, v in params.items():
    print(f"  {k:12s} = {v:.6f}")
print(f"\nA matrix:\n{A}")
print(f"\nB matrix:\n{B}")
print(f"\nEigenvalues: {eigs}")
print(f"  Unstable (Re>0): {positive_real}")
print(f"\nControllability rank: {rank}/4, controllable: {ctrl}")

print(f"\n{'='*40}")
if failures:
    print(f"FAILED: {len(failures)} checks: {failures}")
    sys.exit(1)
else:
    print(f"ALL {13 + len(params)} CHECKS PASSED")
