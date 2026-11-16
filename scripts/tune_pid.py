#!/usr/bin/env python3
"""
Tune LQR and PID gains via headless simulation sweeps.

Usage:
    python scripts/tune_pid.py
    python scripts/tune_pid.py --tilt 3.0 --duration 15
"""
import argparse
import sys
import logging
import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

logging.basicConfig(level=logging.WARNING)


def run_sim(controller, duration=10.0, tilt=3.0):
    from balancing_robot.config import load_robot_config, load_simulation_config
    from balancing_robot.simulation import Simulation
    robot_cfg = load_robot_config()
    sim_cfg = load_simulation_config()
    sim = Simulation(
        sim_cfg=sim_cfg, robot_cfg=robot_cfg, controller=controller,
        gui=False, initial_tilt_deg=tilt, duration_s=duration,
        experiment_id="tune", run_id="tune", seed=42,
    )
    result = sim.run()
    if not result.rows:
        return False, 0.0, 999.0, 999.0
    thetas = [abs(r["tilt_rad"]) for r in result.rows]
    rms = math.sqrt(sum(t**2 for t in thetas) / len(thetas))
    max_t = max(thetas)
    return result.success, result.fall_time or duration, rms, max_t


def tune_lqr():
    import numpy as np
    from balancing_robot.config import load_robot_config, load_lqr_config
    from balancing_robot.dynamics import compute_model_params, compute_state_space
    from balancing_robot.controllers.lqr import LQRController

    robot_cfg = load_robot_config()
    params = compute_model_params(robot_cfg)
    A, B = compute_state_space(params)

    best = {"score": 999, "Q": None, "R": None, "K": None}

    # Sweep q_theta and R; fix q_x=1 to prevent drift
    for q_theta in [50, 100, 200, 400, 800]:
        for q_thetadot in [5, 10, 20]:
            for R_val in [0.01, 0.05, 0.1, 0.5]:
                Q = np.diag([1.0, 1.0, float(q_theta), float(q_thetadot)])
                R = np.array([[R_val]])
                try:
                    ctrl = LQRController(A=A, B=B, Q=Q, R=R, max_torque_total=10.0)
                except Exception:
                    continue
                if not ctrl.is_stable():
                    continue
                success, fall_t, rms, max_t = run_sim(ctrl, duration=15.0, tilt=3.0)
                score = rms if success else 999 - fall_t
                if score < best["score"]:
                    best.update({"score": score, "Q": Q.diagonal().tolist(),
                                 "R": R_val, "K": ctrl.K.tolist(),
                                 "success": success, "rms": rms, "fall_t": fall_t})
                    print(f"  q_theta={q_theta} q_td={q_thetadot} R={R_val}: "
                          f"success={success} rms={rms:.4f}rad fall={fall_t:.2f}s [BEST]")

    print(f"\nBest LQR: Q={best['Q']} R={best['R']}")
    print(f"  K = {best['K']}")
    print(f"  rms={best.get('rms','?'):.4f}rad success={best.get('success','?')}")
    return best


def tune_pid():
    from balancing_robot.controllers.pid import PIDController

    best = {"score": 999, "Kp": None, "Ki": None, "Kd": None}

    for Kp in [35, 55, 80, 110, 150]:
        for Kd in [2.5, 4.0, 6.0, 9.0]:
            for Ki in [0.0, 0.5, 1.0]:
                cfg = {"Kp": float(Kp), "Ki": float(Ki), "Kd": float(Kd),
                       "integral_clamp": 10.0, "derivative_filter_alpha": 0.7,
                       "max_torque_total": 10.0, "theta_ref": 0.0}
                ctrl = PIDController.from_config(cfg)
                success, fall_t, rms, max_t = run_sim(ctrl, duration=15.0, tilt=3.0)
                score = rms if success else 999 - fall_t
                if score < best["score"]:
                    best.update({"score": score, "Kp": Kp, "Ki": Ki, "Kd": Kd,
                                 "success": success, "rms": rms, "fall_t": fall_t})
                    print(f"  Kp={Kp} Kd={Kd} Ki={Ki}: "
                          f"success={success} rms={rms:.4f}rad fall={fall_t:.2f}s [BEST]")

    print(f"\nBest PID: Kp={best['Kp']} Ki={best['Ki']} Kd={best['Kd']}")
    print(f"  rms={best.get('rms','?'):.4f}rad success={best.get('success','?')}")
    return best


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["lqr", "pid", "both"], default="both")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--tilt", type=float, default=3.0)
    args = parser.parse_args()

    if args.controller in ("lqr", "both"):
        print("=== Tuning LQR ===")
        tune_lqr()
    if args.controller in ("pid", "both"):
        print("\n=== Tuning PID ===")
        tune_pid()
