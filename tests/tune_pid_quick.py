"""Quick PID tuning test."""
import sys, math, logging
logging.basicConfig(level=logging.WARNING)
sys.path.insert(0, 'src')
from balancing_robot.config import load_robot_config, load_simulation_config
from balancing_robot.controllers.pid import PIDController
from balancing_robot.simulation import Simulation

robot_cfg = load_robot_config()
sim_cfg = load_simulation_config()

results = []
for Kp in [60, 80, 100, 130]:
    for Kd in [4.0, 6.0, 8.0, 12.0]:
        for Ki in [0.0, 0.3]:
            cfg = {'Kp': float(Kp), 'Ki': float(Ki), 'Kd': float(Kd),
                   'integral_clamp': 10.0, 'derivative_filter_alpha': 0.7,
                   'max_torque_total': 10.0, 'theta_ref': 0.0}
            ctrl = PIDController.from_config(cfg)
            sim = Simulation(sim_cfg=sim_cfg, robot_cfg=robot_cfg, controller=ctrl,
                gui=False, initial_tilt_deg=3.0, duration_s=10.0,
                experiment_id='tune', run_id='tune', seed=42)
            result = sim.run()
            if result.rows:
                thetas = [abs(r['tilt_rad']) for r in result.rows]
                import math as m
                rms = m.sqrt(sum(t**2 for t in thetas) / len(thetas))
            else:
                rms = 999
            results.append((result.success, result.fall_time or 0, rms, Kp, Kd, Ki))
            if result.success:
                print(f"SUCCESS Kp={Kp} Kd={Kd} Ki={Ki} rms={rms:.4f}rad")

results.sort(key=lambda x: (not x[0], -x[1], x[2]))
print("\nTop 5:")
for s, ft, rms, Kp, Kd, Ki in results[:5]:
    print(f"  success={s} fall={ft:.2f}s rms={rms:.4f} Kp={Kp} Kd={Kd} Ki={Ki}")
