"""Quick LQR validation after sensor LPF fix."""
import sys, math, logging
import numpy as np
logging.basicConfig(level=logging.WARNING)
sys.path.insert(0, 'src')
from balancing_robot.config import load_robot_config, load_simulation_config
from balancing_robot.dynamics import compute_model_params, compute_state_space
from balancing_robot.controllers.lqr import LQRController
from balancing_robot.physics import PhysicsEngine
from balancing_robot.robot import Robot
from balancing_robot.sensors import Sensors

robot_cfg = load_robot_config()
sim_cfg = load_simulation_config()
params = compute_model_params(robot_cfg)
A, B = compute_state_space(params)

Q = np.diag([1.0, 1.0, 100.0, 10.0])
R = np.array([[0.1]])
ctrl = LQRController(A=A, B=B, Q=Q, R=R, max_torque_total=10.0)
print('K =', ctrl.K)

phys = PhysicsEngine(sim_cfg, gui=False)
robot = Robot('assets/robot/balancing_robot.urdf', phys.client, robot_cfg, initial_tilt_deg=3.0)
sensors = Sensors(robot.robot_id, phys.client)
ctrl.reset()

dt = sim_cfg['physics']['timestep']
print('Step | t(s)  | theta(deg) | thetadot   | u(Nm)')
fell = False
for i in range(1000):
    state = sensors.get_state()
    if abs(state['theta']) > math.radians(45):
        print(f"FELL at t={i*dt:.3f}s theta={math.degrees(state['theta']):.1f}deg")
        fell = True
        break
    u = ctrl.compute(state, dt)
    robot.apply_torque(u)
    phys.step()
    if i % 100 == 0:
        print(f"{i:4d} | {i*dt:.3f} | {math.degrees(state['theta']):9.4f} | {state['theta_dot']:10.4f} | {u:6.3f}")
if not fell:
    print(f"SUCCESS - still balancing at t={1000*dt:.2f}s, theta={math.degrees(state['theta']):.3f}deg")
phys.disconnect()
