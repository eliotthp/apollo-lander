from gnc import navigation, guidance, control
from sim import simulation
from states import PolarState, LVLHState, ControlState, GuidanceState
import config as cfg

# Testing File to Verify code functionality

# Test Polar -> LVLH
nav = navigation.Navigation(cfg, 0, 0)
polar_state = PolarState(cfg.r_moon + 1000, -10, 0, 0, 0)

lvlh_state = nav._polar_to_LVLH(polar_state)

print(lvlh_state == LVLHState(1000, -10, 0, 0, 0))

# Test Zero-Thrust -> No Mass Change
m_initial = 10
dt = 1

ctrl = ControlState(0, 0, 0, 0)
sim = simulation.Simulation(cfg, PolarState(100, 0, 0, 0, m_initial))

for i in range(10):
    sim.step(dt, ctrl)

print(sim.state.m == m_initial)

# Test Thrust Limiter Behavior
ctrl_state = ControlState(10 * cfg.T_max, 0, 0, 0)

ctrl = control.Control(cfg, ctrl_state)
ctrl._thrust_limiter()

print(ctrl.control_state.T_ctrl == cfg.T_max)
