import numpy as np
import config as cfg
from gnc.navigation import Navigation
from gnc.guidance import Guidance
from gnc.control import Control
from sim.simulation import Simulation
import matplotlib.pyplot as plt
from states import PolarState, LVLHState, ControlState

# --- Initial Conditions (From Apollo 11 Event B) ---
S0 = PolarState(
    r=14_878 + cfg.r_moon,
    dr=0,
    theta=0,
    dtheta=np.sqrt(cfg.mu / (cfg.r_moon + 14_878)) / (cfg.r_moon + 14_878),
    m=cfg.m0,
)
landing = True
dt = 0.1
t = 0

nav = Navigation(cfg, 1, 42)
gd = Guidance()
ct = Control(cfg, ControlState(0, 0, 0, 0))
sim = Simulation(cfg, S0)

t_hist = []
r_hist = []
theta_hist = []

# --- Main Loop ---
while landing:
    # Navigation Step
    nav_state = nav.step(sim.state)

    # Guidance Step
    guid_state = gd.step(dt, nav_state)

    # Control Step
    control_state = ct.step(dt, nav_state, guid_state)

    # Check landing
    if sim.state.r - cfg.r_moon < 0:
        landing = False

    # Logging
    t_hist.append(t)
    r_hist.append(sim.state.r - cfg.r_moon)
    theta_hist.append(sim.state.theta)

    # Simulation Step
    sim.step(dt, control_state)

    t += dt

plt.plot(t_hist, r_hist)
plt.show()
