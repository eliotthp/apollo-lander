import numpy as np
import config as cfg
from gnc import navigation as nav
from gnc import guidance as gd
from gnc import control as ct
from sim import simulation as sim
from viz import visualization as vis
import matplotlib.pyplot as plt
from states import PolarState

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

nav = nav.Navigation(cfg, 1, 42)
gd = gd.Guidance()
ct = ct.Control(cfg, ct.ControlState(0, 0, 0, 0))
sim = sim.Simulation(cfg, S0)


# --- Main Loop ---
while landing:
    # Navigation Step
    nav_state = nav.step(sim.state)

    # Guidance Step
    guid_state = gd.step(dt, nav_state)

    # Control Step
    control_state = ct.step(dt, nav_state, guid_state)

    # Simulation Step
    sim.step(dt, control_state)

    # Check landing
    if sim.state.r - cfg.r_moon < 0:
        landing = False
