# External Libraries
import time
import config as cfg

# Internal Libraries
from telemetry import Logger
from gnc.navigation import Navigation
from gnc.guidance import Guidance
from gnc.control import Control
from sim.simulation import Simulation


# --- Initial Conditions (From Apollo 11 Event B) ---
landing = True
t = 0
dt = 0.1
t_max = 1000

nav = Navigation(cfg, 1, 42)
gd = Guidance()
ct = Control(cfg, cfg.C0)
sim = Simulation(cfg, cfg.S0)
logger = Logger(
    [
        "t",
        "m",
        "r",
        "dr",
        "z",
        "dz",
        "x",
        "dx",
        "T_cmd",
        "T_ctrl",
        "alpha_cmd",
        "alpha_ctrl",
        "t_elapsed",
    ]
)

# --- Main Loop ---
start = time.time()
while landing:
    # Navigation Step
    nav_state = nav.step(dt, sim.state)

    # Guidance Step
    guid_state = gd.step(dt, nav_state)

    # Control Step
    ctrl_state = ct.step(dt, nav_state, guid_state)

    # Logging
    logger.log(t, nav_state, guid_state, ctrl_state, sim.state)

    # Simulation Step
    if sim.state.r - cfg.r_moon < 0 or t > t_max:
        landing = False
    sim.step(dt, ctrl_state)

    t += dt
end = time.time()

# --- Logging and Plotting ---
logger.output_stats(t, start, end, sim.state)
logger.plot()
