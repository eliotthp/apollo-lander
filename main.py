# External Libraries
import time

# Internal Libraries
import config as cfg
from telemetry import Logger
from gnc.navigation import Navigation
from gnc.guidance import Guidance
from gnc.control import Control
from sim.simulation import Simulation

# --- Initial Conditions (From Apollo 11 Event B) ---
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
        "theta",
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


def main_loop(
    t=0, dt=0.1, t_max=1000, nav=nav, gd=gd, ct=ct, sim=sim, logger=logger
) -> tuple[float, float, float]:
    """
    Main loop for the simulation. This function steps through the navigation, guidance,
    control, logging, and simulation steps until the lander reaches the surface of the
    moon or the maximum time is exceeded.

    Parameters:
    t (float): The starting time for the simulation. Defaults to 0.
    dt (float): The time step for the simulation. Defaults to 0.1.
    t_max (float): The maximum time for the simulation. Defaults to 1000.
    nav (Navigation): The navigation object.
    gd (Guidance): The guidance object.
    ct (Control): The control object.
    sim (Simulation): The simulation object.
    logger (Logger): The logger object.

    Returns:
    tuple: A tuple containing the final time, the start time, and the end time.
    """
    landing = True
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

    return t, start - end, landing


if __name__ == "__main__":
    t, t_elapsed, _ = main_loop()

    # --- Logging and Plotting ---
    logger.output_stats(t, t_elapsed, sim.state)
    logger.plot_telemetry()
    logger.plot_trajectory()
