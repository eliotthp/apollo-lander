import numpy as np
import config as cfg
from telemetry import Logger
from gnc.navigation import Navigation
from gnc.guidance import Guidance
from gnc.control import Control
from sim.simulation import Simulation
import matplotlib.pyplot as plt
from states import PolarState, ControlState


def find_stage_change(t_elapsed: list[float]) -> list[float]:
    time_stages = []
    for i in range(len(t_elapsed) - 1):
        if t_elapsed[i + 1] == 0:
            time_stages.append(logger.records["t"][i])
    return time_stages


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
t_max = 1000

nav = Navigation(cfg, 1, 42)
gd = Guidance()
ct = Control(cfg, ControlState(0, -np.pi / 2, 0, -np.pi / 2))
sim = Simulation(cfg, S0)
logger = Logger(
    [
        "t",
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
while landing:
    # Navigation Step
    nav_state = nav.step(sim.state)

    # Guidance Step
    guid_state = gd.step(dt, nav_state)

    # Control Step
    control_state = ct.step(dt, nav_state, guid_state)

    # Check landing
    if sim.state.r - cfg.r_moon < 0 or t > t_max:
        landing = False

    # Logging
    logger.log(t, nav_state, guid_state, control_state)

    # Simulation Step
    sim.step(dt, control_state)

    t += dt

# --- Printing Results ---
print("--- Results ---")
print(f"Time: {t:.2f} s")
print(f"Final Altitude: {sim.state.r - cfg.r_moon:.2f} m")
print(f"Final Velocity: {np.sqrt(sim.state.dr**2 + sim.state.dtheta**2):.2f} m/s")


# --- Plotting ---
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle("Descent Simulation Telemetry")
fig.subplots_adjust(
    bottom=0.08, left=0.08, right=0.92, top=0.92, wspace=0.22, hspace=0.22
)

axs[0, 0].plot(logger.records["t"], logger.records["z"])
axs[0, 0].set_title("Altitude vs. Time")
axs[0, 0].set_xlabel("Time (s)")
axs[0, 0].set_ylabel("Altitude (m)")
axs[0, 0].grid(True)

axs[0, 1].plot(logger.records["t"], logger.records["dz"], label="Vertical Velocity")
axs[0, 1].plot(logger.records["t"], logger.records["dx"], label="Horizontal Velocity")
axs[0, 1].set_title("Velocity vs. Time")
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Velocity (m/s)")
axs[0, 1].legend()
axs[0, 1].grid(True)

axs[1, 0].plot(
    logger.records["t"], logger.records["T_cmd"], "r--", label="Commanded Thrust"
)
axs[1, 0].plot(
    logger.records["t"], logger.records["T_ctrl"], "b-", label="Actual Thrust"
)
axs[1, 0].set_title("Thrust vs. Time")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Thrust (N)")
axs[1, 0].set_ylim(0, cfg.T_max * 1.5)
axs[1, 0].legend()
axs[1, 0].grid(True)

axs[1, 1].plot(
    logger.records["t"],
    np.rad2deg(logger.records["alpha_cmd"]),
    "r--",
    label="Commanded Pitch",
)
axs[1, 1].plot(
    logger.records["t"],
    np.rad2deg(logger.records["alpha_ctrl"]),
    "b-",
    label="Actual Pitch",
)
axs[1, 1].set_title("Pitch Angle vs. Time")
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Pitch Angle (degrees)")
axs[1, 1].legend()
axs[1, 1].grid(True)

time_stages = find_stage_change(logger.records["t_elapsed"])

"""for time in [time_stages]:
    for ax_row in axs:
        for ax in ax_row:
            ax.axvline(time, color="k", linestyle="--", alpha=0.5)"""

plt.show()
