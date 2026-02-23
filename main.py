# External Libraries
import time
import numpy as np
import matplotlib.pyplot as plt
import config as cfg

# Internal Libraries
from telemetry import Logger
from gnc.navigation import Navigation
from gnc.guidance import Guidance
from gnc.control import Control
from sim.simulation import Simulation


def find_stage_change(t_elapsed: list[float]) -> list[float]:
    time_stages = []
    for i in range(len(t_elapsed) - 1):
        if t_elapsed[i + 1] == 0:
            time_stages.append(logger.records["t"][i])
    return time_stages


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

real_t = end - start
delta_V = cfg.Isp * cfg.G_earth * np.log(cfg.m0 / sim.state.m)

# --- Printing Results ---
print("--- Results ---")
print(f"Time: {t:.2f} s")
print(f"Real Time: {real_t:.2f} s")
print(f"Effective sim speed (x real time) {t / real_t:.2f}")
print(f"Final Altitude: {sim.state.r - cfg.r_moon:.2f} m")
v_final = np.sqrt(sim.state.dr**2 + (sim.state.dtheta * sim.state.r) ** 2)
print(f"Final Velocity: {v_final:.2f} m/s")
print(f"Delta-V: {delta_V:.2f} m/s")
print(f"--- Safe Landing: {v_final < 5} ---")


# --- Plotting ---
fig, axs = plt.subplots(2, 3, figsize=(16, 8))
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

axs[0, 2].plot(
    logger.records["t"],
    np.array(logger.records["m"]) - cfg.m_empty,
    label="Mass of Propellant",
)
axs[0, 2].set_title("Propellant Mass vs. Time")
axs[0, 2].set_xlabel("Time (s)")
axs[0, 2].set_ylabel("Propellant Mass (kg)")
axs[0, 2].grid(True)

time_stages = find_stage_change(logger.records["t_elapsed"])

for i in [time_stages[0], time_stages[1]]:
    for ax_row in axs:
        for ax in ax_row:
            ax.axvline(i, color="k", linestyle="--", alpha=0.5)

r_array = np.array(logger.records["r"])
dr_array = np.array(logger.records["dr"])
z_array = np.array(logger.records["z"])
dz_array = np.array(logger.records["dz"])

axs[1, 2].plot(
    logger.records["t"],
    r_array - cfg.r_moon - z_array,
    label="Position",
)
axs[1, 2].plot(
    logger.records["t"],
    dr_array - dz_array,
    label="Velocity",
)
axs[1, 2].set_xlabel("Time (s)")
axs[1, 2].set_ylabel("Error (m)")
axs[1, 2].set_title("Error between radar and true z-quantities vs. Time | Filtered")
axs[1, 2].legend()
axs[1, 2].grid(True)
axs[1, 2].plot()
plt.show()

# Calculate RMS
rms_r = np.sqrt(np.mean((r_array - cfg.r_moon - z_array) ** 2))
rms_dr = np.sqrt(np.mean((dr_array - dz_array) ** 2))

print(f"RMS Error in Position: {rms_r:.5f} m")
print(f"RMS Error in Velocity: {rms_dr:.5f} m/s")

plt.savefig("figs/telemetry.png")
