import numpy as np
import config as env
from gnc import navigation as nav
from gnc import guidance as gd
from gnc import control as ct
from sim import simulation as sim
from viz import visualization as vis
from sensors import sensor as sns
import matplotlib.pyplot as plt

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu
m0 = env.m0
m_empty = env.m_empty

# --- Initial Conditions (From Apollo 11 Event B) ---
S0 = [14_878 + r_moon, 0, 0, np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878), m0]

# Initialize Simulation Parameters
landing = True
t = 0  # Simulation time
dt = 1  # Loop every second
h = dt / 100  # Plant run for accuracy

# Data logging for post-flight analysis
log_keys = [
    "S",
    "LVLH",
    "LVLH_nav",
    "t",
    "alpha_cmd",
    "alpha_ctrl",
    "T_cmd",
    "T_ctrl",
    "targets",
]
data = {key: [] for key in log_keys}
t_max = 1_000

# Initial state setup
S = S0
LVLH = nav.polar_to_LVLH(S0)
alpha_ctrl = -np.pi / 2
stage = 1
# Initialize Sensors
altimeter = sns.altimeter(0.5, 67)
# Initialize Guidance
staging = gd.Staging(1)
# --- Main Loop ---
while landing:
    # --- Navigation: Process sensor data to estimate state ---
    LVLH = nav.polar_to_LVLH(S)
    # --- Termination Condition ---
    if t >= t_max or LVLH[0] < 1:
        break
    LVLH_nav = LVLH.copy()
    LVLH_nav[0] = altimeter.measure(LVLH[0])

    # --- Guidance: Generate commanded accelerations based on target ---
    targets, t_go, stage = staging.get_guidance_targets(dt, LVLH_nav)
    _, _, ddz_cmd = gd.poly_guidance(
        0, [LVLH_nav[0], targets[0], LVLH_nav[1], targets[1]], t_go
    )
    _, _, ddx_cmd = gd.poly_guidance(
        0, [LVLH_nav[2], targets[2], LVLH_nav[3], targets[3]], t_go
    )

    # --- Control: Translate accelerations to actuator commands ---
    T_cmd, alpha_cmd = ct.control(t, LVLH_nav, [ddz_cmd, ddx_cmd])
    T_ctrl = ct.thrust_limiter(T_cmd)
    alpha_ctrl = ct.slew_limiter(dt, alpha_cmd, alpha_ctrl)

    # --- Dynamics: Propagate the physics model ---
    S = sim.propagate(h, dt, S, [T_ctrl, alpha_ctrl])

    # --- Log mission data ---
    for key in log_keys:
        data[key].append(locals()[key])
    # --- Loop ---
    t += dt

# --- Post-Simulation Analysis ---
for key in data:
    data[key] = np.array(data[key])

# --- Visualization: Generate plots and mission report ---
plt.close("all")
plt.plot(data["t"][:], data["LVLH"][:, 0])
plt.plot(data["t"], data["LVLH_nav"][:, 0])
plt.plot(data["t"], data["targets"][:, 0], "r--")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Altitude vs Time")
plt.legend(["True Altitude", "Estimated Altitude", "Target Altitude"])
plt.grid(True)
plt.show()

vis.trajectory(data["S"][:, 2], data["LVLH"][:, 0])
vis.telemetry(
    data["t"],
    [data["LVLH"][:, 1], data["LVLH"][:, 3]],
    data["alpha_cmd"],
    data["T_cmd"],
    data["alpha_ctrl"],
    data["T_ctrl"],
    data["S"][:, -1] - m_empty,
)
vis.end_state_metrics(data["t"], data["S"][-1])
plt.show()
