import numpy as np
import navigation as nav
import guidance as gd
import controller as ct
import environment as env
import simulation as sim
import visualization as vis
import matplotlib.pyplot as plt
import sensor as sns

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
S_hist = []
LVLH_hist = []
LVLH_nav_hist = []
t_hist = []
alpha_cmd_hist = []
alpha_ctrl_hist = []
T_cmd_hist = []
T_ctrl_hist = []
t_max = 714

# Initial state setup
S = S0
LVLH = nav.polar_to_LVLH(S0)
alpha_ctrl = -np.pi / 2
# Initialize Sensors
altimeter = sns.altimeter(0.5, 67)

# --- Main Loop ---
while landing:
    if t >= t_max or LVLH[0] <= 0:
        landing = False
    t_go = max(t_max - t, dt)

    # --- Navigation: Process sensor data to estimate state ---
    LVLH = nav.polar_to_LVLH(S)
    LVLH_nav = LVLH.copy()
    LVLH_nav[0] = altimeter.measure(LVLH[0])

    # --- Guidance: Generate commanded accelerations based on target ---
    _, _, ddz_cmd = gd.poly_guidance(0, [LVLH_nav[0], 0, LVLH_nav[1], 0], t_go)
    _, _, ddx_cmd = gd.poly_guidance(0, [LVLH_nav[2], 480_000, LVLH_nav[3], 0], t_go)

    # --- Control: Translate accelerations to actuator commands ---
    T_cmd, alpha_cmd = ct.control(t, LVLH_nav, [ddz_cmd, ddx_cmd])
    T_ctrl = ct.thrust_limiter(T_cmd)
    alpha_ctrl = ct.slew_limiter(dt, alpha_cmd, alpha_ctrl)

    # --- Dynamics: Propagate the physics model ---
    S = sim.propagate(h, dt, S, [T_ctrl, alpha_cmd])

    # --- History ---
    S_hist.append(S)
    LVLH_hist.append(LVLH)
    LVLH_nav_hist.append(LVLH_nav)
    t_hist.append(t)
    alpha_cmd_hist.append(alpha_cmd)
    alpha_ctrl_hist.append(alpha_ctrl)
    T_cmd_hist.append(T_cmd)
    T_ctrl_hist.append(T_ctrl)
    t += dt

# --- Post-Simulation Analysis ---
S_hist = np.array(S_hist)
LVLH_hist = np.array(LVLH_hist)
LVLH_nav_hist = np.array(LVLH_nav_hist)
t_hist = np.array(t_hist)

# --- Visualization: Generate plots and mission report ---
plt.close("all")
plt.plot(t_hist, LVLH_hist[:, 0])
plt.plot(t_hist, LVLH_nav_hist[:, 0])
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Altitude vs Time")
plt.legend(["True Altitude", "Estimated Altitude"])
plt.grid(True)
plt.show()

vis.trajectory(S_hist[:, 2], LVLH_hist[:, 0])
vis.telemetry(
    t_hist,
    [LVLH_hist[:, 1], LVLH_hist[:, 3]],
    alpha_cmd_hist,
    T_cmd_hist,
    alpha_ctrl_hist,
    T_ctrl_hist,
    S_hist[:, -1] - m_empty,
)
vis.end_state_metrics(t_hist, S_hist[-1])
plt.show()
