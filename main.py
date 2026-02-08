import numpy as np
import navigation as nav
import guidance as gd
import controller as ct
import environment as env
import simulation as sim
import visualization as vis
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

# Initialize
S = S0
LVLH = nav.polar_to_LVLH(S0)
alpha_ctrl = -np.pi / 2

# History
S_hist = []
LVLH_hist = []
t_hist = []
alpha_cmd_hist = []
alpha_ctrl_hist = []
T_cmd_hist = []
T_ctrl_hist = []
t_max = 714

# --- Main Loop ---
while landing:
    if t >= t_max or LVLH[0] <= 0:
        landing = False
    t_go = max(t_max - t, dt)

    # --- Navigation ---
    LVLH = nav.polar_to_LVLH(S)

    # --- Guidance ---
    _, _, ddz_cmd = gd.poly_guidance(0, [LVLH[0], 0, LVLH[1], 0], t_go)
    _, _, ddx_cmd = gd.poly_guidance(0, [LVLH[2], 480_000, LVLH[3], 0], t_go)

    # --- Control ---
    T_cmd, alpha_cmd = ct.control(t, LVLH, [ddz_cmd, ddx_cmd])
    T_ctrl = ct.thrust_limiter(T_cmd)
    alpha_ctrl = ct.slew_limiter(dt, alpha_cmd, alpha_ctrl)

    # --- Dynamics ---
    S = sim.propagate(h, dt, S, [T_ctrl, alpha_cmd])

    # --- History ---
    S_hist.append(S)
    LVLH_hist.append(LVLH)
    t_hist.append(t)
    alpha_cmd_hist.append(alpha_cmd)
    alpha_ctrl_hist.append(alpha_ctrl)
    T_cmd_hist.append(T_cmd)
    T_ctrl_hist.append(T_ctrl)
    t += dt

# --- Post-Simulation Analysis ---
S_hist = np.array(S_hist)
LVLH_hist = np.array(LVLH_hist)
t_hist = np.array(t_hist)

# --- Visualization ---
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
