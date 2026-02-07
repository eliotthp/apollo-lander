import numpy as np
import navigation as nav
import guidance as gd
import controller as ct
import environment as env
import simulation as sim
import matplotlib.pyplot as plt

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu
m0 = env.m0

# --- Initial Conditions (From Apollo 11 Event B) ---
"""
r0 = r_moon + 14_878  # m
dr0 = -1.22  # m/s
theta0 = np.radians(40)  # rad
dtheta0 = -np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878)  # rad/s
"""
S0 = [14_878 + r_moon, 0, 0, np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878), m0]

# Initialize Simulation Parameters
landing = True
t = 0  # Simulation time
dt = 1  # Loop every second
h = dt / 100  # Plant run for accuracy

# Initialize state and trackers
S = S0
S_hist = []
t_hist = []
t_max = 500
# --- Main Loop ---
while landing:
    if t > t_max or S[0] < 0:
        landing = False
    # --- Navigation ---
    LVLH = nav.polar_to_LVLH(S)
    alt = nav.altitude(LVLH)
    # --- Guidance ---
    _, _, ddz_cmd = gd.poly_guidance(t, [LVLH[0], 0, LVLH[1], 0], t_max)
    _, _, ddx_cmd = gd.poly_guidance(t, [LVLH[2], 480_000, LVLH[3], 0], t_max)
    # --- Control ---
    T_cmd, alpha_cmd = ct.control(t, LVLH, [ddz_cmd, ddx_cmd])
    # --- Dynamics ---
    S = sim.propagate(h, dt, S, [T_cmd, alpha_cmd])
    S_hist.append(S)
    t_hist.append(t)
    t += dt

# --- Post-Simulation Analysis ---
S_hist = np.array(S_hist)
t_hist = np.array(t_hist)

fig, ax = plt.subplots(2, 2)
ax[0, 0].plot(t_hist, S_hist[:, 0] - r_moon)
ax[0, 0].axhline(y=0, color="grey", linestyle="--")

ax[0, 1].plot(t_hist, S_hist[:, 1])
ax[0, 1].set_xlabel("Time (s)")
ax[0, 1].set_ylabel("dr (m/s)")
ax[0, 1].grid()

ax[1, 1].plot(t_hist, S_hist[:, 3])
ax[1, 1].set_xlabel("Time (s)")
ax[1, 1].set_ylabel("dtheta (rad/s)")
ax[1, 1].grid()

plt.tight_layout()
plt.show()
