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
S0 = [14_878 + r_moon, 0, 0, -np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878), m0]

# Initialize Simulation Parameters
landing = True
t = 0  # Simulation time
dt = 1  # Loop every second
h = dt / 100  # Plant run for accuracy

# Initialize state and trackers
S = S0
S_hist = []
t_hist = []
# --- Main Loop ---
while landing:
    if t > 500 or S[0] < 0:
        landing = False
    # --- Navigation ---
    LVLH = nav.polar_to_LVLH(S)
    alt = nav.altitude(LVLH)
    # --- Guidance ---

    # --- Control ---
    S = sim.propagate(h, dt, S, [0, 0])
    S_hist.append(S)
    t_hist.append(t)
    t += dt

# --- Post-Simulation Analysis ---
S_hist = np.array(S_hist)
t_hist = np.array(t_hist)
plt.plot(t_hist, S_hist[:, 0] - r_moon)
plt.axhline(y=0, color="grey", linestyle="--")
plt.show()
