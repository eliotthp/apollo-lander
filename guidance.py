import numpy as np
import matplotlib.pyplot as plt

# --- Constants & Environment ---
G = 6.67408e-11  # m^3/kg/s^2
G_earth = 9.81  # m/s^2
r_moon = 1737e3  # m
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2

# --- Initial Conditions (From Apollo 11 Event B) ---
r0 = r_moon + 14_878  # m
dr0 = -1.22  # m/s
theta0 = np.radians(40)  # rad
dtheta0 = -np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878)  # rad/s
m0 = 15_240  # kg
m_empty = 4_280  # kg
Isp = 311  # s
T_max = 45_000  # N

# --- Target Conditions ---
target_r = r_moon  # m
target_dr = 0  # m/s
target_theta = theta0 - 480_000 / r_moon  # rad
target_dtheta = 0  # rad/s

T = 714
t = np.linspace(0, T, 100)

# --- Polynomial Guidance (r) ---
a = np.array([[0, 0, 0, 1], [T**3, T**2, T, 1], [0, 0, 1, 0], [3 * T**2, 2 * T, 1, 0]])
b = np.array([[r0], [target_r], [dr0], [target_dr]])

a, b, c, d = np.linalg.solve(a, b)
r = a * t**3 + b * t**2 + c * t + d

# --- Polynomial Guidance (theta) ---
a = np.array([[0, 0, 0, 1], [T**3, T**2, T, 1], [0, 0, 1, 0], [3 * T**2, 2 * T, 1, 0]])
b = np.array([[theta0], [target_theta], [dtheta0], [target_dtheta]])

a, b, c, d = np.linalg.solve(a, b)
theta = a * t**3 + b * t**2 + c * t + d

fig, axs = plt.subplots(2, 2, figsize=(14, 8))

axs[0, 0].plot(theta * 180 / np.pi, r - r_moon)
axs[0, 0].set_xlabel("Theta (deg)")
axs[0, 0].set_ylabel("Radius (m)")
axs[0, 0].set_title("Radius vs. Theta (Descending Orbit)")

axs[0, 1].plot(t, theta)
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Theta (rad)")
axs[0, 1].set_title("Theta vs. Time")

axs[1, 1].plot(t, r - r_moon)
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Radius (m)")
axs[1, 1].set_title("Radius vs. Time")

plt.show()
