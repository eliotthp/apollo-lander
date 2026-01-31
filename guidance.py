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

T = 580
t = np.linspace(0, T, 100)


def poly_guidance(t, S, tf):
    a = np.array(
        [[0, 0, 0, 1], [tf**3, tf**2, tf, 1], [0, 0, 1, 0], [3 * tf**2, 2 * tf, 1, 0]]
    )
    b = S

    a, b, c, d = np.linalg.solve(a, b)

    f = a * t**3 + b * t**2 + c * t + d
    df = 3 * a * t**2 + 2 * b * t + c
    ddf = 6 * a * t + 2 * b
    return f, df, ddf


r, dr, ddr = poly_guidance(t, [r0, target_r, dr0, target_dr], T)

theta, dtheta, ddtheta = poly_guidance(
    t, [theta0, target_theta, dtheta0, target_dtheta], T
)

# --- Thrust Curve ---
F_r_max = []
F_theta_max = []
F_max = []
T = np.arange(550, 650, 10)
for tf in T:
    t = np.linspace(0, tf, 100)
    r, dr, ddr = poly_guidance(t, [r0, target_r, dr0, target_dr], tf)
    theta, dtheta, ddtheta = poly_guidance(
        t, [theta0, target_theta, dtheta0, target_dtheta], tf
    )

    F_r = m0 * (ddr + mu / r**2 - r * dtheta**2)
    F_theta = m0 * (r * ddtheta - 2 * dr * dtheta)
    F_r_max.append(max(F_r))
    F_theta_max.append(max(F_theta))
    F_max.append(np.sqrt(F_r_max[-1] ** 2 + F_theta_max[-1] ** 2))
"""
# --- Calculations ---
F_r = m0*(ddr + mu/r**2 - r*dtheta**2)
F_theta = m0*(r*ddtheta - 2*dr*dtheta)
F_r_max = max(F_r)
F_theta_max = max(F_theta)

# --- Printing ---
print(f"Maximum Radial Thrust: {F_r_max:.2f} N")
print(f"Maximum Angular Thrust: {F_theta_max:.2f} N")
"""
# --- Plotting ---
fig, axs = plt.subplots(2, 2, figsize=(14, 8))

axs[0, 0].plot(theta * 180 / np.pi, r - r_moon)
axs[0, 0].set_xlabel("Theta (deg)")
axs[0, 0].set_ylabel("Radius (m)")
axs[0, 0].set_title("Radius vs. Theta (Descending Orbit)")

axs[1, 1].plot(T, F_max, label="F_r")
axs[1, 1].set_xlabel("Thrust (N)")
axs[1, 1].set_ylabel("Force (N)")
axs[1, 1].legend()

axs[1, 0].plot(t, F_r, label="F_r")
axs[1, 0].plot(t, F_theta, label="F_theta")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Force (N)")
axs[1, 0].legend()

ax1 = axs[0, 1]
ax2 = ax1.twinx()
ax1.plot(t, theta, color="b", label="Theta (rad)")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (rad)", color="b")
ax1.tick_params(axis="y", colors="b")
ax2.plot(t, r - r_moon, color="r", label="Radius (m)")
ax2.set_ylabel("Radius (m)", color="r")
ax2.tick_params(axis="y", colors="r")
ax1.set_title("Theta and Radius vs. Time")
ax1.legend(loc="lower left")
ax2.legend(loc="upper right")

plt.show()
