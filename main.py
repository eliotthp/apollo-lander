import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

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

S0 = [r0, dr0, theta0, dtheta0, m0]

guidance_timeline = {
    0: [0.10, 93.0],
    26: [0.95, 87.0],
    176: [0.95, 71.0],
    258: [0.95, 72.0],
    384: [0.95, 73.0],
    402: [0.95, 70.7],
    506: [0.3835, -11.0],
    606: [0.20, 00.0],
    714: [0.00, 00.0],
}


# --- Functions ---
def controller(t, S):
    current_stage = 0
    for t0 in sorted(guidance_timeline.keys()):
        if t >= t0:
            current_stage = t0
    thrust_pct, alpha_deg = guidance_timeline[current_stage]
    return thrust_pct, alpha_deg


def dynamics(t, S):
    r, dr, theta, dtheta, m = S
    thrust_pct, alpha_deg = controller(t, S)

    # 1. Controller Logic (Functional)
    alpha = np.radians(alpha_deg)
    T = T_max * thrust_pct

    # 2. Fuel Guardrail
    if m <= m_empty:
        T, dm = 0, 0
    else:
        dm = -T / (G_earth * Isp)

    # 3. Equations of Motion
    ddr = (T / m) * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = (1 / r) * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)

    return [dr, ddr, dtheta, ddtheta, dm]


def surface_contact(t, S):
    return S[0] - r_moon


surface_contact.terminal = True
surface_contact.direction = -1

sol = solve_ivp(
    lambda t, S: dynamics(t, S),
    (0, 1000),
    S0,
    method="RK45",
    events=[surface_contact],
)

# --- Choose time ---
check_time = 606
idx = np.argmin(np.abs(sol.t - check_time))
# --- Quick Maths ---
v_tangential = sol.y[0][idx] * sol.y[3][idx]
v_inertial = np.sqrt(sol.y[1][idx] ** 2 + v_tangential**2)
# --- Print Stats ---
print(f"Time: {sol.t[idx]:.2f} s")
print(f"Altitude: {sol.y[0][idx] - r_moon:.2f} m")
print(f"Radial Velocity: {sol.y[1][idx]:.2f} m/s")
print(f"Tangential Velocity: {v_tangential:.2f} m/s")
print(f"Inertial Velocity: {v_inertial:.2f} m/s")

# Target Conditions
target_r = r_moon  # m
target_dr = 0  # m/s
target_theta = theta0 - 480_000 / r_moon  # rad
target_dtheta = 0  # rad/s

# Final Conditions
final_r = sol.y[0][-1]
final_dr = sol.y[1][-1]
final_theta = sol.y[2][-1]
final_dtheta = sol.y[3][-1]
final_m = sol.y[4][-1]

# Final Stats
miss_distance = (sol.y[2][-1] - target_theta) * r_moon
print("--- MISSION DATA ---")
print(
    f"Impact Velocity: {np.sqrt(final_dr**2 + (final_dtheta * final_r) ** 2):.2f} m/s"
)
print(f"Miss Distance: {miss_distance / 1000:.2f} km")
print(f"Remaining Propellant: {(final_m - m_empty):.2f} kg")

# Convert rad to deg
sol.y[2] *= 180 / np.pi
theta0 *= 180 / np.pi
target_theta *= 180 / np.pi
final_theta *= 180 / np.pi

# Plotting
fig, axs = plt.subplots(2, 2, figsize=(14, 8))

# The trajectory
axs[0, 0].plot(sol.y[2], sol.y[0] - r_moon, label="Eagle Path")
axs[0, 0].axhline(y=0, color="gray", linestyle="--", label="Moon Surface")
axs[0, 0].scatter(target_theta, target_r - r_moon, color="red", label="Target")
axs[0, 0].scatter(
    final_theta, final_r - r_moon, color="black", marker="x", label="Impact"
)
axs[0, 0].set_xlim(theta0, target_theta - 1)
axs[0, 0].set_ylim(-500, max(sol.y[0] - r_moon) + 500)
axs[0, 0].set_xlabel("Theta (°)")
axs[0, 0].set_ylabel("Radius (m)")
axs[0, 0].set_title("Radius vs. Theta (Descending Orbit)")
axs[0, 0].legend()
axs[0, 0].grid(True)

# Altitude vs. Time
axs[0, 1].plot(sol.t, sol.y[0] - r_moon)
axs[0, 1].set_ylim(-500, max(sol.y[0] - r_moon) + 500)
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Altitude (m)")
axs[0, 1].set_title("Altitude vs. Time")
axs[0, 1].grid(True)

# Velocity Components
axs[1, 0].plot(sol.t, sol.y[1], label="Radial")
axs[1, 0].plot(sol.t, -(sol.y[3] * sol.y[0]), label="Angular")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Velocity (m/s)")
axs[1, 0].set_title("Velocity Components")
axs[1, 0].legend()
axs[1, 0].grid(True)

# Fuel
axs[1, 1].plot(sol.t, sol.y[4])
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Mass (kg)")
axs[1, 1].set_title("Fuel Consumption")
axs[1, 1].grid(True)
plt.tight_layout()
plt.show()
