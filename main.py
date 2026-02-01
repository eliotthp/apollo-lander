import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import controller as ctrl  # Importing the controller module
import visualization as viz  # Importing the visualization module


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
alpha0 = np.pi / 2  # rad
m_empty = 4_280  # kg
Isp = 311  # s
T_max = 45_000  # N

# Target Conditions
target_r = [2346.96 + r_moon, r_moon]  # m
target_dr = [-44.2, 0]  # m/s
target_theta = [np.radians(24.5), theta0 - 480_000 / r_moon]  # rad
target_dtheta = [-8.828e-5, 0]  # rad/s

S0 = [r0, dr0, theta0, dtheta0, m0, alpha0]


# --- Functions ---
def dynamics(t, S):
    r, dr, theta, dtheta, m, alpha = S

    T_cmd, alpha_cmd = ctrl.control(t, S)

    # Controller Logic (Functional)
    T = T_cmd
    if alpha_cmd - alpha > 0:
        dalpha = np.deg2rad(np.clip(5, 0, alpha_cmd - alpha))
    else:
        dalpha = -np.deg2rad(np.clip(5, alpha_cmd - alpha, 0))

    # Fuel Guardrail
    if m <= m_empty:
        T, dm = 0, 0
    else:
        dm = -T / (G_earth * Isp)

    # Equations of Motion
    ddr = (T / m) * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = (1 / r) * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)

    return [dr, ddr, dtheta, ddtheta, dm, dalpha]


# --- Simulation ---
def surface_contact(t, S):
    return S[0] - r_moon


surface_contact.terminal = True

sol = solve_ivp(
    lambda t, S: dynamics(t, S),
    (0, 1000),
    S0,
    method="RK45",
    events=[surface_contact],
    max_step=1,
)

# --- Results ---
mission_params = {
    "r_moon": r_moon,
    "target_theta": target_theta,
    "target_r": target_r,
    "theta0": theta0,
    "m0": m0,
    "m_empty": m_empty,
    "Isp": Isp,
    "G_earth": G_earth,
}

reconstructed_alpha = []
reconstructed_thrust = []

for i in range(len(sol.t)):
    t_val = sol.t[i]
    s_val = sol.y[:, i]
    pct, rad = ctrl.control(t_val, s_val)
    deg = np.rad2deg(rad)
    pct /= T_max
    reconstructed_alpha.append(deg)
    reconstructed_thrust.append(pct * 100)  # As percentage

plt.plot(sol.t, reconstructed_alpha, label="Pitch (deg)", color="orange")
plt.plot(sol.t, reconstructed_thrust, label="Thrust (%)", color="purple")
plt.title("Control Commands Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot ---
viz.plot_mission_results(sol, mission_params)
