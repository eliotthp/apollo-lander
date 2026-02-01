import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import guidance  # Importing the guidance module
import visualization  # Importing the visualization module


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

# Target Conditions
target_r = [2346.96, r_moon]  # m
target_dr = [-44.2, 0]  # m/s
target_theta = [np.radians(24.5), theta0 - 480_000 / r_moon]  # rad
target_dtheta = [0.0657, 0]  # rad/s

S0 = [r0, dr0, theta0, dtheta0, m0]


# --- Functions ---
def controller(t, S):
    # Navigation
    r_act, dr_act, theta_act, dtheta_act, m = S

    # Braking Phase
    if r_act - r_moon > 2346.96:
        tf_total = 8 * 60 + 26
        # Guidance
        _, _, ddr_req = guidance.poly_guidance(
            t, [r0, target_r[0], dr0, target_dr[0]], tf_total
        )
        _, _, ddtheta_req = guidance.poly_guidance(
            t, [theta0, target_theta[0], dtheta0, target_dtheta[0]], tf_total
        )

        Fr = m * (ddr_req + mu / r_act**2 - r_act * dtheta_act**2)
        Ft = m * (r_act * ddtheta_req + 2 * dr_act * dtheta_act)

        thrust_req = np.sqrt(Fr**2 + Ft**2)
        alpha_req = np.arctan2(Ft, Fr)

        # Apply Limits
        thrust_pct = 0.95
    else:
        # Determine time remaining
        tf_total = 580
        t_go = max(0.01, tf_total - t)

        # Guidance
        _, _, ddr_req = guidance.poly_guidance(
            t, [r0, target_r[0], dr0, target_dr[0]], tf_total
        )
        _, _, ddtheta_req = guidance.poly_guidance(
            t, [theta0, target_theta[0], dtheta0, target_dtheta[0]], tf_total
        )

        # Control
        Fr = m * (ddr_req + mu / r_act**2 - r_act * dtheta_act**2)
        Ft = m * (r_act * ddtheta_req + 2 * dr_act * dtheta_act)

        thrust_req = np.sqrt(Fr**2 + Ft**2)
        alpha_req = np.arctan2(Ft, Fr)

        # Apply Limits
        thrust_pct = np.clip(thrust_req / T_max, 0, 1)

        if r_act <= r_moon + 0.1:
            thrust_pct = 0

    return thrust_pct, np.degrees(alpha_req)


def dynamics(t, S):
    r, dr, theta, dtheta, m = S

    thrust_pct, alpha_deg = controller(t, S)

    # Controller Logic (Functional)
    alpha = np.radians(alpha_deg)
    T = T_max * thrust_pct

    # Fuel Guardrail
    if m <= m_empty:
        T, dm = 0, 0
    else:
        dm = -T / (G_earth * Isp)

    # Equations of Motion
    ddr = (T / m) * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = (1 / r) * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)

    return [dr, ddr, dtheta, ddtheta, dm]


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
    pct, deg = controller(t_val, s_val)
    reconstructed_alpha.append(deg)
    reconstructed_thrust.append(pct * 100)  # As percentage

plt.plot(sol.t, reconstructed_alpha, label="Pitch (deg)", color="orange")
plt.plot(sol.t, reconstructed_thrust, label="Thrust (%)", color="purple")
plt.title("Control Commands Over Time")
plt.legend()
plt.tight_layout()
plt.show()

# --- Plot ---
visualization.plot_mission_results(sol, mission_params)
