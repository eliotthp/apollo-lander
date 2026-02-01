import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import controller as ctrl  # Importing the controller module
import visualization as viz  # Importing the visualization module
import environment as env  # Importing the enviromental variables

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu
Isp = env.Isp
m_empty = env.m_empty
m0 = env.m0
T_max = env.T_max

# --- Initial Conditions (From Apollo 11 Event B) ---
r0 = r_moon + 14_878  # m
dr0 = -1.22  # m/s
theta0 = np.radians(40)  # rad
dtheta0 = -np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878)  # rad/s
alpha0 = np.pi / 2  # rad

# --- Target Conditions ---
zf = [2346.96, r_moon]  # m
dzf = [-44.2, 0]  # m/s
xf = [400_000, 480_000]  # m
dxf = [-44.2, 0]  # m/s
targets = np.array([zf, dzf, xf, dxf])

S0 = [r0, dr0, theta0, dtheta0, m0, alpha0]


# --- Functions ---
def dynamics(t, S):
    """
    Calculates the derivatives of the state vector for the lunar lander.

    Args:
        t (float): Current simulation time.
        S (list): Current state vector [r, dr, theta, dtheta, m, alpha].

    Returns:
        list: Derivatives of the state vector [dr, ddr, dtheta, ddtheta, dm, dalpha].
    """
    r, dr, theta, dtheta, m, alpha = S

    T_cmd, alpha_cmd = ctrl.control(t, S, targets[:, 0].tolist())

    # Fuel Guardrail
    if m <= m_empty:
        T, dm = 0, 0
    else:
        dm = -T / (G_earth * Isp)

    # Equations of Motion
    ddr = (T / m) * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = (1 / r) * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)
    dalpha = alpha_cmd

    return [dr, ddr, dtheta, ddtheta, dm, dalpha]


# --- Simulation ---
def surface_contact(t, S):
    """
    Event function to detect when the lander reaches the lunar surface.

    Args:
        t (float): Current simulation time.
        S (list): Current state vector.

    Returns:
        float: Altitude above the lunar surface (zero at contact).
    """
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
    "target_theta": xf[-1] / r_moon,
    "target_r": zf[-1] + r_moon,
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
    pct, rad = ctrl.control(t_val, s_val, targets[:, 0].tolist())
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
