import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 6.67408e-11  # m^3/kg/s^2
G_earth = 9.81  # m/s^2
t_total = 1000  # s
r_moon = 1737e3  # m
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2

# Initial Conditions
r0 = r_moon + 15_240
dr0 = 0
theta0 = 40 * np.pi / 180
dtheta0 = np.sqrt(mu / (r_moon + 15_240)) / (r_moon + 15_240)
m0 = 15_240

# Target Conditions
target_r = r_moon
target_dr = 0
target_theta = theta0 + 480_000 / r_moon
target_dtheta = 0


class Lander:
    def __init__(self, S, Isp, T_max, m_e):
        self.S = S
        self.Isp = Isp
        self.T_max = T_max
        self.m_e = m_e

    def controller(self, S):
        r, dr, theta, dtheta, m = S
        T_max = self.T_max
        alt = r - r_moon

        # Determine Phase angle
        phase = theta - target_theta

        # Determine Angle of Thrust
        v_theta = dtheta * r
        gamma = np.arctan2(-v_theta, -dr)

        # Phase 0 Floating
        T, alpha = 0, 0
        # Phase 1 Braking
        if phase < 0.2:
            T = T_max
            alpha = np.pi / 2 - 0.05
        # Phase 2: Approach
        # elif alt < 8_000:
        #  T = T_max
        #  alpha = gamma
        # Phase 3: Descent
        # elif alt < 3_000:
        #  return
        return T, alpha

    def dynamics(self, t, S):
        r, dr, theta, dtheta, m = S
        T, alpha = self.controller(S)

        # Radial
        dr = dr
        ddr = T / m * np.cos(-alpha) - mu / r**2 + r * dtheta**2
        # Angular
        dtheta = dtheta
        ddtheta = 1 / r * (T / m * np.sin(-alpha) - 2 * dr * dtheta)
        # Mass
        dm = -T / (G_earth * self.Isp)

        return [dr, ddr, dtheta, ddtheta, dm]

    def propagate(self, S, duration):
        def surface_contact(t, S):
            return S[0] - r_moon

        surface_contact.terminal = True
        surface_contact.direction = -1

        sol = solve_ivp(
            lambda t, S: self.dynamics(t, S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
            events=[surface_contact],
            max_step=1,
        )
        return sol


# Starting Conditions (Burn Ignition State)
Apollo = Lander(np.array([r0, dr0, theta0, dtheta0, m0]), 311, 45000, 4280)
sol = Apollo.propagate(Apollo.S, t_total)

# Apollo Transform
x = sol.y[0] * np.cos(sol.y[2])
y = sol.y[0] * np.sin(sol.y[2])

# Target Transform
x_target = target_r * np.cos(target_theta)
y_target = target_r * np.sin(target_theta)

# Moon Transform
theta_circle = np.linspace(0, 2 * np.pi, 1000)
x_moon = r_moon * np.cos(theta_circle)
y_moon = r_moon * np.sin(theta_circle)

fig, axs = plt.subplots(2, 2, figsize=(14, 8))

# The trajectory
axs[0, 0].plot(sol.y[2], sol.y[0], label="Eagle Path")
axs[0, 0].axhline(y=r_moon, color="gray", linestyle="--", label="Moon Surface")
axs[0, 0].scatter(target_theta, target_r, color="red", label="Target")
axs[0, 0].set_xlim(max(sol.y[2]) + 0.05, min(sol.y[2]) - 0.05)
axs[0, 0].set_xlabel("Theta (rad)")
axs[0, 0].set_ylabel("Radius (m)")
axs[0, 0].set_title("Radius vs. Theta (Descending Orbit)")
axs[0, 0].legend()
axs[0, 0].grid(True)

# Altitude vs. Time
axs[0, 1].plot(sol.t, sol.y[0] - r_moon)
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
