import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 6.67408e-11  # m^3/kg/s^2
G_earth = 9.81  # m/s^2
t_total = 2000  # s
r_moon = 1737e3  # m
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2

# Starting Conditions
r0 = r_moon + 15_000  # m
dr0 = 0  # m/s
v0 = -np.sqrt(mu / r0)
theta0 = np.pi / 2  # rad
dtheta0 = v0 / r0  # rad/s

# Target Conditions
target_r = r_moon  # m
target_dr = 0  # m/s
target_theta = 480_000 / r_moon  # rad
target_dtheta = 0  # rad/s


class Lander:
    def __init__(self, S, Isp, T_max, m_e):
        """
        Initialize the Lander object

        :param self: The Lander object
        :param S: Initial state of the lander (numpy array of 5 elements: r, dr, theta, dtheta, m)
        :param Isp: Specific impulse of the lander (s)
        :param T: Maximum thrust of the lander (N)
        :param m_e: Empty mass of the lander (kg)
        """
        self.S = S
        self.Isp = Isp
        self.T_max = T_max
        self.m_e = m_e

    def controller(self, S):
        r, dr, theta, dtheta, m = S
        T_max = self.T_max
        alt = r - r_moon

        T = 0
        alpha = 0
        if theta < 0.4:
            T = T_max
            alpha = np.pi
        if alt < 5_000:
            T = T_max * 0.6
            alpha = np.pi / 4
        if alt < 500:
            T = T_max * 0.2
            alpha = 0

        return T, alpha

    def dynamics(self, t, S):
        r, dr, theta, dtheta, m = S
        T, alpha = self.controller(S)

        # Radial
        dr = dr
        ddr = T / m * np.cos(alpha) - mu / r**2 + r * dtheta**2
        # Angular
        dtheta = dtheta
        ddtheta = 1 / r * (T / m * np.sin(alpha) - 2 * dr * dtheta)
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
        )
        return sol


Apollo = Lander(np.array([r0, dr0, theta0, dtheta0, 15200]), 311, 45000, 4280)
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
axs[0, 0].plot(x, y, label="Apollo")
axs[0, 0].plot(x_moon, y_moon, "gray", label="Moon Surface")
axs[0, 0].scatter(x_target, y_target, color="red", label="Target")
axs[0, 0].set_xlim(x_target - 25_000, x_target + 10_000)
axs[0, 0].set_ylim(y_target - 10_000, y_target + 75_000)
axs[0, 0].set_xlabel("x (m)")
axs[0, 0].set_ylabel("y (m)")
axs[0, 0].set_title("Apollo Trajectory")
axs[0, 0].legend()

# Altitude vs. Time
axs[0, 1].plot(sol.t, sol.y[0] - r_moon)
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Altitude (m)")
axs[0, 1].set_title("Altitude vs. Time")

# Velocity Components
axs[1, 0].plot(sol.t, sol.y[1])
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Velocity (m/s)")
axs[1, 0].set_title("Velocity Components")

# Fuel
axs[1, 1].plot(sol.t, sol.y[4])
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Mass (kg)")
axs[1, 1].set_title("Fuel Consumption")
plt.show()
