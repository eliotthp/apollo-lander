import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 6.67408e-11  # m^3/kg/s^2
G_earth = 9.81  # m/s^2
t_total = 1000  # s
r_moon = 1737e3  # m
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2

# Starting Conditions
r0 = r_moon + 15_000  # m
dr0 = 0  # m/s
theta0 = 0  # rad
dtheta0 = 0  # rad/s

# Target Conditions
target_r = r_moon  # m
target_dr = 0  # m/s
target_theta = 0  # rad
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
        T = self.T_max
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
        sol = solve_ivp(
            lambda t, S: self.dynamics(t, S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
        )
        return sol


Apollo = Lander(np.array([r0, dr0, theta0, dtheta0, 15200]), 311, 45000, 4280)

sol = Apollo.propagate(Apollo.S, t_total)
