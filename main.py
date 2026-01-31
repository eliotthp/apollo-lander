import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 1.625  # m/s^2
G0 = 9.81  # m/s^2
t_total = 500  # s


class Lander:
    def __init__(self, S, Isp, T, m_e):
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e
        self.m_p = S[2] - m_e

    def controller(self, S):
        T_min = self.T * 0.2
        T_max = self.T * 0.6
        target_y = 0
        target_v = 0

        Kp = 0.5
        Kd = 1

        u = Kp * (target_y - S[0]) + Kd * (target_v - S[1])
        return np.clip(u, T_min, T_max)

    def dynamics(self, S):
        m = S[2]
        # State Space Matrices
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1 / m]])
        f = np.array([0, -G])
        C = np.array([1, 0])
        # Input
        u = self.controller(S)
        # Dynamics
        x_dot = A @ S[0:2] + (B @ np.array([u])).flatten() + f
        # Mass
        m_dot = -u / (G0 * self.Isp)
        return [x_dot[0], x_dot[1], m_dot]

    def propagate(self, S, duration):
        sol = solve_ivp(
            lambda t, S: self.dynamics(S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
        )
        return sol


Apollo = Lander(np.array([1000, -10, 15200]), 311, 45000, 4280)

# Plotting
sol = Apollo.propagate(Apollo.S, t_total)
fig, ax = plt.subplots()
ax.plot(sol.t, sol.y[0], label="Altitude (km)", color="b")
ax.plot(sol.t, -sol.y[1], label="Velocity (km/s)", color="g")
ax.set_xlabel("Time (s)")
ax.set_ylabel("")
ax2 = ax.twinx()
ax2.plot(sol.t, sol.y[2], label="Mass (kg)", color="r")
ax2.set_ylim([0, Apollo.S[2]])
ax2.set_ylabel("")
ax2.set_title("Apollo Lander Landing on Moon")
ax.set_xlim([0, t_total])
ax.set_ylim([0, np.max([np.max(sol.y[0]), np.max(-sol.y[1])])])
plt.legend()
plt.show()
