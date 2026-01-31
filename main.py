import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Simulation Constants
G = 1.625  # m/s^2
G0 = 9.81  # m/s^2
t_total = 1000  # s

x_start = 0  # m
v_x_start = 1  # m/s
y_start = 1000  # m
v_y_start = 0  # m/s

target_x = 3000  # m
target_v_x = 0  # m/s
target_y = 0  # m
target_v_y = 0  # m/s


class Lander:
    def __init__(self, S, Isp, T, m_e):
        """
        Initialize the Lander object

        :param self: The Lander object
        :param S: Initial state of the lander (numpy array of 5 elements: x, v_x, y, v_y, m)
        :param Isp: Specific impulse of the lander (s)
        :param T: Maximum thrust of the lander (N)
        :param m_e: Empty mass of the lander (kg)
        """
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e

    def controller(self, S):
        m = S[4]
        T_min = self.T * 0.2
        T_max = self.T * 0.6

        # Horizontal PD Controller
        Kp_x, Kd_x = 0.3489, 87.158
        # Vertical PD Controller
        Kp_y, Kd_y = 0.41, 99

        u_x = Kp_x * (target_x - S[0]) + Kd_x * (target_v_x - S[1])
        u_y = m * G + Kp_y * (target_y - S[2]) + Kd_y * (target_v_y - S[3])
        u = np.array([u_x, u_y])
        mag = np.linalg.norm(u)
        if mag > T_max:
            u = u / mag * T_max
        elif mag < T_min:
            u = u / mag * T_min
        return u

    def dynamics(self, t, S):
        x, v_x, y, v_y, m = S

        # State Space Matrices
        A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
        B = np.array([[0, 0], [1 / m, 0], [0, 0], [0, 1 / m]])
        f = np.array([0, 0, 0, -G])

        # Landed
        if y <= 0:
            return [0, 0, 0, 0, 0]
        # Out of fuel
        if S[4] <= self.m_e:
            u = [0, 0]
        else:
            # Input
            u = self.controller(S)
        # Dynamics
        x_dot = A @ np.array([x, v_x, y, v_y]) + (B @ u).flatten() + f
        # Mass
        m_dot = -np.linalg.norm(u) / (G0 * self.Isp)
        return [*x_dot, m_dot]

    def propagate(self, S, duration):
        sol = solve_ivp(
            lambda t, S: self.dynamics(t, S),
            (0, duration),
            self.S,
            method="RK45",
            t_eval=np.linspace(0, duration, duration * 10),
        )
        return sol


Apollo = Lander(
    np.array([x_start, v_x_start, y_start, v_y_start, 15200]), 311, 45000, 4280
)

sol = Apollo.propagate(Apollo.S, t_total)
landed_idx = np.where(sol.y[2] <= 0)[0]
if landed_idx.size > 0:
    first_touchdown = landed_idx[0]
    landed_x = sol.y[0][first_touchdown]
    landed_v_x = sol.y[1][first_touchdown]
    landed_y = sol.y[2][first_touchdown]
    landed_v_y = sol.y[3][first_touchdown]
    landed_time = sol.t[first_touchdown]
print(
    f"Landed! Distance: {landed_x:.2f} m | X_Velocity: {landed_v_x:.2f} m/s | Y_Velocity: {landed_v_y:.2f} m/s | Time: {landed_time:.2f} s"
)

# Trajectory Plot
plt.figure(figsize=(10, 5))
plt.plot(sol.y[0], sol.y[2], color="purple", label="Flight Path")
plt.axhline(0, color="black", linestyle="--")  # The Ground
plt.scatter(target_x, 0, color="red", marker="X", label="Target")  # The Target
plt.xlabel("Horizontal Distance (m)")
plt.ylabel("Altitude (m)")
plt.title("2D Lunar Landing Trajectory")
plt.legend()
plt.grid(True)
plt.show()
