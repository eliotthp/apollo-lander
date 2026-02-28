import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# Dynamics of system
def update_state(u, dt, x, v):
    x = x + v * dt
    v = v + u * dt - 10 * dt
    return x, v


def run_sim(u_vec, dt, t_span):
    x = 0
    v = -20
    x_hist = np.zeros(len(t_span))
    for i in range(len(t_span)):
        x_hist[i] = x
        x, v = update_state(u_vec[i], dt, x, v)
    xT = x
    vT = v
    return xT, vT, x_hist


def objective(u_vec, dt, t_span, rho):
    effort = 0.0
    xT, vT, x_hist = run_sim(u_vec, dt, t_span)
    effort = np.sum(u_vec**2) * dt
    J = effort + rho * ((xT - 40) ** 2 + (vT + 9) ** 2)
    return J


def main():
    # Parameters
    dt = 0.1
    t_span = np.arange(0, 10 + dt, dt)
    rho = 1e5
    # Optimize cost function
    results = minimize(
        objective, x0=np.zeros(len(t_span)), args=(dt, t_span, rho), tol=1e-6
    )
    # Plot optimal effort
    u_vec = results.x
    xT, vT, x_hist = run_sim(u_vec, dt, t_span)

    plt.plot(t_span, x_hist, "b", label="Trajectory")
    plt.grid(True)
    plt.plot(t_span, u_vec, "r", label="Effort")
    plt.title("Optimal Effort")
    plt.ylabel("Altitude (m)")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
