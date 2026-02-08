import numpy as np
import environment as env

# --- Constants & Environment ---
mu = env.mu
Isp = env.Isp
G_earth = env.G_earth
m_empty = env.m_empty


def get_derivatives(S, C):
    """
    Calculates the second-order derivatives of the state variables.

    Args:
        S (list): Current state vector [r, dr, theta, dtheta, m].
        C (list): Control inputs [T, alpha] (Thrust and Pitch).

    Returns:
        list: Derivatives [ddr, ddtheta].
    """
    # Unpack state
    r, dr, theta, dtheta, m = S
    T, alpha = C
    if m - m_empty <= 0:
        T = 0
    # Equations of Motion
    ddr = T / m * np.cos(alpha) - mu / r**2 + r * dtheta**2
    ddtheta = 1 / r * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)
    # Propellant Change
    dm = -T / (Isp * G_earth)

    return [ddr, ddtheta, dm]


def update_state(dt, S, dS):
    """
    Updates the state vector using Euler integration.

    Args:
        dt (float): Time step for the update.
        S (list): Current state vector [r, dr, theta, dtheta, m].
        dS (list): Derivatives [ddr, ddtheta].

    Returns:
        list: Updated state vector.
    """
    # Unpack states
    r, dr, theta, dtheta, m = S
    ddr, ddtheta, dm = dS
    # Update velocities
    dr += ddr * dt
    dtheta += ddtheta * dt
    # Update positions
    r += dr * dt
    theta += dtheta * dt
    # Update mass
    m += dm * dt

    return [r, dr, theta, dtheta, m]


def propagate(h, dt, S, C):
    """
    Propagates the state forward in time over a duration dt using a smaller step h.

    Args:
        h (float): Integration time step.
        dt (float): Total propagation duration.
        S (list): Initial state vector.
        C (list): Constant control inputs for this duration.

    Returns:
        list: Final state vector after duration dt.
    """
    # Ensure that n is exactly 100
    n = int(round(dt / h))
    for _ in range(n):
        dS = get_derivatives(S, C)
        S = update_state(h, S, dS)

    return S
