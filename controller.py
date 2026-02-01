import guidance
import numpy as np
import environment as env

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu


def control(t, S, targets):
    """
    Calculates the required thrust and pitch angle to follow a guided trajectory.

    Args:
        t (float): Current simulation time.
        S (list): Current state vector [r, dr, theta, dtheta, m, alpha].
        targets (list): Target conditions [zf, dzf, xf, dxf].

    Returns:
        tuple: (T_cmd, alpha_cmd) Commanded thrust in Newtons and pitch angle in radians.
    """
    # Unpack State
    r, dr, theta, dtheta, m, alpha = S
    zf, dzf, xf, dxf = targets
    g = mu / r**2

    # Convert to LVLH
    z = r - r_moon
    dz = dr
    x = r_moon * theta
    dx = r_moon * dtheta

    # Constraints
    alpha_limit = np.deg2rad(5)  # rad/s

    # Braking Phase
    if z > 2346.96:
        tf = 500

        # Guidance
        _, _, ddz_cmd = guidance.poly_guidance(0, [z, zf, dz, dzf], tf)
        _, _, ddx_cmd = guidance.poly_guidance(0, [x, xf, dx, dxf], tf)
    else:
        return 0, 0

    T_cmd = m * np.sqrt((ddz_cmd + g) ** 2 + ddx_cmd**2)
    alpha_cmd = np.arctan2(ddx_cmd, ddz_cmd + g)

    # Attitude rate limit
    if alpha_cmd > alpha_limit:
        alpha_cmd = alpha_limit
    elif alpha_cmd < -alpha_limit:
        alpha_cmd = -alpha_limit

    return T_cmd, alpha_cmd
