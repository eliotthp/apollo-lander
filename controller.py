import guidance
import numpy as np
import enviroment as env

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu


def control(t, S):
    # Unpack State
    r, dr, theta, dtheta, m, alpha = S
    g = mu / r**2

    # Convert to LVLH
    z = r - r_moon
    dz = dr
    x = r_moon * theta
    dx = r_moon * dtheta

    # Braking Phase
    if z > 2346.96:
        tf = 500

        # Guidance
        _, _, ddz_cmd = guidance.poly_guidance(0, [z, 2346.96, dz, -44.20], tf)
        _, _, ddx_cmd = guidance.poly_guidance(0, [x, 400_000, dx, 44.20], tf)
    else:
        ddz_cmd = 0
        ddx_cmd = 0

    T_cmd = m * np.sqrt((ddz_cmd + g) ** 2 + ddx_cmd**2)
    alpha_cmd = np.arctan2(ddx_cmd, ddz_cmd + g)

    return T_cmd, alpha_cmd
