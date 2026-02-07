import numpy as np
import environment as env

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu


def control(t, S, targets):
    # Unpack states
    z, dz, x, dx, m = S
    ddz_cmd, ddx_cmd = targets
    # Calculate distance from moon center
    r = r_moon + z
    dtheta = dx / r
    # Components of thrust req
    Tz = ddz_cmd + (mu / r**2) - (r * dtheta**2)
    Tx = ddx_cmd + (2 * dz * dtheta)
    # Calculate control inputs
    alpha_cmd = np.arctan2(Tx, Tz)
    T_cmd = m * np.sqrt(Tx**2 + Tz**2)

    return T_cmd, alpha_cmd
