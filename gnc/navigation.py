import config as env

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu


def polar_to_LVLH(S):
    """
    Converts polar coordinates to Local Vertical Local Horizontal (LVLH) coordinates.
    With z (altitude, +up), and x (horizontal distance, +downrange)

    Args:
        S (list): Current state vector [r, dr, theta, dtheta, m].

    Returns:
        LVLH (list): LVLH state vector [z, dz, x, dx, m].
    """
    # Unpack global polar state: radius, radial velocity, angle, angular velocity, mass
    r, dr, theta, dtheta, m = S
    # Project to LVLH: z is altitude above surface, x is arc-length downrange
    z = r - r_moon  # Altitude (m)
    dz = dr  # Vertical velocity (m/s)
    x = r * theta  # Downrange distance (m)
    dx = r * dtheta  # Horizontal velocity (m/s)
    # Pack new state for use in guidance and control loops
    LVLH = [z, dz, x, dx, m]
    return LVLH
