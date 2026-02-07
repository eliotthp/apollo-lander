import environment as env

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
        LVLH (list): LVLH state vector [z, dz, x, dx].
    """
    # Unpack state
    r, dr, theta, dtheta, m = S
    # Project to LVLH
    z = r - r_moon
    dz = dr
    x = r * theta
    dx = r * dtheta
    # Pack new state
    LVLH = [z, dz, x, dx, m]
    return LVLH
