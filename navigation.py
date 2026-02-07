import environment as env

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu


def polar_to_LVLH(S):
    """
    Converts polar coordinates to Local Vertical Local Horizontal (LVLH) coordinates.

    Args:
        S (list): Current state vector [r, dr, theta, dtheta, m].

    Returns:
        LVLH (list): LVLH state vector [z, dz, x, dx].
    """
    # Unpack state
    r, dr, theta, dtheta, m = S
    # Project to LVLH
    z = r_moon - r
    dz = -dr
    x = r_moon * theta
    dx = r_moon * dtheta
    # Pack new state
    LVLH = [z, dz, x, dx, m]
    return LVLH


def altitude(LVLH):
    """
    Calculates the altitude above the lunar surface from LVLH coordinates.

    Args:
        LVLH (list): LVLH state vector [z, dz, x, dx].

    Returns:
        h (float): Altitude in meters.
    """
    z, _, _, _, _ = LVLH
    h = -z
    return h
