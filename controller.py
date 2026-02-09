import numpy as np
import environment as env

# --- Constants & Environment ---
G_earth = env.G_earth
r_moon = env.r_moon
mu = env.mu
m_empty = env.m_empty
T_max = env.T_max
dalpha_max = env.dalpha_max


def control(t, S, targets):
    """
    Calculates the required thrust and pitch angle to achieve commanded accelerations.
    With z (altitude, +up), and x (horizontal distance, +downrange)

    Args:
        t (float): Current simulation time.
        S (list): Current LVLH state vector [z, dz, x, dx, m].
        targets (list): Commanded accelerations [ddz_cmd, ddx_cmd].

    Returns:
        tuple: (T_cmd, alpha_cmd) Commanded thrust (N) and pitch angle (rad).
    """
    # Unpack states
    z, dz, x, dx, m = S
    ddz_cmd, ddx_cmd = targets

    # Calculate distance from moon center and angular velocity
    r = r_moon + z
    dtheta = dx / r

    # Determine required thrust components in the LVLH frame
    # Compensates for gravity and centrifugal/coriolis effects
    Tz = ddz_cmd + (mu / r**2) - (r * dtheta**2)
    Tx = ddx_cmd + (2 * dz * dtheta)

    # Convert required force components to vehicle thrust and pitch angle
    if m > m_empty:
        alpha_cmd = np.arctan2(Tx, Tz)
        T_cmd = m * np.sqrt(Tx**2 + Tz**2)
    else:
        alpha_cmd = 0
        T_cmd = 0

    return T_cmd, alpha_cmd


def thrust_limiter(T_cmd):
    """
    Limits the commanded thrust based on the Apollo Descent Propulsion System (DPS)
    constraints: either fixed at 100% or throttleable between 10% and 65%.

    Args:
        T_cmd (float): The raw commanded thrust from the controller.

    Returns:
        T_ctrl (float): The actual thrust (N) after applying hardware limits.
    """
    throttle = T_cmd / T_max * 100  # Throttle (%)
    # Apollo DPS cannot throttle between 65% and 100% reliably
    if throttle >= 65:
        T_ctrl = T_max
    elif throttle >= 10:
        T_ctrl = T_max * throttle / 100
    else:
        T_ctrl = T_max * 0.1
    return T_ctrl


def slew_limiter(dt, alpha_cmd, alpha_ctrl):
    """
    Limits the rate of change of the pitch angle to simulate gimbal/slew rate limits.

    Args:
        dt (float): Time step (s).
        alpha_cmd (float): The target pitch angle (rad) from the controller.
        alpha_ctrl (float): The current pitch angle (rad) from the previous step.

    Returns:
        alpha_ctrl (float): The new pitch angle (rad) after applying the slew rate limit.
    """
    # Calculate the desired change in angle
    dalpha = -(alpha_ctrl - alpha_cmd)
    delta_alpha_max = dalpha_max * dt

    # Apply slew rate limit
    if abs(dalpha) > delta_alpha_max:
        alpha_ctrl += np.sign(dalpha) * delta_alpha_max
    else:
        alpha_ctrl = alpha_cmd
    return alpha_ctrl
