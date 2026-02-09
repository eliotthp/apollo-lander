import numpy as np


def poly_guidance(t, S, tf):
    """
    Returns the reference position, velocity, and acceleration
    for a cubic trajectory.

    Args:
        t (float or ndarray): Current time(s).
        S (list): Desired conditions [f(0), f(tf), df(0), df(tf)].
        tf (float): Final time.

    Returns:
        tuple: (f, df, ddf) Position, velocity, and acceleration at time t.
    """
    # Set up the system of equations: M * coeffs = [f(0), f(tf), df(0), df(tf)]
    # The matrix represents the polynomial terms at t=0 and t=tf
    a_mat = np.array(
        [[0, 0, 0, 1], [tf**3, tf**2, tf, 1], [0, 0, 1, 0], [3 * tf**2, 2 * tf, 1, 0]]
    )
    b_vec = S

    # Solve for the polynomial coefficients [a, b, c, d]
    coeffs = np.linalg.solve(a_mat, b_vec)
    a, b, c, d = coeffs

    # Calculate position, velocity, and acceleration based on the cubic form
    f = a * t**3 + b * t**2 + c * t + d
    df = 3 * a * t**2 + 2 * b * t + c
    ddf = 6 * a * t + 2 * b

    return f, df, ddf
