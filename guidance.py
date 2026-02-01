import numpy as np


def poly_guidance(t, S, tf):
    """
    Returns the reference position, velocity, and acceleration
    for a cubic trajectory.
    """
    # Coefficients of the polynomial
    a_mat = np.array(
        [[0, 0, 0, 1], [tf**3, tf**2, tf, 1], [0, 0, 1, 0], [3 * tf**2, 2 * tf, 1, 0]]
    )
    b_vec = S

    # Solving for coefficients a, b, c, d
    coeffs = np.linalg.solve(a_mat, b_vec)
    a, b, c, d = coeffs

    f = a * t**3 + b * t**2 + c * t + d
    df = 3 * a * t**2 + 2 * b * t + c
    ddf = 6 * a * t + 2 * b

    return f, df, ddf
