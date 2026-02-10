import numpy as np


class Staging:
    def __init__(self, stage):
        # stage: current staging level, targets: staging targets (z, dz, x, dx, t_stage)
        self.stage = stage
        self.x_hold = None
        self.t_elapsed = 0

    def get_guidance_targets(self, dt, LVLH):
        # Check stage then return target
        self.stage = self.check_stage(LVLH)
        if self.stage == 1:
            targets, t_stage = (2290, -44.2, 478_000, 0), 600
        elif self.stage == 2:
            targets, t_stage = (156, -4.9, 479_000, 0), 50
        elif self.stage == 3:
            targets, t_stage = (0, 0, self.x_hold, 0), 50
        t_go = max(t_stage - self.t_elapsed, dt)
        self.step(dt)
        return targets, t_go, self.stage

    def check_stage(self, LVLH):
        z, dz, x, dx, m = LVLH
        # Update staging only downwards
        if self.stage == 1 and z <= 2290:
            self.stage = 2
            self.t_elapsed = 0
        elif self.stage == 2 and abs(dx) <= 5:
            self.stage = 3
            self.t_elapsed = 0
            self.x_hold = x
        return self.stage

    def step(self, dt):
        self.t_elapsed += dt


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
