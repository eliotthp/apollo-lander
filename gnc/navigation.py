from state import LVLHState, PolarState
import numpy as np


class Navigation:
    def __init__(self, config, bias, seed):
        self.cfg = config
        self.bias = bias
        self.rng = np.random.default_rng(seed)

    def step(self, polar_state: PolarState):
        self.LVLH_state = self._polar_to_LVLH(polar_state)
        z_meas = self._measure(self.LVLH_state.z)
        return LVLHState(
            z_meas,
            self.LVLH_state.dz,
            self.LVLH_state.x,
            self.LVLH_state.dx,
            self.LVLH_state.m,
        )

    def _measure(self, z_true):
        # Calculate standard deviation based on Apollo-like landing radar specs
        σ_z = (0.015 * z_true + 1.52) / 3  # 1σ noise
        n = self.rng.normal(0, σ_z)  # One noise sample
        z_meas = z_true + self.bias + n
        return z_meas

    def _polar_to_LVLH(self, polar_state):
        """
        Converts polar coordinates to Local Vertical Local Horizontal (LVLH) coordinates.
        With z (altitude, +up), and x (horizontal distance, +downrange)

        Args:
            state (list): Current state vector [r, dr, theta, dtheta, m].

        Returns:
            LVLH (list): LVLH state vector [z, dz, x, dx, m].
        """
        # Project to LVLH: z is altitude above surface, x is arc-length downrange
        z = polar_state.r - self.cfg.r_moon  # Altitude (m)
        dz = polar_state.dr  # Vertical velocity (m/s)
        x = polar_state.r * polar_state.theta  # Downrange distance (m)
        dx = polar_state.r * polar_state.dtheta  # Horizontal velocity (m/s)
        m = polar_state.m  # Mass (kg)
        return LVLHState(z, dz, x, dx, m)
