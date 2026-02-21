import numpy as np
from states import LVLHState, PolarState
from config import Config


class Navigation:
    def __init__(self, config: Config, bias: float, seed: int):
        self.cfg = config
        self.bias = bias
        self.rng = np.random.default_rng(seed)
        self.z_filtered = self.cfg.S0.r - self.cfg.r_moon
        self.dz_filtered = 0.0
        self.alpha = 0.02
        self.beta = 0.0004

    def step(self, dt, polar_state: PolarState) -> LVLHState:
        self.LVLH_state = self._polar_to_LVLH(polar_state)
        # Measure with instruments
        self.z_meas = self._measure(dt, self.LVLH_state.z)
        # Predict
        self.z_predict, self.dz_predict = self._predict(dt)
        # Filter out noise
        self.z_filtered, self.dz_filtered = self._alpha_beta_filter(dt)
        # Return estimated state
        return LVLHState(
            self.z_filtered,
            self.dz_filtered,
            self.LVLH_state.x,
            self.LVLH_state.dx,
            self.LVLH_state.m,
        )

    def _measure(self, dt: float, z_true: float) -> float:
        z_meas = self._altitude_sensor(z_true)
        return z_meas

    def _predict(self, dt: float) -> float:
        z_predict = self.z_filtered + self.dz_filtered * dt
        dz_predict = self.dz_filtered
        return z_predict, dz_predict

    def _alpha_beta_filter(self, dt: float) -> float:
        z_filtered = self.z_predict + self.alpha * (self.z_meas - self.z_predict)
        dz_filtered = self.dz_predict + self.beta * (self.z_meas - self.z_predict) / dt
        return z_filtered, dz_filtered

    def _altitude_sensor(self, z_true: float) -> float:
        # Calculate standard deviation based on Apollo-like landing radar specs
        o_z = (0.015 * z_true + 1.52) / 3  # 1σ noise
        n = self.rng.normal(0, o_z)  # One noise sample
        z_meas = z_true + self.bias + n
        return z_meas

    def _polar_to_LVLH(self, polar_state: PolarState) -> LVLHState:
        # Project to LVLH: z is altitude above surface, x is arc-length downrange
        z = polar_state.r - self.cfg.r_moon  # Altitude (m)
        dz = polar_state.dr  # Vertical velocity (m/s)
        x = polar_state.r * polar_state.theta  # Downrange distance (m)
        dx = polar_state.r * polar_state.dtheta  # Horizontal velocity (m/s)
        m = polar_state.m  # Mass (kg)
        return LVLHState(z, dz, x, dx, m)
