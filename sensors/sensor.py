import numpy as np


class altimeter:
    """Simulates a radar altimeter with constant bias and altitude-dependent noise."""

    def __init__(self, bias, seed):
        # bias: constant measurement offset, seed: random number generator seed
        self.bias = bias
        self.rng = np.random.default_rng(seed)

    def measure(self, z_true):
        # Calculate standard deviation based on Apollo-like landing radar specs
        σ_z = (0.015 * z_true + 1.52) / 3  # 1σ noise
        n = self.rng.normal(0, σ_z)  # One noise sample
        z_meas = z_true + self.bias + n
        return z_meas


class filter:
    """Implements signal processing filters for sensor data smoothing."""

    def __init__(self, dt, tau, x0=0):
        # dt: sampling period, tau: filter time constant
        self.alpha = dt / (tau + dt)
        self.x_prev = x0

    def apply(self, x_meas):
        # Discrete-time first-order low-pass filter
        self.x_prev = self.alpha * x_meas + (1 - self.alpha) * self.x_prev
        return self.x_prev
