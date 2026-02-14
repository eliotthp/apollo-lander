import numpy as np
from state import State


class Simulation:
    def __init__(self, config, inital_state: State):
        self.cfg = config
        self.state = inital_state

    def step(self, control, dt):
        dstate = self._get_derivatives(control)
        self.state = self._euler(self.state, dstate, dt)
        return self.state

    def _get_derivatives(self, control):
        # Unpack state
        r = self.state.r
        dr = self.state.dr
        theta = self.state.theta
        dtheta = self.state.dtheta
        m = self.state.m
        T, alpha = control

        # Cut thrust if propellant is exhausted
        if m - self.cfg.m_empty <= 0:
            T = 0
        # Equations of Motion in polar coordinates
        ddr = T / m * np.cos(alpha) - self.cfg.mu / r**2 + r * dtheta**2
        ddtheta = 1 / r * ((T / m) * np.sin(alpha) - 2 * dr * dtheta)
        # Mass flow rate based on ideal rocket equation
        dm = -T / (self.cfg.Isp * self.cfg.G_earth)

        return [ddr, ddtheta, dm]

    def _euler(self, S, dS, dt):
        # Unpack states
        r = self.state.r
        dr = self.state.dr
        theta = self.state.theta
        dtheta = self.state.dtheta
        m = self.state.m

        ddr, ddtheta, dm = dS
        # Update velocities
        dr += ddr * dt
        dtheta += ddtheta * dt
        # Update positions
        r += dr * dt
        theta += dtheta * dt
        # Update mass
        m += dm * dt

        return State(r, dr, theta, dtheta, m)
