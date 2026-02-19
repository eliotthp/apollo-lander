import numpy as np
from states import PolarState, ControlState


class Simulation:
    def __init__(self, config, inital_state: PolarState):
        self.cfg = config
        self.state = inital_state

    def step(self, dt, control):
        dstate = self._get_derivatives(control)
        self._euler(dstate, dt)

    def _get_derivatives(self, control: ControlState):
        # Cut thrust if propellant is exhausted
        if self.state.m - self.cfg.m_empty <= 0:
            control.T_ctrl = 0
        # Equations of Motion in polar coordinates
        ddr = (
            control.T_ctrl / self.state.m * np.cos(control.alpha_ctrl)
            - self.cfg.mu / self.state.r**2
            + self.state.r * self.state.dtheta**2
        )
        ddtheta = (
            1
            / self.state.r
            * (
                (control.T_ctrl / self.state.m) * np.sin(control.alpha_ctrl)
                - 2 * self.state.dr * self.state.dtheta
            )
        )
        # Mass flow rate based on ideal rocket equation
        dm = -control.T_ctrl / (self.cfg.Isp * self.cfg.G_earth)

        return [ddr, ddtheta, dm]

    def _euler(self, dstate, dt):
        # Unpack derivatives
        ddr, ddtheta, dm = dstate
        # Update velocities
        self.state.dr += ddr * dt
        self.state.dtheta += ddtheta * dt
        # Update positions
        self.state.r += self.state.dr * dt
        self.state.theta += self.state.dtheta * dt
        # Update mass
        self.state.m += dm * dt
