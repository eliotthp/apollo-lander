import numpy as np
from states import PolarState, ControlState
from config import Config


class Simulation:
    def __init__(self, config: Config, inital_state: PolarState) -> None:
        self.cfg = config
        self.state = inital_state

    def step(self, dt: float, control: ControlState) -> None:
        self._rk4(dt, control)

    def _get_derivatives(self, state: PolarState, control: ControlState) -> list[float]:
        # Equations of Motion in polar coordinates
        ddr = (
            control.T_ctrl / state.m * np.cos(control.alpha_ctrl)
            - self.cfg.mu / state.r**2
            + state.r * state.dtheta**2
        )
        ddtheta = (
            1
            / state.r
            * (
                (control.T_ctrl / state.m) * np.sin(control.alpha_ctrl)
                - 2 * state.dr * state.dtheta
            )
        )

        # Update velocities
        dr = state.dr
        dtheta = state.dtheta

        # Mass flow rate based on ideal rocket equation
        dm = -control.T_ctrl / (self.cfg.Isp * self.cfg.G_earth)

        return [dr, ddr, dtheta, ddtheta, dm]

    def _rk4(self, dt: float, control: ControlState) -> None:
        x0 = [
            self.state.r,
            self.state.dr,
            self.state.theta,
            self.state.dtheta,
            self.state.m,
        ]
        k1 = self._get_derivatives(PolarState(*x0), control)
        x_k2 = [x0[i] + (dt / 2) * k1[i] for i in range(len(x0))]
        temp = PolarState(*x_k2)
        k2 = self._get_derivatives(temp, control)
        x_k3 = [x0[i] + (dt / 2) * k2[i] for i in range(len(x0))]
        temp = PolarState(*x_k3)
        k3 = self._get_derivatives(temp, control)
        x_k4 = [x0[i] + dt * k3[i] for i in range(len(x0))]
        temp = PolarState(*x_k4)
        k4 = self._get_derivatives(temp, control)

        x_next = [
            x0[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
            for i in range(len(x0))
        ]

        self.state = PolarState(x_next[0], x_next[1], x_next[2], x_next[3], x_next[4])
