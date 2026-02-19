from states import LVLHState, ControlState, GuidanceState
from config import Config
import numpy as np


class Control:
    def __init__(self, config: Config, control_state: ControlState):
        self.cfg = config
        self.control_state = control_state

    def step(
        self, dt: float, nav_state: LVLHState, guid_state: GuidanceState
    ) -> ControlState:
        # Calculate distance from moon center and angular velocity
        r = self.cfg.r_moon + nav_state.z
        dtheta = nav_state.dx / r
        # Determine required thrust components in the LVLH frame
        # Compensates for gravity and centrifugal/coriolis effects
        Tz = guid_state.ddz + (self.cfg.mu / r**2) - (r * dtheta**2)
        Tx = guid_state.ddx + (2 * nav_state.dz * dtheta)

        self.control_state.alpha_cmd = np.arctan2(Tx, Tz)
        self.control_state.T_cmd = nav_state.m * np.sqrt(Tx**2 + Tz**2)

        # Thrust limiter
        self._thrust_limiter()

        # Slew rate limiter
        self._slew_limiter(dt)

        # Propellant limit
        self._propellant_limit(nav_state.m)

        return self.control_state

    def _propellant_limit(self, m: float):
        if m > self.cfg.m_empty:
            pass
        else:
            self.control_state.T_ctrl = 0
            self.control_state.alpha_ctrl = 0

    def _thrust_limiter(self):
        throttle = self.control_state.T_cmd / self.cfg.T_max * 100  # Throttle (%)
        # Apollo DPS cannot throttle between 65% and 100% reliably
        if throttle >= 65:
            self.control_state.T_ctrl = self.cfg.T_max
        elif throttle >= 10:
            self.control_state.T_ctrl = self.cfg.T_max * throttle / 100
        else:
            self.control_state.T_ctrl = self.cfg.T_max * 0.1

    def _slew_limiter(self, dt: float):
        # Calculate the desired change in angle
        dalpha = -(self.control_state.alpha_ctrl - self.control_state.alpha_cmd)
        delta_alpha_max = self.cfg.dalpha_max * dt

        # Apply slew rate limit
        if abs(dalpha) > delta_alpha_max:
            self.control_state.alpha_ctrl += np.sign(dalpha) * delta_alpha_max
        else:
            self.control_state.alpha_ctrl = self.control_state.alpha_cmd
