import numpy as np
from states import GuidanceState, LVLHState


class Guidance:
    def __init__(self) -> None:
        # stage: current staging level, targets: staging targets (z, dz, x, dx, t_stage)
        self.guidance_state = GuidanceState(0, 0, 0, 0, 0, 0, 1, 0, 0)
        self.x_hold = None

    def step(self, dt: float, LVLH: LVLHState) -> GuidanceState:
        self.guidance_state.t_elapsed += dt
        # Check stage
        self._check_stage(LVLH)
        # Get targets
        self._get_guidance_targets(dt, LVLH)
        # Set guidance
        t_go = max(self.guidance_state.t_stage - self.guidance_state.t_elapsed, dt)
        _, _, self.guidance_state.ddz = self._poly_guidance(
            0.0,
            t_go,
            LVLH.z,
            self.guidance_state.z,
            LVLH.dz,
            self.guidance_state.dz,
        )
        _, _, self.guidance_state.ddx = self._poly_guidance(
            0.0,
            t_go,
            LVLH.x,
            self.guidance_state.x,
            LVLH.dx,
            self.guidance_state.dx,
        )

        return self.guidance_state

    def _check_stage(self, LVLH: LVLHState) -> None:
        # Braking
        if self.guidance_state.stage == 1 and LVLH.z <= 2_500:
            # Approach
            print(f"Approach Stage after {self.guidance_state.t_elapsed:.2f} s")
            self.guidance_state.stage = 2
            self.guidance_state.t_elapsed = 0
        elif self.guidance_state.stage == 2 and LVLH.z <= 150:
            # Final Phase
            print(f"Final Stage after {self.guidance_state.t_elapsed:.2f} s")
            self.guidance_state.stage = 3
            self.guidance_state.t_elapsed = 0
            self.x_hold = LVLH.x

    def _get_guidance_targets(self, dt: float, LVLH: LVLHState) -> None:
        if self.guidance_state.stage == 1:
            # Braking
            self.guidance_state.z = 0
            self.guidance_state.dz = -50
            self.guidance_state.x = 480_000
            self.guidance_state.dx = 0
            self.guidance_state.t_stage = 660
        elif self.guidance_state.stage == 2:
            # Approach
            self.guidance_state.z = 0
            self.guidance_state.dz = 0
            self.guidance_state.x = 480_000
            self.guidance_state.dx = 0
            self.guidance_state.t_stage = 160
        elif self.guidance_state.stage == 3:
            # Final Phase
            self.guidance_state.z = 0
            self.guidance_state.dz = 0
            self.guidance_state.x = 480_000
            self.guidance_state.dx = 0
            self.guidance_state.t_stage = 120

    def _poly_guidance(
        self, t: float, tf: float, f0: float, ff: float, df0: float, dff: float
    ) -> tuple[float, float, float]:
        # Set up the system of equations: M * coeffs = [f(0), f(tf), df(0), df(tf)]
        # The matrix represents the polynomial terms at t=0 and t=tf
        a_mat = np.array(
            [
                [0, 0, 0, 1],
                [tf**3, tf**2, tf, 1],
                [0, 0, 1, 0],
                [3 * tf**2, 2 * tf, 1, 0],
            ]
        )
        b_vec = [f0, ff, df0, dff]

        # Solve for the polynomial coefficients [a, b, c, d]
        coeffs = np.linalg.solve(a_mat, b_vec)
        a, b, c, d = coeffs

        # Calculate position, velocity, and acceleration based on the cubic form
        f = a * t**3 + b * t**2 + c * t + d
        df = 3 * a * t**2 + 2 * b * t + c
        ddf = 6 * a * t + 2 * b

        return f, df, ddf
