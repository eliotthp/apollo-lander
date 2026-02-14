from dataclasses import dataclass


@dataclass
class PolarState:
    r: float
    theta: float
    dr: float
    dtheta: float
    m: float


@dataclass
class LVLHState:
    z: float
    dz: float
    x: float
    dx: float
    m: float
