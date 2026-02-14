from dataclasses import dataclass


@dataclass
class State:
    x: float
    y: float
    z: float
    dx: float
    dy: float
    dz: float
    m: float
