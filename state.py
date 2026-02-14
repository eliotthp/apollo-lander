from dataclasses import dataclass


@dataclass
class State:
    r: float
    theta: float
    dr: float
    dtheta: float
    m: float
