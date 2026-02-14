import config as cfg
from state import State
from sim.simulation import Simulation

# Create initial state object
S0 = State(
    r=14_878 + cfg.r_moon,
    dr=0,
    theta=0,
    dtheta=0,
    m=cfg.m0,
)

# Pass the State object into Simulation
simulation = Simulation(cfg, S0)

for i in range(10):
    simulation.step([0, 0], 1)

print(int(simulation.state.dr))
