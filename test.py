import config as cfg
from state import PolarState
from sim.simulation import Simulation
from gnc.navigation import Navigation

# Create initial state object
S0 = PolarState(
    r=14_878 + cfg.r_moon,
    dr=0,
    theta=0,
    dtheta=0,
    m=cfg.m0,
)

# Pass the State object into Simulation
simulation = Simulation(cfg, S0)
navigation = Navigation(cfg, 1, 42)
for i in range(20):
    navigation.step(simulation.state)
    simulation.step([0, 0], 1)

print(simulation.state.r - cfg.r_moon)
print(navigation.LVLH_state.z)
