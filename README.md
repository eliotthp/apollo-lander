# Apollo Lunar Module Powered Descent GNC Simulation

Python-based closed-loop simulation of an Apollo-style lunar powered descent.

The goal was not to recreate Apollo perfectly, but to build a modular navigation–guidance–control architecture and explore how guidance laws, actuator limits, sensor imperfections, and mass depletion interact in a realistic descent problem.

---

## What This Models

- 2D lunar translational dynamics (polar coordinates)
- Continuous mass depletion from thrust
- LVLH frame navigation
- Polynomial boundary-condition guidance
- Throttle limits and gimbal rate limits
- Discrete GNC loop driving a continuous plant
- Altitude measurement with injected sensor noise

---

## Architecture

The code is intentionally separated:

- `sim/` — truth dynamics (r, θ, m)
- `gnc/navigation.py` — LVLH state mapping with noisy altitude measurement
- `gnc/guidance.py` — acceleration command generation
- `gnc/control.py` — thrust + pitch allocation with limits
- `telemetry.py` — logging and plotting

The structure mirrors real flight software partitioning rather than a single-script trajectory solver.

---

## Telemetry

![Telemetry Plot](figs/telemetry.png)

Outputs include:

- Altitude and vertical velocity
- Horizontal velocity
- Commanded vs actual thrust
- Commanded vs actual pitch
- Remaining propellant mass
- Radar (noisy) vs true altitude comparison

Termination reports:

- Time of flight  
- Impact velocity  
- Delta-V used  
- Remaining propellant  

---

## Key Technical Ideas Explored

- Frame discipline (operate GNC in LVLH, not polar)
- Solving cubic boundary-value guidance each cycle
- Handling actuator saturation and rate limits
- Mass depletion coupling into dynamics
- Separation between plant and control timing
- Effects of altitude sensor noise on guidance performance
- Telemetry-driven validation instead of eyeballing trajectories

---

## Current Limitations

- Translational dynamics only (no attitude dynamics)
- Single-axis sensor noise (altitude only)
- No full state estimator (no Kalman filter)
- No terrain model
- Simplified thrust vector geometry

---

## References

- NASA Manned Spacecraft Center, *Apollo Lunar Descent and Ascent Trajectories*  
  https://ntrs.nasa.gov/api/citations/19700024568/downloads/19700024568.pdf  

- *Mission Planning for Lunar Module Descent and Ascent*  
  https://ntrs.nasa.gov/api/citations/19720018205/downloads/19720018205.pdf  

- Apollo 11 Mission Report  
  https://sma.nasa.gov/SignificantIncidents/assets/a11_missionreport.pdf  

---

## Status

Active development. Planned next steps:

- Add full state estimation (EKF)
- Monte Carlo dispersions with sensor noise
- Higher-order integration refinement
- Hardware-in-the-loop experimentation
