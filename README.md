# Apollo Lunar Module Descent Simulation

![Language](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Active_Development-orange)

## 🚀 Project Overview
This project is a high-fidelity, 3-Degree-of-Freedom (3-DOF) flight dynamics simulation of the **Apollo 11 Lunar Module (LM)** descent sequence. It models the critical **Braking Phase (P63)** and **Approach Phase (P64)** using a closed-loop Guidance, Navigation, and Control (GNC) architecture.

The simulation solves the non-linear equations of motion in a polar coordinate frame, accounting for the Moon's spherical gravity, centrifugal relief, and Coriolis effects. It implements a **Receding Horizon Polynomial Guidance** algorithm to generate optimal acceleration commands in real-time, subject to physical actuator constraints (Thrust-to-Weight ratios and Reaction Control System slew rates).

## 🛠 Engineering & Technical Highlights

### 1. Dynamics & Physics Engine
* **Spherical Mechanics:** Migrated from a flat-moon assumption to a full polar coordinate system $(r, \theta)$ to accurately model orbital mechanics, including centrifugal and Coriolis forces.
* **Numerical Integration:** Utilizes `scipy.integrate.solve_ivp` with the **Runge-Kutta 4(5)** method (RK45) for adaptive time-stepping and high-precision state propagation.
* **Mass Properties:** dynamic modeling of fuel depletion based on $I_{sp}$ (311 s) and throttle commands, affecting the vehicle's inertia and acceleration capability over time.

### 2. Guidance, Navigation, & Control (GNC)
* **Polynomial Guidance:** Implements a cubic spline guidance law that calculates required velocity and acceleration to meet boundary conditions (position/velocity) at a target time ($t_{go}$).
* **Receding Horizon Control:** Features a "shrinking horizon" logic where the guidance solution is re-solved at every time step based on the vehicle's current state deviation, ensuring convergence even with initial errors.
* **Multi-Phase Logic:** Automatically handles state transitions between the **Braking Phase** (high thrust, orbital velocity reduction) and **Approach Phase** (terminal descent).

### 3. Actuator & Hardware Constraints
* **Thrust Vector Control (TVC):** Constraints applied to the descent engine's max thrust (45,000 N) and throttleability.
* **Slew Rate Limiting:** Models the inertia of the physical lander by limiting the rate of pitch change ($\dot{\alpha}$), simulating the realistic lag of the Reaction Control System (RCS) rather than assuming instantaneous rotation.

## 📊 Simulation Visualizations

*(Note: Add screenshots of your plots here. I recommend adding 'trajectory.png' and 'telemetry.png' to a /docs folder)*

* **Trajectory Plot:** Tracks altitude vs. downrange distance relative to the landing site.
* **Telemetry:** Real-time logging of Pitch ($\alpha$), Thrust (%), Velocity Components, and Propellant Mass.
* **Phase Plane:** Visualization of Radial vs. Angular velocity convergence.

## 📂 Project Structure

```bash
├── main.py             # Simulation loop, RK45 integrator, and event handling
├── guidance.py         # Polynomial guidance library (Cubic Spline generation)
├── visualization.py    # Plotting and telemetry reconstruction
├── README.md           # Project documentation
└── .gitignore          # Python/Environment exclusion patterns
```
## 📚 References & Data Sources

This simulation relies on historical flight data and technical specifications from the following official documentation:

1.  **NASA Manned Spacecraft Center**, *"Apollo 11 Mission Report"* (MSC-00171), Nov 1970.  
    [NASA Technical Reports Server (NTRS)](https://ntrs.nasa.gov/api/citations/19700024568/downloads/19700024568.pdf)  
    *Primary source for trajectory timeline, event times (P63/P64 transition), and propellant usage.*

2.  **NASA Safety & Mission Assurance**, *"Apollo 11 Mission Report"* (Significant Incidents Context).  
    [Available PDF](https://sma.nasa.gov/SignificantIncidents/assets/a11_missionreport.pdf)  
    *Used for verifying safety constraints and abort boundaries.*

3.  **Aircraft Engine Historical Society**, *"The Lunar Module Descent Engine (LMDE)"*.  
    [EngineHistory.org](https://www.enginehistory.org/Rockets/RPE09.46/RPE09.46.shtml)  
    *Source for TRW Descent Engine performance data ($I_{sp}$, Thrust curves, and throttling limits).*

4.  **Wikipedia Contributors**, *"Apollo Lunar Module"* & *"Descent Propulsion System"*.  
    [Apollo Lunar Module](https://en.wikipedia.org/wiki/Apollo_Lunar_Module) | [DPS](https://en.wikipedia.org/wiki/Descent_propulsion_system)  
    *General physical properties (Dry Mass, Tank Capacities, Dimensions).*
