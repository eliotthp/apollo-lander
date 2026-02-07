# Apollo Lunar Module Descent Simulation

![Language](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Active_Development-orange)

## 🚀 Project Overview
This project is 3-DOF flight dynamics simulation of the **Apollo 11 Lunar Module (LM)** during the early portion of the powered descent to the lunar surface. The focus of the current implementation is the **Braking Phase (P63)**, where the LM transitions from a low-altitude descent orbit to a controlled, near-vertical descent.

The simulation is structured as a modular Guidance, Navigation, and control (GNC) system, with clear seperation between:
* physical dynamics (the "plant"),
* guidance law generation,
* control and actuator logic,
* and visualization/post-processing.
The project is under **active development**, with additional descent phases (High Gate / Low Gate / P64-style approach) planned but not yet fully implemented.

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

### **Trajectory Plot:**  
![Trajectory Plot](figs/trajectory.png)  
Tracks altitude vs. downrange distance relative to the landing site.
### **Telemetry Plots:**  
![Telemetry Plots](figs/telemetry.png)  
Real-time logging of Pitch ($\alpha$), Thrust (%), Velocity Components, and Propellant Mass.

## 📂 Project Structure

```bash
├── README.md
├── controller.py
├── environment.py
├── guidance.py
├── LICENSE
├── main.py
├── navigation.py
├── simulation.py
├── test.py
└── visualization.py
```

## 🔧 Current Limitations
* Only the **Braking Phase (P63)** is fully implemented.
* Time-to-go is currently fixed rather than state-scheduled.
* Attitude dynamics are simplified (single-angle model).
* Throttle and attitude rate limits are not implemented.
* No terrain model or landing leg dynamics.

## 🔜 Planned Extensions
* Explicit **High Gate / Low Gate** phase transitions.
* Tracking control layered on top of guidance references.
* State-dependent time-to-go scheduling.
* Improved actuator dynamics and saturation handling.
* Validation against published Apollo descent data.

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
