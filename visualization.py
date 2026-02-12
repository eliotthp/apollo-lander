import matplotlib.pyplot as plt
import numpy as np
import environment as env

# --- Constants & Environment ---
m_empty = env.m_empty
T_max = env.T_max

# In future auto-update graphs to be displayed in GitHub


def trajectory(theta, alt):
    """
    Plots the altitude of the lunar module relative to its angular position.

    Args:
        theta (ndarray): Array of angular positions (degrees).
        alt (ndarray): Array of altitudes (meters).
    """
    # Create the trajectory plot showing altitude vs angular position
    plt.figure(figsize=(8, 6))
    plt.plot(theta, alt, label="Trajectory")
    plt.xlabel(r"Theta ($\degree$)")
    plt.ylabel("Altitude (m)")
    plt.gca().invert_xaxis()
    plt.title("Trajectory of Lunar Module")
    plt.legend()
    plt.grid(True)
    # Ensure layout is clean before saving to the figs directory
    plt.tight_layout()
    plt.savefig("figs/trajectory.png")


def telemetry(t, vel_components, alpha_cmd, thrust_cmd, alpha_ctrl, thrust_ctrl, m_p):
    """
    Generates a 2x2 grid of plots showing the vehicle's telemetry over time.

    Args:
        t (ndarray): Time array.
        vel_components (list): List containing vertical (vz) and horizontal (vx) velocities.
        alpha_cmd (ndarray): Commanded pitch angle array.
        thrust_cmd (ndarray): Commanded thrust array.
        alpha_ctrl (ndarray): Actual pitch angle array.
        thrust_ctrl (ndarray): Actual thrust array.
        m_p (ndarray): Propellant mass array.
    """
    # Initialize a 2x2 subplot figure for comprehensive telemetry review
    vz = vel_components[0]
    vx = vel_components[1]
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # Plot 1: Vertical and Horizontal velocity components
    axs[0, 0].plot(t, vz, label="Vertical Velocity")
    axs[0, 0].plot(t, vx, label="Horizontal Velocity")
    axs[0, 0].set_title("Velocity Components Over Time")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Velocity (m/s)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Plot 2: Compare commanded pitch vs actual vehicle pitch (converted to degrees)
    axs[0, 1].plot(t, -np.rad2deg(alpha_cmd), "r--", label="Pitch Command")
    axs[0, 1].plot(t, -np.rad2deg(alpha_ctrl), "b", label="Pitch Actual")
    axs[0, 1].set_title("Pitch Angle: Command vs Actual")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel(r"Pitch from vertical ($\degree$)")
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # Plot 3: Compare commanded thrust vs actual thrust output
    axs[1, 0].plot(t, thrust_cmd, "r--", label="Thrust Command")
    axs[1, 0].plot(t, thrust_ctrl, "g", label="Thrust Actual")
    axs[1, 0].set_title("Thrust: Command vs Actual")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Thrust (N)")
    axs[1, 0].set_ylim([0, T_max + T_max * 0.1])
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Plot 4: Track propellant consumption over the course of the landing
    axs[1, 1].plot(t, m_p, color="purple", label="Propellant Mass")
    axs[1, 1].set_title("Propellant Mass Over Time")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Mass (kg)")
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig("figs/telemetry.png")


def end_state_metrics(t, final_state):
    """
    Calculates and prints the final mission performance metrics.

    Args:
        t (ndarray): Time array from the simulation.
        final_state (list): The final state vector [zf, dzf, xf, dxf, mf].
    """
    # Unpack final state
    zf, dzf, xf, dxf, mf = final_state
    # Constants
    Isp = env.Isp
    G_earth = env.G_earth
    m0 = env.m0
    # Calculate magnitude of the velocity vector at touchdown
    impact_velocity = np.sqrt(dzf**2 + dxf**2)
    # Determine how much fuel is left in the tanks
    remaining_propellant = mf - m_empty
    # Use Tsiolkovsky rocket equation for total Delta-V expended
    delta_v = Isp * G_earth * np.log(m0 / mf)

    # Prepare the data
    metrics = {
        "Time of Flight": f"{t[-1]:.2f} s",
        "Impact Velocity": f"{impact_velocity:.2f} m/s",
        "Delta-V": f"{delta_v:.2f} m/s",
        "Vertical Vel": f"{dzf:.2f} m/s",
        "Horizontal Vel": f"{dxf:.2f} m/s",
        "Remaining Fuel": f"{remaining_propellant:.2f} kg",
    }
    # Export results to a Markdown file for the mission report
    with open("data/MISSION_LOG.md", "w", encoding="utf-8") as f:  # Added encoding here
        f.write("### 🚀 Mission End State Metrics\n\n")
        f.write("| Metric | Value |\n")
        f.write("| :--- | :--- |\n")
        for key, value in metrics.items():
            f.write(f"| **{key}** | `{value}` |\n")
