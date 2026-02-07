import matplotlib.pyplot as plt
import numpy as np
import environment as env

# --- Constants & Environment ---
m_empty = env.m_empty

# In future auto-update graphs to be displayed in GitHub


def trajectory(theta, alt):
    """
    Plots the altitude of the lunar module relative to its angular position.

    Args:
        theta (ndarray): Array of angular positions (degrees).
        alt (ndarray): Array of altitudes (meters).
    """
    plt.plot(theta, alt, label="Trajectory")
    plt.xlabel(r"Theta ($\degree$)")
    plt.ylabel("Altitude (m)")
    plt.gca().invert_xaxis()
    plt.title("Trajectory of Lunar Module")
    plt.legend()
    plt.grid(True)
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
    vz = vel_components[0]
    vx = vel_components[1]
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # Velocity Components
    axs[0, 0].plot(t, vz, label="Vertical Velocity")
    axs[0, 0].plot(t, vx, label="Horizontal Velocity")
    axs[0, 0].set_title("Velocity Components Over Time")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Velocity (m/s)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Pitch Command vs Actual
    axs[0, 1].plot(t, -np.rad2deg(alpha_cmd), "r--", label="Pitch Command")
    axs[0, 1].plot(t, -np.rad2deg(alpha_ctrl), "b", label="Pitch Actual")
    axs[0, 1].set_title("Pitch Angle: Command vs Actual")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel(r"Pitch from vertical ($\degree$)")
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # Thrust Command vs Actual
    axs[1, 0].plot(t, thrust_cmd, "r--", label="Thrust Command")
    axs[1, 0].plot(t, thrust_ctrl, "g", label="Thrust Actual")
    axs[1, 0].set_title("Thrust: Command vs Actual")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Thrust (N)")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Mass of Propellant
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
        final_state (list): The final state vector [r, dr, theta, dtheta, m, alpha].
    """
    # Unpack final state
    r_final, dr_final, theta_final, dtheta_final, m_final, alpha_final = final_state
    # Constants
    Isp = env.Isp
    G_earth = env.G_earth
    m0 = env.m0
    # Find impact velocity
    impact_velocity = np.sqrt(dr_final**2 + (r_final * dtheta_final) ** 2)
    # Find remaining prop
    remaining_propellant = m_final - m_empty
    # Calculate delta-V
    delta_v = Isp * G_earth * np.log(m0 / m_final)

    print("-" * 30)
    print("MISSION END STATE METRICS")
    print("-" * 30)
    print(f"Time of Flight:    {t[-1]:.2f} s")
    print(f"Impact Velocity:   {impact_velocity:.2f} m/s")
    print(f"Delta-V:           {delta_v:.2f} m/s")
    print(f"Vertical Vel:      {dr_final:.2f} m/s")
    print(f"Horizontal Vel:    {r_final * dtheta_final:.2f} m/s")
    print(f"Remaining Fuel:    {remaining_propellant:.2f} kg")
    print("-" * 30)
