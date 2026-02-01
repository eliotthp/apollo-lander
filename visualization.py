import matplotlib.pyplot as plt
import numpy as np


def plot_mission_results(sol, params):
    # Unpack the parameters dictionary for cleaner math
    r_moon = params["r_moon"]
    target_theta = params["target_theta"]
    target_r = params["target_r"]
    theta0 = params["theta0"]
    m0 = params["m0"]
    m_empty = params["m_empty"]
    Isp = params["Isp"]
    G_earth = params["G_earth"]

    # Final Conditions
    final_r = sol.y[0][-1]
    final_dr = sol.y[1][-1]
    final_theta = sol.y[2][-1]
    final_dtheta = sol.y[3][-1]
    final_m = sol.y[4][-1]
    delta_v = Isp * G_earth * np.log(m0 / m_empty)

    # Final Stats
    miss_distance = (sol.y[2][-1] - target_theta[-1]) * r_moon

    print(f"--- MISSION DATA --- Final Time: {sol.t[-1]:.2f} s ---")
    print(
        f"Impact Velocity: {np.sqrt(final_dr**2 + (final_dtheta * final_r) ** 2):.2f} m/s"
    )
    print(f"Miss Distance: {miss_distance / 1000:.2f} km")
    print(f"Remaining Propellant: {(final_m - m_empty):.2f} kg")
    print(f"Total Delta V Expended: {delta_v:.2f} m/s")

    # Convert rad to deg
    sol.y[2] *= 180 / np.pi
    theta0 *= 180 / np.pi
    target_theta[-1] *= 180 / np.pi
    final_theta *= 180 / np.pi

    # Plotting
    fig, axs = plt.subplots(2, 2, figsize=(14, 8))

    # The trajectory
    axs[0, 0].plot(sol.y[2], sol.y[0] - r_moon, label="Eagle Path")
    axs[0, 0].axhline(y=0, color="gray", linestyle="--", label="Moon Surface")
    axs[0, 0].scatter(
        target_theta[-1], target_r[-1] - r_moon, color="red", label="Target"
    )
    axs[0, 0].scatter(
        final_theta, final_r - r_moon, color="black", marker="x", label="Impact"
    )
    axs[0, 0].set_xlim(theta0, target_theta[-1] - 1)
    axs[0, 0].set_ylim(-500, max(sol.y[0] - r_moon) + 500)
    axs[0, 0].set_xlabel("Theta (°)")
    axs[0, 0].set_ylabel("Radius (m)")
    axs[0, 0].set_title("Radius vs. Theta (Descending Orbit)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Altitude vs. Time
    axs[0, 1].plot(sol.t, sol.y[0] - r_moon)
    axs[0, 1].set_ylim(-500, max(sol.y[0] - r_moon) + 500)
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Altitude (m)")
    axs[0, 1].set_title("Altitude vs. Time")
    axs[0, 1].grid(True)

    # Velocity Components
    axs[1, 0].plot(sol.t, sol.y[1], label="Radial")
    axs[1, 0].plot(sol.t, -(sol.y[3] * sol.y[0]), label="Angular")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Velocity (m/s)")
    axs[1, 0].set_title("Velocity Components")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Fuel
    axs[1, 1].plot(sol.t, sol.y[4])
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Mass (kg)")
    axs[1, 1].set_title("Fuel Consumption")
    axs[1, 1].grid(True)
    plt.tight_layout()
    plt.show()
