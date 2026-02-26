from states import LVLHState, GuidanceState, ControlState, PolarState
import config as cfg
import matplotlib.pyplot as plt
import numpy as np


class Logger:
    def __init__(self, desired_keys: list[str]) -> None:
        self.records: dict[str, list[float]] = {k: [] for k in desired_keys}

    def log(
        self,
        t: float,
        lvlh: LVLHState,
        guid: GuidanceState,
        ctrl: ControlState,
        plr: PolarState,
    ) -> None:
        self.records["t"].append(t)
        self.records["m"].append(plr.m)
        self.records["r"].append(plr.r)
        self.records["dr"].append(plr.dr)
        self.records["z"].append(lvlh.z)
        self.records["dz"].append(lvlh.dz)
        self.records["x"].append(lvlh.x)
        self.records["dx"].append(lvlh.dx)
        self.records["T_cmd"].append(ctrl.T_cmd)
        self.records["T_ctrl"].append(ctrl.T_ctrl)
        self.records["alpha_cmd"].append(ctrl.alpha_cmd)
        self.records["alpha_ctrl"].append(ctrl.alpha_ctrl)
        self.records["t_elapsed"].append(guid.t_elapsed)

    def find_stage_change(self, t_elapsed: list[float]) -> list[float]:
        time_stages = []
        for i in range(len(t_elapsed) - 1):
            if t_elapsed[i + 1] == 0:
                time_stages.append(self.records["t"][i])
        return time_stages

    def output_stats(self, t: float, start: float, end: float, sim: PolarState):
        real_t = end - start
        delta_V = cfg.Isp * cfg.G_earth * np.log(cfg.m0 / sim.m)

        # --- Printing Results ---
        print("--- Results ---")
        print(f"Time: {t:.2f} s")
        print(f"Real Time: {real_t:.2f} s")
        print(f"Effective sim speed (x real time) {t / real_t:.2f}")
        print(f"Final Altitude: {sim.r - cfg.r_moon:.2f} m")
        v_final = np.sqrt(sim.dr**2 + (sim.dtheta * sim.r) ** 2)
        print(f"Final Velocity: {v_final:.2f} m/s")
        print(f"Delta-V: {delta_V:.2f} m/s")
        print(f"--- Safe Landing: {v_final < 5} ---")

    def plot(self):
        # --- Plotting ---
        fig, axs = plt.subplots(2, 3, figsize=(16, 8))
        fig.suptitle("Descent Simulation Telemetry")
        fig.subplots_adjust(
            bottom=0.08, left=0.08, right=0.92, top=0.92, wspace=0.22, hspace=0.22
        )

        axs[0, 0].plot(self.records["t"], self.records["z"])
        axs[0, 0].set_title("Altitude vs. Time")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Altitude (m)")
        axs[0, 0].grid(True)

        axs[0, 1].plot(self.records["t"], self.records["dz"], label="Vertical Velocity")
        axs[0, 1].plot(
            self.records["t"], self.records["dx"], label="Horizontal Velocity"
        )
        axs[0, 1].set_title("Velocity vs. Time")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Velocity (m/s)")
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        axs[1, 0].plot(
            self.records["t"], self.records["T_cmd"], "r--", label="Commanded Thrust"
        )
        axs[1, 0].plot(
            self.records["t"], self.records["T_ctrl"], "b-", label="Actual Thrust"
        )
        axs[1, 0].set_title("Thrust vs. Time")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Thrust (N)")
        axs[1, 0].set_ylim(0, cfg.T_max * 1.5)
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        axs[1, 1].plot(
            self.records["t"],
            np.rad2deg(self.records["alpha_cmd"]),
            "r--",
            label="Commanded Pitch",
        )
        axs[1, 1].plot(
            self.records["t"],
            np.rad2deg(self.records["alpha_ctrl"]),
            "b-",
            label="Actual Pitch",
        )
        axs[1, 1].set_title("Pitch Angle vs. Time")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Pitch Angle (degrees)")
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        axs[0, 2].plot(
            self.records["t"],
            np.array(self.records["m"]) - cfg.m_empty,
            label="Mass of Propellant",
        )
        axs[0, 2].set_title("Propellant Mass vs. Time")
        axs[0, 2].set_xlabel("Time (s)")
        axs[0, 2].set_ylabel("Propellant Mass (kg)")
        axs[0, 2].grid(True)

        time_stages = self.find_stage_change(self.records["t_elapsed"])

        for i in [time_stages[0], time_stages[1]]:
            for ax_row in axs:
                for ax in ax_row:
                    ax.axvline(i, color="k", linestyle="--", alpha=0.5)

        r_array = np.array(self.records["r"])
        dr_array = np.array(self.records["dr"])
        z_array = np.array(self.records["z"])
        dz_array = np.array(self.records["dz"])

        axs[1, 2].plot(
            self.records["t"],
            r_array - cfg.r_moon - z_array,
            label="Position",
        )
        axs[1, 2].plot(
            self.records["t"],
            dr_array - dz_array,
            label="Velocity",
        )
        axs[1, 2].set_xlabel("Time (s)")
        axs[1, 2].set_ylabel("Error (m)")
        axs[1, 2].set_title(
            "Error between radar and true z-quantities vs. Time | Filtered"
        )
        axs[1, 2].legend()
        axs[1, 2].grid(True)
        axs[1, 2].plot()
        plt.savefig("figs/telemetry.png")
        plt.show()

        # Calculate RMS
        rms_r = np.sqrt(np.mean((r_array - cfg.r_moon - z_array) ** 2))
        rms_dr = np.sqrt(np.mean((dr_array - dz_array) ** 2))

        print(f"RMS Error in Position: {rms_r:.5f} m")
        print(f"RMS Error in Velocity: {rms_dr:.5f} m/s")
