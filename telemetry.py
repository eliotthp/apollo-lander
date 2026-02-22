from states import LVLHState, GuidanceState, ControlState, PolarState


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
