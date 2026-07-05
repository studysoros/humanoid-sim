"""Battery pack model: 13S4P INR18650 Li-ion per specs/robot.json.

Simulates state of charge, terminal voltage sag under load, and the BMS
protection thresholds (over-current, under-voltage) documented for Asimov 1.
"""
import json
from pathlib import Path

SPECS = Path(__file__).resolve().parent.parent / "specs" / "robot.json"

# Open-circuit voltage curve for a Li-ion cell, (soc, volts) breakpoints.
OCV_CURVE = [(0.0, 3.0), (0.05, 3.3), (0.1, 3.45), (0.2, 3.55), (0.4, 3.65),
             (0.6, 3.75), (0.8, 3.95), (0.9, 4.05), (1.0, 4.2)]
CELL_IR = 0.035  # ohms per INR18650 cell


class BMSFault(Exception):
    pass


class Battery:
    def __init__(self, soc=1.0):
        spec = json.loads(SPECS.read_text())["battery"]
        self.spec = spec
        self.series = 13
        self.parallel = 4
        self.capacity_ah = spec["capacity_ah"]
        self.soc = soc
        self.oc_limit = spec["bms"]["oc_limit_a"]
        self.cell_uv = spec["bms"]["cell_uv"]
        self.energy_used_wh = 0.0
        self.peak_current = 0.0
        self.fault = None
        # pack internal resistance: series cells add, parallel strings divide
        self.pack_ir = CELL_IR * self.series / self.parallel

    def ocv(self):
        pts = OCV_CURVE
        s = max(0.0, min(1.0, self.soc))
        for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
            if s <= s1:
                cell = v0 + (v1 - v0) * (s - s0) / (s1 - s0)
                return cell * self.series
        return pts[-1][1] * self.series

    def step(self, power_w, dt):
        """Draw power_w for dt seconds. Returns terminal voltage.

        Raises BMSFault on over-current or cell under-voltage, exactly the
        conditions the real protection circuit module trips on.
        """
        if self.fault:
            raise BMSFault(self.fault)
        v_oc = self.ocv()
        # solve V*I = P with V = Voc - I*R  ->  I = (Voc - sqrt(Voc^2-4RP)) / 2R
        disc = v_oc * v_oc - 4 * self.pack_ir * power_w
        if disc < 0:
            self.fault = "over-current (load exceeds pack capability)"
            raise BMSFault(self.fault)
        current = (v_oc - disc ** 0.5) / (2 * self.pack_ir)
        v_term = v_oc - current * self.pack_ir
        self.peak_current = max(self.peak_current, current)
        if current > self.oc_limit:
            self.fault = f"over-current ({current:.1f} A > {self.oc_limit} A BMS limit)"
            raise BMSFault(self.fault)
        if v_term / self.series < self.cell_uv:
            self.fault = f"under-voltage ({v_term/self.series:.2f} V/cell)"
            raise BMSFault(self.fault)
        self.soc -= current * dt / 3600.0 / self.capacity_ah
        self.energy_used_wh += power_w * dt / 3600.0
        return v_term
