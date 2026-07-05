"""CAN branch model matching the Asimov 1 wiring: six daisy-chained branches
off the Motion Control Board, five at 1 Mbps and the neck at 500 kbps.

Models bus utilisation to verify the control rate fits the wire — a real
integration failure mode (too many actuators polled too fast saturates CAN).
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Standard CAN 2.0A data frame with 8-byte payload incl. stuffing overhead.
BITS_PER_FRAME = 130
FRAMES_PER_ACTUATOR_PER_CYCLE = 2  # command down + telemetry back


class CanBranch:
    def __init__(self, name, bitrate, actuator_ids):
        self.name = name
        self.bitrate = bitrate
        self.actuator_ids = actuator_ids

    def utilisation(self, control_hz):
        bits = len(self.actuator_ids) * FRAMES_PER_ACTUATOR_PER_CYCLE * BITS_PER_FRAME * control_hz
        return bits / self.bitrate

    def latency_s(self):
        """Worst-case time to serialise one cycle of frames down the chain."""
        return len(self.actuator_ids) * FRAMES_PER_ACTUATOR_PER_CYCLE * BITS_PER_FRAME / self.bitrate


def build_branches():
    robot = json.loads((ROOT / "specs" / "robot.json").read_text())
    acts = json.loads((ROOT / "specs" / "actuators.json").read_text())["actuators"]
    branches = []
    for name, cfg in robot["electronics"]["can_branches"].items():
        ids = [a["id"] for a in acts if a["branch"] == name]
        branches.append(CanBranch(name, cfg["bitrate"], ids))
    return branches


def check_control_rate(control_hz):
    """Return (ok, report) for running the whole robot at control_hz."""
    lines, ok = [], True
    for b in build_branches():
        u = b.utilisation(control_hz)
        ok &= u < 0.8  # keep 20% headroom, standard CAN practice
        lines.append(f"{b.name:10s} {len(b.actuator_ids)} actuators  "
                     f"{b.bitrate/1000:.0f} kbps  util {u*100:5.1f}%  "
                     f"latency {b.latency_s()*1e3:.2f} ms")
    return ok, "\n".join(lines)
