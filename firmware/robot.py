"""Robot-level firmware: wires joint controllers to the MuJoCo model, meters
power through the battery, and enforces the CAN control-rate budget.

Mirrors the real stack: Motion Control Board runs the joint loops at
CONTROL_HZ over six CAN branches, powered from the 13S4P pack.
"""
import json
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from electronics.battery import Battery
from electronics.can_bus import check_control_rate
from firmware.joint_controller import JointController

CONTROL_HZ = 500  # verified against CAN bandwidth at import of Firmware
COMPUTE_IDLE_W = 15.0  # Pi 5 + Radxa CM5 + peripherals


class Firmware:
    def __init__(self, model, data, battery=None, check_can=True):
        self.model = model
        self.data = data
        self.battery = battery or Battery()
        if check_can:
            ok, report = check_control_rate(CONTROL_HZ)
            if not ok:
                raise RuntimeError(f"CAN budget exceeded at {CONTROL_HZ} Hz:\n{report}")
        acts = json.loads((ROOT / "specs" / "actuators.json").read_text())["actuators"]
        by_joint = {a["joint"]: a for a in acts}
        self.controllers = {}
        for i in range(model.nu):
            jname = model.actuator(i).name + "_joint"
            self.controllers[jname] = JointController(by_joint[jname])
        self.bus_voltage = self.battery.ocv()
        self.total_power = 0.0

    def set_targets(self, targets, duration=1.0):
        """targets: {joint_name: angle_rad}; unlisted joints keep their target.

        duration: seconds to interpolate over. Per-joint ramp rates are set
        so the slowest move finishes on time, capped by the actuator's
        velocity limit — commanding whole-body steps at raw motor speed
        shock-loads the stance and tips the robot (found in stage 4).
        """
        for name, q in targets.items():
            if name in self.controllers:
                c = self.controllers[name]
                lo, hi = c.spec["range"]
                c.q_cmd = max(lo, min(hi, q))
                rate = abs(c.q_cmd - c.q_des) / max(duration, 1e-3)
                c.ramp_rate = min(max(rate, 0.05), 0.5 * c.vel_limit)

    def step_control(self, dt):
        """One control cycle: PD torques -> data.ctrl, power -> battery."""
        power = COMPUTE_IDLE_W
        for i in range(self.model.nu):
            jname = self.model.actuator(i).name + "_joint"
            c = self.controllers[jname]
            jid = self.model.joint(jname).id
            q = self.data.qpos[self.model.jnt_qposadr[jid]]
            dof = self.model.jnt_dofadr[jid]
            dq = self.data.qvel[dof]
            # gravity/Coriolis feed-forward, capped at rated torque so the
            # FF path alone can never push an actuator into overload
            ff = max(-c.rated, min(c.rated, float(self.data.qfrc_bias[dof])))
            tau = c.torque(q, dq, dt, tau_ff=ff)
            self.data.ctrl[i] = tau
            power += c.power(tau, dq)
        self.total_power = power
        self.bus_voltage = self.battery.step(power, dt)

    def hold_pose(self):
        """Set every controller target to the current joint angle."""
        for jname, c in self.controllers.items():
            jid = self.model.joint(jname).id
            q = float(self.data.qpos[self.model.jnt_qposadr[jid]])
            c.q_cmd = c.q_des = q


def load_stage(stage="full"):
    """Build (if needed) and load a stage model; returns (model, data)."""
    xml_path = ROOT / "model" / f"asimov_{stage}.xml"
    if not xml_path.exists():
        sys.path.insert(0, str(ROOT / "model"))
        from build_mjcf import Builder
        xml_path.write_text(Builder(stage).build())
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return model, mujoco.MjData(model)
