"""Standing balance controller (ankle strategy).

Joint-space PD alone cannot hold a squat: gravity feed-forward only sees the
hanging subtree, so loaded knees sag, the thigh tilts back, and the CoM
walks off the heels (found in stage 4). Real humanoids close a balance loop
around the CoM: trim ankle pitch so the CoM tracks the support center.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

KP_COM = 3.0     # rad of ankle trim per m of CoM error
KD_COM = 0.6
TRIM_LIMIT = 0.30  # stay within the ankle's 0.35 rad envelope
SUPPORT_CENTER_X = 0.03  # ankle sits 1/3 from heel; polygon center is forward


class StandingController:
    """Wraps Firmware with a CoM-feedback ankle trim for double support."""

    def __init__(self, fw, model, data):
        self.fw = fw
        self.model = model
        self.data = data
        self.base = {name: 0.0 for name in fw.controllers}
        self.prev_e = None
        self.pelvis_id = model.body("pelvis").id
        self.lfoot = model.body("left_foot").id
        self.rfoot = model.body("right_foot").id
        self.l_ankle = fw.controllers["left_ankle_pitch_joint"]
        self.r_ankle = fw.controllers["right_ankle_pitch_joint"]
        self.ankle_base = 0.0      # current (ramped) ankle base, left convention
        self.ankle_tgt = 0.0
        self.ankle_rate = 0.0

    def set_pose(self, targets, duration=1.5):
        self.base.update(targets)
        self.fw.set_targets({k: v for k, v in targets.items()
                             if "ankle_pitch" not in k}, duration)
        # ankle base is owned by the balance loop and must ramp in step with
        # the other joints — jumping it ahead pitches the robot over
        if "left_ankle_pitch_joint" in targets:
            self.ankle_tgt = targets["left_ankle_pitch_joint"]
            self.ankle_rate = abs(self.ankle_tgt - self.ankle_base) / max(duration, 1e-3)

    def com_error(self):
        com_x = self.data.subtree_com[self.pelvis_id][0]
        feet_x = 0.5 * (self.data.xpos[self.lfoot][0] + self.data.xpos[self.rfoot][0])
        return com_x - (feet_x + SUPPORT_CENTER_X)

    def step(self, dt):
        e = self.com_error()
        de = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e
        step = max(-self.ankle_rate * dt, min(self.ankle_rate * dt,
                                              self.ankle_tgt - self.ankle_base))
        self.ankle_base += step
        # negative trim = dorsiflexion = lean forward; mirrored on the right
        trim = max(-TRIM_LIMIT, min(TRIM_LIMIT, KP_COM * e + KD_COM * de))
        for ctrl, sign in ((self.l_ankle, 1.0), (self.r_ankle, -1.0)):
            lo, hi = ctrl.spec["range"]
            ctrl.q_cmd = max(lo, min(hi, sign * (self.ankle_base + trim)))
            ctrl.ramp_rate = 2.0  # balance trim must not lag behind the ramp
        self.fw.step_control(dt)
