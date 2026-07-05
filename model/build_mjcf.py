"""Procedural 'CAD' stage: generate the Asimov 1 MJCF model from the spec files.

The published docs give exact joint/actuator data but only STEP CAD for geometry,
so link dimensions and masses come from specs/robot.json geometry_estimates.

Usage:
    python model/build_mjcf.py [--stage STAGE] [--out PATH]

Stages support the assembly simulation: each stage emits a partial robot
mounted the way the real assembly verifications fixture it.
    legs        pelvis + both legs, pelvis welded to a stand (leg swing tests)
    lower_body  pelvis + legs, free floating base
    upper_body  torso + arms + head, welded at the waist mount (arm tests)
    full        complete robot, floating base (default)
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SPECS = ROOT / "specs"


def load_specs():
    robot = json.loads((SPECS / "robot.json").read_text())
    act = json.loads((SPECS / "actuators.json").read_text())
    return robot, {a["joint"]: a for a in act["actuators"]}, act["passive_joints"]


def sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def fmt(*vals):
    return " ".join(f"{v:.4g}" for v in vals)


class Builder:
    def __init__(self, stage="full"):
        self.robot, self.acts, self.passive = load_specs()
        self.g = self.robot["geometry_estimates"]
        self.m = self.g["mass_budget_kg"]
        self.stage = stage
        self.joints_used = []

    # -- joints ---------------------------------------------------------
    def joint(self, body, name, axis):
        """Add a hinge joint using the spec range. Right-side pitch-type joints
        use a flipped axis so spec sign conventions (mirrored ranges) hold."""
        a = self.acts[name]
        lo, hi = a["range"]
        # gearbox reflected inertia: bigger motors -> more armature
        armature = 0.05 if a["peak_torque"] >= 75 else 0.01
        sub(body, "joint", name=name, axis=fmt(*axis), range=fmt(lo, hi),
            damping=1.0, armature=armature, limited="true")
        self.joints_used.append(name)

    def pitch_axis(self, side):
        return (0, -1, 0) if side == "right" else (0, 1, 0)

    def housing(self, body, mass=0.35):
        """Actuator housing mass at a joint cluster (non-colliding)."""
        sub(body, "geom", type="sphere", size="0.03", mass=mass,
            contype=0, conaffinity=0, rgba="0.25 0.25 0.28 1")

    # -- limbs ----------------------------------------------------------
    def leg(self, parent, side):
        g, m = self.g, self.m
        sgn = 1 if side == "left" else -1
        p = self.pitch_axis(side)

        hip = sub(parent, "body", name=f"{side}_hip", pos=fmt(0, sgn * g["hip_offset_y"], 0))
        self.joint(hip, f"{side}_hip_pitch_joint", p)
        self.housing(hip, 0.5)
        hip2 = sub(hip, "body", name=f"{side}_hip2", pos="0 0 0")
        self.joint(hip2, f"{side}_hip_roll_joint", (-1, 0, 0))
        self.housing(hip2, 0.4)
        thigh = sub(hip2, "body", name=f"{side}_thigh", pos="0 0 0")
        self.joint(thigh, f"{side}_hip_yaw_joint", (0, 0, 1))
        sub(thigh, "geom", type="capsule", fromto=fmt(0, 0, -0.03, 0, 0, -g["thigh_length"] + 0.03),
            size="0.055", mass=m["thigh"])

        shank = sub(thigh, "body", name=f"{side}_shank", pos=fmt(0, 0, -g["thigh_length"]))
        self.joint(shank, f"{side}_knee_joint", p)
        sub(shank, "geom", type="capsule", fromto=fmt(0, 0, -0.03, 0, 0, -g["shank_length"] + 0.03),
            size="0.045", mass=m["shank"])

        ankle = sub(shank, "body", name=f"{side}_ankle", pos=fmt(0, 0, -g["shank_length"]))
        self.joint(ankle, f"{side}_ankle_pitch_joint", p)
        self.housing(ankle, 0.2)
        foot = sub(ankle, "body", name=f"{side}_foot", pos="0 0 0")
        self.joint(foot, f"{side}_ankle_roll_joint", (-1, 0, 0))
        # foot sole: box from heel to toe joint, ankle sits 1/3 from heel
        fl, fw, ah = g["foot_length"], g["foot_width"], g["ankle_height"]
        heel_x = -fl / 3.0
        sole_len = fl - g["toe_length"]
        cx = heel_x + sole_len / 2.0
        sub(foot, "geom", type="box", pos=fmt(cx, 0, -ah + 0.015),
            size=fmt(sole_len / 2, fw / 2, 0.015), mass=m["foot"])

        toe = sub(foot, "body", name=f"{side}_toe", pos=fmt(heel_x + sole_len, 0, -ah + 0.015))
        tj = next(pj for pj in self.passive if pj["joint"] == f"{side}_toe_joint")
        sub(toe, "joint", name=tj["joint"], axis=fmt(*self.pitch_axis(side)),
            range=fmt(*tj["range"]), stiffness=tj["stiffness"], damping=0.05, limited="true")
        sub(toe, "geom", type="box", pos=fmt(g["toe_length"] / 2, 0, 0),
            size=fmt(g["toe_length"] / 2, fw / 2, 0.012), mass=m["toe"])

    def arm(self, parent, side, shoulder_z):
        g, m = self.g, self.m
        sgn = 1 if side == "left" else -1
        p = self.pitch_axis(side)

        sh = sub(parent, "body", name=f"{side}_shoulder",
                 pos=fmt(0, sgn * g["shoulder_offset_y"], shoulder_z))
        self.joint(sh, f"{side}_shoulder_pitch_joint", p)
        self.housing(sh, 0.4)
        sh2 = sub(sh, "body", name=f"{side}_shoulder2", pos="0 0 0")
        self.joint(sh2, f"{side}_shoulder_roll_joint", (-1, 0, 0))
        self.housing(sh2, 0.3)
        ua = sub(sh2, "body", name=f"{side}_upper_arm", pos="0 0 0")
        self.joint(ua, f"{side}_shoulder_yaw_joint", (0, 0, 1))
        sub(ua, "geom", type="capsule", fromto=fmt(0, 0, -0.02, 0, 0, -g["upper_arm_length"] + 0.02),
            size="0.04", mass=m["upper_arm"])

        fa = sub(ua, "body", name=f"{side}_forearm", pos=fmt(0, 0, -g["upper_arm_length"]))
        self.joint(fa, f"{side}_elbow_joint", p)
        sub(fa, "geom", type="capsule", fromto=fmt(0, 0, -0.02, 0, 0, -g["forearm_length"] + 0.02),
            size="0.035", mass=m["forearm"])

        hand = sub(fa, "body", name=f"{side}_hand", pos=fmt(0, 0, -g["forearm_length"]))
        self.joint(hand, f"{side}_wrist_yaw_joint", (0, 0, 1))
        sub(hand, "geom", type="box", pos="0 0 -0.04", size="0.03 0.02 0.05", mass=m["hand"])

    def torso_chain(self, parent):
        g, m = self.g, self.m
        torso = sub(parent, "body", name="torso", pos=fmt(0, 0, 0.10))
        self.joint(torso, "waist_yaw_joint", (0, 0, 1))
        # torso structure + battery pack (2 kg, low in the torso like the real pack)
        sub(torso, "geom", name="torso_geom", type="box", pos=fmt(0, 0, g["torso_length"] / 2),
            size=fmt(0.09, 0.13, g["torso_length"] / 2), mass=m["torso"])
        sub(torso, "geom", name="battery_pack", type="box", pos="0 -0.0 0.06",
            size="0.06 0.09 0.035", mass=m["battery_in_torso"],
            contype=0, conaffinity=0, rgba="0.8 0.6 0.1 1")
        sub(torso, "site", name="imu", pos="0 0 0.15")

        neck = sub(torso, "body", name="neck", pos=fmt(0, 0, g["torso_length"]))
        self.joint(neck, "neck_yaw_joint", (0, 0, 1))
        self.housing(neck, 0.15)
        head = sub(neck, "body", name="head", pos=fmt(0, 0, g["neck_length"]))
        self.joint(head, "neck_pitch_joint", (0, 1, 0))
        sub(head, "geom", type="sphere", pos=fmt(0, 0, g["head_radius"] * 0.6),
            size=g["head_radius"], mass=m["head"])
        sub(head, "site", name="camera_mount", pos=fmt(g["head_radius"], 0, g["head_radius"] * 0.6))

        self.arm(torso, "left", g["torso_length"] - 0.04)
        self.arm(torso, "right", g["torso_length"] - 0.04)

    # -- assembly -------------------------------------------------------
    def build(self):
        g, m = self.g, self.m
        root = ET.Element("mujoco", model=f"asimov1_{self.stage}")
        sub(root, "compiler", angle="radian", autolimits="true")
        sub(root, "option", timestep="0.002", integrator="implicitfast")
        default = sub(root, "default")
        sub(default, "geom", friction="1.0 0.005 0.0001", rgba="0.75 0.78 0.82 1")

        # frame the default viewer camera on the robot, not the floor plane
        if self.stage == "upper_body":
            top = 1.0 + 0.10 + g["torso_length"] + g["neck_length"] + g["head_radius"] * 1.6
        elif self.stage == "full":
            top = (g["pelvis_height"] + 0.10 + g["torso_length"]
                   + g["neck_length"] + g["head_radius"] * 1.6)
        else:
            top = g["pelvis_height"] + (0.25 if self.stage == "legs" else 0.0) + 0.12
        sub(root, "statistic", center=fmt(0, 0, top / 2), extent=fmt(top * 1.1))
        visual = sub(root, "visual")
        sub(visual, "global", azimuth="120", elevation="-15")

        world = sub(root, "worldbody")
        sub(world, "light", pos="0 0 3", dir="0 0 -1")
        sub(world, "geom", name="floor", type="plane", size="5 5 0.1", rgba="0.3 0.35 0.3 1")

        fixture = self.stage in ("legs", "upper_body")
        if self.stage in ("legs", "lower_body", "full"):
            z = g["pelvis_height"] + (0.25 if self.stage == "legs" else 0.0)
            pelvis = sub(world, "body", name="pelvis", pos=fmt(0, 0, z))
            if not fixture:
                sub(pelvis, "freejoint", name="root")
            sub(pelvis, "geom", name="pelvis_geom", type="box", pos="0 0 0.05",
                size="0.08 0.13 0.07", mass=m["pelvis"])
            sub(pelvis, "site", name="pelvis_site", pos="0 0 0")
            self.leg(pelvis, "left")
            self.leg(pelvis, "right")
            if self.stage == "full":
                self.torso_chain(pelvis)
        elif self.stage == "upper_body":
            mount = sub(world, "body", name="waist_mount", pos="0 0 1.0")
            sub(mount, "geom", type="cylinder", size="0.05 0.05", mass="50")
            self.torso_chain(mount)

        # adjacent-part contact exclusions (parts overlap at the joints by design)
        contact = sub(root, "contact")
        have_legs = self.stage in ("legs", "lower_body", "full")
        have_torso = self.stage in ("upper_body", "full")
        if have_legs:
            for s in ("left", "right"):
                sub(contact, "exclude", body1="pelvis", body2=f"{s}_thigh")
        if have_torso:
            for s in ("left", "right"):
                sub(contact, "exclude", body1="torso", body2=f"{s}_upper_arm")
        if self.stage == "full":
            for s in ("left", "right"):
                sub(contact, "exclude", body1=f"{s}_hand", body2=f"{s}_thigh")
                sub(contact, "exclude", body1=f"{s}_forearm", body2=f"{s}_thigh")

        # actuators: pure torque motors; firmware supplies the control law
        actuators = sub(root, "actuator")
        sensors = sub(root, "sensor")
        for name in self.joints_used:
            a = self.acts[name]
            sub(actuators, "motor", name=name.replace("_joint", ""), joint=name,
                ctrlrange=fmt(-a["peak_torque"], a["peak_torque"]))
            sub(sensors, "jointpos", name=f"{name}_pos", joint=name)
            sub(sensors, "jointvel", name=f"{name}_vel", joint=name)
        if self.stage in ("lower_body", "full"):
            sub(sensors, "gyro", name="imu_gyro", site="imu" if self.stage == "full" else "pelvis_site")
            sub(sensors, "accelerometer", name="imu_acc", site="imu" if self.stage == "full" else "pelvis_site")

        ET.indent(root)
        return ET.tostring(root, encoding="unicode")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="full",
                    choices=["legs", "lower_body", "upper_body", "full"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    xml = Builder(args.stage).build()
    out = Path(args.out) if args.out else ROOT / "model" / f"asimov_{args.stage}.xml"
    out.write_text(xml)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
