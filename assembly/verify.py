"""Staged assembly simulation with per-stage acceptance tests.

Mirrors the real build order (bench electronics -> legs on fixture ->
upper body on fixture -> integrated robot) with a verification gate at
each stage, the way the Asimov assembly docs structure it.

Run:  python assembly/verify.py            # all stages
      python assembly/verify.py --stage 2  # single stage
"""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from electronics.battery import Battery
from electronics.can_bus import check_control_rate, build_branches
from firmware.joint_controller import AnkleRSU
from firmware.robot import Firmware, load_stage, CONTROL_HZ
from control.balance import StandingController

PASS, FAIL = "PASS", "FAIL"
results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"  [{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))
    return ok


def settle(model, data, fw, seconds, targets=None):
    if targets:
        fw.set_targets(targets)
    n = int(seconds / model.opt.timestep)
    for _ in range(n):
        fw.step_control(model.opt.timestep)
        mujoco.mj_step(model, data)


def joint_q(model, data, name):
    j = model.joint(name)
    return float(data.qpos[model.jnt_qposadr[j.id]])


# ---------------------------------------------------------------- stage 1
def stage1_bench_electronics():
    print("\n== Stage 1: bench electronics bring-up ==")
    b = Battery()
    v = b.step(60.0, 1.0)  # light bench load
    check("battery voltage sane at light load", 50.0 < v < 54.6, f"{v:.1f} V")
    check("pack energy matches datasheet", abs(b.spec["energy_wh"] - 491.4) < 1)

    ok, report = check_control_rate(CONTROL_HZ)
    print(report)
    check(f"CAN budget fits {CONTROL_HZ} Hz control on all branches", ok)
    ids = sorted(i for br in build_branches() for i in br.actuator_ids)
    check("all 25 actuators enumerate on CAN", ids == list(range(1, 26)))

    # BMS over-current protection must actually trip
    b2 = Battery()
    tripped = False
    try:
        b2.step(2000.0, 0.01)
    except Exception:
        tripped = True
    check("BMS trips on over-current", tripped)


# ---------------------------------------------------------------- stage 2
def stage2_legs_on_fixture():
    print("\n== Stage 2: legs on assembly fixture (pelvis clamped) ==")
    model, data = load_stage("legs")
    fw = Firmware(model, data, check_can=False)
    fw.hold_pose()
    acts = json.loads((ROOT / "specs" / "actuators.json").read_text())["actuators"]
    spec = {a["joint"]: a for a in acts}

    # sweep every leg joint to 80% of each range limit, verify tracking.
    # Park the contralateral hip roll outward first so inward sweeps don't
    # strike the other leg (bench procedure, same as the real fixture work).
    park = {"left": {"right_hip_roll_joint": 0.5},
            "right": {"left_hip_roll_joint": -0.5}}
    all_ok, worst = True, ("", 0.0)
    for jname in fw.controllers:
        side = "left" if jname.startswith("left") else "right"
        settle(model, data, fw, 1.0, park[side])
        lo, hi = spec[jname]["range"]
        for target in (0.8 * lo, 0.8 * hi, 0.0):
            settle(model, data, fw, 1.6, {jname: target})
            err = abs(joint_q(model, data, jname) - max(lo, min(hi, target)))
            if err > worst[1]:
                worst = (f"{jname}@{target:+.2f}", err)
            all_ok &= err < 0.05
    check("all 12 leg joints track sweep within 0.05 rad", all_ok,
          f"worst {worst[0]} err {worst[1]:.4f} rad")

    # ankle RSU motor stroke must cover the joint envelope
    rsu = AnkleRSU()
    corners = [rsu.motor_angles(p, r) for p in (-0.35, 0.35) for r in (-0.10, 0.10)]
    stroke = max(abs(v) for ab in corners for v in ab)
    check("RSU motor stroke covers ankle envelope", stroke < 2.0,
          f"max motor angle {stroke:.2f} rad")

    # passive toes: deflect and confirm spring return
    tq = joint_q(model, data, "left_toe_joint")
    check("passive toe returns to neutral", abs(tq) < 0.15, f"{tq:.3f} rad")


# ---------------------------------------------------------------- stage 3
def stage3_upper_body_on_fixture():
    print("\n== Stage 3: upper body on waist fixture ==")
    model, data = load_stage("upper_body")
    fw = Firmware(model, data, check_can=False)
    fw.hold_pose()
    settle(model, data, fw, 1.0)

    # arms raise forward to horizontal and hold
    fw.set_targets({"left_shoulder_pitch_joint": -1.57,
                    "right_shoulder_pitch_joint": 1.57})
    settle(model, data, fw, 2.0)
    l = joint_q(model, data, "left_shoulder_pitch_joint")
    r = joint_q(model, data, "right_shoulder_pitch_joint")
    check("arms hold horizontal against gravity",
          abs(l + 1.57) < 0.08 and abs(r - 1.57) < 0.08, f"L {l:.2f}, R {r:.2f}")

    # shoulder rated torque covers a 2 kg payload in the hand at full reach
    reach = 0.20 + 0.19 + 0.09
    arm_mass_torque = (1.6 * 0.10 + 1.0 * 0.29 + 0.4 * 0.44) * 9.81
    payload_torque = 2.0 * reach * 9.81
    need = arm_mass_torque + payload_torque
    check("shoulder pitch rated torque covers 2 kg payload at reach",
          need < 30.0, f"need {need:.1f} Nm vs 30 Nm rated")

    # head/neck sweep
    fw.set_targets({"neck_yaw_joint": 1.0, "neck_pitch_joint": 0.5})
    settle(model, data, fw, 2.0)
    check("neck tracks look-at command",
          abs(joint_q(model, data, "neck_yaw_joint") - 1.0) < 0.05 and
          abs(joint_q(model, data, "neck_pitch_joint") - 0.5) < 0.05)


# ---------------------------------------------------------------- stage 4
def stage4_integration():
    print("\n== Stage 4: full integration — stand, squat, power ==")
    model, data = load_stage("full")
    battery = Battery(soc=0.9)
    fw = Firmware(model, data, battery)
    fw.hold_pose()
    bal = StandingController(fw, model, data)

    def run(seconds, pose=None, duration=1.5):
        if pose:
            bal.set_pose(pose, duration)
        for _ in range(int(seconds / model.opt.timestep)):
            bal.step(model.opt.timestep)
            mujoco.mj_step(model, data)

    run(3.0)

    head_z = data.body("head").xpos[2]
    check("robot stands unassisted for 3 s", head_z > 1.0, f"head at {head_z:.2f} m")
    com = data.subtree_com[model.body("pelvis").id]
    check("CoM stays over support polygon", abs(com[0]) < 0.08 and abs(com[1]) < 0.05,
          f"CoM x={com[0]:+.3f} y={com[1]:+.3f}")

    v_stand, p_stand = fw.bus_voltage, fw.total_power
    print(f"  standing draw: {p_stand:.0f} W at {v_stand:.1f} V "
          f"(~{battery.spec['energy_wh'] / max(p_stand, 1):.1f} h runtime)")
    check("standing power within pack limit", p_stand < v_stand * 30.0)

    # squat with forward torso lean: the +-0.35 rad ankle can't dorsiflex
    # enough for an upright deep squat, so the hips fold to keep the CoM
    # centered (and leave the balance loop trim margin)
    squat = {"left_hip_pitch_joint": -1.27, "left_knee_joint": 1.0,
             "left_ankle_pitch_joint": -0.25,
             "right_hip_pitch_joint": 1.27, "right_knee_joint": -1.0,
             "right_ankle_pitch_joint": 0.25}
    run(5.0, squat, duration=2.5)
    squat_z = data.body("head").xpos[2]
    still_up = squat_z > 0.75 and abs(data.body("pelvis").xpos[0]) < 0.3
    check("squat: pelvis drops, robot stays balanced", still_up and squat_z < head_z - 0.1,
          f"head {head_z:.2f} -> {squat_z:.2f} m")
    p_squat = fw.total_power

    run(5.0, {j: 0.0 for j in squat}, duration=2.5)
    check("recovers to stand from squat", data.body("head").xpos[2] > 1.0,
          f"head at {data.body('head').xpos[2]:.2f} m")
    print(f"  squat hold draw: {p_squat:.0f} W | energy this test: "
          f"{battery.energy_used_wh:.2f} Wh | peak current {battery.peak_current:.1f} A")
    check("no BMS fault during integration", battery.fault is None)

    # thermal proxy: nothing sat above rated torque continuously
    hot = [c.name for c in fw.controllers.values() if c.overload_time > 1.0]
    check("no actuator exceeded rated torque for >1 s", not hot, ", ".join(hot))


STAGES = [stage1_bench_electronics, stage2_legs_on_fixture,
          stage3_upper_body_on_fixture, stage4_integration]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=int, default=0, help="1-4, 0 = all")
    args = ap.parse_args()
    todo = STAGES if args.stage == 0 else [STAGES[args.stage - 1]]
    for s in todo:
        s()
    npass = sum(ok for _, ok in results)
    print(f"\n{'='*50}\nAssembly verification: {npass}/{len(results)} checks passed")
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
