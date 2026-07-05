"""Commissioning run: the fully assembled robot executes a demo routine
(stand -> squats -> arm wave -> head scan) with full telemetry.

    python run_asimov.py            # headless, writes telemetry.csv
    python run_asimov.py --view     # live MuJoCo viewer
"""
import argparse
import csv
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from electronics.battery import Battery
from firmware.robot import Firmware, load_stage
from control.balance import StandingController


def mirror_legs(hip, knee, ankle):
    return {"left_hip_pitch_joint": hip, "left_knee_joint": knee,
            "left_ankle_pitch_joint": ankle,
            "right_hip_pitch_joint": -hip, "right_knee_joint": -knee,
            "right_ankle_pitch_joint": -ankle}


SQUAT = mirror_legs(-1.27, 1.0, -0.25)
STAND = mirror_legs(0.0, 0.0, 0.0)

ROUTINE = [
    # (label, pose targets, move duration s, hold s)
    ("stand",      STAND, 1.0, 3.0),
    ("squat 1",    SQUAT, 2.5, 2.0),
    ("stand",      STAND, 2.5, 1.0),
    ("squat 2",    SQUAT, 2.5, 2.0),
    ("stand",      STAND, 2.5, 1.0),
    ("wave up",    {"right_shoulder_pitch_joint": 2.8, "right_elbow_joint": -1.2}, 1.5, 0.5),
    ("wave out",   {"right_elbow_joint": -0.4}, 0.6, 0.2),
    ("wave in",    {"right_elbow_joint": -1.2}, 0.6, 0.2),
    ("wave out",   {"right_elbow_joint": -0.4}, 0.6, 0.2),
    ("arm down",   {"right_shoulder_pitch_joint": 0.0, "right_elbow_joint": 0.0}, 1.5, 0.5),
    ("look left",  {"neck_yaw_joint": 1.2}, 1.0, 0.5),
    ("look right", {"neck_yaw_joint": -1.2}, 1.5, 0.5),
    ("look ahead", {"neck_yaw_joint": 0.0, "neck_pitch_joint": 0.0}, 1.0, 1.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="live viewer")
    ap.add_argument("--soc", type=float, default=1.0, help="initial state of charge")
    args = ap.parse_args()

    model, data = load_stage("full")
    battery = Battery(soc=args.soc)
    fw = Firmware(model, data, battery)
    fw.hold_pose()
    bal = StandingController(fw, model, data)
    dt = model.opt.timestep

    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    log_path = ROOT / "telemetry.csv"
    t = 0.0
    with open(log_path, "w", newline="") as f:
        log = csv.writer(f)
        log.writerow(["t", "phase", "head_z", "com_err_x", "power_w",
                      "bus_v", "soc", "peak_a"])
        for label, pose, move, hold in ROUTINE:
            bal.set_pose(pose, duration=move)
            for i in range(int((move + hold) / dt)):
                bal.step(dt)
                mujoco.mj_step(model, data)
                t += dt
                if i % 25 == 0:
                    log.writerow([f"{t:.3f}", label,
                                  f"{data.body('head').xpos[2]:.4f}",
                                  f"{bal.com_error():+.4f}",
                                  f"{fw.total_power:.1f}",
                                  f"{fw.bus_voltage:.2f}",
                                  f"{battery.soc:.5f}",
                                  f"{battery.peak_current:.2f}"])
                    if viewer:
                        viewer.sync()
            head = data.body("head").xpos[2]
            print(f"[{t:7.2f}s] {label:10s} head {head:.2f} m  "
                  f"{fw.total_power:5.1f} W  {fw.bus_voltage:.1f} V  "
                  f"SOC {battery.soc*100:.2f}%")
            if head < 0.6:
                print("!! robot fell — aborting routine")
                break

    est_runtime_h = battery.spec["energy_wh"] / max(fw.total_power, 1)
    print(f"\nRoutine complete in {t:.1f} s simulated.")
    print(f"Energy used: {battery.energy_used_wh:.2f} Wh of "
          f"{battery.spec['energy_wh']:.0f} Wh | peak current "
          f"{battery.peak_current:.1f} A (BMS limit {battery.oc_limit:.0f} A)")
    print(f"Estimated runtime at final draw: {est_runtime_h:.1f} h")
    print(f"Telemetry: {log_path}")
    if viewer:
        print("Viewer open — close the window to exit.")
        import time
        while viewer.is_running():
            time.sleep(0.2)


if __name__ == "__main__":
    main()
