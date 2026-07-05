"""Interactive build-tour server.

Runs the MuJoCo simulation headless and serves a browser dashboard that
walks through the Asimov 1 build stage by stage. The browser renders the
scene with three.js from streamed geom poses — no MuJoCo UI involved.

    python tour/server.py     ->  http://localhost:8321
"""
import json
import sys
import threading
import time
from pathlib import Path

import mujoco
from flask import Flask, jsonify, request, send_from_directory

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from electronics.battery import Battery
from electronics.can_bus import check_control_rate, build_branches
from firmware.robot import Firmware, load_stage, CONTROL_HZ
from control.balance import StandingController

BENCH_XML = """
<mujoco model="bench">
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.3 0.35 0.3 1"/>
    <body name="table" pos="0 0 0.4">
      <geom type="box" size="0.5 0.35 0.02" rgba="0.55 0.42 0.30 1"/>
      <geom type="box" pos="0.45 0.30 -0.2" size="0.02 0.02 0.19" rgba="0.4 0.3 0.2 1"/>
      <geom type="box" pos="-0.45 0.30 -0.2" size="0.02 0.02 0.19" rgba="0.4 0.3 0.2 1"/>
      <geom type="box" pos="0.45 -0.30 -0.2" size="0.02 0.02 0.19" rgba="0.4 0.3 0.2 1"/>
      <geom type="box" pos="-0.45 -0.30 -0.2" size="0.02 0.02 0.19" rgba="0.4 0.3 0.2 1"/>
      <geom name="battery" type="box" pos="-0.2 0 0.06" size="0.09 0.13 0.045" rgba="0.85 0.65 0.1 1"/>
      <geom name="mcb" type="box" pos="0.15 0.12 0.03" size="0.08 0.06 0.008" rgba="0.1 0.5 0.2 1"/>
      <geom name="pi5" type="box" pos="0.15 -0.12 0.03" size="0.045 0.03 0.008" rgba="0.1 0.45 0.25 1"/>
      <geom name="cm5" type="box" pos="0.32 -0.12 0.03" size="0.04 0.028 0.008" rgba="0.15 0.4 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"""

STAGE_MODELS = {
    "intro": "full", "electronics": None, "legs": "legs",
    "upper_body": "upper_body", "full": "full",
}


def mirror_legs(hip, knee, ankle):
    return {"left_hip_pitch_joint": hip, "left_knee_joint": knee,
            "left_ankle_pitch_joint": ankle,
            "right_hip_pitch_joint": -hip, "right_knee_joint": -knee,
            "right_ankle_pitch_joint": -ankle}


SQUAT = mirror_legs(-1.27, 1.0, -0.25)
STAND = mirror_legs(0.0, 0.0, 0.0)


class Tour:
    def __init__(self):
        self.lock = threading.Lock()
        self.results = []
        self.script = []          # queued moves: dicts with targets/until/check
        self.script_t_end = 0.0
        self.stage = None
        self.set_stage("intro")
        threading.Thread(target=self.loop, daemon=True).start()

    # ---------------------------------------------------------- stages
    def set_stage(self, name):
        with self.lock:
            self.stage = name
            self.results = []
            self.script = []
            model_kind = STAGE_MODELS[name]
            if model_kind is None:
                self.model = mujoco.MjModel.from_xml_string(BENCH_XML)
                self.data = mujoco.MjData(self.model)
                self.fw = self.bal = None
            else:
                self.model, self.data = load_stage(model_kind)
                self.fw = Firmware(self.model, self.data, Battery(), check_can=False)
                self.fw.hold_pose()
                self.bal = (StandingController(self.fw, self.model, self.data)
                            if model_kind == "full" else None)
            mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------------- sim loop
    def loop(self):
        dt = 0.002
        while True:
            t0 = time.perf_counter()
            with self.lock:
                for _ in range(10):  # 20 ms of physics per tick
                    self._advance_script()
                    if self.bal is not None:
                        self.bal.step(dt)
                    elif self.fw is not None:
                        self.fw.step_control(dt)
                    mujoco.mj_step(self.model, self.data)
            time.sleep(max(0.0, 0.02 - (time.perf_counter() - t0)))

    def _advance_script(self):
        if not self.script:
            return
        step = self.script[0]
        if not step.get("applied"):
            step["applied"] = True
            step["until"] = self.data.time + step["duration"] + step.get("hold", 0.5)
            targets = step.get("targets")
            if targets:
                if self.bal is not None:
                    self.bal.set_pose(targets, duration=step["duration"])
                else:
                    self.fw.set_targets(targets, duration=step["duration"])
        if self.data.time >= step["until"]:
            check = step.get("check")
            if check:
                self.results.append(check())
            self.script.pop(0)

    def joint_q(self, name):
        j = self.model.joint(name)
        return float(self.data.qpos[self.model.jnt_qposadr[j.id]])

    # ---------------------------------------------------------- actions
    def run_action(self, action):
        with self.lock:
            fn = getattr(self, f"action_{action}", None)
            if fn is None:
                return False
            fn()
            return True

    def _track_check(self, jname, target, tol=0.05):
        def check():
            err = abs(self.joint_q(jname) - target)
            short = jname.replace("_joint", "")
            return (f"{short} reaches {target:+.2f} rad", err < tol,
                    f"error {err:.3f} rad")
        return check

    def action_bench_tests(self):
        b = Battery()
        v = b.step(60.0, 1.0)
        self.results.append(("battery voltage sane at light load",
                             50.0 < v < 54.6, f"{v:.1f} V"))
        b2, tripped = Battery(), False
        try:
            b2.step(2000.0, 0.01)
        except Exception:
            tripped = True
        self.results.append(("BMS trips on over-current (2 kW dead short)", tripped,
                             "protection circuit opened"))
        ok, report = check_control_rate(CONTROL_HZ)
        for line in report.split("\n"):
            self.results.append((f"CAN {line.strip()}", True, ""))
        self.results.append((f"all branches fit {CONTROL_HZ} Hz control with 20% headroom", ok, ""))
        ids = sorted(i for br in build_branches() for i in br.actuator_ids)
        self.results.append(("all 25 actuators enumerate on the bus",
                             ids == list(range(1, 26)), ""))

    def action_leg_sweep(self):
        seq = [("right_hip_roll_joint", 0.5, None),      # park right leg out
               ("left_hip_pitch_joint", -1.67, 0.05),
               ("left_hip_pitch_joint", 0.0, None),
               ("left_knee_joint", 1.2, 0.05),
               ("left_knee_joint", 0.0, None),
               ("left_ankle_pitch_joint", -0.28, 0.05),
               ("left_ankle_pitch_joint", 0.0, None),
               ("left_hip_roll_joint", -0.5, 0.05),
               ("left_hip_roll_joint", 0.0, None),
               ("right_hip_roll_joint", 0.0, None)]
        for jname, target, tol in seq:
            self.script.append({
                "targets": {jname: target}, "duration": 1.2, "hold": 0.6,
                "check": self._track_check(jname, target, tol) if tol else None})

    def action_arm_raise(self):
        self.script.append({
            "targets": {"left_shoulder_pitch_joint": -1.57,
                        "right_shoulder_pitch_joint": 1.57},
            "duration": 1.5, "hold": 1.0,
            "check": self._track_check("left_shoulder_pitch_joint", -1.57, 0.08)})
        self.script.append({
            "targets": {"left_shoulder_pitch_joint": 0.0,
                        "right_shoulder_pitch_joint": 0.0},
            "duration": 1.5, "hold": 0.5, "check": None})

    def action_neck_test(self):
        self.script.append({
            "targets": {"neck_yaw_joint": 1.0, "neck_pitch_joint": 0.5},
            "duration": 1.2, "hold": 0.8,
            "check": self._track_check("neck_yaw_joint", 1.0)})
        self.script.append({
            "targets": {"neck_yaw_joint": 0.0, "neck_pitch_joint": 0.0},
            "duration": 1.2, "hold": 0.3, "check": None})

    def action_stand_check(self):
        def check():
            h = float(self.data.body("head").xpos[2])
            return ("robot stands unassisted", h > 1.0, f"head at {h:.2f} m")
        self.script.append({"targets": None, "duration": 3.0, "check": check})

    def action_squat(self):
        def check():
            h = float(self.data.body("head").xpos[2])
            return ("deep squat held in balance", 0.75 < h < 1.0, f"head at {h:.2f} m")
        self.script.append({"targets": SQUAT, "duration": 2.5, "hold": 1.5, "check": check})
        self.script.append({"targets": STAND, "duration": 2.5, "hold": 1.0,
                            "check": lambda: ("recovers to stand",
                                              float(self.data.body("head").xpos[2]) > 1.0, "")})

    def action_wave(self):
        self.script.append({"targets": {"right_shoulder_pitch_joint": 2.8,
                                        "right_elbow_joint": -1.2},
                            "duration": 1.5, "hold": 0.2, "check": None})
        for elbow in (-0.4, -1.2, -0.4):
            self.script.append({"targets": {"right_elbow_joint": elbow},
                                "duration": 0.6, "hold": 0.1, "check": None})
        self.script.append({"targets": {"right_shoulder_pitch_joint": 0.0,
                                        "right_elbow_joint": 0.0},
                            "duration": 1.5, "hold": 0.2, "check": None})

    def action_look_around(self):
        for yaw in (1.2, -1.2, 0.0):
            self.script.append({"targets": {"neck_yaw_joint": yaw},
                                "duration": 1.0, "hold": 0.3, "check": None})

    # ---------------------------------------------------------- state
    def scene(self):
        with self.lock:
            m = self.model
            geoms = []
            for i in range(m.ngeom):
                geoms.append({
                    "type": int(m.geom_type[i]),
                    "size": [float(x) for x in m.geom_size[i]],
                    "rgba": [float(x) for x in m.geom_rgba[i]],
                })
            return {"geoms": geoms, "stage": self.stage}

    def state(self):
        with self.lock:
            d, m = self.data, self.model
            tel = None
            if self.fw is not None:
                b = self.fw.battery
                tel = {"power": round(self.fw.total_power, 1),
                       "voltage": round(self.fw.bus_voltage, 2),
                       "soc": round(b.soc, 4),
                       "peak_a": round(b.peak_current, 2),
                       "energy_wh": round(b.energy_used_wh, 3)}
                if self.bal is not None:
                    tel["com_err_mm"] = round(self.bal.com_error() * 1000, 1)
                    tel["head_z"] = round(float(d.body("head").xpos[2]), 3)
            return {
                "t": round(d.time, 2),
                "busy": bool(self.script),
                "stage": self.stage,
                "pos": [[round(float(v), 4) for v in d.geom_xpos[i]] for i in range(m.ngeom)],
                "mat": [[round(float(v), 4) for v in d.geom_xmat[i]] for i in range(m.ngeom)],
                "telemetry": tel,
                "results": [{"name": n, "ok": bool(ok), "detail": det}
                            for n, ok, det in self.results],
            }


tour = Tour()
app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"),
            static_url_path="")


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/scene")
def scene():
    return jsonify(tour.scene())


@app.get("/api/state")
def state():
    return jsonify(tour.state())


@app.post("/api/stage")
def set_stage():
    name = request.json.get("stage")
    if name not in STAGE_MODELS:
        return jsonify({"ok": False}), 400
    tour.set_stage(name)
    return jsonify({"ok": True})


@app.post("/api/action")
def action():
    ok = tour.run_action(request.json.get("action", ""))
    return jsonify({"ok": ok})


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8321, threaded=True)
