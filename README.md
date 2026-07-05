# Asimov 1 Humanoid — Virtual Build

A high-fidelity software construction of the [Menlo Asimov 1](https://docs.menlo.ai/asimov/1)
open-source humanoid (~$16k to build physically), simulated end-to-end in MuJoCo:
mechanics, electronics, firmware, staged assembly, verification, and commissioning.

Every number that Menlo publishes is used verbatim; everything they don't publish
(link dimensions, masses) is estimated and clearly marked as such in the specs.

## The robot

1.23 m, 35 kg, 25 actuated DOF + 2 passive toes. Six CAN branches off a Motion
Control Board, 13S4P Li-ion pack (46.8 V, 10.5 Ah, 491 Wh), Raspberry Pi 5 +
Radxa CM5 compute. MuJoCo is the engine Menlo's own `robotics-sim` repo uses.

## Layout — mirrors the real engineering stack

| Directory | Real-world analogue | Contents |
|---|---|---|
| `specs/` | Datasheets & BOM | Actuator table (all 25 motors: ranges, Kt, rated/peak torque, velocity limits), battery, CAN topology, geometry estimates |
| `model/` | CAD & fabrication | `build_mjcf.py` generates the MJCF robot from the specs, including partial builds for assembly stages |
| `electronics/` | Power & comms | Battery model (OCV curve, sag, SOC, BMS trips), CAN bandwidth/latency budget |
| `firmware/` | Motor drivers + MCB | Per-joint PD loops with torque/velocity limits, trajectory slew limiting, gravity feed-forward, current & power draw, RSU ankle kinematics |
| `control/` | Locomotion controller | Standing balance via CoM-feedback ankle strategy |
| `assembly/` | Assembly & verification docs | 4-stage build with 18 acceptance checks |

## Run it

Install with [uv](https://docs.astral.sh/uv/) (`winget install astral-sh.uv` on Windows,
or `curl -LsSf https://astral.sh/uv/install.sh | sh` on macOS/Linux):

```
uv venv
uv pip install -r requirements.txt
```

**Start here — the interactive build tour** (guided, no robotics background needed):

```
uv run tour/server.py         # then open http://localhost:8321
```

Five stages walk through the build with plain-English explanations, live 3D,
one-click tests, and real telemetry: meet the robot → bench electronics →
legs on fixture → upper body → full integration (squat, wave, balance).

Engineer's tools:

```
uv run assembly/verify.py     # staged assembly, 18 acceptance checks
uv run run_asimov.py          # commissioning routine, writes telemetry.csv
uv run run_asimov.py --view   # same, with raw MuJoCo debug viewer
```

Regenerate models after editing specs: `uv run model/build_mjcf.py --stage full`
(stages: `legs`, `lower_body`, `upper_body`, `full`).

## Assembly stages & what they check

1. **Bench electronics** — battery voltage/energy, BMS over-current trip,
   CAN enumeration of all 25 actuators, bus utilisation at 500 Hz control
   (worst branch: leg, 78% of 1 Mbps).
2. **Legs on fixture** — every leg joint swept to 80% of range with tracking
   < 0.05 rad; RSU ankle motor stroke covers the joint envelope; passive toe
   spring return.
3. **Upper body on fixture** — arms hold horizontal against gravity; shoulder
   rated torque covers a 2 kg payload at full reach; neck look-at tracking.
4. **Integration** — unassisted standing, CoM over support polygon, deep squat
   and recovery, power/current/BMS margins, actuator thermal proxy.

## Engineering log — issues found and fixed during the virtual build

These all surfaced from the simulation, the same way they would on a real build:

1. **Adjacent parts collide at joints** — thigh/pelvis, arm/torso, hand/thigh
   overlapped by design; added contact exclusions (real robots rely on
   mechanical clearance the primitive geometry doesn't capture).
2. **BMS over-current on motion start** — stepping PD targets demanded 54 A
   (> 30 A limit). Fixed with trajectory slew limiting in the firmware.
3. **PD droop on horizontal limbs** — 0.09 rad sag holding a leg out; fixed
   with model-based gravity feed-forward, capped at rated torque.
4. **Squat topples backward** — loaded knees sag (gravity FF can't see ground
   contact), thigh tilts back, CoM walks off the heels. Fixed three ways:
   stiffer stance gains (10× rated), a CoM-feedback ankle-strategy balance
   loop, and a squat pose with forward torso lean — the ±0.35 rad ankle can't
   dorsiflex enough for an upright deep squat, so the hips must fold, and the
   pose must leave the balance loop trim margin instead of parking the ankle
   on its range limit.

## Commissioning results

Stand → 2× deep squat → wave → head scan, 32 s simulated: balance held
throughout, 55 W standing / 78 W squatting, peak 1.9 A (BMS limit 30 A),
0.57 Wh used → ~9 h estimated standing runtime on the 491 Wh pack.

## Known simplifications

- Link geometry/masses estimated (Menlo ships STEP CAD only); total mass
  matches the 35 kg spec.
- RSU ankle simulated as two orthogonal hinges with joint-space limits from
  the spec table; the parallel linkage is modelled kinematically for motor
  stroke verification, not as physical pushrods.
- Motor thermals are an overload-time proxy, not a thermal circuit; winding
  resistance assumed (0.3 Ω) since it isn't published.
- No walking controller yet — locomotion on the real robot is a trained RL
  policy; this repo's balance layer is the classical baseline underneath.
