# Operator guide — running the FR3 connector-mating cell

*Written 2026-09-22, after the workspace restructure and the tracking work.
Start here if you are about to use the cell. This guide is task-oriented and
points at the authorities rather than repeating them; where it disagrees with
an older document, see [DOCS.md](DOCS.md).*

---

## 0. What changed on 2026-09-22

| Change | What it means for you |
|---|---|
| `move_l` is now **`mating_node`** | The old name was MELFA's "MoveL". The ROS node name `connector_mating_node` is UNCHANGED, so every `/connector_mating_node/*` service and every params file still works |
| Packages split | `mating_controller` (portable core), `melfa/` (parked MELFA side), `fr3_mating_controllers` (FR3 impedance controller) |
| `setpoint_slew_mps` / `_rps` are **live** | You can change the equilibrium speed limit without a cleanup/reconfigure cycle. Slew, not stiffness, is the real speed limit |
| Panel has **TRACK / STOP TRACKING** | Continuous marker following. **Do not use yet — see section 5** |
| [GUIDE.md](GUIDE.md), [DOCS.md](DOCS.md) | This guide, and an index saying which document to believe |

---

## 1. Before you power anything

- **Mains power.** Not battery. Two stack crashes on 2026-09-22 were traced to
  the laptop running on battery: the CPU cannot hold its clocks, the 1 ms FCI
  deadline slips, and libfranka aborts `ros2_control_node` with
  `communication_constraints_violation`. `fr3_preflight` FAILs on battery;
  the governor check alone would not catch it - it still reads `performance`.
- **RT throttling off:** `cat /proc/sys/kernel/sched_rt_runtime_us` should be
  `-1` (`fr3_preflight` FAILs otherwise). Set with
  `sudo sysctl -w kernel.sched_rt_runtime_us=-1`. That lasts until the next
  reboot; to make it permanent, once:
  `echo 'kernel.sched_rt_runtime_us = -1' | sudo tee /etc/sysctl.d/99-fr3-rt.conf`.
- **Desk:** unlock the joints, activate FCI, then **close the Desk browser
  tab**. Its persistent HTTPS connections share the robot link
  (`fr3_preflight` WARNs while one is open).
- **Payload is in Desk**, not the panel: the active end-effector profile is
  "Franka Hand with D405" at 0.83 kg, CoM `[-5, -5, 32]` mm. The panel's
  payload field therefore gets **0**. Never count it twice.

---

## 2. Bring-up

Two terminals, each with the cell environment (`~/.bashrc` on this PC
already sources it in every shell):

```bash
source "/home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src/tools/fr3/fr3_env.sh"
```

It pins DDS off the robot NIC and sources ROS 2, `~/franka_ros2_ws` and this
workspace itself, in that order - so the old "source `fr3_env.sh` LAST" rule
is gone: the order it protected is now fixed inside the script. It
prints a `[FAIL]` line if `fr3_mating_controllers` does not resolve to this
workspace; then build it (`colcon build --symlink-install`) and source again.

```bash
# T1 - preflight; the driver + MoveIt start only if it has 0 FAILs
fr3_preflight && ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP

# T2 - the cell: impedance controller (INACTIVE), hand-eye TF, vision,
#      tracking node and the cell panel
fr3_cell
```

`fr3_cell` is `ros2 launch tools/fr3/fr3_cell.launch.py` from any directory;
`fr3_cell --show-args` lists its arguments. What it does and does not do:

- **Spawns `cartesian_impedance_stroke_controller` inactive** and exits.
  Relaunching T2 while T1 runs is safe: an already-active controller is left
  running untouched - the spawner then prints "Failed to configure
  controller" and exits, which is expected. Edits to
  `cartesian_impedance_stroke.yaml` take effect only after restarting T1,
  then T2. **Restarting only T1** (after a reflex) is fine: the panel sees the
  controller manager come back without the impedance controller and, 15 s
  later, runs the same spawner itself (log: "loaded again, inactive").
- **Hand-eye TF from `tools/fr3/calib/handeye.yaml`**, the one copy; the log
  line says whether an override was used instead.
- **In-process capture** (`vision_source:=realsense`, the default): no image
  topic ever reaches DDS - "defence 0" in `tools/fr3/README.md`.
  `vision_source:=topic` expects a separate `realsense2_camera` driver.
- **The panel** starts with it (`start_panel:=false` to run
  `python3 tools/fr3/cell/cell.py` yourself; `fr3_cell mock:=true` runs it
  against a fake cell, no robot - see `tools/fr3/cell/README.md`). Ctrl-C in T2 lets the panel
  hand the arm back before it exits: normally a second, up to about a minute
  if a service is not answering - do not kill it meanwhile.
- **No `mating_node`.** The autonomous phase machine is parked
  ([melfa/parked/README.md](melfa/parked/README.md)).

Check it came up:

```bash
ros2 control list_controllers      # fr3_arm_controller active,
                                   # cartesian_impedance_stroke_controller inactive
ros2 topic hz /aruco/debug_image   # ~4-5 Hz once the panel subscribes
ros2 topic hz /aruco/pose          # only while the marker is actually seen
```

If `debug_image` flows but `pose` is silent, the camera is fine and the
marker is not being detected - look at the panel's camera view, which is
exactly the question it answers. Check `marker_id` too (default `0`, the
marker now on the cell): cam_pub discards every other id.

---

## 3. The commissioning ladder

Full detail and the physics:
[fr3_mating_controllers/README.md](fr3_mating_controllers/README.md). Rungs
0-3 have passed on this arm; rung 4 has not.

| Rung | Press | Pass looks like | Stop if |
|---|---|---|---|
| 0 | **PRE-FLIGHT** (payload `0`, CoM `0,0,0`) | "ladder is unlocked", `\|F ext\|` at rest **under ~2 N** | rest force above 5 N, or the arm controller does not come back |
| 1 | **FLOAT** | weightless, holds height when you let go, RT pill 100% | buzz, kicks, RT below 99%, or a slow sag (payload wrong) |
| 2 | **HOLD** | does not move; ~15 N moves it 10 cm sideways, ~12 N moves it 1.5 cm on tool Z | ringing beyond one small overshoot, buzz, drift |
| 3 | **SETPOINT** (up first) | glides at the slew limit, settles | it lurches, or refuses repeatedly for a reason you do not understand |
| 4 | dispatched stroke | - | **not attempted** - needs the force thresholds tuned first |

**It will not reach the setpoint exactly, and that is physics, not a fault.**
Joint friction gives a deadband of `F_friction / k` - about 3.5-6.5 N at the
TCP, so ~6 mm at k=600 and ~1.8 mm at k=3000. TODO lesson 10 has the numbers.

PRE-FLIGHT is **per driver session**. Relaunch the driver, run it again.

---

## 4. Live tuning

The gain fields and, new since 2026-09-22, the slew limits apply while the
controller is holding - no deactivate, no re-seed.

| Knob | Range | Notes |
|---|---|---|
| `k lateral` / `k tool Z` | 0-3000 N/m | Set both equal for an isotropic spring; an anisotropic K deflects force off the commanded direction when the tool is tilted |
| `k roll/pitch`, `k yaw` | 0-300 Nm/rad | **~90 is the measured ceiling** - noise and vibration above it |
| `damping zeta` | 0.1-2.0 | Leave at **1.0**. Zero overshoot in 32 clean steps; friction is itself a damper |
| `setpoint_slew_mps` | 0.001-0.25 m/s | The real speed limit. A 50 mm step took 1.05 s against a 0.96 s slew floor |
| `setpoint_slew_rps` | 0.001-1.0 rad/s | |

Out-of-range values are rejected **as a whole set** - the controller never
applies half a set.

---

## 5. Continuous tracking

**Run it:** HOLD → **ALIGN** (100 mm target) → **3b. TRACK**. TRACK holds the
camera where ALIGN leaves it, so it starts at zero error; without ALIGN the goal
can be far away and the node just holds. **STOP TRACKING** sits in the window
header on every tab; ALIGN's STOP NOW also stops the tracker.

The banner shows the node's own status:

| Banner | Meaning |
|---|---|
| `TRACKING` | following; error, lead and policy in the subtitle |
| `HOLDING - <reason>` | still armed, arm holding still; follows again **by itself** once the reason clears |
| `STOPPED - <reason>` | tracking ended and your gains are restored; TRACK starts it again |

It **holds** by itself when the goal is past the lead cap (60 mm / 15 deg,
with the over-lead dropdown on `hold`), below the Z floor (100 mm above the
base), outside the drawer's workspace box, when it would pull a joint within
8 deg of its end stop further in, or when vision is not fresh (no raw
detection within 0.25 s). Workspace limits always hold, never stop. It
**stops** by itself when a joint **buzzes** (more than 3.5 Nm rms above 20 Hz),
when the impedance controller leaves ACTIVE, or past the cap with the dropdown
on `stop`.

TRACK keeps **your applied gains** (`tracking_use_operator_gains: true`,
since 2026-09-24); the node scales its friction deadband and extra-pull caps
to that stiffness, and logs both at START. With it set to false, TRACK uses
the track profile: k 1500 N/m, k_rot 90 Nm/rad, **ζ 0.5**, slew 0.10 m/s.
ζ was 1.0 until 2026-09-23, when it drove a 40 Hz wrist buzz (1 kHz data in
[TRACKING_SPEC.md](TRACKING_SPEC.md) O1); 0.5 is **not yet proven on the arm**,
so keep the recorder running on the first runs:

```bash
ros2 run mating_controller state_recorder     # 1 kHz CSV into runs/<day>/
python3 tools/fr3/analyse_trace.py <tracking_*.jsonl>   # includes the buzz level
```

Design is in [TRACKING_SPEC.md](TRACKING_SPEC.md); what is left is in
[TODO.md](TODO.md).

### The camera pane

The marker view is **on when the panel opens** - ALIGN cannot work blind,
and that is the configuration the proven alignment runs of 2026-09-11 and
09-15 used. `/aruco/debug_image` is capped at `debug_max_hz` (5 Hz) and
travels on loopback only.

The **show the marker view** checkbox stays, because the subscription is
what costs anything: the publisher only encodes the overlay while something
is subscribed, so unticking it takes the frames off DDS entirely rather than
just hiding them. Untick it if you want the quietest possible graph during
torque work - though none of the three stack deaths on 2026-09-16/22 was
traced to it (two were the laptop on battery, one correlated with the 1 kHz
state relay, which is a different subscription).
