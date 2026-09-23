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
  `communication_constraints_violation`. `fr3_preflight.sh` does **not** check
  this - the governor still reads `performance` on battery.
- **RT throttling off:** `cat /proc/sys/kernel/sched_rt_runtime_us` should be
  `-1`. Set with `sudo sysctl -w kernel.sched_rt_runtime_us=-1`.
- **Desk:** unlock the joints, activate FCI, then **close the Desk browser
  tab**. Its persistent HTTPS connections share the robot link.
- **Payload is in Desk**, not the panel: the active end-effector profile is
  "Franka Hand with D405" at 0.83 kg, CoM `[-5, -5, 32]` mm. The panel's
  payload field therefore gets **0**. Never count it twice.

---

## 2. Bring-up

Every terminal starts with this, and **`fr3_env.sh` must be last**:

```bash
cd "/home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src"
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source tools/fr3/fr3_env.sh          # LAST
```

Sourcing `franka_ros2_ws/install/setup.bash` afterwards rebuilds
`AMENT_PREFIX_PATH` from its own chain and silently drops this workspace. The
DDS pin survives, so everything looks fine until the spawner fails with
"Failed loading controller".

```bash
# T1 - preflight, then driver + MoveIt
tools/fr3/fr3_preflight.sh $FR3_ROBOT_IP        # want 9 ok / 0 fail
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP

# T2 - impedance controller, loaded INACTIVE (exits when done)
ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
    --inactive --param-file "$(pwd)/fr3_mating_controllers/config/cartesian_impedance_stroke.yaml"

# T3 - vision + hand-eye TF + tracking node
ros2 launch tools/fr3/fr3_mating.launch.py robot_ip:=$FR3_ROBOT_IP \
    vision_source:=realsense start_mating_node:=false

# T4 - the cell panel
python3 tools/fr3/cell_panel.py
```

**Why those two launch arguments:**

- `vision_source:=realsense` makes `cam_pub` capture in-process through
  pyrealsense2, so no image topic ever reaches DDS. The default (`topic`)
  instead expects a separate `realsense2_camera` driver to be running, and
  puts the frames on the graph. With a 1 kHz FCI loop next door, prefer
  in-process - this is "defence 0" in `tools/fr3/README.md`.
- `start_mating_node:=false` leaves the autonomous phase machine out.
  Given a marker, `mating_node` aligns the arm **by itself**, which is not
  what you want while driving the cell from the panel. Drop the argument
  when you actually want an autonomous mating run (and a working MoveIt).

Check it came up:

```bash
ros2 control list_controllers      # fr3_arm_controller active,
                                   # cartesian_impedance_stroke_controller inactive
ros2 topic hz /aruco/debug_image   # ~4-5 Hz once the panel subscribes
ros2 topic hz /aruco/pose          # only while the marker is actually seen
```

If `debug_image` flows but `pose` is silent, the camera is fine and the
marker is not being detected - look at the panel's camera view, which is
exactly the question it answers.

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

## 5. Continuous tracking — blockers fixed, still unproven

The panel has **3b. TRACK** and **STOP TRACKING**, and a **camera pane**
(off by default). All five blockers the 2026-09-22 reviews raised are now
fixed:

| Was | Now |
|---|---|
| RELEASE left an orphaned tracker streaming at 50 Hz | RELEASE stops tracking before the switch, **and** the node self-halts when the impedance controller leaves ACTIVE - it no longer depends on the panel remembering |
| STOP raced the 50 Hz tick, so the last message could be the lead | One mutex serialises `halt_tracking()` against the tick's read-modify-publish |
| STOP queued behind an in-flight START (~11 s) | STOP has its own callback group |
| Ctrl-C could not restore the gains | Restored from a **pre**-shutdown callback, while the executor still spins; if it still fails the node logs FATAL and names the profile left in force |
| Panel timeout 6 s < the node's ~11 s start | 15 s, and a timeout now says "the node may be tracking; press STOP TRACKING" rather than claiming failure |

**It has still never run on the arm, and the fixes have not been reviewed.**
Outstanding from the same reviews, not blockers but know them before you
press TRACK:

- tracking stalls permanently and silently if the goal is ever more than
  60 mm from the arm - the veto that stops it is the same thing that would
  let it recover
- no angular equivalent of that 60 mm lead cap
- the 50 Hz callback group does a synchronous disk write every tick
- worst-case commanded force is **30 N**, not the 15 N `TRACKING_SPEC.md`
  section 6 implies - that table bounds the integrator, not the whole wrench

Design is in [TRACKING_SPEC.md](TRACKING_SPEC.md); the fix list and what is
left are in [TODO.md](TODO.md).

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
