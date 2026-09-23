# Project state — Robotic Connector Mating Cell

*Snapshot: 2026-09-22. The short, current answer to "where are we?".
Detail lives in [STATUS.md](STATUS.md) (capabilities), [TODO.md](TODO.md)
(tasks, plus lessons 1–11 that should not be re-learned),
[TRACKING_SPEC.md](TRACKING_SPEC.md) (continuous tracking) and
[SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md) (reference manual).
[DOCS.md](DOCS.md) indexes all of them and says which copy to believe.*

## In one paragraph

A vision-guided connector-mating cell on a **Franka FR3** (the original target
was a Mitsubishi RV-5AS). A wrist-mounted RealSense D405 finds the connector's
ArUco marker, the arm aligns to it, and a compliant Cartesian-impedance stroke
is meant to insert. **Alignment works on the real robot. Insertion has never
run.** The impedance controller HAS now commanded torque on hardware:
commissioning rungs 0-3 (pre-flight, float, hold, setpoint) passed on
2026-09-16. Rung 4, the dispatched stroke, waits on the force thresholds.

## Progress at a glance

| Stage | State | Evidence / notes |
|---|---|---|
| FCI link and ROS stack | Working | DDS pinned to loopback (`tools/fr3/fr3_env.sh`). The stack still drops occasionally; see Risks |
| Camera: D405, 90 fps with depth | Working | Out-of-ROS capture; USB no-suspend udev rule; self-healing capture |
| Marker orientation measurement | Working | IPPE mirror flip (62% of frames) and 2.6° bias fixed — TODO lesson 1 |
| Hand-eye calibration | Done 2026-09-15, committed | Tsai, 21 poses, residual 3.17 mm / 1.57°. Validated: 8.14 mm static-marker scatter vs 170.6 mm for the old guess |
| Camera→marker alignment | Working | Cartesian backend in `tools/fr3/cell_panel.py`, ~1 mm, all 6 DOF including in-plane |
| Servo alignment backend | Parked, never validated | Stall and noise root-caused (TODO lesson 6); position-controller fix built but never run |
| Impedance insertion controller | **Rungs 0-3 run on hardware 2026-09-16** | Float smooth, hold solid, setpoints track; friction deadband measured (TODO lesson 10) |
| Continuous marker tracking | **Written, never run** | `mating_controller`'s `tracking_node`; compiles, pure law unit-tested, no adversarial review yet, no operator button yet |
| Force thresholds | Not tuned | Shared by the MoveIt stroke and the impedance stroke |
| Connector offsets | Not taught | `tools/fr3/fr3_params.yaml` |
| Full mate | Not attempted | |

## Where we are right now

- **2026-09-22: continuous tracking is written, and has never run.**
  - [TRACKING_SPEC.md](TRACKING_SPEC.md) was written first, against the
    measured numbers, and then **amended where the implementation proved it
    wrong** - the vision rate, the reachable static accuracy, the gain-profile
    mechanism and the frame the error is computed in. The amendments are
    marked in place.
  - The controller side is done: `setpoint_slew_mps` / `setpoint_slew_rps`
    are now **live** parameters, which matters because the slew, not the
    stiffness, is the speed limit (a 50 mm step took 1.05 s against a 0.96 s
    slew floor).
  - `mating_controller/src/tracking_node.cpp` streams `~/equilibrium_pose` at
    50 Hz from `standoff_goal` plus a bounded integral of the *measured*
    error; the pure law is in `tracking_law.hpp` and unit-tested. Starting it
    snapshots the controller's live gains and stopping restores them, so it
    never has to guess what the operator had. It refuses to start if it
    cannot read those gains, if `float_mode` is on, or if it cannot measure
    the `fr3_hand_tcp` → controller-EE offset.
  - **Three things are honestly missing.** The node has **never been run** -
    not on hardware, not against a live graph. It has had **no adversarial
    review**; an RT-safety critique called an earlier draft unsound and its
    fixes went in unverified. And the panel's **START / STOP TRACKING buttons
    do not exist**, so the only way to start it today is a `ros2 service
    call` - the one motion path in this cell that is not behind a GUI button.
  - The workspace was restructured the same day:
    `melfa_rv5as_masterclass` split into `mating_controller` (the active FR3
    work) and `melfa/` (parked), and `move_l` became `mating_node`. The ROS
    node name `connector_mating_node` is unchanged.
- **2026-09-16: the impedance controller ran on the real arm, rungs 0-3.**
  - Payload is configured in **Desk** ("Franka Hand with D405", 0.83 kg at
    [-5 -5 32] mm), so the panel's own payload is entered as **0 kg** - never
    both. PRE-FLIGHT reported |F ext| at rest of **1.0 N**.
  - FLOAT smooth, RT success 1.000 (min 0.94) throughout. HOLD solid: settled
    drift 0.01 mm/s. SETPOINT tracks, with a friction deadband.
  - **The one real finding: compliant control here has a Coulomb friction
    deadband of `F_friction / k`** - about 3.5 N at the TCP and 0.6 Nm at the
    wrist, so 23 mm at k=150, 6.8 mm at k=600, 3.4 deg at k_rot=10. Confirmed
    three ways; full write-up is TODO **lesson 10**. It is the arm, not the
    controller: CM at 1000 Hz, nothing clipped, no joint within 25 deg of a
    limit. Consequence: the stroke is fine (4.4 mm at k_z=800, covered by the
    10 mm overdrive), but **tracking and mating want opposite stiffness** -
    that is a gain schedule, and the spec for it is still to be written.
  - `cell_panel.py` (IMPEDANCE & TRACK tab) aborted at exit (daemon spin thread outliving the
    rclpy context). The arm HAD been handed back first, by luck of the race,
    not by construction. Fixed and mutation-tested - TODO lesson 11.
- **Earlier: two review rounds on the impedance work, all fixes applied.**
  - *Round 1* (verified audit of the controller, panel and ladder): live
    gains range-checked in the controller and the panel, with identical
    limits enforced by a test; malformed setpoints dropped; configure-time
    bounds; a setpoint handoff that never blocks the 1 kHz loop; the
    PRE-FLIGHT rung (payload + collision thresholds, robot idle); a Z floor;
    release decided by the controller manager's state; DRIVER DOWN
    messaging; a 50 Hz trace with joints; `fr3_env.sh` sources this
    workspace.
  - *Round 2* (adversarial review of round 1; 10 verified findings plus 13
    lower-ranked ones, all addressed):
    - the window no longer closes while a release cannot be confirmed
    - pressing HOLD again no longer walks the Z floor down
    - PRE-FLIGHT restores the arm controller by asking the controller
      manager, and the exit path restores it too
    - Ctrl+C and SIGTERM take the guarded close path (tkinter swallowed
      Ctrl+C; rclpy's default handler swallowed SIGTERM)
    - the live view reschedules before drawing, so one error cannot freeze it
    - gain sets are applied atomically
    - a stuck state relay shows NO ROBOT STATE, not DRIVER DOWN
    - PRE-FLIGHT reports the resting |F ext| bias
    - configure-time parameters can be changed after a cleanup
    - gtests build without franka and cannot hang
    - docs fixed (unquoted spawner path, overstated claims)
  - That review also confirmed against the franka sources that releasing the
    arm controller does put the robot in IDLE, so PRE-FLIGHT can succeed, and
    that the `set_load` units, frame and inertia ordering are right.
- **Checked live, no robot needed:** SIGTERM closes the panel through the
  guarded path; the state-relay child dies with its parent; a symlinked
  `fr3_env.sh` finds the plugin from any directory.
- **Tests (2026-09-22):** 170 pytest in `tools/fr3`, 47 in `roscam`, 17 gtests
  in `fr3_mating_controllers`, 52 in `mating_controller`. All pass.
- **Robot stack:** up since 2026-09-16 (driver + move_group + MoveIt), with
  `fr3_arm_controller` active and the impedance controller loaded inactive.
  The FCI link measured 0% loss over 60 packets, 0.34 ms max - the cleanest it
  has been; keep Desk's browser tab closed.
- **Uncommitted:** everything since commit `4080be5` - the workspace
  restructure (`mating_controller` / `melfa/`, `move_l` -> `mating_node`),
  the tracking spec and its amendments, `tracking_node.cpp` with
  `tracking_law.hpp` and its gtests, the live slew parameters, and the docs
  including this file and the new `DOCS.md`. Commit as Kaan Ural once
  approved - and see next action 4 before any of it drives the arm.

## Next actions, in order

1. Commit (author Kaan Ural) and push. The review fixes are applied and all
   three suites pass.
2. ~~Bring the cell up.~~ ~~**Impedance ladder, rungs 0-3.**~~ Both done
   2026-09-16. To repeat, source ROS, `~/franka_ros2_ws/install/setup.bash`
   and `tools/fr3/fr3_env.sh` in **every** terminal, then:
   ```bash
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP
   ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
       --inactive --param-file "$(pwd)/fr3_mating_controllers/config/cartesian_impedance_stroke.yaml"
   python3 tools/fr3/cell_panel.py
   ```
3. ~~**Write the tracking spec**~~ done 2026-09-22
   ([TRACKING_SPEC.md](TRACKING_SPEC.md)), and amended after implementation.
4. **Review `tracking_node.cpp` adversarially, before it goes near the arm.**
   It is ~800 lines of new code that commands a torque controller, it has
   never run, and the one critique it has had called an earlier draft unsound
   on RT-safety grounds. Nothing about it should be trusted yet.
5. **Build the panel's START / STOP TRACKING buttons** so tracking obeys the
   same rule as every other motion here: a human presses something. Until
   then it starts from a service call, which is a gap, not a design.
6. **Then run V1-V6** from the spec, in order, on the real arm. Expect V1 to
   land near 5 mm, not 2 mm - that is the friction deadband at the shipped
   `track` stiffness, and the spec now says so.
7. Then the rest of the staged plan:
   - tune the force thresholds on the MoveIt stroke
   - teach the connector offsets
   - rung 4, the dispatched impedance stroke
   - an ArUco board for a steadier orientation
   - measure true end-to-end latency
   - Kalman-filter velocity feedforward
   - only then, combined position + orientation tracking

## Decisions already made

- **Impedance, not moveit_servo, is the path** to insertion and tracking.
  Streaming into the effort-mode trajectory controller stalled the arm
  (90 s commanded, 0.00 mm/s measured) and buzzed. Servo stays parked.
- **Alignment stays on the cartesian backend**, which works.
- **The enabling device is left as is.** It is invisible over FCI: no
  robot-state field changes when it is held or released. The operator controls
  are PAUSE and the robot-state gate; safety is the E-stop.
- **Commissioning reflex thresholds: 40 N / 40 Nm Cartesian.** That sits
  about 10 N above the controller's 30 N force ceiling, less the bias of the
  estimated wrench the reflex actually watches (PRE-FLIGHT reports it), and far
  below the 100 N that libfranka's example uses.
- **Every robot motion stays behind a GUI button** (human in the loop).
  RELEASE is one click away whenever no other action is running, and every
  exit path - close, Ctrl+C, SIGTERM, an exception - hands the arm back first.

## Known risks and open problems

- **Any libfranka reflex kills `ros2_control_node`.** franka_hardware does not
  catch the exception. The robot stops (it is not freed), and the stack must be
  relaunched. The upstream fix is outside this repo and not done.
- **The FCI link still drops.** A `communication_constraints_violation` killed
  the stack mid-run on 2026-09-15. Earlier packet loss was traced to Desk
  browser connections, so keep Desk closed while running.
- **franka_hardware 2.0.2 crashes on a combined cross-mode controller switch**
  (position↔effort). Use two separate calls, as `cell_panel` does.
  Effort↔effort swaps (impedance ↔ arm controller) are safe as a single call.
- **The payload is not configured yet.** Until PRE-FLIGHT sets it, expect a
  1–3 N bias in the wrench estimate, against mating thresholds of 8 N / 12 N.
- **The workspace path contains spaces**, which some colcon setup scripts
  handle badly; `fr3_env.sh` works around it.
- **Housekeeping:** `roscam/handeye_samples.yaml` is an untracked duplicate of
  the archived `tools/fr3/handeye_samples_20260915.yaml`.

## Where things live

| Path | What |
|---|---|
| `tools/fr3/cell_panel.py` | Camera-alignment dashboard (cartesian backend; servo parked) |
| `tools/fr3/cell_panel.py` | Impedance commissioning ladder, rungs 0–4 |
| `fr3_mating_controllers/` | The torque controller; safety logic in `impedance_detail.hpp`, gtests in `test/` |
| `tools/fr3/fr3_env.sh` | Per-terminal environment: DDS isolation, robot IP, this workspace |
| `mating_controller/` | The FR3 nodes: `mating_node` (phase machine), `tracking_node` (continuous tracking), pure logic in `include/` with gtests in `test/` |
| `tools/fr3/logs/` | Per-run JSONL traces (gitignored), including `tracking_*.jsonl` when `tracking_log_dir` is set |
| `tools/fr3/test_*.py` | Pure-logic tests, no ROS or robot needed (`python3 -m pytest tools/fr3 -q`) |
| `TODO.md` → Lessons | 1–5 from the 2026-09-11 bring-up, 6–9 from the 2026-09-15 servo and impedance work |
