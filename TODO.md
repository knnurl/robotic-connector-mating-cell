# TODO — Connector Mating Cell

Full state of the project: [STATUS.md](STATUS.md). Setup/usage reference:
[SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md). Original audit
handoff: [HANDOFF.md](HANDOFF.md) (2026-07-05, historical).

## ► Critical path — the single next action

**Superseded 2026-09-11: the robot HAS now run.** The FR3 moved under
closed-loop vision control (vision → MoveIt Cartesian → real FCI), so the
"first live controller run" gate is closed. Camera-relative alignment works
and the marker-orientation measurement is now trustworthy.

**Hand-eye is now CALIBRATED (2026-09-15).** 21 poses, Tsai, residual
3.17 mm / 1.57 deg; all four solvers agreed to 0.05 mm / 0.01 deg. Samples
archived in `tools/fr3/handeye_samples_20260915.yaml`; the result is the
default in `tools/fr3/fr3_mating.launch.py`.

The big finding: the old `handeye_quat` default was identity, and the true
rotation is **89.94 deg about Z** — the camera is mounted rotated ~90 deg,
so camera X/Y were effectively swapped for anything trusting that transform.
The translation guess was only 13 mm out; the rotation was the real bug.

Re-run `handeye_calib` if the bracket is reprinted or reseated
([hardware/camera_mount/](hardware/camera_mount/)):

    python3 -m roscam.handeye_calib --ros-args \
        -p base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp

(`ros2 run roscam ...` does not work — roscam is not colcon-installed here —
and the frame defaults are the MELFA ones, so the overrides are mandatory.)

**The single next action is now the validation ladder**: confirm tilt
converges end-to-end with `align_gui` AUTO-CONVERGE (this also validates the
new hand-eye in the loop), then teach connector offsets, then the ladder in
the FR3 section below.

Residual caveat: 1.57 deg rotational residual is above the "well under
1 deg" the tool asks for. Usable, and vastly better than the identity it
replaced, but more rotational diversity would tighten it if alignment ever
looks systematically off.

**Superseded 2026-09-15 (evening): the next action is the IMPEDANCE
commissioning ladder.** Camera alignment works through the cartesian backend
(~1 mm), so the open risk is the insertion backend, which has never run on
hardware and commands torque. Run the ladder with
`tools/fr3/impedance_panel.py` — steps 1-3 need no connector, no vision and
no force thresholds, so they can be done any time the cell is up.

The servo alignment backend is **parked**, not abandoned: it works in
simulation and unit tests, but it needs a controller swap on every run and
was never validated after the position-controller fix. Alignment stays on
the cartesian backend meanwhile.

### ⚠ Blocker before any further robot motion

**Packet loss on the FCI link: 4.7% measured 2026-09-11** (was 10.5%; it
improved but is not clean). libfranka tolerates almost none, and this has
already killed the stack twice mid-run — once as
`Connection reset by peer`, once as `libfranka: Timeout` while idle. NIC
counters are clean (0 errors/dropped/carrier) and RTT is 0.13 ms when
packets arrive, so they are being dropped at the robot end or on the wire,
not locally. Suspects, in order: the browser holding ~37 persistent HTTPS
connections to Desk on the same link, then cabling, then the robot's own
controller. `fr3_preflight.sh` only pings 20 packets and reports avg/max,
so it can miss this — check loss over a few hundred packets:

    ping -c 300 -i 0.01 -q 172.16.0.2      # want ~0% loss

## MELFA path — blocking real-world use (only if deploying on the RV-5AS)

- [ ] Copy repo to the robot PC and `colcon build` there (`plc_`/`hmi_`
      build automatically once `melfa_msgs` is found)
- [ ] Fake-hardware dry run (`use_fake_hardware:=true`,
      `enable_insertion: false`): verify phase transitions and motion
      directions in RViz — **first-ever live run of the controller**
  - [ ] Confirm the marker-frame flip convention matches the RV-5AS TCP
        (tool must align tool-Z INTO the surface; fix in
        `mating_geometry::standoff_goal` + tests if not)
  - [ ] Confirm Pilz LIN accepts the small frequent goals (if fussy: check
        `pilz_cartesian_limits.yaml`, else consider `computeCartesianPath`)
- [ ] Hand-eye calibration on the real cell (`ros2 run roscam handeye_calib`),
      replace the guessed static TF in the bringup
- [ ] Teach target geometry in `rv5as_params.yaml`:
      `connector_offset_x/y/z` (currently 0,0,0 = marker centre!),
      `tool_yaw_offset_deg`, `standoff_height_m`, `insertion_depth_m`
- [ ] Validation ladder: fake HW → real HW insertion-disabled (incl.
      occlusion hold test) → full mate at reduced `insert_speed`
- [ ] Tune convergence on real kinematics (step clamps, tolerances,
      `filter_alpha`) if alignment oscillates or crawls

## FR3 cell (this PC is the FR3 control PC — PREEMPT_RT, franka_ros2_ws)

- [x] Real-FR3 scaffolding (`tools/fr3/`): params (insertion off by
      default), cell launch, CycloneDDS interface isolation, low-bandwidth
      RealSense config, preflight script — done 2026-07-12
- [x] Bring up `eno1` on the FCI subnet — done; robot at 172.16.0.2, this
      PC 172.16.0.5/24. NOTE `fr3_preflight.sh` defaults to 172.16.0.3 and
      takes the IP as a POSITIONAL arg only (`ROBOT_IP=` env is ignored):
      `tools/fr3/fr3_preflight.sh 172.16.0.2`
- [x] Switch `CYCLONEDDS_URI` to `tools/fr3/cyclonedds_fr3.xml` — done
      2026-09-11, set in `~/.bashrc` so every shell inherits it, plus
      `tools/fr3/fr3_env.sh` to source explicitly. This was NOT cosmetic:
      the old fr3_act config has no `<Interfaces>` block and DDS on the
      robot NIC caused an FCI `Connection reset by peer` mid-motion.
      Required adding a `<Discovery>` block to the config too — see the
      "lessons" section at the end of this file.
- [x] First live controller run — done 2026-09-11 on REAL hardware (not
      fake): FCI activated in Desk, MoveIt + ros2_control up, arm driven
      by closed-loop vision through `tools/fr3/align_gui.py`. Supersedes
      both the fake-hardware run and the parked mock dry run.
- [x] Marker-orientation measurement made trustworthy — done 2026-09-11.
      Was unusable: two separate defects (see lessons). Now 0% flips and
      unbiased. This blocked all orientation alignment.
- [ ] Verify tilt now converges to ~0 in a full `align_gui` AUTO-CONVERGE
      run (`tilt_change_deg` should be consistently negative in the trace);
      the previous run stalled on the measurement, not the controller
- [ ] Hand-eye calibrate on the FR3 (`base_frame:=fr3_link0`,
      `tcp_frame:=fr3_hand_tcp`, `filter_frame:=''` during collection),
      pass result via `handeye_xyz`/`handeye_quat` launch args
- [ ] Teach connector offsets in `tools/fr3/fr3_params.yaml`, then enable
      insertion and run the validation ladder at `insert_speed: 0.02`
- [ ] Tune force-guard thresholds on the real cell: watch
      `/franka_robot_state_broadcaster/external_wrench_in_base_frame`
      during one manual mate, set `contact_force_n` above the estimate's
      bias, verify contact→MATED and early-contact→FAULT behaviours
- [x] `moveit_servo` installed and wired into `align_gui` as the "servo"
      backend — done 2026-09-15. Continuous 6-DOF streaming with command
      shaping, an oscillation watchdog and a stall watchdog; signs pinned
      by `tools/fr3/test_servo_signs.py`.
- [ ] **Re-validate the servo backend on hardware after the controller
      change.** It now streams joint POSITIONS to
      `fr3_servo_position_controller` (loaded inactive by
      `tools/fr3/fr3_servo.launch.py`; `align_gui` swaps controllers around
      each run) instead of trajectories into the effort-mode
      `fr3_arm_controller`, which stalled the arm outright — lessons 6-7.
      Ladder: conservative until converged, then moderate, then brisk.
      Watch for audible buzz, check `outcome` in the trace, and confirm
      `fr3_arm_controller` is active again afterwards
      (`ros2 control list_controllers`) — the GUI says "NOT RESTORED" and
      the SERVO pill reads `CTRL STUCK` if the hand-back ever fails.
- [x] Out-of-ROS camera capture ("camera outside ROS, only end-data in"):
      `roscam/rs_capture.py` (SDK wrapper + self-test CLI),
      `source:=topic|realsense|external` on cam_pub/connector_pose,
      `vision_standalone` executable (one camera, ArUco+ICP, poses only).
      Default stays `topic` — nothing rewired — done 2026-07-12
- [x] `pip install pyrealsense2` + hardware-verify the out-of-ROS path —
      done 2026-09-11 (pyrealsense2 2.58.4, `--user`). D405 serial
      130322273822, FW 5.17.0.10. Verified 89.9 fps at 640x480 colour+depth
      with 0.2 ms jitter, ~138 MB/s kept off the DDS graph.
- [x] Run the camera at 90 fps (was 15) — done. Needed a fix to the depth
      plane fit, which at first held the pipeline to 22 fps; see lessons.
- [ ] Install a USB udev rule on any NEW machine:
      `sudo cp tools/fr3/99-realsense-no-suspend.rules /etc/udev/rules.d/`
      then `sudo udevadm control --reload-rules` and
      `sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=8086`.
      Without it the kernel autosuspends the D405 after 2 s, it
      re-enumerates with a new USB device number, and every open handle
      dies as a silent "frame timeout". `RsCapture` now self-heals
      (hardware_reset after 8 consecutive timeouts, bounded to 5) but the
      rule prevents it rather than papering over it.
- [ ] Verify no `communication_constraints_violation` across a full mating
      cycle with the camera live (then once more with Foxglove attached
      over WiFi); prefer `vision_source:=realsense` if it ever recurs

## Improvements (not blocking)

- [ ] Finish the FR3 mock dry-run harness (`tools/dryrun_fr3/`) — pipeline
      config already fixed, never re-run; success = MATED in the log
      (HANDOFF §6). Lower priority now that `tools/fr3/` targets real HW
- [x] `computeCartesianPath` stroke planner (`insert_planner: cartesian`) —
      path-guaranteed INSERT/retract without Pilz; on by default in the FR3
      profile — done 2026-07-12
- [x] Force-aware insertion (`wrench_topic`): contact→MATED (with
      min-depth jam detection), lateral snag→FAULT, guarded async
      execution; enabled in the FR3 profile, off for MELFA (no F/T
      source) — done 2026-07-12 (thresholds need real-cell tuning)
- [x] Servo alignment mode (`align_mode: servo`): ALIGN publishes clamped
      Cartesian twists for moveit_servo, zero-twist deadman on every hold
      path, INSERT stays discrete; `tools/fr3/fr3_servo.yaml` provided —
      done 2026-07-12 (runtime-unverified: moveit_servo not installed)
- [x] Cartesian-impedance insertion backend (`fr3_mating_controllers`):
      ControllerInterface plugin, tau = J^T(K dx − D v) + nullspace +
      coriolis at 1 kHz, tool-frame K (soft lateral/rot, firm Z),
      slew-limited equilibrium, torque-rate saturation, float_mode
      commissioning switch; `move_l` dispatch (`insert_backend: impedance`)
      with controller switching, 50 Hz equilibrium ramp + preload
      overdrive, wrench-judged outcomes — done 2026-07-12. Compiled clean
      against franka_ros2; **never run on hardware** (fake HW cannot
      integrate torques)
- [x] Commissioning rig for that ladder — done 2026-09-15:
      `tools/fr3/impedance_panel.py`, one button per rung (FLOAT / HOLD /
      SETPOINT / RELEASE) with the arm's pose, external wrench and
      `control_command_success_rate` live, a JSONL trace per session, and
      RELEASE one click away whenever no other action is running (close,
      Ctrl+C and exit also hand the arm back). Logic
      covered by `tools/fr3/test_impedance_panel.py`.
- [x] Pre-contact guards in the controller — done 2026-09-15, before it
      ever commanded torque: a Cartesian force/moment ceiling
      (`max_force_n` 30 N, `max_torque_nm` 10 Nm — stiffness × error alone
      is unbounded, and a blocked arm with a slewing equilibrium reaches
      80 N in two seconds), a per-joint ceiling (`tau_max_nm`, default the
      FR3's own limits), non-finite guards on state and torque, zeroed
      commands on deactivate, and live-tunable gains so the ladder does not
      have to deactivate to change stiffness. `float_mode` is now live too
      and **re-seeds the equilibrium on float→hold**, so step 2 cannot snap
      the arm back to where step 1 activated it.
- [x] Review fixes before first contact — done 2026-09-15 (evening), after a
      verified audit of the impedance turn:
      live gains range-checked in the controller AND the panel (same limits,
      a test fails if they drift; `damping_ratio` 0 / negative was accepted
      before, i.e. an undamped or energy-injecting spring); malformed
      setpoints dropped; configure-time parameters bounded and rejected live;
      the setpoint handoff no longer blocks the 1 kHz loop and the float→hold
      log moved out of it (gtests in `fr3_mating_controllers/test/`);
      PRE-FLIGHT rung sets payload + collision thresholds with the robot idle
      (franka accepts them only then) and restores the arm controller on
      every path; Z floor for setpoints; release decided by the controller
      manager's state on RELEASE, close and exit; DRIVER DOWN instead of
      "use the E-stop" when the driver is gone; 50 Hz trace with joints;
      `fr3_env.sh` sources this workspace; ladder wording corrected (small
      overshoot is normal: effective damping ~0.5–0.7).
      NOT done (outside this repo): franka_hardware does not catch reflex
      exceptions in `read()`/`write()`, so every reflex still kills
      `ros2_control_node`.
- [x] Second review round on those fixes — done 2026-09-15 (night). An
      adversarial review of the round above left 10 verified findings and 13
      lower-ranked ones, all addressed:
      the panel refuses to close while a release cannot be confirmed; HOLD
      pressed again no longer ratchets the Z floor down; PRE-FLIGHT restores
      the arm controller by the controller manager's answer, and exit restores
      it too; Ctrl+C / SIGTERM go through the guarded close (tkinter swallowed
      Ctrl+C, rclpy's default handler swallowed SIGTERM); the live view
      reschedules before drawing; gain sets are applied atomically; a stuck
      state relay reads NO ROBOT STATE, not DRIVER DOWN; PRE-FLIGHT reports
      the resting |F ext| bias; `on_cleanup` lets configure-time parameters
      change; gtests build without franka and cannot hang; the spawner path is
      quoted. The review confirmed against the franka sources that releasing
      the arm controller does put the robot in IDLE, so PRE-FLIGHT can work.
- [ ] **Run the ladder on the real FR3** with `tools/fr3/impedance_panel.py`
      (`fr3_mating_controllers/README.md`). **Rungs 0-3 PASSED 2026-09-16**
      (payload via Desk, rest bias 1.0 N, RT 100%, float smooth, hold solid,
      setpoints tracking with the friction deadband of lesson 10). Rung 4
      still outstanding:
      0. PRE-FLIGHT — weigh camera + bracket first (0 kg if Desk's end
         effector already includes them; never the hand twice).
      1. FLOAT — free-float by hand, smooth, no buzz; RT pill ≥ 99%; pushes
         under 10 N. Ragged here = network/RT, not gains
         (`tools/fr3/fr3_preflight.sh`). Slow sag = payload wrong.
      2. HOLD — arm does not move when HOLD is pressed; ~15 N per 10 cm
         sideways, ~12 N per 1.5 cm on Z; one small overshoot on release is
         normal, ringing is not.
      3. SETPOINT — 10–20 mm UP first; glides at ≤ 5 cm/s, settles within a
         few mm.
      4. Dispatched stroke — AFTER the force-guarded MoveIt stroke works,
         since it shares the thresholds still being tuned there.
- [ ] **Write the spec for continuous marker tracking on the impedance
      backend** - design first, then code. Lesson 10 is why one gain set
      cannot serve both tracking and mating:
      - **named gain profiles** in the controller yaml (`track`: stiff and
        well damped; `mate`: today's soft set), applied atomically on phase
        transitions. Tracking to ~1 mm needs k around 3000 N/m (deadband
        `3.5/k`); mating needs 150 for self-alignment
      - **make `setpoint_slew_mps` / `setpoint_slew_rps` live**, not
        configure-time: tracking wants 0.1-0.25 m/s, the stroke 0.005. Keep
        the range checks
      - **`damping_ratio` toward 2.0 for tracking.** `D = 2*zeta*sqrt(K)` is
        critical for 1 kg, so effective zeta is `zeta / sqrt(apparent mass)`
        - about 0.5 on this arm at zeta 1.0
      - **orientation target must come from VISION, not the measured pose.**
        The panel republishes the measured quaternion, which is fine for
        stepping but would ratchet the 3.4 deg angular deadband when tracking
      - **bench tests before closed loop**: static (steady error vs k, expect
        `3.5/k`), step (bandwidth + overshoot), sine at 0.1/0.2/0.5 Hz (phase
        lag gives end-to-end latency, closing that open item too)
      - stays on impedance, NOT moveit_servo: effort-to-effort needs no
        command-mode swap, so lessons 6 and 7 do not apply
- [ ] `fr3_backend` (~/fr3_backend, joint-impedance WebSocket testbed):
      keep as a hands-on stiffness-feel/tuning rig; do NOT run alongside
      franka_ros2 (both need the exclusive FCI connection)
- [x] Failure policy beyond FAULT-hold: `~/retract` service (pull back
      along the stroke to standoff), reset refused while mid-INSERT
      interrupted, pause/resume covers remaining stroke depth — done
      2026-07-12
- [x] Kalman filter in vision (replaces EMA; outlier gate + ≤0.3 s
      dropout prediction) — done 2026-07-06
- [x] Connector-level 6-DOF pose via depth ICP against CAD STL
      (`connector_pose` node; marker = prior, gated fallback) — done
      2026-07-06; needs real-data tuning + STL export of the real connector
- [x] Multi-marker / ArUco-board support in `cam_pub`
      (`board_markers_x/y`): grid board, pose = board centre, any visible
      subset suffices (occlusion-robust) — done 2026-07-12; verified in
      synthetic tests incl. a half-occluded board
- [x] Machine-readable cell state (/mating/phase, /mating/error_*,
      /diagnostics) + GUI options: Foxglove layout, configurable tkinter
      operator panel, rqt recipe (tools/gui/); operational stop/pause/
      resume/reset services + buttons — done 2026-07-06
- [x] Consolidated single cell launch (`cell.launch.py`): hand-eye TF +
      vision + controller in one, `vision_source`/`filter_frame`/
      `template_stl` args — done 2026-07-12 (launch introspection OK;
      full run still needs the MELFA MoveIt packages)
- [x] Auto-teach connector offsets via ICP (`roscam teach_offsets`):
      pairs marker + connector poses, prints paste-ready offsets, replaces
      the caliper loop — done 2026-07-12 (math unit-tested; needs the real
      cell + STL to exercise end-to-end)
- [ ] Force-guarded insertion (F/T sensor or MELFA force option) for
      tight-tolerance connectors — hardware decision first

## Code quality

- [x] Extract the mating sequence into a pure, unit-tested state machine
      (`mating_phase_machine.hpp`): the whole transition matrix — raw-vision
      arming, interrupted-insert latch, retract recovery, drift-back,
      plan-failure accounting, force outcomes — is now 14 gtests instead of
      untested node code; `move_l.cpp` drives it — done 2026-07-12 (32
      C++ tests total)
- [ ] Split `melfa_rv5as_masterclass` into `mating_controller` (portable) +
      a `melfa_cell` package (PLC/HMI/bringup); the name misleads and the
      graph flags low cohesion in the controller community
- [ ] Automated end-to-end regression: wrap the FR3 mock dry run in
      `launch_testing`, assert MATED is reached headless
- [ ] CI (GitHub Actions on `ros:humble`: build + gtest + pytest + flake8)

## Lessons from the 2026-09-11 bring-up (do not re-learn these)

**1. Measure the measurement before tuning the controller.** Orientation
alignment "failed" for ~18 robot motions and the controller was innocent
both times. Two stacked defects in the marker orientation:

- *The flip.* `cv2.solvePnP(..., SOLVEPNP_IPPE_SQUARE)` returns only ONE of
  the two mirror solutions a planar square admits. On a small near-face-on
  marker their reprojection errors are nearly equal, so the pick alternated:
  the normal's in-plane direction flipped >120 deg on **62%** of frames
  while its magnitude looked stable. Every correction undid the last one
  (visible as the wrist joint alternating sign every step). Fixed with
  `solvePnPGeneric` (returns both) + depth plane normal to choose.
- *The bias.* IPPE's out-of-plane MAGNITUDE reads systematically high —
  measured 3.95 deg vs 2.11 deg from the depth fit, a 2.6 deg gap. A loop
  driving the biased number bottoms out on the bias and cannot reach zero
  (observed as a tilt floor that would not go below ~7 deg). Fixed by
  taking the normal from depth (`tilt_source: depth`).

Net: ArUco corners for position (7-10 micron) and in-plane rotation
(0.18 deg); depth plane for out-of-plane (0.18 deg, unbiased). Use each
sensor for what it is good at. Diagnose with `tools/fr3/analyse_trace.py`,
which detects the flip signature automatically.

**2. Orientation gotchas worth knowing.** J7 cannot fix tilt: the camera's
optical axis is only 2.23 deg off TCP Z, so J7 spins the camera about its
own line of sight (1 deg of J7 moves the optical axis by 0.039 deg). Tilt
is wrist pitch, J5/J6. Conversely the IN-PLANE rotation is measurable and
rock-solid (0.18 deg) and IS roughly J7 — and nothing currently commands
it, so that alignment DOF is unused.

**3. A camera-frame position servo does NOT need hand-eye translation.**
Three 15 mm probe moves recover `R_cam_base` empirically
(`col_i(R) = -(p_after - p_before)/h`); the unknown lever arm cancels out
of the translation math. Saved as the pose-independent `R_cam_tcp`, so it
survives sessions and arm poses. Verified to machine precision over 300
randomised trials. Rotation about the TCP still swings the camera by the
unknown lever arm, so levelling perturbs translation — alternate the two.

**4. DDS/tooling traps.**
- `lo` is not multicast-capable, so pinning DDS to loopback silently
  disables multicast and SPDP falls back to unicast, which only probes
  participant indices 0..9 by default. One `moveit.launch.py` is ~9
  participants: nodes above the cap are never discovered and it surfaces as
  `move_group` never receiving `/joint_states` ("no current robot state")
  with no error anywhere. Hence the `<Discovery>` block.
- Under unicast SPDP, `ros2 node list` / `ros2 topic list` under-report.
  Trust `ros2 topic hz <known topic>` and direct service calls instead.
- Mixed DDS configs discover each other but do NOT exchange data. Every
  process in the cell must use the same `CYCLONEDDS_URI`.
- `udevadm trigger` defaults to `--action=change`; a rule matching only
  `add` silently does nothing. Verify with
  `udevadm test --action=add /sys/bus/usb/devices/<dev>`.
- `sudo tee <<'EOF'` can create an EMPTY file if the heredoc body does not
  survive the paste. Always `wc -c` the result.

**5. MoveIt/FR3 specifics.**
- `GetCartesianPath` in this version has NO velocity-scaling field: setting
  one in the request does nothing. Slow motion requires retiming the
  returned trajectory (stretch `time_from_start`, scale velocities by 1/k
  and accelerations by 1/k^2).
- Cancelling the `ExecuteTrajectory` goal can let the queued trajectory
  play out. For a true mid-motion stop publish `"stop"` to
  `/trajectory_execution_event` (TrajectoryExecutionManager::stopExecution).
- The motion-generator interfaces (joint/Cartesian position and velocity)
  are policed for continuity — `*_motion_generator_*_discontinuity`
  reflexes. The torque interface is only bounded by
  `controller_torque_discontinuity` (1 Nm/ms), which is why the impedance
  backend is the right home for streamed, jumpy vision setpoints.

## Lessons from the 2026-09-15 servo work (do not re-learn these)

**6. Do not stream into an effort-mode trajectory controller.**
`fr3_arm_controller` is a `JointTrajectoryController` on the **effort**
interface, and `moveit_servo` replaces its trajectory every 10 ms. Each new
trajectory is seeded at the *measured* position, so the tracking error — and
with it the commanded torque — stays tiny. Measured: a conservative servo
run commanded ~10 mm/s and 7 deg/s for **90 s while the arm moved
0.00 mm/s** (error flat at 12.7 mm / 8.8 deg), and at higher speeds the
100 Hz torque steps were clearly audible and shook the arm into
`cartesian_reflex`. Stream joint *positions* to a forward position
controller instead and let libfranka's own joint-impedance controller track
them. Corollary: a servo backend needs a stall watchdog — commanded motion
with no measured progress must abort, not stream forever.

**7. franka_hardware 2.0.2 cannot swap command modes in ONE switch call.**
`perform_command_mode_switch` walks the modes in a fixed order in a single
pass: asked for effort in and position out, it starts torque control and
*then* calls `stopRobot()` for the outgoing position mode, leaving `write()`
with no active control — `std::runtime_error`, ros2_control_node dead, whole
launch down. Verified twice with the arm stationary. Always **deactivate the
old controller, pause, then activate the new one** (`switch_controllers` in
`align_gui.py` does this). Switching *to* position is fine; it is the way
back that bites. Test mode switches with the arm stopped and nothing
commanded — that is how this was caught instead of mid-motion.

**8. The enabling device is invisible over FCI.** Holding and releasing the
cell's enabling device changes *no* field of `FrankaRobotState` — not
`robot_mode`, not the error flags. There is no libfranka signal for it, so
software cannot gate on it. `align_gui`'s gate is therefore a **robot-state**
gate (pauses whenever the robot leaves MOVE, e.g. a real user stop, and ends
the run on REFLEX), plus a PAUSE/RESUME button. If a held-to-run interlock
is ever required, it has to come from the robot's own safety configuration
or from separate hardware.

**9. Do not subscribe to the 1 kHz robot state from a Python GUI.** Decoding
`FrankaRobotState` at 1000 Hz costs **86% of a CPU core**; taking the
messages raw and decoding a few costs 31% (rclpy still dispatches every
callback). A C++ `topic_tools throttle` child relaying at 50 Hz costs 5%,
which is what `align_gui` starts. Subscribe **best-effort, depth 1** so a
slow reader can never back-pressure the realtime publisher.

## Lessons from the 2026-09-16 impedance commissioning (do not re-learn these)

**10. Compliant control has a friction deadband, and it is `F_friction / k`.**
The arm tracks a commanded setpoint only until the spring force drops below
joint breakaway, then stops and stays stopped. Measured on this cell:
**~3.5 N at the TCP** and **~0.6 Nm about the wrist**. Three independent
confirmations from one session:

| deadband | predicted `F/k` | observed |
|---|---|---|
| position, `k_pos_tool` 150 N/m | 23 mm | 21.6 mm |
| position, `k_pos_tool` 600 N/m | 5.8 mm | 6.8 mm |
| orientation, `k_rot_tool` 10 Nm/rad | 3.4 deg | 3.37 deg |

The proof that it is Coulomb friction and not a controller fault: raising
stiffness 4x left the **stall force unchanged** (3.2 N -> 4.1 N) and shrank
the **lag 3x**. Nothing was being clipped - controller manager at 1000 Hz, RT
success 1.000, the 30 N wrench cap engaged in 3% of samples, no joint within
25 deg of a limit.

*The measurement trap:* at the panel's 60 mm lead cap a 150 N/m spring can
never make more than 9 N, so "it stalls at 8 N" measures the CAP, not
friction. Raise `k` and re-measure - the force at stall is the friction, the
lag is just `F/k`.

*What it costs, per axis:*
- **The insertion stroke is fine.** At `k_z` 800 the deadband is 4.4 mm and
  `impedance_overdrive_m` is already 10 mm of preload lead.
- **Lateral self-alignment needs > 3.5 N of chamfer force** before the arm
  yields at all. Check this against the real connector's insertion force.
- **Angular deadband is 3.4 deg** at `k_rot_tool` 10 Nm/rad - the one to
  watch for keyed connectors. `k_rot_tool` 40 brings it to ~0.85 deg, at the
  cost of angular compliance.
- **~2 N of friction lands in the external-wrench estimate** with nothing
  touching the connector, against `contact_force_n` 8 N. Thinner margin than
  it looks when tuning the force thresholds.
- **A soft spring cannot track to 1 mm.** Tracking wants stiff (deadband
  `3.5/k`), mating wants soft. That is a gain-schedule, not one tuning.

Symptom to recognise: "it moves the right way but seems to hit resistance,
and the heading recovers a bit after it stops". The partial recovery is the
spring unwinding until it re-sticks - Coulomb hysteresis, not oscillation.
Raw data: `tools/fr3/logs/impedance_20260916_162613.jsonl` (40 MB, three gain
regimes).

**11. A daemon spin thread outliving rclpy's context ABORTS the process.**
`impedance_panel.py` exited 1 with `terminate called without an active
exception` on an ordinary window close: `rclpy.shutdown()` ran while a daemon
thread was still inside `rclpy.spin()`, taking a DDS thread down with the
context. The abort landed *after* the arm was handed back - by luck of the
race, not by construction, and it aborts in the one path that guarantees the
handoff. Fixed by owning the executor and tearing down in order
(`shutdown_ros()`: hand back -> `executor.shutdown()` -> `join` -> destroy ->
`rclpy.shutdown()`), with a mutation-checked test. Corollary: **never read a
GUI exit status as evidence the arm was released** - ask the controller
manager (`ros2 control list_controllers`).

## Housekeeping

- [ ] Rename the workspace directory to remove spaces + trailing space
      (breaks colcon's `install/setup.sh`; workarounds in HANDOFF §2)
- [ ] Modernize or delete `pick_n_place_` (legacy demo, still old patterns)
- [ ] Push the repo to a remote (currently local-only git)
- [ ] Set a global git identity on this machine (commits currently use
      per-command `-c user.name/email`)
