# TODO — Connector Mating Cell

Where the project stands: [PROJECT_STATE.md](PROJECT_STATE.md). What exists
and what is proven: [STATUS.md](STATUS.md). Setup/usage reference:
[SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md). Which document to
believe when two disagree: [DOCS.md](DOCS.md).

## ► Critical path (2026-09-25)

1. **The next arm session: [ARM_CHECKLIST.md](ARM_CHECKLIST.md).**
   Everything after `401cf24` was built offline and needs the arm:
   - the marker distance from depth;
   - the re-solved hand-eye;
   - the `/object/*` contract (the Phase 1 exit);
   - PLACE AT B.

   Rebuild roscam first: the installed copy publishes no `/object/*`.
2. **Perception Phase 3 (shadow mode) is built offline** and replay-tested
   ([PERCEPTION_PLAN.md](PERCEPTION_PLAN.md)). Its arm sessions come after
   step 1: ARM_CHECKLIST section 6.
3. **Perception Phase 4 (`depth_checked`) is built offline** as well: depth
   drives and the marker vetoes. Its arm runs come after Phase 3's
   (ARM_CHECKLIST section 7).

Resolved blockers:
- The 09-11 FCI packet loss was Desk's browser tab on the robot link.
- The 09-24 comm reflexes were an unoptimised controller build (C10, lesson
  12).

### Hand-eye

Calibrated 2026-09-15 (Tsai, 21 poses; the samples are in
`tools/fr3/handeye_samples_20260915.yaml`). The true rotation is 89.94° about
Z: the camera is mounted rotated, and the old identity default had swapped
camera X and Y.

**Re-solved 2026-09-25 for the depth distance** (lesson 17). The xyz moved
3.9 mm along the optical axis, and the old solve is kept as `xyz_aruco_range`.
Both are in `tools/fr3/calib/handeye.yaml`, the one copy, and the launch picks
the xyz that matches `range_source`.

A fresh calibration with the depth distance supersedes both. Also re-run it if
the bracket is reprinted or reseated
([hardware/camera_mount/](hardware/camera_mount/)):

    python3 -m roscam.handeye_calib --ros-args \
        -p base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp

The frame overrides are mandatory, because the defaults are the MELFA ones.

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
      by closed-loop vision through `tools/fr3/cell_panel.py`. Supersedes
      both the fake-hardware run and the parked mock dry run.
- [x] Marker-orientation measurement made trustworthy — done 2026-09-11.
      Was unusable: two separate defects (see lessons). Now 0% flips and
      unbiased. This blocked all orientation alignment.
- [x] Verify tilt converges in a full ALIGN run — done 2026-09-23 on the
      arm: 239.6 mm → 1.6 mm and 9.7° → 0.49° from 100 mm. ALIGN now lives in
      the panel, `tools/fr3/cell/`
- [x] Hand-eye calibrate on the FR3 — done 2026-09-15, and re-solved
      2026-09-25 for the depth distance (see "Hand-eye" above). The launch
      reads `tools/fr3/calib/handeye.yaml`
- [ ] Teach connector offsets in `tools/fr3/fr3_params.yaml`, then enable
      insertion and run the validation ladder at `insert_speed: 0.02`
- [ ] Tune force-guard thresholds on the real cell: watch
      `/franka_robot_state_broadcaster/external_wrench_in_base_frame`
      during one manual mate, set `contact_force_n` above the estimate's
      bias, verify contact→MATED and early-contact→FAULT behaviours
- [x] `moveit_servo` installed and wired into `cell_panel` as the "servo"
      backend — done 2026-09-15. Continuous 6-DOF streaming with command
      shaping, an oscillation watchdog and a stall watchdog; signs pinned
      by `tools/fr3/test_servo_signs.py`.
- [x] ~~Re-validate the servo backend on hardware~~ — **dropped
      2026-09-23: servo is archived** (tag `pre-cleanup-2026-09-23`).
      Impedance is the path; lessons 6-7 say why
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
      over WiFi). The five reflexes of 2026-09-24 were the unoptimised
      controller build (C10, lesson 12). None has occurred since the Release
      build, but no full mating cycle has run yet
- [x] Continuous tracking (TRACK) on the arm — done 2026-09-23/24.
      `tracking_node` runs behind the panel's TRACK button. It is:
      - camera-centred, with an over-lead policy (hold / clamp / stop);
      - glides the goal between camera frames;
      - holds at the joint limits and at the workspace box;
      - stops itself on buzz (3.5 Nm);
      - keeps the operator's gains, and has a FAST switch.

      `tools/fr3/sim/tracking_smoke.py` runs the real node in a fake cell;
      run it before any arm session
- [ ] TRACKING_SPEC V1-V3 as a recorded table → [ARM_CHECKLIST.md](ARM_CHECKLIST.md)
      section 2 (V4-V6 later)
- [x] One-page PySide6 panel, `tools/fr3/cell/` — done 2026-09-24, replacing
      the Tk `cell_panel.py`
- [x] GRIP the 55 mm cube — done 2026-09-25: 3 cycles on the arm, under
      impedance
- [ ] PLACE AT B on the arm. It is built and mock-tested (`75df4d1`)
      → ARM_CHECKLIST section 1
- [ ] Controller-side items C1-C5 and C7-C9
      ([tools/fr3/cell/README.md](tools/fr3/cell/README.md)). Suggested
      order: C8 with C1, C9, C2/C3, C7. C4 and C5 need franka_ros2 changes
- Marker-free perception ([PERCEPTION_PLAN.md](PERCEPTION_PLAN.md)):
  - [ ] **Phase 0, measure the sensor.** Done: the range bias, latency
        24 ms and the depth settings (`ba26387`, `8345442`). The missing
        recordings, the sticker and the tilt reference → ARM_CHECKLIST
        sections 3-4
  - [ ] **Phase 1, the `/object/*` contract.** Built offline (`a8809d7`);
        the on-arm exit → ARM_CHECKLIST section 2
  - [ ] **Phase 2, the estimator.** `roscam/object_pose.py` (`c8991e6`,
        `c75a878`) is scored on the recordings that exist
        (`runs/2026-09-25/analysis/`). Two exits wait on missing
        recordings: the bare cube, and no false accepts on junk and hand
        frames. The tilt offset against the marker (0.5-0.7° at 100 mm,
        1.3-2.1° at 300 mm, systematic) is open: which one is right?
  - [ ] **Phase 3, shadow mode.** Built offline 2026-09-25 (`1cce687`):
        `roscam/object_shadow.py` and `vision_standalone.Shadow`, run with
        `fr3_cell vision_source:=standalone object_shadow:=true`. Replayed
        through the real frame loop (`tools/fr3/vision/loop_replay.py`,
        `runs/2026-09-25/analysis/phase3_*`):
        - 3,181 of 3,183 static marker frames valid, within ±0.51 mm of the
          marker;
        - the whole frame p95 44.8 ms on one E-core, against the 66.7 ms
          period, with 1 overrun in 3,655 frames.

        The on-arm sessions and the estimator on/off A/B → ARM_CHECKLIST
        section 6. The object KF moved to Phase 4 (PERCEPTION_PLAN
        [AS BUILT])
  - [ ] **Phase 4, `depth_checked`.** Built offline 2026-09-26 (`1cce687`,
        the C++ estimator `8fb0abd`):
        `roscam/depth_checked.py` and `vision_standalone.DepthRunner`,
        chosen from the panel's source dropdown. Two adversarial reviews
        found no blocker; their findings are fixed. On the replayed static
        sessions (`runs/2026-09-25/analysis/phase4b_rules_*`), with the
        2026-09-26 rules (veto tilt 4°, a drop pauses acquisition) and the
        C++ estimator:
        - a raw pose on 98-99 % of marker frames out to 275 mm and 94.5 % at
          275-350 mm, still;
        - 79.5 % in hand-guided motion;
        - the depth raw goes out at p95 23 ms after frame pickup (the TRACK
          row asks for 40).

        The arm runs → ARM_CHECKLIST section 7
  - [ ] Phases 5-7: the marker as a seed only, no marker, then the connector

## Improvements (not blocking)

- [x] ~~Finish the FR3 mock dry-run harness (`tools/dryrun_fr3/`)~~ —
      **dropped 2026-09-23**: it was parked in `melfa/parked/` with
      `mating_node`. The no-robot checks are now `tools/fr3/cell/mock_cell.py`
      and `tools/fr3/sim/tracking_smoke.py`
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
      commissioning switch; `mating_node` dispatch (`insert_backend: impedance`)
      with controller switching, 50 Hz equilibrium ramp + preload
      overdrive, wrench-judged outcomes — done 2026-07-12. Compiled clean
      against franka_ros2; **never run on hardware** (fake HW cannot
      integrate torques)
- [x] Commissioning rig for that ladder — done 2026-09-15:
      `tools/fr3/cell_panel.py`, one button per rung (FLOAT / HOLD /
      SETPOINT / RELEASE) with the arm's pose, external wrench and
      `control_command_success_rate` live, a JSONL trace per session, and
      RELEASE one click away whenever no other action is running (close,
      Ctrl+C and exit also hand the arm back). Logic
      covered by `tools/fr3/test_cell_panel.py`.
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
- [ ] **Run the ladder on the real FR3** with the panel, `tools/fr3/cell/`
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
- [x] **Spec for continuous marker tracking on the impedance backend** -
      written 2026-09-22 against measured numbers:
      [TRACKING_SPEC.md](TRACKING_SPEC.md). Supersedes the bullet list that
      used to live here; two of its guesses were wrong and the spec says so.
- [x] **Implement the tracking loop per [TRACKING_SPEC.md](TRACKING_SPEC.md)**
      — done 2026-09-22/24; it runs on the arm (see the FR3 section).
      - C2, the live `setpoint_slew_mps/rps`: done.
      - C1, named gain profiles: superseded. TRACK keeps the operator's
        gains (`tracking_use_operator_gains`).
      - The damping ratio did change: the 40 Hz wrist buzz of 2026-09-23 was
        the rotational damping, so `track_damping_ratio` went from 1.0 to 0.5.
      - Still open: the V1-V6 table.
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
      2026-07-06; needs real-data tuning + STL export of the real connector.
      For the marked cube it is superseded by `roscam/object_pose.py`
      (depth plus colour edges, perception Phase 2). Plain ICP was biased
      where the D405 returns no depth (lesson 16)
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
      untested node code; `mating_node.cpp` drives it — done 2026-07-12 (32
      C++ tests total)
- [x] **Workspace restructured 2026-09-22** (the FR3 is the only target):
      - `move_l` -> `mating_node`. The old name was MELFA's "MoveL" and
        described a linear move, not a mating controller. The ROS node name
        `connector_mating_node` is UNCHANGED - it keys the params files and
        every `/connector_mating_node/*` service the GUIs call.
      - MELFA packages grouped under `melfa/` (`melfa_cell`,
        `melfa_masterclass_msgs`, `BRINGUP.txt`), parked but intact.
      - `mating_node` put behind `option(BUILD_MATING_NODE ON)` so this
        machine's broken MoveIt install cannot take the whole package -
        gtests included - down with it. Build the testable core with
        `--cmake-args -DBUILD_MATING_NODE=OFF`.
      - [DOCS.md](DOCS.md) added: an index of all twelve documents with the
        redundant content named. Nothing was deleted.
- [x] Split `melfa_rv5as_masterclass` into `mating_controller` (portable:
      phase machine, pose maths, tracking law, mating_node, tracking_node) +
      `melfa_cell` (plc_/hmi_/legacy demo) - done 2026-09-22 when the
      tracking work needed a home and the FR3 became the only target. The
      namespaces (`mating_geometry`, `mating_phase_machine`, `tracking_law`)
      were already package-agnostic, so only include paths and launch
      package names moved.
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
`cell_panel.py` (ALIGN tab) does this). Switching *to* position is fine; it is the way
back that bites. Test mode switches with the arm stopped and nothing
commanded — that is how this was caught instead of mid-motion.

**8. The enabling device is invisible over FCI.** Holding and releasing the
cell's enabling device changes *no* field of `FrankaRobotState` — not
`robot_mode`, not the error flags. There is no libfranka signal for it, so
software cannot gate on it. `cell_panel`'s gate is therefore a **robot-state**
gate (pauses whenever the robot leaves MOVE, e.g. a real user stop, and ends
the run on REFLEX), plus a PAUSE/RESUME button. If a held-to-run interlock
is ever required, it has to come from the robot's own safety configuration
or from separate hardware.

**9. Do not subscribe to the 1 kHz robot state from a Python GUI.** Decoding
`FrankaRobotState` at 1000 Hz costs **86% of a CPU core**; taking the
messages raw and decoding a few costs 31% (rclpy still dispatches every
callback). A C++ `topic_tools throttle` child relaying at 50 Hz costs 5%,
which is what `cell_panel` starts. Subscribe **best-effort, depth 1** so a
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
`cell_panel.py` (IMPEDANCE & TRACK tab) exited 1 with `terminate called without an active
exception` on an ordinary window close: `rclpy.shutdown()` ran while a daemon
thread was still inside `rclpy.spin()`, taking a DDS thread down with the
context. The abort landed *after* the arm was handed back - by luck of the
race, not by construction, and it aborts in the one path that guarantees the
handoff. Fixed by owning the executor and tearing down in order
(`shutdown_ros()`: hand back -> `executor.shutdown()` -> `join` -> destroy ->
`rclpy.shutdown()`), with a mutation-checked test. Corollary: **never read a
GUI exit status as evidence the arm was released** - ask the controller
manager (`ros2 control list_controllers`).

## Lessons from 2026-09-23 to 09-25 (do not re-learn these)

**12. Split the RT success rate by controller mode before blaming the
network.**
- *What happened:* five comm reflexes on 09-24, all in HOLD or TRACK and
  never in FLOAT.
- *The cause:* since the 09-23 rebuild the controllers had had no
  `CMAKE_BUILD_TYPE` (no `-O2`), so the 1 kHz impedance law ran unoptimised
  Eigen.
- *Measured:* 8.7-28.7 % of HOLD samples were below 97 % RT success
  unoptimised, and 0.0 % after a Release build.
- *The rule:* an RT failure that depends on the mode points at the
  controller's compute. Check the build flags before the network, the
  scheduler or the GUI.
- Both CMakeLists now default to Release (C10).

**13. Cartesian tracking can drive a joint into its end stop.**
- *What happened:* two `joint_velocity_violation` reflexes on 09-24.
- *The cause:* J2 was at -103.7° and -104.6°, past its -102.2° limit. There
  the FR3's position-dependent velocity limit toward the stop is zero, so a
  slow drift of 0.06-0.16 rad/s trips it.
- *Misleading sign:* "the robot state froze first" was the reflex itself.
- *The rule:* check q against the joint limits (libfranka
  `rate_limiting.h`) before suspecting the driver.
- `tracking_node` now holds within 0.14 rad of a stop.

**14. Test node-level ROS plumbing before the arm.**
- *What happened:* the first hardware TRACK segfaulted in code no unit test
  reached: a range-for over a temporary from `future.get()`.
- *The fix:* `tools/fr3/sim/tracking_smoke.py` runs the real node in a fake
  cell. Since then it has caught a fake robot state with no joint positions,
  and a glide regression in the lead cap.
- *The rule:* run it before asking anyone to press TRACK.

**15. `PR_SET_PDEATHSIG` fires when the forking THREAD exits.**
- *What happened:* for a day, the panel's REC said "recording" and recorded
  nothing.
- *The cause:* the panel forked `ros2 bag record` from a short-lived worker
  thread with PDEATHSIG set to SIGINT. The bag got SIGINT as soon as that
  thread ended, before it had made its folder.
- *The rule:* fork long-lived children from a thread that lives as long as
  they should.
- *Related gotcha:* rosbag2 writes nothing under a path that contains a
  backslash, such as pytest's `tmp_path` on this domain account.

**16. The D405 is passive stereo: plain plastic returns no depth.**
- *What happened:* the cube's bare yellow faces leave bites in the depth
  region, so a depth-only outline, or plain ICP, is biased by millimetres.
- *Misleading sign:* the bias looked like depth erosion, then like a sticker
  offset.
- *Why:* the sticker and the blue patch are textured, so they do return
  depth.
- *The rule:* take the in-plane pose from colour edges, and z and tilt from
  depth (`roscam/object_pose.py`).

**17. The ArUco distance reads long with distance squared, and the hand-eye
absorbs it.**
- *Measured:* +1, +3.5 and +10 mm at 100, 200 and 300 mm. The marker reads
  about 0.7 px narrow, but the ray direction is right, so the range now
  comes from depth along the ArUco ray.
- *The trap:* the 09-15 hand-eye had been solved on the biased ranges and
  carried 3.9 mm of the error along the optical axis. Fixing the range alone
  would have moved the cube up by about 3.5 mm, and GRIP with it.
- *The rule:* change a measurement and its calibration together.

**18. numpy's OpenBLAS spreads tiny solves over every core.**
- *What happened:* a replay ran at 1744 % CPU for 6x6 solves and small SVDs,
  slower than on one thread.
- *The rule:* any numpy process on the cell PC gets
  `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS` and `MKL_NUM_THREADS` set to 1.
  The `fr3_cell` launch sets them for vision, and vision runs on E-cores
  12-19.

**19. A quaternion from a rotation matrix via `copysign` of the off-diagonal
terms is wrong at 180°.**
- *Why it matters:* every straight-down TCP pose is a 180° rotation, so GRIP
  got the wrong yaw in testing.
- *The rule:* use Shepperd's method (`grip_logic.R2q`), and test the
  half-turns explicitly.

**20. ROS callbacks never touch widgets, least of all under a lock.**
- *What happened:* the Tk panel froze whenever tracking changed state, and
  STOP TRACKING was unreachable.
- *The cause:* the spin thread called Tk inside `trace()` while holding a
  lock that the Tk thread was waiting on.
- *The rule:* in the PySide6 panel, the modules that host ROS callbacks cannot
  even import Qt, and the window hears from them only through `post()`
  (`test_cell_pins.py::test_ros_side_never_touches_qt`).

## Housekeeping

- [ ] Rename the workspace directory to remove spaces + trailing space
      (breaks colcon's `install/setup.sh`; workarounds in HANDOFF §2)
- [ ] **This machine's MoveIt install is broken** - `moveit_core` and
      `geometric_shapes` export imported targets (`tl::expected`,
      `random_numbers::random_numbers`) that are not found, so any
      `find_package(moveit_ros_planning_interface)` fails at configure.
      `mating_node` therefore cannot be built here, and the previously built
      binary went with the old install tree. Needed before the next full
      cell run; suspect a partial/mismatched ROS install (try
      `sudo apt install --reinstall ros-humble-moveit-core ros-humble-geometric-shapes ros-humble-random-numbers`)
- [ ] Consider making `tools/fr3/` a real package. It is a de-facto one
      (launch, params, operator GUIs, tests, testdata) but is reached by
      path, which is why `cell_panel.py` (IMPEDANCE & TRACK tab) needs `sys.path` surgery to
      import `tools/gui/mating_panel.py`. Deferred 2026-09-22: the churn
      would touch every doc, the env script and the test paths, for no
      functional gain today
- [ ] Modernize or delete `pick_n_place_` (legacy demo, still old patterns)
- [x] Push the repo to a remote — done: GitHub, branch `fr3-cell-panel`
- [ ] Merge the PR from `fr3-cell-panel` into `main`. It was opened
      2026-09-25, and the branch has moved on since
- [ ] Set a global git identity on this machine (commits currently use
      per-command `-c user.name/email`)
