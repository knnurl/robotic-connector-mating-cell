# Project state — Robotic Connector Mating Cell

*Snapshot: 2026-09-26, branch `fr3-cell-panel`: pushed up to `a8809d7`;
`50c65fc`, `1cce687`, `8fb0abd` and this docs commit are local, not pushed.
`main` is behind (PR open). The short, current answer to "where are we?". The next
arm session's to-do list is [ARM_CHECKLIST.md](ARM_CHECKLIST.md). Detail lives
in [TODO.md](TODO.md) (tasks, plus the lessons that should not be re-learned),
[PERCEPTION_PLAN.md](PERCEPTION_PLAN.md) (marker-free perception),
[TRACKING_SPEC.md](TRACKING_SPEC.md) (continuous tracking) and
[GUIDE.md](GUIDE.md) (operating the cell). [DOCS.md](DOCS.md) indexes all of
them and says which copy to believe.*

## In one paragraph

A vision-guided connector-mating cell on a **Franka FR3**. The original target
was a Mitsubishi RV-5AS, now parked. A wrist-mounted RealSense D405 finds a
marked part, and the arm aligns to it, tracks it and grips it under Cartesian
impedance control. Every motion starts from a button on a one-page operator
panel. **ALIGN, TRACK and GRIP work on the real arm. PLACE AT B is built but
has only run against the mock. Insertion has never run:** rung 4 of the
commissioning ladder waits on the connector choice and the force thresholds.
The current work is **marker-free perception** (PERCEPTION_PLAN.md):
- the depth and colour-edge pose estimator is built and scored offline;
- the `/object/*` pose contract is in place, so TRACK, GRIP and ALIGN can
  switch pose source with a parameter;
- both still need the arm to confirm them.

## Progress at a glance

| Stage | State | Evidence / notes |
|---|---|---|
| FCI link and ROS stack | Working | The 5 comm reflexes of 2026-09-24 came from an unoptimised controller build. Release is now the default (`da51731`, C10). A reflex still kills `ros2_control_node` (C5) |
| Camera: D405 | Working | 640x480 at 15 fps with depth. High Accuracy preset plus the spatial filter (half the depth noise up to 200 mm). The vision process is pinned to E-cores 12-19 |
| Marker pose | Working; **new distance not yet on the arm** | The ArUco distance read +1 to +10 mm long at 100-300 mm. It now comes from the depth plane around the marker (`8345442`); on replay it is within 1.4 mm of the kinematics. Latency measured at 24 ms |
| Hand-eye calibration | Re-solved 2026-09-25 | The 09-15 samples were re-solved for the depth distance: xyz moved 3.9 mm along the optical axis. The old solve is kept for `range_source:=aruco`. A fresh calibration would supersede both |
| Operator panel | Working on the arm | PySide6, one page, `tools/fr3/cell/` (`78c232e`). REC fixed (`401cf24`) |
| ALIGN | Working on the arm | 239.6 mm → 1.6 mm, 9.7° → 0.49° (09-23). New and not yet on the arm: it refuses a raw pose older than 0.25 s |
| Impedance ladder | Rungs 0-3 passed 2026-09-16 | Rung 4, the dispatched stroke, waits on the connector and the force thresholds |
| TRACK | **Working on the arm** (09-24) | Glides between camera frames; holds at the joint limits and the workspace box; keeps the operator's gains; FAST switch; buzz stop at 3.5 Nm. Smoke test 29/29. The formal V1-V3 table is not recorded yet |
| GRIP | **Working on the arm** (09-25) | 3 cycles on the 55 mm cube, under impedance, not MoveIt |
| PLACE AT B | Built, mock-tested only | Its first arm run is in ARM_CHECKLIST.md |
| Perception phase 0 (measure) | Partly done | Range bias, latency and depth settings measured. 10 recordings still missing |
| Perception phase 1 (contract) | Built offline | `/object/*`, with the marker as the source. The on-arm exit is pending |
| Perception phase 2 (estimator) | Built offline, scored on replay | `roscam/object_pose.py`: within ±0.45 mm of the marker at 100-300 mm, 100 % valid, yaw scatter 0.13-0.20° at 100-150 mm. Tracking p95 is 23-27 ms on one E-core. Tilt is offset from the marker's, consistently: about 0.5-0.7° at 100 mm, 1.3-2.1° at 300 mm. No independent reference yet |
| Perception phase 3 (shadow mode) | Built offline, replayed | The estimator runs beside the marker in the live frame loop and only reports (`object_shadow:=true`, off by default). Replayed on the recordings: 3,181 of 3,183 static marker frames valid; the whole frame p95 44.8 ms on one E-core against a 66.7 ms period. Its arm sessions are in ARM_CHECKLIST.md section 6 |
| Perception phase 4 (`depth_checked`) | Built offline, replayed, reviewed | Depth drives `/object/*`, and the same frame's marker vetoes anything more than 3 mm, 2° in-plane or 4° tilt away. Replayed: a raw pose on 98-99 % of marker frames out to 275 mm when still, 79.5 % in hand-guided motion. The estimator runs in C++ (`object_pose_cpp`, the same poses as the Python one); the depth raw goes out at p95 23 ms (the target is 40). Chosen from the panel's source dropdown; its arm runs are ARM_CHECKLIST.md section 7 |
| Force thresholds | Not tuned | |
| Connector | Not chosen | Offsets not taught |
| Full mate | Not attempted | |

## Where we are right now

- **2026-09-26, later: the estimator in C++, the Phase 4 rules revised, and
  exposure made live** (`50c65fc`, `8fb0abd`).
  - **`object_pose_cpp`** is a new colcon package: a pybind11 port of the
    estimator. It gives the same poses as the Python version to rounding, on
    91 recorded and 11 synthetic cases including corrupted input. On an
    E-core it runs at p95 8.5 ms instead of 30.7.
  - **The Phase 4 rules changed, with the user's OK:** the veto's tilt limit
    is 4°, and a dropped frame pauses acquisition instead of restarting it.
    `depth_checked` availability in hand-guided motion went from 25 % to
    80 %.
  - **Exposure and gain change live,** and every recorded frame carries
    them. That's for the blur test in ARM_CHECKLIST section 3.
- **2026-09-26: perception Phase 4 (`depth_checked`) built offline**
  (`1cce687`).
  - `roscam/depth_checked.py` holds the per-frame rules. A raw pose is only
    the frame's own depth estimate, and only after it passed its gates, the
    marker's veto (3 mm / 2°) and the object filter's jump gate, and only
    after 5 agreeing frames in a row. None while GRIP holds the part.
  - Any source switch leaves at least 0.34 s with nothing on `/object/*`, so
    TRACK holds across it.
  - Two adversarial reviews found no blocker. Their fixes include:
    - drawing the debug outline only after the markers were detected;
    - a switch gap by time, not only frames;
    - launch-only veto thresholds;
    - stale quality fields dropping off the panel.
  - The replay also settled what the tilt disagreement is: a consistent
    offset, 0.5-0.7° at 100 mm growing to 1.3-2.1° at 300 mm, not noise.
- **2026-09-25, night: perception Phase 3 (shadow mode) built offline**
  (`1cce687`).
  - `roscam/object_shadow.py` runs the estimator in `vision_standalone`
    after the marker, seeded from it. Each frame's `/object/pose_quality`
    gains the estimate, and its agreement with the marker.
  - The panel chip reads `MARKER · depth Δ0.4 mm 0.3°`.
  - `tools/fr3/vision/loop_replay.py` put the four recorded sessions
    through the real frame loop.
  - Two fixes came from it, so that the estimator never makes the marker
    frame late: the estimator's pixel grid is built at start-up, and blurred
    frames are reported, not re-solved from depth.
  - Deviations from the plan are marked **[AS BUILT]** in
    PERCEPTION_PLAN.md. The main one: the object Kalman filter moves to
    Phase 4.
- **2026-09-25, afternoon and evening: perception, mostly offline.**
  - *Phase 0 on the arm:*
    - REC now records camera frames in-process (`ba26387`).
    - The first sessions showed the ArUco distance growing too long with
      distance squared, while depth matched the robot kinematics.
    - Latency is 24 ms.
    - High Accuracy plus the spatial filter halves the depth noise.
  - *Then the arm and camera became unavailable, and everything after
    `401cf24` was built against the recordings:*
    - the marker distance from depth, with the hand-eye re-solved to match
      (`8345442`);
    - the marker-free estimator (`c8991e6`, speed pass `c75a878`);
    - the `/object/*` contract (`a8809d7`).
  - *Key finding:* the D405 is passive stereo, so the cube's plain plastic
    returns no depth. A depth-only outline is therefore biased, and the
    estimator takes its in-plane pose from colour edges instead.
  - **Before the next arm session, rebuild roscam** (`colcon build
    --packages-select roscam` in `src/`). The installed copy publishes no
    `/object/*`, and TRACK, GRIP and the panel now read those topics.
- **2026-09-25, morning:**
  - GRIP was built and works on the arm (`d17a3c9`).
  - PLACE AT B was built; it has run against the mock only (`75df4d1`).
  - TRACK now keeps the operator's gains.
  - PERCEPTION_PLAN.md was written and critic-checked (`3dfacf8`).
  - The TRACK lead cap now judges the frame's target, not the glide
    (`f40b69e`, caught by the smoke test).
- **2026-09-24:**
  - The PySide6 panel replaced the Tk one (`78c232e`).
  - TRACK ran on the arm: the user said "tracking works very well".
  - Two `joint_velocity_violation` reflexes were J2 driven past its limit.
    That led to the joint-margin hold, the workspace-box hold and the glide
    between frames (`c8fe175`).
  - The comm reflexes were the unoptimised build (`da51731`).
- **2026-09-23:**
  - A snapshot commit and the tag `pre-cleanup-2026-09-23`.
  - Camera-centred TRACK with an over-lead policy.
  - Two-terminal bring-up (`fr3_cell`); servo, `mating_node` and the MELFA
    tools archived.
  - The first hardware TRACK segfaulted (fixed in `87debcb`), and the
    no-robot smoke test followed (`2ec122e`).
  - A 40 Hz wrist buzz traced to rotational damping (ζ 1.0 → 0.5, plus a
    buzz watchdog).
- **2026-09-16:** impedance rungs 0-3 on the real arm.
  - Payload is set in **Desk** (0.83 kg), so the panel's payload is 0.
  - The Coulomb friction deadband is `F_friction / k` (TODO lesson 10).
- **Tests:** `tools/run_tests.sh` covers the colcon build, 159 colcon tests,
  354 pytest and the tracking smoke test (29/29, on DDS domain 87). It never
  touches the robot or the camera.

## Next actions, in order

1. **The next arm session: [ARM_CHECKLIST.md](ARM_CHECKLIST.md).** Rebuild
   roscam first. Then:
   - the depth-distance check;
   - the Phase 1 exit: the bag comparison, TRACK V1-V3, ALIGN from 3 starts,
     5 GRIPs, 3 PLACE AT B, and the baseline table;
   - the missing Phase 0 recordings;
   - the sticker and tilt references.
2. **Phase 3 on the arm, after step 1:** the shadow sessions (20 min or
   more inside the budgets), and the estimator on/off A/B for the RT
   metrics (ARM_CHECKLIST section 6).
3. **Phase 4 on the arm, after step 2** (ARM_CHECKLIST section 7): TRACK V1,
   V2 and V4, ALIGN, 10 GRIPs and 5 PLACE AT B in `depth_checked`, against
   the Phase 1 baseline. Open going in:
   - which side the tilt offset is on;
   - availability in fast motion at 100 mm, and how much a shorter exposure
     helps (the blur test).
4. **Controller-side items C1-C5 and C7-C9**
   ([tools/fr3/cell/README.md](tools/fr3/cell/README.md)). Suggested order:
   C8 with C1, then C9, then C2/C3, then C7. C4 and C5 need franka_ros2
   changes.
5. **Merge the PR** from `fr3-cell-panel` into `main`.
6. **When the connector is chosen:** perception Phase 7, the connector
   offsets, the force thresholds, rung 4, then the full mate.

## Decisions already made

- **FR3 only** (2026-09-23). MELFA is parked under `melfa/`.
- **Impedance, not moveit_servo,** for tracking and insertion.
  - Streaming into the effort-mode trajectory controller stalled the arm and
    made it buzz.
  - Servo is archived (tag `pre-cleanup-2026-09-23`).
- **Mating path:** the panel, `tracking_node` and an impedance stroke. The
  autonomous `mating_node` phase machine is parked.
- **TRACK aims camera-centred,** at the pose ALIGN leaves. The connector offset
  is applied only in a later approach or stroke step.
- **The marker-free end goal** (PERCEPTION_PLAN.md):
  - an incremental rollout behind the `/object/*` contract: marker, then
    shadow, then `depth_checked`, then the marker only as a seed, then no
    marker;
  - a classical depth-plus-edges engine on the CPU, generic over STL parts
    (the user chose "Generic STL" over "cube first");
  - a learned model only if the connector needs one.
- **The enabling device is left as is.** It is invisible over FCI. The operator
  controls are PAUSE, STOP and the robot-state gate; safety is the E-stop.
- **Commissioning reflex thresholds: 40 N / 40 Nm Cartesian.**
- **Every robot motion stays behind a GUI button** (human in the loop).
  RELEASE is one click away whenever no other action is running, and every
  exit path hands the arm back first.
- **Run logs** go to `runs/YYYY-MM-DD/` beside `src/`, outside the repo
  (`FR3_LOG_DIR`).

## Known risks and open problems

- **The new marker distance and the re-solved hand-eye are unproven on the
  arm.** They move GRIP and PLACE heights by millimetres. The rollback is
  `fr3_cell range_source:=aruco`.
- **Any libfranka reflex kills `ros2_control_node`** (C5). franka_hardware does
  not catch the exception. The robot stops but is not freed, and T1 must be
  relaunched. The panel reloads the impedance controller by itself afterwards.
- **Joint limits during TRACK.** Cartesian tracking once drove J2 past its limit,
  where the FR3 allows zero velocity toward the stop. The joint-margin hold
  (0.14 rad) now stops that, but only inside TRACK.
- **Tilt: depth and the marker disagree by a consistent offset,** about
  0.5-0.7° at 100 mm and growing to 1.3-2.1° at 300 mm, with the same sign in
  every recorded session.
  - It is a systematic bias in one of the two, not noise. The signed
    components show it; the earlier unsigned figure mixed bias and noise.
  - Nothing yet says which one is right (ARM_CHECKLIST section 4).
  - Beyond about 250 mm it makes `depth_checked`'s 2° veto take over.
- **Plain plastic gives no depth.** The estimator leans on colour edges, and
  it has not yet seen a lighting change, a hand or clutter (the missing Phase 0
  recordings).
- **franka_hardware 2.0.2 crashes on a combined cross-mode controller switch**
  (position↔effort). Use two separate calls. Effort↔effort swaps are safe.
- **Stop T1 before rebuilding the controllers,** and do not run
  `tools/run_tests.sh` while the arm is up: it rebuilds the C++ packages.
- **The workspace path contains spaces,** which some colcon setup scripts
  handle badly. `fr3_env.sh` works around it.

## Where things live

| Path | What |
|---|---|
| `tools/fr3/cell/` | The operator panel (PySide6), `grip_node` (GRIP / PLACE / PLACE AT B), `mock_cell` (no-robot mode on DDS domain 88) |
| `tools/fr3/fr3_cell.launch.py` | T2 of the bring-up: controller spawner, hand-eye TF, vision, tracking node, grip node, panel |
| `tools/fr3/fr3_params.yaml`, `tools/fr3/calib/handeye.yaml` | Cell parameters (the pose topics point at `/object/*`), the one hand-eye file |
| `roscam/` | Vision: `cam_pub` (marker), `vision_standalone` (camera owner, frame recorder), `object_contract` (`/object/*`), `object_pose` (the marker-free estimator) |
| `tools/fr3/vision/`, `tools/fr3/parts/` | `depth_quality`, `latency_fit` and `replay_eval`; the part files (`cube55.yaml`) |
| `mating_controller/` | `tracking_node` (with the pure law in `tracking_law.hpp`), `state_recorder` (1 kHz CSV) |
| `fr3_mating_controllers/` | The torque controller; safety logic in `impedance_detail.hpp`, gtests in `test/` |
| `tools/run_tests.sh`, `tools/fr3/sim/tracking_smoke.py` | Every test, and the real `tracking_node` in a fake cell |
| `../runs/YYYY-MM-DD/` | Run logs, bags, recordings and their analyses, outside git |
