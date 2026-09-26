# Next arm session: checklist

*Written 2026-09-25. Everything since the REC fix (401cf24) was built and
checked offline, on replayed recordings and mocks, because the arm and camera
were not available. This list is what only the arm and the camera can check.
Work down it in order. Each step says what passes and what to keep. The
analysis afterwards happens off the arm: record everything, then hand over the
run folder (`runs/<date>/`).*

The usual rules apply: every motion comes from a panel button, and the E-stop
stays in reach. Nothing on this list needs new code.

## 0. Before you launch

- [ ] **Rebuild roscam and object_pose_cpp, with T1 and T2 down.**
  - The installed roscam predates the `/object/*` topics. TRACK, GRIP and the
    panel now read those topics, so without the rebuild they get no pose at
    all.
  - `object_pose_cpp` is new: the depth estimator in C++. It gives the same
    poses as the Python one, about 4x faster.
  ```bash
  cd "/home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src"
  colcon build --symlink-install --packages-select roscam object_pose_cpp
  source install/local_setup.bash
  python3 -c "import roscam.object_contract, object_pose_cpp"               # no error
  ```
  - Keep `--symlink-install`. Since 2026-09-26 roscam is installed that way
    (Python edits then need no rebuild). A build without it now fails with
    "existing path cannot be removed: Is a directory".
  - Nothing else needs a rebuild: no robot-side C++ has changed since your
    17:20 build.
  - With the estimator on, the T2 log says `estimator cpp`. If it says
    `python`, `object_pose_cpp` isn't built; the poses are the same, just
    slower.
- [ ] **Check that no mock or replay process is left over.**
  `pgrep -af 'mock_cell|replay_eval|tracking_smoke'` should print nothing.
- [ ] **Bring up as usual** ([GUIDE.md](GUIDE.md) sections 1-2), but start
  T2 with `fr3_cell vision_source:=standalone`. REC records camera frames only
  in this mode.
- [ ] **Read the T2 log.** It should show:
  - `capture settings:` with High Accuracy and the spatial filter on;
  - `hand-eye TF from calib/handeye.yaml (...)`, without `xyz_aruco_range`.
- [ ] **Check the panel.** The vision chip should read `VISION <ms> · MARKER`.
- [ ] **Start the 1 kHz recorder for the whole session:**
  `ros2 run mating_controller state_recorder`. It gives the RT metrics with
  vision now pinned to the E-cores.

**Rollback** for anything in sections 1 and 2:
- `fr3_cell range_source:=aruco` goes back to the ArUco distance and the old
  hand-eye together.
- `vision_source:=realsense` goes back to the old camera owner. The
  `/object/*` topics still come out in that mode.

## 1. The marker distance from depth (8345442)

Do this section first, because everything else sits on top of it.

- [ ] **Still frames with REC on.** Hold the camera straight down over the
  cube at 100, 200 and 300 mm, and keep the arm still for at least 3 s at each
  height (HOLD, then SETPOINT up).
  - **Pass:** the cube's base-frame height agrees within about 1.5 mm across
    the three. With the ArUco distance it drifted from 202.6 to 194.2 mm.
  - I check this afterwards with `plateaus.py`.
- [ ] **TRACK as usual:** a minute of moving the cube by hand. It should feel
  the same as before.
- [ ] **5 GRIPs, started from different heights between 150 and 250 mm.**
  - **Pass:** the fingers close at the same height every time, within about
    1 mm, with the cube centred between them.
- [ ] **PLACE AT B, 3 times.** This is its first run on the arm.
  - B is ArUco id 1 (DICT_6X6_250), printed at 21 mm like id 0, lying flat
    on the table.
  - The arm must be still when B is first seen.
  - Note where the cube lands relative to B.

## 2. The Phase 1 exit (the /object/* contract)

Most of this reuses the presses from section 1, with REC and the recorder
running, so do both sections in one go.

- [ ] **60 s with everything still, REC on.** Afterwards, `/object/pose_raw`
  must equal `/aruco/pose_raw` message for message.
- [ ] **The ALIGN raw gate.** Cover the marker and press ALIGN: it must refuse,
  with a NO POSE or POSE STALE banner. Uncover the marker: ALIGN runs.
- [ ] **TRACK V1-V3** ([TRACKING_SPEC.md](TRACKING_SPEC.md) section 7), REC on.
- [ ] **ALIGN from 3 different start poses**, REC on. The time to converge
  goes into the baseline.
- [ ] **GRIP x5 and PLACE AT B x3.** The runs from section 1 count.

Afterwards I build the **baseline table** from these recordings:
- the marker's raw and filtered σ per axis;
- availability and hunting;
- ALIGN time;
- the GRIP and PLACE results;
- RT success with vision pinned.

## 3. The Phase 0 recordings that are still missing

Run with `fr3_cell vision_source:=standalone`. Press REC for each one, for
10-20 s, with the arm still unless the item says otherwise.

- [ ] Static at 80 mm, the closest distance.
- [ ] An oblique view, with 2-3 faces of the cube in view. Tilt the camera
  30-45° with FLOAT.
- [ ] The GRIP viewpoint with the fingers in view: the gripper open at 75 mm,
  at the approach height.
- [ ] A bare cube, with no marker in view. Turn the cube over, or use an
  unmarked cube.
- [ ] A hand touching and moving the cube.
- [ ] The cube and B in view together.
- [ ] Junk frames: first no cube in view, then clutter (tools, cables)
  without the cube.
- [ ] A lighting change: room lights off, then on, while recording.
- [ ] TRACK-like motion at 100-200 mm: in FLOAT, move the camera by hand over
  the cube. The 18:06 latency run covered only 200-350 mm.
- [ ] **Standalone against realsense:** 60 s still with
  `vision_source:=standalone`, then 60 s with `vision_source:=realsense` (bag
  only in that mode). If the two match, standalone can become the default
  (your call).
- [ ] Optional: 848x480 (`capture_width:=848`) at 100 and 200 mm.
- [ ] **The blur test: exposure** (needs T2 with
  `vision_source:=standalone object_shadow:=true`).
  - Blur is exposure time x image speed, and the depth pose drops in fast
    motion. The exposure and gain change live, and every recorded frame
    carries the ones it was shot with.
  - **The motion:** at 100 mm (TRACK's distance), in FLOAT, move the camera
    by hand over the cube as fast as TRACK FAST would (about 100 mm/s), with
    REC on. Do 30 s of that at each setting, then 10 s still (for the depth
    quality).
  - **The settings,** in a terminal, in this order:
    `ros2 param set /aruco_pose_publisher capture_exposure_us <us>` with
    `-1` (auto), then `8000`, `4000` and `2000`.
  - **If the image gets too dark:** raise the gain with
    `ros2 param set /aruco_pose_publisher capture_gain <g>`, or add light.
  - **On the D405, colour and depth share one sensor,** so the still part
    shows what a short exposure does to the depth.
  - **Afterwards I compare** the lost frames against exposure x image speed.

## 4. By hand

- [ ] **Measure the sticker on the cube with calipers.** Measure the offset
  from the marker centre to the centre of the top face (x and y), and the
  marker's rotation against the cube's edges.
  - Depth says the sticker is centred to within 0.05 mm. This is the
    independent check.
  - The measurement sets `T_marker_object` in `tools/fr3/parts/cube55.yaml`.
- [ ] **Find an independent tilt reference.** Depth and the marker disagree on
  the cube's tilt by a consistent offset: about 0.5-0.7° at 100 mm, growing
  to 1.3-2.1° at 300 mm, with the same sign in every session. That is a
  systematic bias in one of the two, and nothing yet says which. It also
  decides how far out `depth_checked` works: its 2° veto takes over beyond
  about 250 mm. Two options:
  - **Touch the table.** With the cube on the table, touch 3 points of the
    table around it with a fingertip in FLOAT, and note the TCP z from the
    panel at each. That gives the table plane in the base frame, and the
    cube's top face is parallel to it.
  - **Use an inclinometer.** Put a digital inclinometer on the table, then on
    the robot's base plate.

## 5. CPU use next to the 1 kHz loop

- [ ] **During TRACK, check the vision process in `htop`** (F4, filter
  `vision`).
  - Note its CPU % and its thread count.
  - Check that it runs only on CPUs 12-19.
  - The RT pill should stay at 99 % or higher.
  - OpenCV's own thread pool is not limited yet. If it spreads over many
    threads, the fix is `cv2.setNumThreads(1)`.

## 6. Optional, if there is time

- [ ] **Re-run `handeye_calib` with the depth distance** (21 poses). It
  replaces the z·(1 - 0.10 z) re-solve in `tools/fr3/calib/handeye.yaml`.
  The command is in [TODO.md](TODO.md) under "Hand-eye", with
  `range_source` left at depth.
- [ ] **Phase 3 shadow mode** (built offline 2026-09-25). Do this only after
  sections 1 and 2 pass: the Phase 1 baseline must be measured without it.
  - Start T2 with `fr3_cell vision_source:=standalone object_shadow:=true`.
    The VISION chip then adds `depth Δ<mm> <deg>`, and the debug image
    shows the estimated outline in green.
  - Keep REC and the 1 kHz recorder running throughout.
  - Do normal work with the cube in view, 20 min or more in total: TRACK
    V1-V3, ALIGN, GRIP and PLACE AT B. While GRIP holds the cube, the chip
    reads `depth —`.
  - **A/B for the RT metrics:** run TRACK V6 (15 min, TRACKING_SPEC section 7)
    with the estimator on. Then run it again after
    `ros2 param set /aruco_pose_publisher object_shadow false`.
  - Afterwards I compute the per-axis agreement from the bags. The exit is
    the budgets in PERCEPTION_PLAN section 1, with RT success of 0.99 or
    higher and no added reflexes with the estimator on.

## 7. Phase 4: depth drives, the marker vetoes (built offline 2026-09-26)

Only after sections 1 and 2 pass and the Phase 3 sessions look right. Every
motion is still a button press, and the marker stays on the cube: it checks
every depth pose.

- [ ] **Start and switch.** Start T2 with
  `fr3_cell vision_source:=standalone`. While idle, set the source dropdown
  in the CAMERA header to `depth_checked`. The chip then reads
  `DEPTH✓MARKER · veto <n>%`. When there is no pose, the banner says why:
  vetoed, acquiring 3/5, or HELD.
- [ ] **Keep recording.** REC and the 1 kHz recorder stay on throughout.
- [ ] **TRACK V1, V2 and V4** (TRACKING_SPEC section 7).
  - V1 must hold within 5 mm / 0.5°.
  - In V4, covering the marker must make the arm hold (no marker, no depth
    pose in this phase), with no lurch when you uncover it.
- [ ] **ALIGN from the Phase 1 baseline's 3 start poses.** It should take no
  more than 1.5x the marker's time.
- [ ] **10 GRIPs** from random placements and yaws, **and 5 PLACE AT B.**
  Pass: 10/10 and 5/5.
- [ ] **If anything looks off:** set the dropdown back to `marker` (idle
  only), or restart T2, which always starts on `marker`.
- **Afterwards I compare** with the Phase 1 baseline. The integrator's
  hunting must be no worse, and the veto rate under 5 %.

## Open, but not for this session

- The controller-side items C1-C5 and C7-C9 in
  [tools/fr3/cell/README.md](tools/fr3/cell/README.md). C4 and C5 are
  franka_ros2 changes.
- Rung 4 (the dispatched stroke), the force thresholds and the connector
  offsets. All of them wait on the connector being chosen.
