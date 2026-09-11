# Agent Handoff — Robotic Connector Mating Cell

*Written 2026-07-05 after a full audit-and-rework session. Read this first;
it explains what exists, what was changed and why, what is verified, and
what to do next. Companion doc: [SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md).*

## 1. Project context

Vision-guided **connector mating** on a Mitsubishi **MELFA RV-5AS** cobot
(ROS 2 Humble + MoveIt 2 + Pilz LIN planner) with an eye-in-hand RealSense
**D405** and one ArUco marker (DICT_6X6_250, id 11, 21 mm) on the work
surface near the connector. Owner: Kaan Ural (k.ural1@salford.ac.uk),
University of Salford.

Two-phase motion profile: fast approach to the target area, then a slow,
decelerated insertion. The session's goals were: fix the (badly broken)
original code, add **full orientation matching (roll + pitch on top of
X/Y/Z + yaw)**, make it reliable/smooth, and make it robot-agnostic.

## 2. Environment — read before running anything

- **This machine is a lab/dev PC, NOT the robot PC.** ROS 2 Humble,
  MoveIt 2, Pilz, OpenCV 4.12, colcon are installed. The MELFA packages
  (`melfa_msgs`, `melfa_bringup`, `melfa_rv5as_moveit_config`) are **absent**
  here — they exist only on the robot PC. Consequently `plc_`/`hmi_` build
  conditionally (skipped here) and the controller has never run against the
  real MELFA move_group.
- **The workspace path contains spaces AND a trailing space**
  (`.../Robotic Connector Handling /src`). colcon's generated
  `install/setup.sh` is broken by this — do NOT source it. Workarounds used:
  `CMAKE_PREFIX_PATH="$PWD/install/<pkg>:$CMAKE_PREFIX_PATH"` for builds,
  `AMENT_PREFIX_PATH="$PWD/install/<pkg>:$AMENT_PREFIX_PATH"` for
  `ros2 run/launch`. Renaming the directory (e.g. `robotic_connector_handling`)
  would fix this properly; the user hasn't done it yet.
- ~/.bashrc sources several other workspaces (franka_ros2_ws, livox, etc.).
  `franka_ros2_ws` matters: it's used by the parked dry-run harness (§6).
- Build/test: `source /opt/ros/humble/setup.sh && colcon build` from the src
  dir. `build/ install/ log/` are gitignored.

## 3. Git history (repo initialized this session)

```
59dc4ad  Baseline: original state before audit fixes   <- untouched original
b2b0856  Rework vision + motion control for reliable connector mating
bd8f5d2  Add setup/calibration guide; make controller robot-agnostic
fb70b47  Add FR3 dry-run harness (unfinished; kept for future validation)
95e4527  Add reset service, geometry unit tests, and hand-eye calibration tool
```

Diff any file against `59dc4ad` to see the original. The original had 5
divergent `cam_pub.py` variants, an `old/` directory, and severe control
bugs (documented in §4) — all deleted/replaced, recoverable from history.

## 4. What was wrong originally (audit summary)

Critical bugs fixed in `b2b0856` — do not reintroduce these patterns:

1. `bool mated` declared but never used → the arm could servo again *after*
   mating (post-mating motion hazard).
2. Entire terminal sequence (pre-final + final blind moves) executed
   back-to-back inside ONE `/cam` subscription callback, blocking it for
   seconds while stale messages queued (depth 10) and were replayed after.
3. Step size `fabs(std::min(0.1, smallest))` where `smallest` mixed signed
   errors across axes → a large negative error produced up to 0.5 m steps.
4. Camera→robot frames hard-coded twice (axis swap in the publisher + sign
   table in the controller) instead of TF; broke for any tool rotation.
5. No occlusion/vision-loss handling anywhere; no state machine (int flags).
6. The "two-phase speed profile" did not exist — velocity scaling set once.
7. Vision used `cv2.aruco.DetectorParameters_create()` (removed in
   OpenCV ≥4.7 — crashed on this machine's 4.12), `SOLVEPNP_ITERATIVE`
   (pose-flip ambiguity on planar markers), published translation-only
   `Point` with axes swapped, `cv2.imshow` inside callbacks.
8. Packaging: dead `setup.py` in an ament_cmake package, missing
   package.xml deps, calibration YAML/npy duplicated in 3 places.

## 5. Current architecture (all verified to build; tests pass)

```
camera driver ──image/camera_info──▶ roscam cam_pub ──/aruco/pose (PoseStamped,
                                                       optical frame)──▶ move_l
move_l ──LIN pose goals──▶ move_group ──▶ robot driver
```

### Vision — `roscam/roscam/cam_pub.py` (entry point `cam_pub`, node `aruco_pose_publisher`)
- Intrinsics from `camera_info` (no calibration files anywhere).
- New ArUco API with legacy fallback; `SOLVEPNP_IPPE_SQUARE` + subpixel
  corners; gating on reprojection error (>2 px reject) and translation jumps
  (>5 cm reject, unless persistent → re-acquire); EMA + slerp low-pass
  (`filter_alpha` 0.35).
- Publishes `/aruco/pose` (filtered), `/aruco/pose_raw`, `/aruco/debug_image`.
- **Verified end-to-end** with a synthetic rendered marker: recovered pose
  within 0.1 mm of ground truth (test script pattern: render marker with
  `cv2.aruco.generateImageMarker`, feed through callbacks, probe output).

### Controller — `melfa_rv5as_masterclass/src/move_l.cpp` (exe `move_l`, node `connector_mating_node`)
- Phase machine: `WAIT_FOR_VISION → ALIGN_COARSE → ALIGN_FINE → INSERT →
  MATED` (+ `FAULT`). MATED/FAULT are **latched**; recover via
  `ros2 service call /connector_mating_node/reset std_srvs/srv/Trigger`.
- Subscription only stores the latest pose (QoS 1, mutex); all planning/
  execution in a dedicated control thread (`run()` loop, `control_period_s`).
- Geometry: marker pose → planning frame via TF2; goal = marker pose ∘
  (connector offset in marker frame + standoff along marker Z + 180° X-flip
  + yaw offset). Full 6-DOF error servoed with per-cycle clamps
  (coarse 5 cm/5°, fine 1 cm/1.5°); orientation locked during INSERT
  (stroke = straight line along current tool Z).
- Guards: vision watchdog (stale → hold, `vision_timeout_s`), joint-jump
  plan rejection (`max_joint_jump_rad`, catches IK branch flips),
  N-consecutive-plan-failures → FAULT, `align_hold_cycles` of fresh-vision
  in-tolerance before committing insertion, drift-back to ALIGN_COARSE if
  the target moves.
- Robot-agnostic: `planning_group`, `EEF_FRAME_ID`, `planning_pipeline`
  (default pilz), `planner_id` (default LIN; empty string = pipeline
  default), everything else parameterized in
  `melfa_rv5as_masterclass/config/rv5as_params.yaml`.
- `enable_insertion: false` = calibration/teach mode (aligns, hovers, logs).
- Pose math lives in `include/melfa_rv5as_masterclass/mating_geometry.hpp`
  — pure functions, covered by `test/test_mating_geometry.cpp`
  (**12 gtests, all passing**: clamps exact, shortest-path rotation,
  marker-tilt roll/pitch tracking, offsets in marker frame, insertion along
  tilted tool axis).

### Calibration tool — `roscam/roscam/handeye_calib.py` (entry point `handeye_calib`)
- Interactive eye-in-hand collection (Enter=record TF base→tcp +
  `/aruco/pose_raw` pair; `s`=solve; samples persist to YAML) and offline
  `--solve file.yaml`.
- Solves with Tsai/Park/Horaud/Daniilidis, ranks by consistency residual
  (marker-fixed-in-base spread), prints a ready-to-paste
  `static_transform_publisher` command (T_tcp→camera_optical).
- **Solver verified against synthetic ground truth**: 1.3 mm / 0.07°
  recovery with realistic noise → OpenCV conventions confirmed correct.

### Launch / docs
- `melfa_rv5as_masterclass/launch/move_l.launch.py`: args `robot_name`,
  `moveit_config_package`, `params_file` (defaults = MELFA RV-5AS).
- `melfa_ros2_bringup.txt`: 6-terminal bringup, new-style
  static_transform_publisher syntax.
- `SETUP_AND_CALIBRATION.md`: full calibration procedure, porting checklist,
  parameter reference, troubleshooting. Keep it in sync with code changes.

## 6. Parked: FR3 dry-run harness (`tools/dryrun_fr3/`)

Goal: first live execution of the controller against a mock robot (Franka
FR3 from `~/franka_ros2_ws`), since the MELFA stack isn't on this machine.
Components: `fr3_dryrun.launch.py` (mock ros2_control + move_group +
static hand-eye TF + controller with FR3 params) and `fake_marker_pub.py`
(marker fixed in base, re-expressed in the moving optical frame via TF,
with noise; only publishes when in front of the camera).

Status when parked (user said "forget franka testing"):
- Run 1: controller started, planning failed ×5 → correctly latched FAULT.
  Root cause was in the **harness**, not the controller: move_group was
  given the legacy single-pipeline config so no pipeline named `ompl`
  existed. **Fixed** (modern `planning_pipelines: ['ompl']` layout) but
  **never re-run** — the FAULT latch behavior is the only runtime evidence
  so far.
- Known harness detail: the stock franka effort-command JTC cannot move
  `fake_components/GenericSystem` (doesn't integrate torques) — that's why
  `fr3_dryrun_controllers.yaml` swaps in a position-command JTC.
- To resume: `source /opt/ros/humble/setup.sh && source
  ~/franka_ros2_ws/install/setup.sh && export AMENT_PREFIX_PATH="$PWD/install/melfa_rv5as_masterclass:$AMENT_PREFIX_PATH"
  && ros2 launch tools/dryrun_fr3/fr3_dryrun.launch.py`. Success = log shows
  ALIGN_COARSE → ALIGN_FINE → INSERT → MATED with error converging
  below 2.5 mm / 1°. Expect to tune: FR3 reachability of the fake marker
  pose (0.45, 0.10, 0.05 in fr3_link0), OMPL time-parameterization vs the
  0.4 s control period.

## 7. Verification matrix — what is and isn't proven

| Component | Status |
|---|---|
| C++ build (zero warnings, -Wall -Wextra -Wpedantic) | ✅ verified |
| Geometry math (clamps, flips, offsets, insertion axis) | ✅ 12 gtests pass |
| Vision node end-to-end (synthetic image → pose) | ✅ 0.1 mm accuracy |
| Hand-eye solver (synthetic ground truth) | ✅ 1.3 mm / 0.07° |
| Controller vs live move_group (phases, execution) | ❌ **never run** |
| Pilz LIN behavior with small frequent goals | ❌ unverified |
| Marker-frame flip convention vs real MELFA TCP frame | ❌ unverified (first RViz run will show it) |
| Hand-eye calibration on the real cell | ❌ not done (tool ready) |
| Connector offsets (`connector_offset_*`) | ❌ **all zero — must be taught**; the old blind 12 cm/3 cm base-frame offsets were deliberately NOT carried over |

## 8. Recommended next steps (in order)

1. **On the robot PC**: clone/copy the repo, `colcon build` (plc_/hmi_ will
   build there), then the fake-hardware dry run
   (`use_fake_hardware:=true`, `enable_insertion: false`) — watch phase
   transitions and motion directions in RViz. This closes the biggest
   verification gap.
2. Run `handeye_calib` on the real cell; replace the guessed static TF in
   the bringup (`-1.57 -1.57 1.57` is a rough guess and currently the
   dominant roll/pitch error source).
3. Teach `connector_offset_x/y/z`, `tool_yaw_offset_deg`,
   `standoff_height_m`, `insertion_depth_m` per §2.4 of the setup guide.
4. Full validation ladder (§4 of setup guide), then live mating at reduced
   `insert_speed`.
5. Optional / deferred by choice:
   - `computeCartesianPath` fallback for Pilz-less robots (INSERT stroke is
     not path-guaranteed under OMPL — documented limitation).
   - `moveit_servo` migration for continuous (non-stepped) tracking.
   - Retract-to-standoff failure policy (currently FAULT = hold; decide
     after seeing real failure modes).
   - Multi-marker / board support in vision for occlusion robustness.
   - Modernize or delete `pick_n_place_` (untouched legacy demo).
   - Rename the workspace directory to kill the spaces-in-path issue.

## 9. Conventions and decisions to preserve

- Marker frame: X/Y in marker plane, Z out of the work surface. Goal tool
  orientation = marker rotation ∘ RotX(180°) ∘ RotZ(yaw_offset) → tool Z
  points INTO the surface. If the real TCP convention differs, fix it in
  `mating_geometry::standoff_goal` (and its tests), nowhere else.
- Vision publishes in the **camera optical frame** (`header.frame_id` from
  the image); the hand-eye TF must chain tcp → ... → optical. RealSense
  provides `camera_link → camera_color_optical_frame` itself.
- INSERT is a *committed* stroke by design: armed only after
  `align_hold_cycles` of fresh-vision fine alignment; marker occlusion by
  the gripper mid-stroke must NOT abort it.
- Never plan/execute in a subscription callback; never bypass the step
  clamps; keep every tunable in the params YAML, not in code.
- Tests: `colcon test --packages-select melfa_rv5as_masterclass`; keep
  `mating_geometry.hpp` pure (no ROS node deps) so it stays testable.
- Commits so far use `git -c user.name="Kaan Ural" -c
  user.email="k.ural1@salford.ac.uk"` since no global git identity is set.
