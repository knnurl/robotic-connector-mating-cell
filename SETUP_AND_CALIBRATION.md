# Connector-Mating Cell — Setup, Calibration & Launch

Vision-guided connector mating with an eye-in-hand RGB camera and an ArUco
marker on the work surface. The stack is **robot-agnostic**: the vision node
knows nothing about the robot, and the controller talks only to MoveIt and
TF. Everything robot-specific is a parameter or a launch argument.

```
 camera driver ──image/camera_info──▶ roscam cam_pub ──/aruco/pose──▶ move_l controller
                                      (ArUco 6-DOF pose,              (phase machine,
                                       optical frame)                 6-DOF servo via MoveIt)
                                                                          │
 robot driver ◀── trajectory execution ── move_group (MoveIt) ◀── LIN pose goals
```

**Mating sequence:** `WAIT_FOR_VISION → ALIGN_COARSE (fast) → ALIGN_FINE →
INSERT (slow, committed) → MATED (latched, no further motion)`.
Stale vision at any point before INSERT = hold position. Repeated planning
failures = FAULT (hold).

---

## 1. What you need

| Component | Requirement |
|---|---|
| Robot | Any arm with a ROS 2 driver + MoveIt config (tested: MELFA RV-5AS) |
| Planner | A straight-line-capable pipeline. Pilz `LIN` preferred; see §6 |
| Camera | Any RGB camera publishing `sensor_msgs/Image` + `camera_info` (tested: RealSense D405), rigidly mounted on the tool |
| Marker | One ArUco marker (default `DICT_6X6_250`, id 11, 21 mm) fixed to the work surface near the connector |
| TF | A complete chain `<planning_frame> → ... → <tcp> → <camera optical frame>` |

Software: ROS 2 (Humble), MoveIt 2, OpenCV ≥ 4.7 (older works too — the node
falls back to the legacy ArUco API), `cv_bridge`.

Build:

```bash
cd <workspace>            # the directory containing this file is the src/ dir
colcon build
source install/setup.bash # avoid spaces in the workspace path
```

`plc_`/`hmi_` (MELFA GPIO demos) build only where `melfa_msgs` exists; the
mating pipeline does not need them.

---

## 2. One-time calibration

Do these once per cell, in this order. Accuracy budget: the final mating
error is roughly *marker pose error + hand-eye error + taught-offset error* —
don't spend hours on one term while another is a guess.

### 2.1 Marker

1. Print the marker (default id 11, `DICT_6X6_250`) with scaling disabled.
2. Measure the printed black square with calipers and put the *measured*
   size in `marker_size_m` (roscam parameter). A 5% size error is a 5% range
   error.
3. Fix it flat and rigid near the connector. Matte paper beats glossy.
4. Verify detection: `ros2 run roscam cam_pub`, then view `/aruco/debug_image`
   in `rqt`. The drawn axes must be stable (no flicker/flips) across the
   whole approach volume.

### 2.2 Camera intrinsics

Nothing to do for cameras that publish factory calibration (RealSense does):
the node reads `camera_info`. For a camera without calibrated
`camera_info`, run the standard ROS camera calibrator
(`ros2 run camera_calibration cameracalibrator`) and load the result into
your camera driver — **not** into this stack; it only ever reads
`camera_info`.

If your topics differ from the RealSense defaults, remap via parameters:
`image_topic`, `camera_info_topic`.

### 2.3 Hand-eye (tool → camera) — the important one

The controller needs TF from the TCP to the camera **optical** frame. A
tape-measure guess caps your roll/pitch accuracy at several degrees; calibrate
it.

Use the built-in tool (robot driver + `cam_pub` must be running):

```bash
ros2 run roscam handeye_calib --ros-args -p base_frame:=rv5as_base -p tcp_frame:=rv5as_default_tcp
```

1. Fix a marker/board in the workspace where it stays put.
2. Jog the robot to 10–15 **diverse** poses (vary all rotations, keep the
   marker in view); press Enter at each to record a sample. Samples are
   saved to `handeye_samples.yaml` as you go.
3. Press `s` to solve. The tool runs four solvers (Tsai, Park, Horaud,
   Daniilidis), reports each one's consistency residual, picks the best, and
   prints a ready-to-paste `static_transform_publisher` command. Residual
   should be a few mm and well under 1°. Re-solve offline any time with
   `ros2 run roscam handeye_calib --solve handeye_samples.yaml`.
4. Publish the printed static TF (put it in your bringup):

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <x> --y <y> --z <z> --qx <qx> --qy <qy> --qz <qz> --qw <qw> \
  --frame-id <tcp_frame> --child-frame-id <camera_link_or_optical_frame>
```

> If you publish tcp→`camera_link` (RealSense), the driver provides
> `camera_link → camera_color_optical_frame`. For other cameras publish the
> transform directly to the frame that appears in the image `header.frame_id`.

**Verification (do not skip):** with the marker in view, run
`ros2 run tf2_ros tf2_echo <planning_frame> <marker check>` — practically:
watch the controller's logged alignment error while jogging the robot around
the stationary marker. The computed goal must stay put (≲2 mm, ≲0.5°) as the
viewpoint changes. If the goal "swims" with robot motion, the hand-eye
transform is wrong.

### 2.4 Teach the connector offset

The controller aims at a point defined **in the marker frame**
(`connector_offset_x/y/z`, marker X/Y in-plane, Z out of the surface), with
tool yaw `tool_yaw_offset_deg`. Defaults are all zero = marker centre.

**Automatic (recommended, needs the connector CAD STL):** if you run the
`connector_pose` ICP node (§6), the offsets can be taught in one shot
instead of by caliper iteration — ICP already knows where the connector is
relative to the marker.

1. In the params file set `enable_insertion: false`, launch the full
   system (§4), let the robot hover at standoff with the marker **and**
   connector in view.
2. Run `connector_pose` with your STL and, critically,
   `-p fallback_to_prior:=false` (a fallback pose *is* the rough prior, so
   teaching from it would echo your guess back).
3. `ros2 run roscam teach_offsets` — it pairs `/aruco/pose_raw` with
   `/connector/pose`, averages ~100 samples, and prints a paste-ready
   block of `connector_offset_x/y/z` + `tool_yaw_offset_deg` (and the
   matching `marker_t_connector_xyz` for `connector_pose` itself). It warns
   if the sample spread is large (unstable ICP → fix the depth data first).
4. Paste into the params file; still set `insertion_depth_m` per step 4
   below (depth is a stroke length, not an offset).

**Manual (uses the built-in calibration mode, no STL needed):**

1. In the params file set `enable_insertion: false` and a generous
   `standoff_height_m` (e.g. `0.10`).
2. Launch the full system (§4). The robot aligns and **holds at standoff**,
   logging `Aligned (x.x mm, x.xx deg) - insertion disabled`.
3. Measure the lateral (X/Y) and yaw offset between the tool axis and the
   actual connector; update `connector_offset_x/y`, `tool_yaw_offset_deg`.
   Repeat until the tool hovers dead-centre over the connector.
4. Measure the vertical gap from TCP to the mate point; set
   `insertion_depth_m = standoff_height_m − gap_at_full_mate` — i.e. the
   stroke length is *(standoff) − (TCP height above mate point when fully
   mated)*. Add `connector_offset_z` if the mate plane is above/below the
   marker plane.
5. Re-enable `enable_insertion: true`. First mating attempts: keep
   `insert_speed` at `0.03` or lower.

---

## 3. Porting to a different robot (checklist)

1. **Bringup**: launch your robot's driver + MoveIt (`move_group`) instead of
   the MELFA ones. No code changes.
2. **Params file**: copy `melfa_rv5as_masterclass/config/rv5as_params.yaml`
   to `<myrobot>_params.yaml` and set:
   - `planning_group` — your MoveIt group name
   - `EEF_FRAME_ID` — your TCP link (empty = MoveIt group default)
   - `planning_pipeline` / `planner_id` — see §6
3. **Hand-eye TF** for your camera mount (§2.3).
4. **Launch**:

```bash
ros2 launch melfa_rv5as_masterclass move_l.launch.py \
  robot_name:=<name> \
  moveit_config_package:=<name>_moveit_config \
  params_file:=/abs/path/<myrobot>_params.yaml
```

5. First run with your driver's fake/mock hardware mode and
   `enable_insertion: false`; watch phases in RViz.

The vision side is untouched by a robot swap. A camera swap only changes the
two topic parameters and the hand-eye TF.

---

## 4. Launch sequence (MELFA RV-5AS reference)

One command per terminal, in order (see `melfa_ros2_bringup.txt`):

```bash
# 1. Robot driver (set use_fake_hardware:=true for a dry run)
ros2 launch melfa_bringup rv5as_control.launch.py use_fake_hardware:=false controller_type:="D" robot_ip:=192.168.0.20

# 2. MoveIt
ros2 launch melfa_rv5as_moveit_config rv5as_moveit.launch.py

# 3. Camera driver (publishes image + camera_info + its internal TF)
ros2 launch realsense2_camera rs_d405_pointcloud_launch.py

# 4. Hand-eye static TF — YOUR calibrated values from §2.3
ros2 run tf2_ros static_transform_publisher --x ... --frame-id rv5as_default_tcp --child-frame-id camera_link

# 5. Vision
ros2 run roscam cam_pub

# 6. Controller
ros2 launch melfa_rv5as_masterclass move_l.launch.py
```

**Shortcut — terminals 4–6 in one:** the consolidated cell launch starts
the hand-eye TF, vision, and controller together (the robot side, 1–2,
stays separate):

```bash
ros2 launch melfa_rv5as_masterclass cell.launch.py \
  handeye_xyz:="<x> <y> <z>" handeye_quat:="<qx> <qy> <qz> <qw>"
# in-process capture (no image topics) + fixed-frame KF + depth-ICP:
ros2 launch melfa_rv5as_masterclass cell.launch.py \
  vision_source:=realsense filter_frame:=rv5as_base \
  template_stl:=/path/connector.stl
```

`handeye_xyz`/`handeye_quat` are **required** (no default — a guessed TF is
the dominant roll/pitch error source); pass the `handeye_calib` output.
Setting `template_stl` also brings up `connector_pose` — point the
controller at it with `pose_topic: /connector/pose`.

### Pre-flight checks

```bash
ros2 topic hz /aruco/pose                      # ~camera rate when marker visible
ros2 run tf2_ros tf2_echo rv5as_base camera_color_optical_frame   # full TF chain resolves
```

Watch the controller log: it prints the phase and the live 6-DOF error
(`[ALIGN_FINE] err: 3.2 mm, 0.8 deg`) every cycle.

### First-run validation (new cell or after recalibration)

1. Fake hardware + `enable_insertion: false` → phases advance, motion
   directions correct in RViz, alignment converges.
2. Real hardware + `enable_insertion: false` → hovers centred over the
   connector at standoff; occlude the marker with your hand → robot holds and
   logs `Marker not visible / stale`; uncover → resumes.
3. Real hardware, insertion enabled, reduced `insert_speed` → full mate;
   verify the log latches `MATED` and nothing moves afterwards.

### Monitoring & GUI

The controller publishes machine-readable state: `/mating/phase` (latched
String, always the real state-machine phase), `/mating/paused` (latched
Bool — the operational pause flag is separate from the phase),
`/mating/error_mm`, `/mating/error_deg`, and `/diagnostics`.
Three ready-made front-ends live in [tools/gui/](tools/gui/README.md):
a Foxglove Studio layout (engineering), a zero-install tkinter operator
panel with YAML-configurable buttons/tolerances (demos/teaching), and an
rqt recipe. `enable_insertion` is now read every cycle, so toggling it
from a GUI or `ros2 param set` takes effect immediately.

### Operator reset & retract

`MATED` and `FAULT` are latched — the controller commands no motion until
reset. After removing the mated connector / clearing the fault:

```bash
ros2 service call /connector_mating_node/reset std_srvs/srv/Trigger
```

The sequence restarts at `WAIT_FOR_VISION`.

**If the FAULT interrupted an insertion stroke** (STOP pressed or planning
failed mid-INSERT), the tool may still be partially engaged, so `reset` is
**refused**: restarting the sequence would command lateral alignment motion
while inside the connector. Pull straight back first:

```bash
ros2 service call /connector_mating_node/retract std_srvs/srv/Trigger
```

Retract re-traces the stroke along the tool axis back to the recorded
standoff pose (orientation locked, `insert_speed`), clears the latch, and
restarts at `WAIT_FOR_VISION`. It is only accepted in `FAULT`. Both GUIs
have a Retract button. Pausing mid-INSERT is also safe: the resumed stroke
covers only the remaining depth.

---

## 5. Parameter reference (`config/rv5as_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `planning_group` | `rv5as` | MoveIt group |
| `EEF_FRAME_ID` | `rv5as_default_tcp` | TCP link used for servoing |
| `pose_topic` | `/aruco/pose` | Marker pose input (filtered) |
| `raw_pose_topic` | `/aruco/pose_raw` | Unpredicted detections; insertion arming requires one this fresh (predictions steer but never arm the stroke) |
| `control_period_s` | 0.4 | State-machine cycle time |
| `vision_timeout_s` | 0.6 | Pose older than this = stale → hold. Staleness uses the message header stamp (arrival time only for zero/skewed stamps) |
| `tf_lookup_timeout_s` | 0.1 | Wait for TF at the image timestamp (correct for a moving camera) before falling back to the latest transform |
| `planning_pipeline` / `planner_id` | pilz / `LIN` | Motion backend (§6) |
| `accel_scaling` | 0.1 | Global acceleration scaling |
| `enable_insertion` | true | false = calibration mode, stop at standoff |
| `connector_offset_x/y/z` | 0 | Mate point in the **marker frame** (§2.4) |
| `tool_yaw_offset_deg` | 0 | Extra tool yaw about the marker normal |
| `standoff_height_m` | 0.08 | Alignment hover height above mate point |
| `insertion_depth_m` | 0.06 | Committed stroke length from standoff |
| `coarse_max_step_m` / `_deg` | 0.05 / 5.0 | Per-cycle clamps, fast approach |
| `coarse_speed` | 0.25 | Velocity scaling, fast approach |
| `coarse_pos_tol_m` / `_rot_tol_deg` | 0.02 / 5.0 | Switch to fine alignment |
| `fine_max_step_m` / `_deg` | 0.01 / 1.5 | Per-cycle clamps, fine alignment |
| `fine_speed` | 0.08 | Velocity scaling, fine alignment |
| `fine_pos_tol_m` / `_rot_tol_deg` | 0.0025 / 1.0 | Insertion arming tolerance |
| `align_hold_cycles` | 3 | Consecutive in-tolerance cycles to arm insertion |
| `insert_speed` | 0.03 | Velocity scaling, insertion stroke |
| `max_joint_jump_rad` | 0.8 | Reject plans with joint jumps (IK-flip guard) |
| `max_consecutive_plan_failures` | 5 | Then FAULT (hold) |
| `wrench_topic` | `''` | WrenchStamped source → force-aware insertion. `''` = blind stroke (default). Axial reaction ≥ `contact_force_n` = contact: **MATED** if ≥ `min_contact_depth_m` travelled, jam → FAULT (retract) if earlier; lateral ≥ `max_lateral_force_n` = snag → FAULT. FR3: the estimated external wrench topic works out of the box |
| `contact_force_n` / `max_lateral_force_n` | 8 / 12 | Contact / snag thresholds (N); tune per connector |
| `min_contact_depth_m` | 0 | Contact earlier than this = obstruction, not seating |
| `insert_planner` | `pipeline` | `cartesian` = `computeCartesianPath` stroke (Pilz-less robots, §6) |
| `align_mode` | `step` | `servo` = ALIGN publishes twists for moveit_servo (continuous tracking); INSERT stays a discrete stroke. Velocity cap = step clamps ÷ `control_period_s` |

Vision node (`roscam cam_pub`): `image_topic`, `camera_info_topic`,
`marker_id` (11), `marker_size_m` (0.021), `aruco_dictionary`
(`DICT_6X6_250`), `board_markers_x`/`board_markers_y` (1×1 = single marker;
set >1 for a grid **board** of ids `marker_id..marker_id+N−1` —
occlusion-robust and more accurate, the published pose is the board centre
so taught offsets are unchanged; print it with `cv2.aruco.GridBoard`),
`board_marker_separation_m` (0.005), `max_reprojection_error_px` (2.0),
`publish_debug_image` (true). Kalman filter: `sigma_accel` (0.08 m/s²),
`sigma_rot_rate_deg` (15), `meas_std_pos` (0.002 m), `meas_std_rot_deg`
(1.0), `gate_sigma` (3.0), `max_prediction_s` (0.3 — how long predicted
poses bridge a detection dropout before the node goes silent and the
controller holds), `rejects_before_reacquire` (5), `filter_frame` ('' —
recommended: set to the robot base frame, e.g. `rv5as_base`. The optical
frame moves with the arm, so robot steps look like marker motion to the
filter and can trip its gate; with `filter_frame` set, detections are
re-expressed via TF at the image stamp and filtered where the marker is
truly static. `/aruco/pose` is then published in that frame — the
controller handles any frame. `/aruco/pose_raw` always stays optical for
the hand-eye tool. Requires the TF chain, so leave it '' during the
initial hand-eye calibration itself).

Timing budget: predicted poses can bridge `max_prediction_s`, and the
controller tolerates `vision_timeout_s` on top, so keep
`control_period_s × align_hold_cycles` comfortably larger than
`vision_timeout_s` (the controller warns at startup otherwise). Arming is
additionally gated on `raw_pose_topic`, so a prediction can never commit
the insertion stroke.

**Frame source (`source` parameter, cam_pub and connector_pose):**
`topic` (default — images from a camera driver via ROS topics, the wiring
described above), `realsense` (capture **in-process** via pyrealsense2:
no image/depth ever enters the DDS graph, only poses — the bandwidth fix
for robots with a 1 kHz network control loop, e.g. FR3; needs
`pip install pyrealsense2`; intrinsics come from the SDK, frames are
stamped node-time − `capture_latency_s`, debug image throttled to
`debug_max_hz`), or `external` (embedder-fed). Only ONE process may own
the camera — when both the marker pipeline and ICP refinement are needed
out-of-ROS, run `ros2 run roscam vision_standalone`, which owns the camera
once and runs both on the same frames (marker-only without `template_stl`;
all cam_pub/connector_pose parameters apply). In `realsense`/standalone
mode, skip the camera-driver terminal entirely.

---

## 6. Optional: connector-level pose refinement (`connector_pose`)

The marker is a proxy — `connector_pose` refines the actual connector's
6-DOF pose by registering a point sample of its CAD model against the depth
cloud (point-to-plane ICP), using the marker pose as the prior. Setup:

1. **Export the connector CAD as STL** (binary or ASCII), modelled so the
   **origin is at the mate point and Z points out of the work surface** —
   then the controller's `connector_offset_*` stay zero.
2. Enable RealSense aligned depth (`align_depth.enable:=true`) so
   `/camera/camera/aligned_depth_to_color/image_raw` exists.
3. Run: `ros2 run roscam connector_pose --ros-args -p template_stl:=/path/connector.stl \
   -p marker_t_connector_xyz:="[x, y, z]"` (rough connector position in the
   marker frame — teach once).
4. Point the controller at it: `pose_topic: /connector/pose` in the params
   file. No other controller change.

Behaviour: ICP output is quality-gated (`min_inlier_fraction` 0.6,
`max_rms_m` 0.004, `max_refine_translation_m` 0.02); on gate failure the
node publishes the marker-derived prior — never worse than marker-only —
and logs why. Accepted poses run through a low-noise Kalman filter (the
connector is static), which averages depth noise down over ~2–3 s.

Measured on synthetic data (worst-case 20 mm part, 0.3 mm depth noise):
XY/Z ≈ **0.1 mm**, yaw ≈ **0.1°**, roll/pitch ≈ **0.7°** (edge-noise
limited; scales with depth-noise ÷ part size, so larger connectors do
proportionally better). Real-D405 tuning pass still required.

## 7. Planner note (robot-agnostic caveat)

ALIGN and INSERT assume the TCP moves in a **straight line** toward each
commanded pose — INSERT especially, since it is the mating stroke.

- **Pilz `LIN`** (default): guaranteed linear TCP path + orientation slerp.
  Available on many industrial drivers (MELFA, UR, Kuka, Fanuc MoveIt
  configs).
- **No Pilz?** Set `planning_pipeline: ompl` (empty `planner_id`) for the
  alignment steps — they converge fine (small steps are near-linear) — and
  set `insert_planner: cartesian` so the **insertion stroke** uses
  `computeCartesianPath`: an interpolated straight Cartesian path,
  time-parameterized at `insert_speed`. This is the path-guaranteed
  Pilz-less configuration (used by the FR3 profile in
  `tools/fr3/fr3_params.yaml`). Still validate in fake hardware first.

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `WAIT_FOR_VISION` forever | No `/aruco/pose`: check camera topics, marker visibility, `marker_id`/dictionary; view `/aruco/debug_image` |
| `TF ... unavailable` warnings | Hand-eye static TF not running, or wrong frame names — check §4 pre-flight |
| Goal position drifts when the robot moves (marker still) | Bad hand-eye calibration (§2.3 verification) |
| Alignment oscillates near tolerance | Lower `filter_alpha` (more smoothing), lower `fine_max_step_m`, or relax `fine_pos_tol_m` slightly |
| Axes on `/aruco/debug_image` flicker/flip | Marker too small/far, glossy print, or bad focus — improve the marker before touching software |
| `Rejecting plan: joint N jumps ...` | Pose near a wrist singularity or IK branch flip — adjust approach yaw (`tool_yaw_offset_deg`) or cell layout so alignment happens away from singular configurations |
| Repeated `LIN planning failed` → FAULT | Target outside reach or Pilz velocity limits missing — check `pilz_cartesian_limits.yaml` in the MoveIt config |
| Tool centred but insertion misses | Re-teach `connector_offset_*` (§2.4); check `insertion_depth_m` arithmetic |
