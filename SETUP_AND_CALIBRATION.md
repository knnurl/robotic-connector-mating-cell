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

1. Fix a marker/board in the workspace where it stays put.
2. Move the robot to 10–15 diverse poses (vary all rotations, keep the
   marker in view). At each pose record:
   - `T_base→tcp` — e.g. `ros2 run tf2_ros tf2_echo <base> <tcp>`
   - `T_cam→marker` — echo `/aruco/pose_raw`
3. Solve with `cv2.calibrateHandEye` (eye-in-hand form), or use the MoveIt
   Hand-Eye Calibration GUI which automates steps 2–3.
4. Publish the result as a static TF (put it in your bringup):

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

Teach procedure (uses the built-in calibration mode):

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

---

## 5. Parameter reference (`config/rv5as_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `planning_group` | `rv5as` | MoveIt group |
| `EEF_FRAME_ID` | `rv5as_default_tcp` | TCP link used for servoing |
| `pose_topic` | `/aruco/pose` | Marker pose input |
| `control_period_s` | 0.4 | State-machine cycle time |
| `vision_timeout_s` | 0.6 | Pose older than this = stale → hold |
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

Vision node (`roscam cam_pub`): `image_topic`, `camera_info_topic`,
`marker_id` (11), `marker_size_m` (0.021), `aruco_dictionary`
(`DICT_6X6_250`), `filter_alpha` (0.35), `max_reprojection_error_px` (2.0),
`max_translation_jump_m` (0.05), `publish_debug_image` (true).

---

## 6. Planner note (robot-agnostic caveat)

ALIGN and INSERT assume the TCP moves in a **straight line** toward each
commanded pose — INSERT especially, since it is the mating stroke.

- **Pilz `LIN`** (default): guaranteed linear TCP path + orientation slerp.
  Available on many industrial drivers (MELFA, UR, Kuka, Fanuc MoveIt
  configs).
- **No Pilz?** Set `planning_pipeline: ompl` and leave `planner_id` empty.
  Alignment still converges (steps are small, so paths are near-linear), but
  the insertion stroke is not path-guaranteed — reduce `fine_pos_tol_m`,
  `insertion_depth_m`, and speeds, and validate in fake hardware first. A
  `computeCartesianPath` fallback is the proper fix if you need a
  Pilz-less deployment; ask for it.

---

## 7. Troubleshooting

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
