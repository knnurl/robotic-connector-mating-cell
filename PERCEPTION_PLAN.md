# Marker-free object pose for the FR3 cell: plan

> **Status: plan, 2026-09-25 - not implemented.** Written by a 4-agent research workflow (a codebase reader, a state-of-the-art researcher, a planner that drafted three approaches and synthesised them, and a completeness critic that checked every cited file, topic and parameter against the code with graphify). Research inputs: [PERCEPTION_RESEARCH.md](PERCEPTION_RESEARCH.md). How the approach was chosen, and the critic's corrections, are in the appendices.

Repo: `/home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src`. Checked:
- TRACK reads `pose_topic` and `tracking_raw_pose_topic` (`tracking_node.cpp:382-383`).
- GRIP reads `tracking_raw_pose_topic` (`grip_node.py:71,160`) and `grip_target_topic`.
- All three keys live in `fr3_params.yaml` under `/**` (20, 148, 208), together with `raw_pose_topic` (21, parked mating_node).
- The panel hardcodes its pose topics (`ros_node.py:63,174`), and its bag recorder hardcodes its topic list (`actions.py:79-82`).
- `vision_standalone.py` calls `ArucoPosePublisher.process_frame(frame.bgr, …)`. It has no per-frame exception guard. It runs ICP only when `template_stl` is set.
- `process_frame` draws its overlay onto the image it is given, and publishes the debug image before returning.
- `icp.icp` returns `(T, rms, inlier_frac)`.
- `/grip_node/status` carries `holding`.
- Docker 29.2.1 is installed; the NVIDIA container toolkit is not.
- E-cores are 12-19, LP-E cores are 20-21; there is no isolcpus.
- Logs go to `$FR3_LOG_DIR/YYYY-MM-DD/`.

## 1. Goal, non-goals, acceptance criteria

**Goal.** Estimate the part's 6-DoF pose from the D405 depth and colour already captured in-process. Publish it on a contract that does not depend on the source, so that TRACK, GRIP, PLACE AT B and ALIGN use it with parameter changes only. Validate each step against the ArUco marker, which stays on the part until Phase 6, and against references independent of the marker.

**Non-goals:**
- Choosing the connector.
- Methods trained per object.
- Clutter or bin picking.
- Autonomous motion: every motion stays behind a panel button.
- A marker-free B.
- Fixing hand-eye (its 8.14 mm scatter stays the floor for base-frame consumers).
- Retuning the tracking law.

**References and metrics.** Each condition uses at least 300 frames.
- **`T_marker_object`**: measured with calipers and a square (the sticker's offset and rotation from the cube's edges), not taught from depth data.
- **Bias**: the mean of `(marker ∘ T_marker_object)⁻¹ · depth` per axis (x, y, z, tilt, in-plane), reported against distance to expose depth-scale bias.
- **Independent checks** that use neither the marker nor hand-eye: object Z parallel to the depth-fitted table normal, and the origin 55 mm (from calipers) above the table plane, both in the same frame.
- **σ**: with arm and part still.
- **Availability**: the fraction of in-view frames with a raw pose.
- **Baseline**: the Phase 1 marker measurements of the same quantities.

| Consumer | Criteria |
|---|---|
| **TRACK** (camera-centred, 100 mm, top face only) | Raw rate ≥ 12 Hz; no raw gap > 0.20 s in a 60 s static run (the gate is 0.25 s); availability ≥ 95%. Filtered `/object/pose` σ ≤ 0.8 mm. Angular σ per axis ≤ 1.5× the marker baseline. Integrator hunting (BuzzMeter, lead activity) no worse than the baseline. Bias ≤ 2 mm, tilt ≤ 1°, in-plane ≤ 0.5°. Frame-to-publish p95 ≤ 40 ms including the TF wait; stamp error ≤ 10 ms (measured). V1-V4 pass. |
| **ALIGN** (panel, camera frame) | Converges to the defaults of 2 mm / 1° / 0.5° in-plane from the baseline start poses, in ≤ 1.5× the marker's time. Filtered in-plane σ ≤ 0.17° (3σ inside 0.5°). |
| **GRIP** (base frame, read once) | Base-frame bias ≤ 2 mm per axis. 5-sample scatter ≤ 2 mm (the gate is 5 mm). Yaw mod 90° ≤ 2°, tilt ≤ 2°. At least 3 raw poses in 0.5 s on ≥ 98% of presses. 19/20 physical grips at random positions and yaws. The estimator adds ≤ 2 mm on top of hand-eye. |
| **PLACE AT B** | B unchanged (marker). The placed cube lands within 3 mm / 3° of B plus its offset, judged by both estimator and marker, and within 1 mm of the marker-GRIP baseline. |
| **Insertion** (provisional) | In the **camera frame** at the camera-centred 100-150 mm standoff: lateral bias ≤ 1 mm, σ ≤ 0.3 mm; keyway yaw ≤ 1°, with the 180° ambiguity resolved once; tilt ≤ 1°; ≥ 10 Hz; availability ≥ 90% on the approach. Base-frame error is bounded by hand-eye and out of scope. |
| **Cell** | Vision process ≤ 1 core on average. In the same session, estimator on versus off: V6 RT success ≥ 0.99, % of time below 97% RT success (the C10 metric) no worse, and no added comm reflexes. |

## 2. The pose contract

The vision process that owns the camera (`vision_standalone`) publishes:

| Topic | Type / frame | Meaning |
|---|---|---|
| `/object/pose_raw` | PoseStamped, `camera_color_optical_frame`, image stamp | A measurement that passed every gate of the current mode, **including the object KF's innovation gate** (as `/aruco/pose_raw` does). Never a prediction, prior or fallback. No message means no valid pose. |
| `/object/pose` | PoseStamped, fr3_link0, KF | Filtered; may coast ≤ 0.3 s. The goal for TRACK and ALIGN. |
| `/object/pose_quality` | `diagnostic_msgs/DiagnosticArray`, stamp = image stamp, every processed frame | Level plus reason. Keys: `source`, `valid`, `rms_mm`, `inlier_frac`, `n_pts`, `weak_dof`, `agree_mm`, `agree_deg`, `sym_index`, `compute_ms`, `tf_wait_ms`, `seeded_from`, `holding`, and `cand_pose` (the depth candidate, optical frame, 7 numbers, **always filled when computed**, so shadow-mode bags carry per-axis data). |
| `/aruco/*` | unchanged | Hand-eye calibration, teach_offsets, B, ground truth. |

**Object frame:**
- The origin is the top-face centre for the cube, and the mate point for a connector.
- Z points outward, toward the camera.
- In-plane X is the symmetry member nearest the previous estimate; at seeding, the member nearest the marker's X composed with `T_marker_object`.
- In marker mode, `T_marker_object` is applied from Phase 3 on (identity in Phase 1 for the bit-identical check), so switching sources never moves the frame.
- The cube resting on a face has C4 symmetry; a connector has C1 or C2.

**Repointing, once, in Phase 1:**
- fr3_params.yaml: `pose_topic` (20) → `/object/pose`; `raw_pose_topic` (21) and `tracking_raw_pose_topic` (148) → `/object/pose_raw`.
- `ros_node.py:63,174` use new constants in `core.py`.
- The `actions.py` `Recorder.TOPICS` list gains `/object/*`, `/aruco/target_pose_raw` and `/grip_node/status`.

**Source selection:**
- The source is chosen by `object_source` on the vision process. The panel sets it only while TRACK and GRIP are idle.
- A change resets the object KF and re-runs the acquisition hysteresis, so a switch always makes a raw gap of at least 0.33 s. Consumers then hold under Decision 5, even if someone changes the parameter from outside the panel.

**Predictions never drive committed motion:**
- The prior and the initialiser only seed ICP.
- Raw poses need 5 agreeing frames after any acquisition.
- Consumers keep gating on raw arrival.
- Neither the C++ nor `grip_node` changes.

**Held part.** While `/grip_node/status` reports `holding=true`, no raw pose is published (level WARN, reason `HELD`). Acquisition restarts after release.

## 3. Phases

### Phase 0: adopt the camera owner, measure the sensor, record ground truth (no estimator yet)

**What is built:**
- **Launch.** `fr3_cell.launch.py` `_vision()` gains the value `vision_source:=standalone`, which runs `vision_standalone`, marker-only, with `/aruco/*` unchanged. `realsense` stays the rollback. New launch arguments `capture_width` and `capture_height`.
- **`vision_standalone.py`:**
  - A per-frame `try/except`, as in cam_pub's `_capture_loop`.
  - It passes `process_frame` a copy it can draw on, and keeps `frame.bgr` untouched for recording and for the estimator later.
- **`cam_pub.py`: one additive hook,** `self.on_publish = None`, called from `_publish` with (publisher, header, t, q). It is the only cam_pub change in the plan.
- **`rs_capture.Frame` gains `t_hw`** (`frame.get_timestamp()` and its timestamp domain).
- **NEW `roscam/roscam/frame_recorder.py`,** called from the vision_standalone loop. Per frame it writes:
  - colour as lossless PNG;
  - depth as uint16 PNG, plus the depth scale;
  - intrinsics and distortion as JSON;
  - `t_host`, `t_hw` and the image stamp;
  - the marker raw and target poses (via `on_publish`).

  A writer thread with a queue of 30 counts drops. It is toggled by the `record_dir` parameter, which the panel's REC button (`actions.Recorder`) sets, so frames and the bag share `$FR3_LOG_DIR/<date>/vision/<session>/`.
- **NEW `tools/fr3/vision/depth_quality.py`,** comparing:
  - top-face fill (marker quad masked ×1.6, the same ring cam_pub uses), plane rms, flying pixels at the silhouette;
  - at 80, 100, 150, 200 and 300 mm;
  - for 640×480 and 848×480;
  - with presets Default versus High Accuracy, spatial post-filter off versus on (no temporal filter: it adds lag), and auto versus locked exposure.
- **NEW `tools/fr3/vision/latency_fit.py`:** the operator jogs the arm past a static marker from the panel. The base-frame marker displacement is regressed against EE velocity derived from `/tf` and `/joint_states` in the bag (ROS stamps). The slope is the capture latency. The state_recorder CSV is used only for RT success.
- **Caliper measurements** of the marker id 0 and id 1 sizes, and of the sticker's offset and rotation on the cube (→ `T_marker_object`).

**Key decisions:**
- Record inside the process that owns the camera, in its loop, where the `Frame` exists. Rejected: image topics (break "images off DDS"); a hook in `process_frame` (it never sees the Frame's timestamps).
- Adopt `vision_standalone` now, marker-only. Rejected: duplicating the recorder in cam_pub's `_capture_loop`.
- Lossless PNG. Rejected: JPEG (edge artefacts).
- Join TF offline from the bag. Rejected: the state_recorder CSV (robot clock only).
- Calipers for marker size and placement. Rejected: inferring them from depth (circular, and confounded with the D405's ±1.4% depth bias).

**Verification:**
- `vision_source:=standalone` versus `realsense`, a 60 s static run each: `/aruco/pose_raw` rate, σ and reprojection statistics equal within noise.
- Recorder: 0 drops at 15 fps, monotonic stamps, recorded marker poses equal to the published ones.
- With caliper `marker_size_m`, `tvec.z` versus depth at the marker centre gives the **depth-scale bias** against distance.

**Exit criteria:**
- Recordings covering:
  - static at 80, 100, 200 and 300 mm;
  - an oblique view with 2-3 faces;
  - TRACK-like motion;
  - the GRIP viewpoint, fingers included;
  - **a bare cube with no marker**;
  - a hand touching and moving the cube;
  - cube and B plate in view together;
  - junk frames;
  - a lighting change.
- `capture_latency_s` with a confidence interval, and the TF-lookup wait p95.
- A depth-quality table and a recorded decision on resolution, preset, filter and exposure.
- If the resolution changes: a hand-eye static-point scatter re-check within 1 mm of 8.14 mm.

**Risks:**
- Holes on the bare top face. If fill is below 50%, Phase 2 leans on edges and the Phase 7 gate opens early.
- PNG CPU cost. The fallback is to record every 2nd frame.
- `vision_standalone` is new on the arm. The rollback is `vision_source:=realsense`.

### Phase 1: the contract, with the marker as the only source (plumbing only)

**What is built:**
- **`vision_standalone` publishes `/object/*`.** With `object_source: marker`, the `on_publish` hook mirrors every raw, filtered **and predicted** `/aruco/pose` publish unchanged. It also publishes `/object/pose_quality`.
- **Parameters.** fr3_params.yaml lines 20, 21 and 148 are repointed as in §2.
- **Panel:**
  - `core.py` gains `POSE_TOPIC` and `RAW_POSE_TOPIC`, used at `ros_node.py:63,174`.
  - The logic.py chip becomes `VISION <ms> · MARKER`, and the banners become NO POSE and POSE STALE.
  - A `view.py` source dropdown, enabled only when the cell is idle.
  - `actions.Recorder.TOPICS` is extended.
- **Mocks.** `mock_cell.py` and `tools/fr3/sim/tracking_smoke.py` publish `/object/*` and quality.
- **Placement.** The vision process runs under `taskset -c 12-19` with thread pools set to 1 (see §4), from now on, so every baseline shares the same placement.
- **Docs.** The contract and the marker-free goal are written into the repo docs (the `tools/fr3/README.md` vision section and TRACKING_SPEC's pose-input section).
- **Separate, explicitly flagged change:** ALIGN gates on `raw_age()` (≤ 0.25 s), with its own test. It is the one intended behaviour change in this phase.
- **Unchanged:** `tracking_node.cpp`, `grip_node.py` and `connector_pose.py`.

**Key decisions:**
- PoseStamped plus a DiagnosticArray. Rejected: a custom message; PoseWithCovarianceStamped.
- Choose the source in the vision process. Rejected: remapping consumers; a mux node.
- One launch value `vision_source:=standalone`. Rejected: an orthogonal `vision_exec` argument, which allows standalone plus topic.

**Verification:**
- Mock plus `tracking_smoke.py`.
- The `test_cell_*` tests: chip, banners, idle-only dropdown, ALIGN raw gate.
- A new pin test: the fr3_params topic names equal the vision node's publisher names.
- On the cell, a 60 s bag shows `/object/pose_raw` bit-identical to `/aruco/pose_raw`.
- The operator runs TRACK V1-V3, ALIGN from 3 start poses, 5 GRIPs and 3 PLACE AT B.

**Exit criteria:**
- Identical poses and behaviour apart from the ALIGN raw gate.
- `MARKER` shown on the panel.
- **The baseline table recorded:** marker raw and filtered σ per axis, availability, hunting, ALIGN time, GRIP and PLACE results, and the RT metrics with the vision process pinned.

**Risks:** stale raw gating may make ALIGN refuse more often under marker flicker. That is measured here.

### Phase 2: the depth estimator, offline only, scored against the references (no robot)

**What is built:**
- **NEW `roscam/roscam/object_pose.py`,** ROS-free: `ObjectPoseEstimator.process(depth, bgr_clean, K, dist, prior) -> (T, valid, quality)`. Its steps:
  1. Undistorted back-projection of an ROI around the projected prior, using the SDK distortion as ArUco does.
  2. Remove the table with `fit_plane_robust` on the ring around the footprint, dropping points within 3 mm of the plane.
  3. Keep the connected cluster nearest the prior; size gate from the part file.
  4. Scene-to-model point-to-plane `icp.icp`.
  5. Gates on object points: inliers ≥ 0.8, rms gate calibrated from replay (start at 1.5 mm, point-to-point to 2 mm voxels), shift ≤ 10 mm, ≥ 200 points.
  6. `weak_dof` from the eigenvalues of the 6×6 normal matrix.
  7. Symmetry snap.
- **`icp.py` gains an optional `return_normal_matrix=False`,** additive only; the default behaviour and `test_icp.py` are unchanged.
- **NEW `tools/fr3/parts/cube55.yaml`:** box or STL, origin convention, C4 about Z, dimensions, and the caliper-measured `T_marker_object`.
- **NEW `tools/fr3/vision/replay_eval.py`:**
  - The marker prior is perturbed by ±5 mm / ±5° **and ±10-20 mm**, to map the edge of the convergence basin the V2 step will hit.
  - It prints bias (per axis and against distance), σ, availability, compute, and the independent table-parallelism and height checks.
  - It derives **per-axis KF measurement noise** for the depth source.
- **Phase 2b, only if 2a misses the TRACK or ALIGN in-plane budget (expected).** In-plane yaw and x/y come from a line fit to colour edges on the **clean** image, in a band around the projected edges of the depth-segmented top face. z and tilt come from depth, the same split `fuse_orientation` uses.

**Key decisions:**
- numpy ICP (about 14 ms). Rejected for now: Open3D, adopted only if p95 exceeds 30 ms on an E-core.
- Plane removal. Rejected: a tighter sphere crop (the research bench shows a 29 mm pull).
- Declared symmetry with a snap to the previous pose. Rejected: ADD-S-only scoring.
- Edge cue. Rejected: coloured ICP; per-frame FPFH or PPF.
- Undistorted back-projection. Rejected: pinhole back-projection (it biases the marker comparison off-centre).

**Verification:** `replay_eval` on the Phase 0 recordings, and NEW `roscam/test/test_object_pose.py`.

**Exit criteria:**
- The §1 TRACK, ALIGN and GRIP bias figures, and raw σ ≤ 1 mm / 0.3° before filtering, on masked-marker **and** bare-cube recordings (bare: fill, independent checks and σ only).
- ≥ 99% convergence from ±5 mm, and the basin edge recorded.
- 0 false accepts on junk frames and hand-touch frames.
- p95 compute ≤ 30 ms on one E-core.

**Risks:**
- Depth holes on the bare face.
- Low-contrast table edges. A matte, contrasting mat is the user's call.

### Phase 3: live shadow mode (the estimator runs, the marker still drives)

**What is built:**
- `object_pose` runs in the `vision_standalone` loop on `frame.bgr` (clean) after `process_frame`.
- **One TF lookup per frame** (image stamp → fr3_link0), shared by cam_pub's filter, the object KF and the prior.
- An object PoseKF in fr3_link0 with the Phase 2 per-axis noise.
- The prior is marker ∘ `T_marker_object` on every frame.
- `object_source: marker`, now with `T_marker_object` applied.
- Quality carries `agree_*` and `cand_pose`.
- The panel chip reads `MARKER · depth Δ1.1 mm 0.4°`.
- The overlay draws the **previous frame's** model outline onto the image passed to `process_frame`, so it rides the existing subscribe-gated debug image. There is no new topic.
- The estimator subscribes to `/grip_node/status` and suspends while `holding`.

**Key decisions:**
- Same process and thread. Rejected: a separate node (would need the images).
- A one-frame-stale overlay. Rejected: moving the debug publish out of `process_frame`.

**Verification:**
- The operator runs normal TRACK V1-V3, ALIGN, GRIP and PLACE sessions, with the bag (extended Recorder) and the 1 kHz recorder.
- An A/B in the same session with the estimator on and off, for the RT metrics.

**Exit criteria:**
- ≥ 20 min inside the §1 bias and σ budgets, computed per axis from `cand_pose`.
- V6 for 15 min with the estimator on meets the Cell row.
- `tf_wait_ms` p95 recorded.

**Risks:** CPU contention, yaw jitter, and TF waits stacking into latency. All are measured before motion depends on depth.

### Phase 4: `depth_checked` mode (depth drives, the marker can veto)

**What is built:**
- `object_source: depth_checked`: raw is published only when the depth pose is valid **and** agrees with marker ∘ `T_marker_object` from the same frame within 3 mm / 2°, modulo symmetry.
- The panel dropdown gains the option; the chip reads `DEPTH✓MARKER` with the veto rate.

**Key decision:** a per-frame veto. Rejected: blending the two sources.

**Verification:**
- On the arm: V1, V2 and V4 (the hand occludes; the arm holds).
- ALIGN from the baseline start poses.
- 10 GRIPs from random placements and 5 PLACE AT B.
- Compared against the Phase 1 baseline.

**Exit criteria:**
- V1 ≤ 5 mm / 0.5°.
- Hunting no worse than the baseline.
- ALIGN row met.
- GRIP 10/10, PLACE 5/5.
- Veto rate < 5%.

**Risks:** depth jitter makes the integrator hunt. The fix is the depth KF's measurement noise, not the gains.

### Phase 5: the marker is only a seed; the tracked prior carries the pose

**What is built:**
- The prior at frame t is the object KF's constant-velocity prediction in fr3_link0 at the image stamp, mapped into the optical frame through that frame's shared TF. This covers both object motion (V2, V3, hand-moved parts) and camera motion.
- On a shift-gate reject or a loss:
  1. Immediately run `acquire()` in an ROI widened around the last pose (the box fit, Phase 6 code).
  2. Fall back to a marker re-seed after 0.5 s.
- `object_source: depth`, `object_seed: marker`, `mask_marker_for_estimator: true`.
- The marker is logged in `agree_*` but no longer vetoes.
- Suspended while `holding`.

**Key decision:** the KF prediction in the base frame plus TF. Rejected: a static-part prior (breaks V2 and V3); a constant-velocity prior in the optical frame (mixes camera and object motion).

**Verification:**
- Replay with the marker withheld after frame 1, to measure drift.
- On the arm: V1-V4, including the 20 mm V2 step; 20 GRIPs; 10 PLACE AT B.

**Exit criteria:**
- 5 min of V1/V3 without a marker re-seed, inside budget.
- Drift ≤ 1 mm/min.
- The V2 step recovers through the local re-acquire, with a raw gap ≤ 0.5 s.
- Re-acquisition after V4.

**Risks:** slip along a weakly observed direction. The `weak_dof` flag withholds raw while the edge cue is missing.

### Phase 6: acquisition without the marker, then removing it

**What is built:** `ObjectPoseEstimator.acquire()`:
- Fit the table plane from depth, with the ROI seeded by the base-frame table height.
- Find clusters above it matching the part file within ±5 mm, excluding the held part (`holding`) and B's plate (by size).
- For boxes: top-face plane plus `cv2.minAreaRect`, then ICP, then the symmetry member nearest the camera's current in-plane target. Open3D FPFH+RANSAC is only for non-box templates.
- 5 consistent frames before the first raw pose.
- The panel shows `ACQUIRED` with the overlay.

**Key decisions:**
- Box fit for the cube. Rejected: FPFH or TEASER++ everywhere.
- No CONFIRM button for the cube. It is required for a connector (§6).

**Verification:**
- 50 random placements with the marker present but masked, plus the bare cube.
- 0 false acquisitions with 3 distractors, B's plate and a hand in view.
- Then **remove the marker**: 20 GRIPs and 10 PLACE AT B, judged physically, with the placed cube re-measured.

**Exit criteria:** acquisition ≥ 98%; 0 false acquisitions; GRIP ≥ 19/20 without the marker.

**Risks:** distractors of similar size. The size and rms gates catch them; otherwise the operator confirms.

### Phase 7 (starts only when the connector is chosen)

**What is built:** a part file (STL with its origin at the mate point, the symmetry and the key feature), and the Phase 0 depth-quality test on the real part at 80, 100, 150 and 200 mm.

**Decision rule:**
- If top-face fill ≥ 70% and plane rms ≤ 1 mm, reuse Phases 2-6 with the connector template. The operator resolves the keyway's 180° ambiguity once on the overlay; continuity holds it after that.
- If the part is black or shiny:
  - **Learned option:** FoundationPose tracking from the Apache-2.0 inference library in a container, fed over shared memory from `vision_standalone` (never over DDS), seeded by the Phase 6 mask and refined by ICP.
  - **No-GPU fallback:** ICG/M3T on the CPU, or keep a marker.

FoundationPose runs only once four things are shown on this host:
- the container toolkit is installed;
- FP16 VRAM ≤ 6 GB;
- tracking ≥ 15 Hz;
- a same-session A/B of V6 with GPU load meets the Cell row.

## 4. Compute budget (shared with the 1 kHz RT controller; images stay in-process)

| Item | Cost per frame at 15 Hz | Source |
|---|---|---|
| ArUco + IPPE + plane normal; `rs.align` | not measured; Phase 0 measures them | — |
| TF lookup at the image stamp (one, shared) | ≤ 50 ms blocking worst case; p95 measured in Phase 0 | cam_pub `_to_filter_frame` timeout |
| Clean-frame copy | about 0.3 ms | estimate |
| ROI back-projection, table plane, clustering | 3-13 ms | synthetic bench |
| numpy point-to-plane ICP, 30 iterations | 4-33 ms, about 14 ms typical (bench unpinned; E-cores at 3.8 GHz are slower, so re-measured in Phase 2) | bench |
| Edge cue (Phase 2b) | about 1-3 ms | estimate |
| Open3D ICP (only if needed) | 2.4 ms | bench |
| Acquisition (box fit or FPFH) | ≤ 110 ms, per acquisition or local re-acquire | bench |
| Recorder (sessions only) | 10-20 ms of PNG work on the writer thread; 10-18 MB/s to disk | estimate |

- **Per-frame budget:** p95 ≤ 45 ms including the TF wait, i.e. ≤ 0.7 core at 15 Hz. At 30 fps, run ICP on every frame only if p95 ≤ 25 ms; otherwise use Open3D.
- **Placement (from Phase 1):**
  - `taskset -c 12-19`, with `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` and `MKL_NUM_THREADS` set to 1, SCHED_OTHER and nice 5.
  - The controller is untouched.
  - The Python TF listener's own CPU at the `/tf` rate is measured in Phase 0.
- **DDS:** about 15 Hz × (2 × 250 B + 800 B) ≈ 20 KB/s. No images or clouds; the overlay stays on the subscribe-gated `/aruco/debug_image`.
- **GPU:** none in Phases 0-6. The driver already runs Xorg; what is unmeasured is GPU compute and memory load against the FCI loop, and only the Phase 7 gate may bring it in.
- **Memory:** under 500 MB.

## 5. Test strategy

**Unit tests (pytest, `roscam/test`):**
- NEW `test_object_pose.py`, synthetic scenes:
  - cube on a table at 80, 100 and 200 mm, with 0.1-0.5 mm noise, holes and flying pixels;
  - the table-pull regression (≤ 1 mm);
  - a hand-like blob touching the cube (rejected or unbiased);
  - `weak_dof` on a top-face-only view;
  - the C4 snap staying next to the previous pose;
  - junk rejection;
  - distractor acquisition;
  - distorted-intrinsics back-projection against the SDK model.
- `test_icp.py` unchanged, plus a case for the normal-matrix return.
- NEW `test_frame_recorder.py`: round trip and drop counting.
- A pure function for the `object_source` rules. Properties: raw is never published from a prior, prediction or fallback; a source switch forces a KF reset plus hysteresis (a raw gap); `holding` suspends raw.
- The quality stamp equals the pose stamp.
- A `vision_standalone` loop test: an exception on one frame does not stop the next.

**Cell tests (`tools/fr3/cell`):**
- The topic-name pin test: fr3_params and ros_node constants against the vision publisher names.
- `test_cell_*` for the chip, NO POSE / POSE STALE, the idle-only dropdown and the ALIGN raw gate.

**Recorded datasets:**
- Stored under `$FR3_LOG_DIR/<date>/vision/`, beside the bags, traces and state CSVs.
- `replay_eval.py` produces each phase's tables.
- About 30 frames per condition are committed as a regression fixture.
- Every on-arm session records the extended `actions.Recorder` bag (`/object/*`, quality, `/aruco/*`, `/grip_node/status`, TF, joint states), the tracking jsonl and the 1 kHz `state_recorder`.

**Mock and simulation:**
- `mock_cell.py` publishes `/object/*` and scripted quality (disagreement, veto, loss, held).
- `tracking_smoke.py` runs on `/object/*`.

**On the arm, operator in the loop:**
- TRACKING_SPEC V1-V6, ALIGN, and N GRIPs and N PLACE AT B per phase, every motion started by a button.
- The marker logs agreement until Phase 6.
- Each phase is compared with the Phase 1 baseline and with the same-session estimator-off A/B.

## 6. Open questions for the user (each with a recommended default)

1. **Marker sizes and placement.** Default: measure ids 0 and 1, and the id 0 sticker's offset and rotation on the cube, with calipers and a square before Phase 0. Set `marker_size_m` and `target_marker_size_m` in the launch.
2. **Replace cam_pub with `vision_standalone` from Phase 0?** Default: yes, as `vision_source:=standalone`, with `realsense` kept as the rollback.
3. **Capture settings (resolution, preset, post-filters, exposure)?** Default: decide from the Phase 0 table. Lean toward 848×480 at 15 fps if top-face σ improves ≥ 30%. No temporal filter. Locked exposure if auto-exposure shifts depth or edges.
4. **Oblique TRACK view?** Default: no; use the edge cue (Phase 2b) and revisit only if Phases 2 and 3 fail the TRACK or ALIGN in-plane rows.
5. **If depth yaw misses ALIGN's 0.5° in-plane tolerance, loosen it per source?** Default: no; require the edge cue first.
6. **CONFIRM button for marker-free acquisition?** Default: not for the cube; yes for the connector.
7. **B marker-free?** Default: keep marker B until the receptacle exists.
8. **Timestamps: SDK hardware stamps or a measured constant?** Default: the measured constant, unless Phase 0 shows jitter above 5 ms.
9. **A matte, contrasting mat?** Default: yes, if Phase 0 shows edge or depth problems at the table boundary.
10. **NVIDIA container toolkit on the RT host?** Default: not now; only if Phase 7's depth test fails, and only after a same-session A/B of GPU load against the FCI loop.
11. **Data volume?** Default: `$FR3_LOG_DIR/<date>/vision/`, about 1 GB per minute at 15 fps, with only small fixtures in git.

---

## Appendix A: how the approach was chosen

CLASSICAL: fit 8, verifiability 7, risk 7, concreteness 8, hardware honesty 9, total 39/50. The estimator is strong: table-plane removal, then scene-to-model point-to-plane ICP with `icp.py` on the CPU, seeded by the marker, then by arm kinematics, then by a depth cluster. But it switches TRACK and GRIP to depth in one step, and its only answer to weak yaw when the camera looks straight down is "prefer oblique views".

LEARNED: fit 4, verifiability 6, risk 3, concreteness 5, hardware honesty 6, total 24/50. This is FoundationPose from NVIDIA's Apache-2.0 inference library in a container, with a mask from the marker or a depth cluster, then ICP refinement. Five things count against it:
- The NVIDIA container toolkit is not installed on this host (checked).
- 8 GB of VRAM at 35 W is at Isaac ROS's stated minimum, and the FP32 peak is about 7 GB.
- Nobody has measured what GPU load does to the 1 kHz PREEMPT_RT loop.
- Frames would have to cross a process boundary without going over DDS.
- A plain, symmetric cube is exactly where this method is weakest.
The fallback without a GPU is the classical ICP.

INCREMENTAL: fit 9, verifiability 9, risk 8, concreteness 7, hardware honesty 8, total 41/50. It first builds an `/object/*` contract that does not care where the pose comes from, with the marker as its only source. It then goes shadow → depth checked by the marker → marker only as a seed → no marker, and measures each step against the marker. Consumers switch by parameter only, and the panel shows the source and how well it agrees with the marker. It is the best path and the easiest to verify, but it says little about the estimator itself.

**Final plan:** the INCREMENTAL skeleton, with the CLASSICAL estimator as its engine. From LEARNED it keeps three things: a part file that declares the symmetry, a silhouette/edge cue for the directions depth cannot see, and a measured decision gate (with a container fed over shared memory) that is used only for the connector.

## Appendix B: the critic's corrections to the first draft

I checked these with graphify first, then by reading the files. Everything else the plan cites was checked as well and is either confirmed or marked NEW:
- fr3_params.yaml lines 20 and 148, and tracking_node.cpp lines 382-383.
- grip_node.py lines 71 and 160.
- The vision_standalone.py frame loop (lines 91-101).
- The cam_pub overlay drawing, at lines 559-562 and 600-602.
- These functions exist: `_to_filter_frame`, `fit_plane_robust`, `quad_mask`, `target_marker_size_m`.
- Docker 29.2.1 is installed and `nvidia-ctk` is not.
- CPU layout: E-cores are 12-19, LP-E cores are 20-21, and the kernel command line has no isolcpus.

**Wrong**
- **grip_node does not read `pose_topic`.** It reads only `tracking_raw_pose_topic` (grip_node.py:71,160) and `grip_target_topic` (98). Repointing still works because the key sits under `/**`. The plan also misses `raw_pose_topic` (fr3_params.yaml:21, used by the parked mating_node), which it leaves on `/aruco/pose_raw`.
- **core.py holds no pose topics.** They are hardcoded in `ros_node.py:63` (`RAW_POSE_TOPIC`) and `:174` (`'/aruco/pose'`). core.py has only `IMAGE_TOPIC` (220).
- **The recorder hook cannot go in `process_frame`.** Its signature is `(color_image, header, depth_m)`, so it never receives the `Frame`, and `t_host`/`t_hw` cannot reach it. The Frame exists only in the capture loops (cam_pub `_capture_loop` 467-479, vision_standalone 91-101).
- **The overlay also corrupts live frames, not just recordings.** `process_frame` draws the marker and axes onto the image it is given whenever `publish_debug` is true, which the launch sets. This happens with or without a subscriber. vision_standalone passes `frame.bgr` directly (97), so the Phase 2b colour-edge cue would read the drawn axes. For the same reason, "draw the model outline on `/aruco/debug_image`" does not work as written: the debug image is published inside `process_frame`, before the estimator runs.
- **The Phase 5 prior wrongly assumes a static part.** TRACK exists to follow a moving part, and TRACKING_SPEC V2 (a 20 mm step made by hand) and V3 (a sine up to 0.5 Hz) move it (TRACKING_SPEC.md:293-294).
  - The prior should be the object KF's constant-velocity prediction in fr3_link0, carried through TF at the image stamp. That covers object motion and camera motion together.
  - A 20 mm step is beyond the 10 mm shift gate. It needs a local re-acquire, not a 0.5 s wait.
- **`T_marker_object` = identity for the cube is an assumption.** The marker is a hand-applied sticker.
  - Teaching the offset from depth-versus-marker data makes the bias metric zero by construction, which is circular.
  - If marker mode does not apply the offset, the marker-to-depth switch shifts TRACK's in-plane target by however far the sticker is rotated.
- **The marker's tilt is not an independent reference.** cam_pub's tilt comes from a depth-plane fit over the marker quad scaled 1.6× (`tilt_source: depth`). The plan masks only 1.2×, so the ring between 1.2× and 1.6× feeds both. Tilt agreement mostly compares depth with depth.
- **An absolute TRACK angular σ ≤ 0.07° is unattainable.** The marker itself gives about 0.18° per-frame tilt, and the KF uses `meas_std_rot_deg` 1°. Getting from 0.3° raw to 0.07° needs about 18 frames of averaging (about 1.2 s), and that is exactly the lag V3 measures. The criterion should be relative to the Phase 1 baseline.
- **`latency_fit.py` cannot use the state_recorder CSV.** The CSV has only robot `time` (`"time,mode,success,…"` in state_recorder.cpp), with no ROS stamp and no EE velocity, so it cannot be joined to image stamps. Use `/tf` and `/joint_states` from the bag instead.
- **The 2% `tvec.z`-versus-depth check confounds two errors.** It cannot separate a wrong `marker_size_m` from the D405's own ±1.4% depth bias. Measure the marker with calipers first; the comparison then measures the depth-scale bias.
- **`~/fr3_data` breaks the repo convention.** Bags, traces and state CSVs already go to `$FR3_LOG_DIR/YYYY-MM-DD/` (fr3_env.sh:39, `core.trace_dir` 223-227).
- **The GPU risk is misdescribed.** The NVIDIA driver is already loaded and running Xorg. What is unmeasured is GPU compute and memory load.
- **Phase 1 says both "plumbing only" and changes ALIGN.** It claims "identical behaviour" but also turns on `raw_age()` gating for ALIGN, which is a behaviour change.
- **The new `vision_exec` argument allows an invalid combination.** It sits beside the existing `vision_source:=realsense|topic`, so standalone plus topic becomes possible.
- **The plan contradicts itself about cam_pub.** It rejects "growing cam_pub", then adds `record_dir` and `last_accepted` to it.

**Missing**
- **Consumers the plan does not cover:**
  - The panel's rosbag recorder (`actions.py:79-82` `Recorder.TOPICS` hardcodes `/aruco/pose` and `/aruco/pose_raw`, and lacks `/aruco/target_pose_raw`, `/object/*` and `/grip_node/status`).
  - ALIGN (`logic.py:735-748`; `core.py:36-50`: 2 mm / 1° / 0.5° in-plane). ALIGN relies on the marker's 0.01° in-plane std, and a top-face-only depth yaw of about 1.2° will not converge. ALIGN also gates TRACK entry.
  - The `NO MARKER` banner in logic.py.
- **vision_standalone has no per-frame exception guard.** cam_pub's `_capture_loop` has one (473-478). One bad frame would kill the only process that owns the camera.
- **No held-cube state.** After GRIP, the cube moves with the hand and sits inside the D405's minimum range. The prior and acquisition would chase it or lock onto B's plate. `/grip_node/status` (`holding`, grip_node.py:256) already exists and can drive a suspend.
- **Failure modes not listed:**
  - The operator's hand touching the cube (V2, V3, V4) and merging with its cluster.
  - Gripper fingers in the near field at the GRIP viewpoint.
  - B's plate as a distractor.
  - Auto-exposure changes.
- **Sensor settings are left undecided.** Depth preset, post-filters (a temporal filter adds lag), exposure lock and depth units (the D405 default is 0.1 mm) are all unset in the code today.
- **Distortion.** `icp.depth_to_points` is pinhole-only (200-212), while ArUco uses the SDK coefficients (vision_standalone:70). The result is a disagreement between the two sources that grows with image radius, and it is largest at the off-centre GRIP view.
- **`icp.icp` returns less than the plan needs.** It returns only `(T, rms, inlier_frac)` (197), with no normal matrix for `weak_dof`. Its `rms` is the point-to-point nearest-neighbour distance to a template sampled on a 2 mm voxel grid, so the 1.5 mm gate needs calibrating.
- **TF lookups can stack up.** Each frame can block up to 50 ms in `_to_filter_frame`, and the object KF and the prior would add more lookups in the same thread. The plan should use one shared lookup per frame and measure its wait.
- **Source-switch semantics are undefined.** A switch should reset the KF and re-run acquisition hysteresis, which causes a deliberate raw gap so consumers hold.
- **`/object/pose_raw` needs the KF innovation gate.** `/aruco/pose_raw` is published only when the KF accepts (cam_pub 537-546), and the new topic should match.
- **Per-source, per-axis KF noise is unset.** The marker values are `meas_std_pos` 2 mm and `meas_std_rot_deg` 1°.
- **Shadow mode logs only scalar agreement.** With only `agree_mm`/`agree_deg`, per-axis bias and σ cannot be computed from on-arm bags.
- **Measurement gaps:**
  - ALIGN's 80 mm minimum (`core.py:34`) and the D405 minimum-Z are not in the distance list.
  - There is no bare-cube recording: masking does not remove the sticker's stereo texture.
  - `capture_width`/`capture_height` are not launch arguments.
  - There is no hand-eye re-check after a resolution change.
  - The replay perturbations (±5 mm) never reach the 20 mm V2 step.
- **Placement and metrics:**
  - The vision process should be pinned from Phase 1, so the baseline and the A/B comparisons do not confound pinning with the estimator.
  - Add the C10 RT metric: % of time below 97% RT success, estimator on versus off (cell README 178-188).
- **Decisions not recorded:**
  - The marker-free goal and contract live only in the user's memory. They should go into a repo doc.
  - The insertion row does not say which frame it is measured in. Base-frame accuracy is bounded by hand-eye (8.14 mm).
- **No test pins the topic names.** Nothing ties the fr3_params topic names to the vision node's publisher names (in the style of `test_cell_pins.py`).
