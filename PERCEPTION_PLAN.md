# Marker-free perception plan (DRAFT)

> **Status: DRAFT, 2026-09-25.** Written by a 4-agent research workflow (codebase reader, state-of-the-art researcher, planner). The completeness critic did NOT finish before a usage limit - names and claims below are not yet cross-checked against the code. Nothing here is implemented yet.

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

---

# Marker-free object pose for the FR3 cell: plan

Repo: `/home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src`. I checked these facts myself while writing the plan:
- TRACK and GRIP both read `pose_topic` and `tracking_raw_pose_topic` (`fr3_params.yaml:20,148`; `tracking_node.cpp:382-383`; `grip_node.py:71,160`), so they can be repointed with parameters alone.
- `vision_standalone.py` already runs `ArucoPosePublisher.process_frame` on each frame in its own loop.
- `cam_pub.process_frame` draws the debug overlay directly onto `color_image`.
- Docker 29 is installed; the NVIDIA container toolkit is not.
- The E-cores are CPUs 12-19, the LP-E cores are 20-21, and there is no `isolcpus`.

## 1. Goal, non-goals, acceptance criteria

**Goal.** Get a 6-DoF pose of the part from the D405 depth and colour that are already captured in-process. Publish it on a contract that does not care where the pose came from, so that TRACK, GRIP and PLACE AT B use it with parameter changes only. Validate each step against the ArUco marker, which stays on the part as ground truth until Phase 6.

**Non-goals:**
- Choosing the connector.
- Methods trained per object.
- Clutter or bin picking.
- Any autonomous motion: every motion stays behind a panel button.
- A marker-free B: B stays marker id 1 until the receptacle part exists.
- Fixing hand-eye: its 8.14 mm validated scatter stays the error floor for GRIP. That is a separate TODO.
- Retuning the tracking law.

**Metrics.** Unless stated otherwise, each is measured against `/aruco/pose_raw` from the same frame, after the offset `T_marker_object` has been taught. That offset is identity for the cube, because the marker is centred on the top face. Each condition uses at least 300 frames.
- **Bias:** the mean difference.
- **σ:** the standard deviation with both arm and part still.
- **Availability:** the fraction of frames with a valid raw pose while the part is fully in view.

| Consumer | Criteria |
|---|---|
| **TRACK** (camera-centred, 100 mm standoff, top face only in view) | Raw rate ≥ 12 Hz; no raw gap > 0.20 s in a 60 s static run (the gate is 0.25 s); availability ≥ 95%. Filtered `/object/pose` in fr3_link0 has σ ≤ 0.8 mm and ≤ 0.07° (3σ inside the 2.5 mm / 0.2° integrator freeze band), and the BuzzMeter shows no more hunting than the marker baseline. Bias ≤ 2 mm, tilt ≤ 1°, in-plane ≤ 0.5°. Frame-to-publish p95 ≤ 40 ms; stamp error ≤ 10 ms (measured). TRACKING_SPEC V1-V4 pass. |
| **GRIP** (base frame, read once) | Base-frame position bias ≤ 2 mm per axis; 5-sample scatter ≤ 2 mm (gate 5 mm); yaw mod 90° ≤ 2°; tilt ≤ 2°. At least 3 raw poses in 0.5 s on ≥ 98% of presses. 19/20 physical grips at random positions and yaws. The estimator may add only 2 mm on top of hand-eye, because the 10 mm-per-side margin is mostly used up already. |
| **PLACE AT B** | B unchanged (marker). The placed cube lands within 3 mm / 3° of B plus its offset, measured by looking at the placed cube with both estimator and marker. That must be within 1 mm of the marker-GRIP baseline. |
| **Insertion** (provisional, part not chosen) | At 100-150 mm: lateral bias ≤ 1 mm, σ ≤ 0.3 mm; keyway yaw ≤ 1°, with the 180° ambiguity resolved once; tilt ≤ 1°; ≥ 10 Hz; availability ≥ 90% during the approach. This fits `fine_pos_tol` 2 mm / 1° and the chamfer capture range. |
| **Cell** | The vision process uses ≤ 1 core on average; FCI success rate ≥ 0.99 over a 15 min V6 run; no comm reflexes added compared with the estimator switched off. |

## 2. The pose contract

The vision process that owns the camera (`vision_standalone`) publishes:

| Topic | Type / frame | Meaning |
|---|---|---|
| `/object/pose_raw` | PoseStamped, `camera_color_optical_frame`, image stamp | **A measurement that passed every gate in the current mode. It is never a prediction, a prior or a fallback.** No message means no valid pose, the same as `/aruco/pose_raw` today. |
| `/object/pose` | PoseStamped, `filter_frame` (fr3_link0), KF | Filtered; may coast ≤ 0.3 s. It is the goal for TRACK and ALIGN. |
| `/object/pose_quality` | `diagnostic_msgs/DiagnosticArray`, header.stamp = image stamp, published **every processed frame** | level OK/WARN/ERROR; message = the reason. Keys: `source` (marker, depth_checked or depth), `valid`, `rms_mm`, `inlier_frac`, `n_pts`, `weak_dof` (e.g. `x,y,yaw`), `agree_mm`, `agree_deg` (against the marker, NaN when it is not seen), `sym_index`, `compute_ms`, `seeded_from`. |
| `/aruco/*` | unchanged | Used by hand-eye calibration, `teach_offsets`, B and the ground-truth logs. |

**Object frame.** The origin is the top-face centre (the mate point for a connector). Z points out of the surface toward the camera. The in-plane X axis is the member of the part's declared symmetry group closest to the previous estimate; at seeding it is the one closest to the marker's X. That way the switch from marker to depth never turns TRACK's 90° in-plane target by a quarter turn. The cube resting on a face has 4-fold symmetry about Z; a connector has 1- or 2-fold symmetry.

**Switching.** Consumers are repointed once, in Phase 1:
- `fr3_params.yaml` `pose_topic` → `/object/pose`.
- `tracking_raw_pose_topic` → `/object/pose_raw`, which covers both TRACK and GRIP.

From then on, the **source is chosen by a parameter `object_source` on the vision process**. The panel can change it only while TRACK and GRIP are idle.

**How the rule "predictions never drive committed motion" is kept:**
- The kinematic prior and the global initialiser only seed ICP. Neither ever reaches `_raw`.
- Consumers still gate on raw arriving, which is Decision 5, unchanged.
- The first raw pose after an acquisition needs 5 consecutive frames that agree with each other.

Consumers read no confidence number: validity is simply whether a raw pose arrived, so neither C++ nor `grip_node` changes.

## 3. Phases

### Phase 0: measure the sensor and record ground truth (no estimator yet)

**What is built:**
- `roscam/roscam/frame_recorder.py`, which writes one set of files per frame:
  - colour as lossless PNG;
  - depth as uint16 PNG, plus the depth scale;
  - the intrinsics as JSON;
  - the host stamp and the hardware stamp;
  - the marker raw pose and its reprojection error.

  A writer thread with a bounded queue of 30 does the writing and counts dropped frames.
- A `record_dir` parameter on `cam_pub` (empty means off). The hook sits in `process_frame` and **copies the image before the debug overlay draws on it**.
- `rs_capture.Frame` gains a `t_hw` field from the SDK metadata.
- `tools/fr3/vision/depth_quality.py`: fill rate on the top face with the marker quad masked out (×1.2), plane rms, and the fraction of flying pixels at the silhouette, at 100, 150, 200 and 300 mm, for 640×480 and 848×480.
- `tools/fr3/vision/latency_fit.py`: the arm jogs past a static marker, and the base-frame marker displacement is regressed against EE velocity from the 1 kHz `state_recorder`. The slope is the capture latency.
- TF, `/aruco/*` and joint states are recorded as a rosbag, which is small, and joined to the frames by stamp offline.

**Key decisions:**
- Record to disk inside the process. Rejected: image topics in a rosbag (breaks "images off DDS"); a second process on the camera (only one owner allowed).
- Use lossless PNG. Rejected: JPEG, whose edge artefacts would bias the yaw from edges.
- Join TF offline. Rejected: looking TF up in the capture thread, which blocks for up to 50 ms.
- Mask the marker quad out of the depth when evaluating. Rejected: trusting depth on a printed face, since the print gives passive stereo texture that a bare part will not have.

**Verification:**
- Recordings have 0 drops at 15 fps, monotonic stamps, and marker poses identical to cam_pub's own.
- The IPPE `tvec.z` is checked against the depth distance at the marker centre. A difference over 2% means `marker_size_m` is wrong, which would make the ground truth wrong.

**Exit criteria:**
- At least 6 recordings: static at 3 distances; oblique with 2-3 faces in view; slow motion like TRACK; the GRIP viewpoint; and junk frames with no cube.
- A measured `capture_latency_s` with a confidence interval.
- A depth-quality table.
- A recorded decision on resolution and frame rate.

**Risks:**
- On the bare top face at 100 mm, the D405 (passive stereo, no projector) may leave holes. If fill is below 50%, Phase 2 must lean on edges, and the decision gate for the learned method opens early.
- PNG encoding costs CPU. The fallback is to record every 2nd frame.

### Phase 1: the contract, with the marker as the only source (plumbing only)

**What is built:**
- `vision_standalone.py` publishes `/object/*`. With `object_source: marker` it copies cam_pub's accepted raw and filtered poses unchanged, using a new `self.last_accepted` handoff attribute in `cam_pub.py` that is set where it already publishes.
- It publishes `/object/pose_quality`.
- `fr3_cell.launch.py` `_vision()` gets the launch argument `vision_exec:=cam_pub|vision_standalone`.
- `fr3_params.yaml` lines 20 and 148 point at `/object/*`.
- Panel:
  - `core.py` and `ros_node.py:63,174` switch topics;
  - the `logic.py` chip becomes `VISION <ms> · MARKER`;
  - a `view.py` source dropdown is disabled unless the cell is idle;
  - `raw_age()` is finally used for ALIGN.
- `mock_cell.py` publishes `/object/*` plus quality.
- `tools/fr3/sim/tracking_smoke.py` gets the new topic parameters.
- **No change to `tracking_node.cpp` or `grip_node.py`.**

**Key decisions:**
- Keep PoseStamped and add a separate quality DiagnosticArray. Rejected: a custom message package (a new build and C++ churn for fields no controller reads); PoseWithCovarianceStamped (changes the types consumers read, and the ICP covariance is not calibrated).
- Choose the source in the vision process. Rejected: remapping consumers for each source (needs a relaunch); a separate mux node (an extra hop and an extra process).
- Adopt `vision_standalone`. Rejected: growing cam_pub, which hand-eye calibration depends on.
- `connector_pose.py` stays untouched. The parked melfa launch uses it; its prior-as-measurement fallback and its `frame_id` bug are not carried over.

**Verification:**
- The mock plus `tracking_smoke.py` pass, as do the `test_cell_*` tests for the chip and the idle-only dropdown.
- On the cell, a 60 s rosbag shows `/object/pose_raw` identical bit-for-bit to `/aruco/pose_raw`.
- The operator runs one TRACK V1, one GRIP and one PLACE AT B; results match the 09-24 and 09-25 runs.

**Exit criteria:** identical poses and identical behaviour, with `MARKER` shown on the panel.

**Risks:** `vision_standalone` has never run on the arm. The launch argument is the rollback.

### Phase 2: the depth estimator, offline only, scored against the marker (no robot)

**What is built:**
- `roscam/roscam/object_pose.py`, which is ROS-free: `ObjectPoseEstimator.process(depth, bgr, K, prior) -> (T, valid, quality)`. Its steps:
  1. Back-project an ROI around the prior projected into the image.
  2. Remove the table with `fit_plane_robust` on the ring around the footprint, dropping points within 3 mm of the plane.
  3. Keep the connected cluster nearest the prior.
  4. Run scene-to-model point-to-plane `icp.icp`.
  5. Apply the gates on object points: inliers ≥ 0.8, rms ≤ 1.5 mm, shift ≤ 10 mm, ≥ 200 points.
  6. Flag weak degrees of freedom from the eigenvalues of the 6×6 point-to-plane normal matrix.
  7. Snap to the nearest symmetry member.
- `tools/fr3/parts/cube55.yaml`: a box primitive or STL, the origin convention, symmetry C4 about Z, and dimensions for the size gate.
- `tools/fr3/vision/replay_eval.py`: runs on Phase 0 recordings with the marker prior perturbed by ±5 mm / ±5°, and prints the bias/σ/availability/compute table per condition.
- **Phase 2b, built only if 2a misses the TRACK in-plane budget.** Expect it to, given the synthetic 1.2° top-face-only result. Take in-plane yaw and x/y from the colour edges of the depth-segmented top face: a line fit in a band around the projected model edges. Take z and tilt from depth. This is the same split `plane_normal.fuse_orientation` already uses successfully, with in-plane from the image and normal from depth.

**Key decisions:**
- Use the numpy ICP from the repo, at about 14 ms. Rejected for now: Open3D (2.4 ms), adopted only if p95 exceeds 30 ms.
- Remove the table with a plane fit. Rejected: a tighter sphere crop (the bench showed table points pulling the pose 29 mm).
- Declare symmetry in the part file and snap to the previous pose. Rejected: ADD-S-only scoring, since TRACK needs a stable X.
- Use an edge cue for weak degrees of freedom. Rejected: coloured ICP (the parts are untextured); FPFH or PPF every frame (100 ms or more, ambiguous on flat faces).

**Verification:** `replay_eval` against the marker; `roscam/test/test_object_pose.py`.

**Exit criteria:**
- The §1 TRACK and GRIP bias figures, and raw σ ≤ 1 mm / 0.3° before filtering, on the masked-marker recordings.
- At least 99% convergence from the perturbed priors.
- 0 false accepts on junk frames.
- p95 compute ≤ 30 ms on one E-core.

**Risks:**
- Depth holes on the bare face.
- Edges near the table on low-contrast backgrounds. Mitigation: a matte, contrasting mat is a cell change for the user to decide.

### Phase 3: live shadow mode (the estimator runs, the marker still drives)

**What is built:**
- `object_pose` is wired into the `vision_standalone` loop, with a PoseKF in fr3_link0 fed through TF at the image stamp (the same mechanism as cam_pub's `_to_filter_frame`).
- The prior is the marker on every frame.
- `object_source` stays `marker`. The quality message carries `agree_*`.
- The panel chip reads `MARKER · depth Δ1.1 mm 0.4°`.
- The model outline is drawn on the existing subscribe-gated `/aruco/debug_image`. There is no new image topic.

**Key decisions:**
- Run in the same process and thread as ArUco. Rejected: a separate node (would need the images).
- Pin the vision process to CPUs 12-19 with thread pools set to 1 (see §4). Rejected: sharing the P-cores with the controller.

**Verification:** the operator runs normal TRACK V1-V3, GRIP and PLACE sessions, with the rosbag of `/object/*`, quality, `/aruco/*` and TF, plus the 1 kHz recorder. V3 gives the phase lag for both sources.

**Exit criteria:**
- At least 20 minutes of real operation inside the §1 agreement budgets.
- V6 run for 15 minutes with the estimator on: FCI success ≥ 0.99 and no added comm reflex.

**Risks:** CPU contention, and yaw jitter from real depth. Both are measured here, before any motion depends on depth.

### Phase 4: `depth_checked` mode (depth drives, the marker can veto)

**What is built:**
- `object_source: depth_checked`: a raw pose is published only when the depth pose is valid **and** agrees with the marker from the same frame within 3 mm / 2°, modulo symmetry.
- The panel dropdown gains this option.
- The chip reads `DEPTH✓MARKER` and shows the veto rate.

**Key decision:** the veto is on a per-frame basis. Rejected: blending the two sources, which would hide which one is driving.

**Verification:** on the arm, V1, V2 and V4 (occlude, arm holds), and 10 GRIPs from random placements. Compare the tracking jsonl and the BuzzMeter with the Phase 1 baseline.

**Exit criteria:**
- V1 ≤ 5 mm / 0.5°.
- Integrator hunting no worse than with the marker.
- GRIP 10/10.
- Veto rate < 5%.

**Risks:** depth jitter makes the integrator hunt. The fix is KF measurement noise in the depth filter, not the tracking gains ("measure before tuning").

### Phase 5: the marker is only a seed; the kinematic prior carries the pose

**What is built:**
- The prior at frame t is `T_optical(t)⁻¹ · T_base_obj(t−1)`, through TF at the image stamp; the part is static in the base frame.
- The marker seeds only on acquisition, or after more than 0.5 s lost.
- `object_source: depth`, `object_seed: marker`.
- A parameter `mask_marker_for_estimator: true` blanks the marker quad in the depth the estimator sees.
- The marker is still logged in `agree_*` but no longer vetoes.

**Key decision:** use the arm-kinematic prior. Rejected: a constant-velocity visual prior, since the camera motion is known exactly from TF.

**Verification:**
- Replay Phase 0 recordings with the marker withheld after frame 1, to measure drift.
- On the arm: V1-V4, 20 GRIPs, 10 PLACE AT B.

**Exit criteria:** 5 minutes of V1/V3 without re-seeding, inside budget; drift ≤ 1 mm/min; re-acquisition works after V4.

**Risks:** slow slip along a weakly observed direction. The `weak_dof` gate catches it by refusing raw poses while the edge cue is missing.

### Phase 6: acquisition without the marker, then removing it

**What is built:** `ObjectPoseEstimator.acquire()`:
- Find the table plane in the workspace ROI; the table height comes from the base frame.
- Find clusters above it whose size matches the part file (±5 mm).
- For box-like parts, take the top-face plane plus `cv2.minAreaRect`, then ICP, then the symmetry member nearest the gripper's current yaw. Open3D FPFH+RANSAC (about 100 ms, run once) is only for templates that are not boxes.
- Hysteresis: 5 consistent frames before the first raw pose.
- The panel shows `ACQUIRED` plus the overlay, and the operator then presses the motion buttons as today.

**Key decisions:**
- Box-fit for the cube. Rejected: FPFH/RANSAC or TEASER++ everywhere (ambiguous on flat faces; TEASER++ needs a source build).
- No extra CONFIRM button for the cube, because all four symmetric orientations are valid. Rejected for the connector (see §6).

**Verification:**
- 50 random placements of the cube, marker still present but masked: acquisition rate, plus 0 false acquisitions with 3 distractors.
- Then **remove the marker**: 20 GRIPs and 10 PLACE AT B, judged physically, with the placed cube re-measured.

**Exit criteria:** acquisition ≥ 98%; 0 false acquisitions; GRIP ≥ 19/20 without the marker.

**Risks:** distractors of similar size. The size gate and the rms gate catch them; if not, the operator confirms.

### Phase 7 (starts only when the connector is chosen)

**What is built:**
- A part file: STL with the origin at the mate point, the symmetry, and the key feature.
- The Phase 0 depth-quality test on the real part at 100, 150 and 200 mm.

**Decision rule:**
- If top-face fill ≥ 70% and plane rms ≤ 1 mm, reuse Phases 2-6 with the connector template. The keyway's 180° ambiguity is resolved once by the operator on the overlay, then held by continuity.
- If the part is black or shiny, choose one of:
  - **Learned option: FoundationPose tracking.** Use the Apache-2.0 inference library in a container. It is fed frames over shared memory from `vision_standalone`, never over DDS. It is seeded by the Phase 6 cluster mask and refined by ICP.
  - **No-GPU fallback:** ICG/M3T on the CPU, or keep a marker for that part.

  FoundationPose runs only after four things are shown on this host: the NVIDIA container toolkit is installed; FP16 VRAM ≤ 6 GB; tracking ≥ 15 Hz; and a 15 min V6 with the GPU loaded keeps FCI success ≥ 0.99.

## 4. Compute budget (shared with the 1 kHz RT controller; images stay in-process)

| Item | Cost per frame at 15 Hz | Source |
|---|---|---|
| ArUco + IPPE + plane normal (today) | not measured; Phase 0 measures it | — |
| ROI back-projection, table plane, clustering | 3-13 ms | synthetic bench (research) |
| numpy point-to-plane ICP, 30 iterations | 4-33 ms, about 14 ms typical | bench |
| Edge cue (Phase 2b) | about 1-3 ms | estimate; Phase 2 measures it |
| Open3D ICP (only if needed) | 2.4 ms | bench |
| Acquisition (box fit or FPFH) | ≤ 110 ms, once per acquisition | bench |
| Recorder (Phase 0 sessions only) | about 10-20 ms of PNG work on the writer thread; 10-18 MB/s to disk | estimate |

- **Per-frame budget:** p95 ≤ 45 ms, i.e. ≤ 0.7 core at 15 Hz. At 30 fps, run ICP on every frame only if p95 ≤ 25 ms; otherwise switch to Open3D.
- **Placement:** `taskset -c 12-19`, with `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` and `MKL_NUM_THREADS` set to 1, SCHED_OTHER and nice 5. This stops the BLAS and Open3D thread pools spreading onto all 22 CPUs. The controller is untouched.
- **DDS:** the new traffic is about 15 Hz × (2 × 250 B + 600 B), roughly 17 KB/s. No images or point clouds go on DDS; the overlay stays on the subscribe-gated `/aruco/debug_image`.
- **GPU:** none in Phases 0-6. The NVIDIA driver on a PREEMPT_RT kernel is an unmeasured source of latency, so only Phase 7's gate may bring it in.
- **Memory:** under 500 MB.

## 5. Test strategy

- **Unit tests (pytest, `roscam/test`):**
  - `test_object_pose.py`, synthetic scenes:
    - cube on a table at 100 and 200 mm, with 0.1-0.5 mm noise, holes and flying pixels;
    - the table-pull regression: the pose must move ≤ 1 mm;
    - the `weak_dof` flag for a top-face-only view;
    - the C4 snap staying next to the previous pose;
    - junk rejection;
    - acquisition with distractors present.
  - `test_frame_recorder.py`: round trip and drop counting.
  - A pure function for the `object_source` rules (marker, depth_checked, depth; raw published or not).
  - A check that the quality stamp equals the pose stamp.
- **Recorded datasets:**
  - Phase 0 frames go under `~/fr3_data/vision/<date>/`, outside git.
  - `replay_eval.py` produces the acceptance tables for each phase.
  - About 30 frames per condition are committed as a regression fixture.
  - Every on-arm session records a rosbag of `/object/*`, quality, `/aruco/*`, TF and joint states, plus the tracking jsonl and the 1 kHz `state_recorder`, which gives RT success and EE velocity.
- **Mock and simulation:**
  - `mock_cell.py` publishes `/object/*` and a scripted quality message covering disagreement, veto and loss.
  - `test_cell_*` covers the chip, the NO/LOST banners and the idle-only source dropdown.
  - `tracking_smoke.py` runs with the `/object/*` parameters.
- **On the arm, operator in the loop:**
  - TRACKING_SPEC V1-V6, plus N GRIPs and N PLACE AT B for each phase, with every motion started by a button.
  - The marker stays on the part and logs agreement until Phase 6.
  - Each phase is compared with the Phase 1 baseline from the marker.

## 6. Open questions for the user (each with a recommended default)

1. **What are the printed sizes of marker id 0 and id 1?** Default: measure them with calipers and set `marker_size_m` and `target_marker_size_m` before Phase 0, because the ground-truth scale depends on them. If the cube's marker covers more than half the top face, make a test cube with a centred marker of about 20 mm.
2. **Should the FR3 launch switch from `cam_pub` to `vision_standalone`?** Default: yes, behind `vision_exec`, with cam_pub kept as the rollback.
3. **Which capture resolution?** Default: decide from Phase 0 data. Lean toward 848×480 at 15 fps if it improves top-face σ by at least 30%.
4. **Should TRACK use an oblique view so the side faces show?** Default: no; add the edge cue (Phase 2b) instead, and revisit only if Phase 2 fails.
5. **Does a marker-free acquisition need an extra CONFIRM button?** Default: not for the cube; yes for the connector, because of the keyway's 180° ambiguity.
6. **Should B go marker-free (a taught pose or the receptacle)?** Default: keep marker B until the receptacle exists, then give it its own part file.
7. **Timestamps: SDK hardware timestamps or a measured constant?** Default: the measured constant, unless Phase 0 shows jitter above 5 ms.
8. **Can a matte, contrasting mat go under the work area?** Default: yes, if Phase 0 shows edge or depth problems at the table boundary.
9. **Is installing the NVIDIA container toolkit on the RT host acceptable?** Default: not now; only if Phase 7's depth test fails, and only after a measurement of GPU load against the FCI loop.
10. **Where is data stored, and how much?** Default: `~/fr3_data/vision`, about 1 GB per minute at 15 fps, with only the small fixtures in git.
