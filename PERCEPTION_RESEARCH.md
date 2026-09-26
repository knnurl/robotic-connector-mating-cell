# Marker-free perception: research notes (2026-09-25)

> Inputs to PERCEPTION_PLAN.md: the codebase ground truth and the state-of-the-art survey, as the workflow agents returned them.

## Codebase ground truth

**Ground truth for a marker-free pose design on the FR3 connector-mating cell (read-only survey, 2026-09-25)**

I started with graphify (query, explain and path) and then read the sources. The roscam unit tests pass: 48 passed, flake8 deselected, run with no bytecode or cache written. I also ran a short synthetic ICP benchmark in the scratchpad (the numbers are marked "my synthetic bench" below). I ran no ros2 commands and changed no repo files.

**Before relying on the docs, three corrections:**
- The "marker-free ICP end goal" is written down only in the user's memory (`fr3-direction-decisions.md`), not in any repo document.
- `PROJECT_STATE.md` (dated 09-22) says TRACK was "never run". That is stale: `fr3_params.yaml:153-176` cites TRACK runs on the arm on 09-23 and 09-24, and `grip_node.py:589-592` cites GRIP runs on 09-25.
- `connector_pose` / ICP has never touched real D405 data. Evidence: `TODO.md:283-285` ("needs real-data tuning + STL export"), `SETUP_AND_CALIBRATION.md:395`, and the only launch that wires it is the parked `melfa/parked/mating_controller/launch/cell.launch.py`.

---

## 1. Vision stack (roscam)

| File | What it does | Inputs → outputs | Maturity |
|---|---|---|---|
| `cam_pub.py` | ArUco detection, IPPE_SQUARE pose with the mirror ambiguity resolved (`solvePnPGeneric` plus the depth plane normal, 361-436), reprojection gate 2.0 px (136, 604-620), depth-fused tilt (122, 509-514), a Kalman filter in `filter_frame` using TF at the image stamp with a 0.05 s blocking timeout (622-642), a static target marker B (580-602) | colour (+ aligned depth) → `/aruco/pose` (filtered; the launch puts it in fr3_link0), `/aruco/pose_raw` (always optical frame; **published only when the filter's gate accepted the detection**, 539-546), `/aruco/target_pose_raw` (optical, no filter, needs depth), `/aruco/debug_image` (≤5 Hz, only while subscribed, 166/570-578) | Launched by `fr3_cell.launch.py:167-188` with source=realsense, filter_frame=fr3_link0 (62), marker_id=0 (69), target_marker_id=1 (71), 15 fps (78). Proven on hardware (alignment runs 09-11 and 09-15). The node class has no unit test |
| `rs_capture.py` | pyrealsense2 capture outside ROS. Default 640×480 @ 15 (39); depth is z16 at the same resolution and rate, aligned to colour with `rs.align` (87-90, 184-185); depth scale taken from the sensor (95-97); **intrinsics from the colour stream profile** (99-102); self-healing after USB re-enumeration (52-70, 112-196); **only one owner per camera** (36-37) | Frame(bgr, depth_m, t_host) | Unit-tested with a fake pipeline (`test_rs_capture.py`). Hardware-verified at **89.9 fps, 640×480 colour+depth, 0.2 ms jitter** (`TODO.md:148-151`). Not set anywhere: depth preset, post-filters, exposure or hardware timestamps |
| `plane_normal.py` | Robust plane fit (quantile trimming, 53-94). `marker_plane_normal` works on the quad scaled 1.6× (≥40 points, rms ≤4 mm, cap 600 points; bbox crop cuts ~12 ms to 0.3 ms, 134-136). `fuse_orientation` takes the normal from depth and the in-plane rotation from ArUco (184-219). `solve_square_by_depth` handles static markers (286-307) | pure numpy/cv2 | `test_plane_normal.py`: <1.5° normal error at 0.2 mm depth noise at 100 mm (81-91) |
| `pose_kf.py` | Constant-velocity translation plus small-angle orientation, Mahalanobis gate at `gate2*3` (134), dt clamped to 0.5 s (101) | pure numpy | `test_pose_kf.py`: prediction drift <10 mm over 0.3 s (85); filtering in a fixed frame survives a camera step (130-171) |
| `icp.py` | STL loader, area-weighted sampling with normals, voxel mean. **Point-to-plane ICP** (linearised 6×6 least squares) or Umeyama point-to-point; trim_fraction defaults to 1.0 on purpose (148-155). `max_corr` 8 mm, 30 iterations. `depth_to_points` is **pinhole only, distortion ignored** (200-212). Sphere crop (215-218). numpy + cKDTree only, "deliberately no Open3D/PCL" (4). Basin: "ICP only refines a few mm / a few degrees" (5-6) | — | `test_icp.py` on a synthetic 20×10×8 mm part with 0.3 mm noise and a prior 4 mm / 3° off: translation <1 mm, rotation <1.2° single frame (97-112); junk scene rejected (115-122); filtered rotation <0.9° (174-207, collected although it sits after the `__main__` block) |
| `connector_pose.py` | Prior = `T_cam_marker @ T_marker_connector` (230) → back-project with stride 2 → 50 mm sphere crop → 2 mm voxels → ICP against the STL template (1500×4 samples, 2 mm voxels; 99-104) → gates: inliers ≥0.6, rms ≤4 mm, shift ≤20 mm (259-279) → static Kalman filter (133-134, rebuilt after 5 rejects, 243-252). On any failure it **publishes the prior as if it were a measurement** (254-257). Runs every 5th frame (84); marker older than 0.5 s by arrival time → silent (227-228) | `/aruco/pose` + aligned depth → `/connector/pose` (PoseStamped, optical frame) | Synthetic tests only. Not in the FR3 launch |
| `vision_standalone.py` | One process owns the camera and runs cam_pub and connector_pose on the same frames (source=external, 91-101) | poses only | "NOT wired into any launch file" (27-29) |
| `teach_offsets.py` | Pairs `/aruco/pose_raw` with `/connector/pose` within 0.15 s (82) and prints `connector_offset_*`; warns above 2 mm / 1.5° spread (132) | — | Maths unit-tested only |
| `handeye_calib.py` | 4 solvers (TSAI, PARK, HORAUD, DANIILIDIS), ranked by a static-marker consistency residual (80-131). Collects `/aruco/pose_raw` plus **the latest TF, not TF at the image stamp** (210-211) | — | Result in `tools/fr3/calib/handeye.yaml:5-24`: Tsai, 21 poses, residual **3.17 mm / 1.57°**, validated static-point scatter **8.14 mm**, TCP→optical `[0.0611, -0.0111, -0.0466]` m, about 90° about Z, optical axis 0.37° off TCP Z |

**Depth path and noise figures quoted in the code:**
- ArUco position 7-10 µm, in-plane rotation 0.18°; IPPE tilt reads about 2.6° high and has a mirror ambiguity; the depth plane gives tilt 0.18° at 0.11 mm fit rms, unbiased (`plane_normal.py:189-192`, `cam_pub.py:118-121`).
- The in-plane angle has 0.01° std (`plane_normal.py:232`, `core.py:44`). This contradicts the 0.18° above.
- The IPPE mirror flipped on 62% of frames before the fix (`TODO.md:343-354`).
- Position always comes from the IPPE tvec, which is scaled by `marker_size_m`, default **0.021 m** (`cam_pub.py:107`). The launch never overrides it, and **the physical sizes of the cube marker (id 0) and B (id 1) are recorded nowhere**. The depth centroid is computed (`plane_normal.py:180`) but never used as a scale cross-check.

**Timestamps and latency:**
- Frames are stamped `now − capture_latency_s` with the default 0.02 s (`cam_pub.py:163, 451-458`). This value is a guess.
- `Frame.t_host` is ignored (467-479), and SDK hardware timestamps are unused.
- End-to-end latency has never been measured (`PROJECT_STATE.md:155`, `TRACKING_SPEC.md:294-299` V3).

**Other known issues in the vision code:**
- In topic mode, connector_pose assumes uint16 depth is in mm (`connector_pose.py:434-435`). D405 depth units should be checked; realsense mode uses the SDK scale correctly.
- **Cost of the existing ICP on this CPU (my synthetic bench, one core, 55 mm cube at 0.3 mm noise, 640×480 stride 2):** preparation 3-13 ms, ICP 4-33 ms per call.
  - A top-face-only view leaves 0.4-1.1 mm in-plane error, because a flat face barely constrains X, Y and yaw.
  - With the crop centred near the table, table points pulled ICP 29 mm off (rms 4.15 mm, inliers 0.64), and the only thing that caught it was the rms gate. **There is no support-plane or background removal.**

## 2. Pose contract (what each consumer actually reads)

All topics are `geometry_msgs/PoseStamped`, with reliable QoS, depth 10 on the publisher side. The rate equals the capture rate (15 Hz as launched) and there is no timer (`TRACKING_SPEC.md:90-122`). There is no covariance, quality value or source flag anywhere.

**tracking_node** (`mating_controller/src/tracking_node.cpp`), 50 Hz tick, period 0.02 s:
- Subscriptions use QoS(1) and store only the latest message and its arrival time (187-203). The raw subscription stores the **arrival time only** (195-202).
- Pose age is taken from the header stamp, falling back to arrival time for zero or future stamps (519-541). The pose must be ≤ `vision_timeout_s` **0.6 s** old (`fr3_params.yaml:23`, tick 1122-1129); otherwise the node holds.
- **Decision 5:** motion also requires a raw arrival within `tracking_raw_timeout_s` **0.25 s**, about 3 frames at 15 fps (`fr3_params.yaml:146-149`; `tracking_law.hpp:113-116`; tick 1131-1139).
- The pose is moved into fr3_link0 **through TF at the image stamp**, falling back to the latest transform, with a zero-timeout lookup (670-708).
- The goal is camera-centred: standoff 0.10 m, in-plane angle 90°, i.e. marker X pointing down in the image (1144-1147; `fr3_params.yaml:127-137`).
- `GoalGlide` interpolates between frame goals and never passes the newest one; its duration is clamped to [0.02, 0.25] s and it adds up to one frame of delay (`tracking_law.hpp:304-361`; tick 1150-1154).
- A hold publishes the measured pose once and keeps the lead (1222-1245).
- What the numbers imply for accuracy:
  - The deadband is 5 mm / 0.007 rad (`fr3_params.yaml:114-115`), and the integrator freezes inside half of it: 2.5 mm / 0.2° (`tracking_law.hpp:237-240`).
  - Lead caps: integrator 10 mm / 0.017 rad (`fr3_params.yaml:112-113`); equilibrium from the arm 60 mm / 0.26 rad (116, 145). The 15 N lead bound is `kLeadForceMaxN` (`tracking_law.hpp:26`).
  - V1 pass criterion is ≤5 mm / ≤0.5° (`TRACKING_SPEC.md:292`).
  - So pose jitter must stay below about F/k = 2.3-4.3 mm at 1500 N/m, and below the 2.5 mm / 0.2° freeze band, or the integrator hunts. Hand-eye error mostly cancels at the camera-centred fixed point.

**grip_node** (`tools/fr3/cell/grip_node.py`), reads **raw only**:
- GRIP needs ≥3 detections in 0.5 s, the newest ≤0.25 s old (481-491).
- It uses the last 5 detections, each through TF at its own stamp with a 0.1 s timeout. It is refused if they scatter more than **5 mm**. It returns the **mean position and the last detection's rotation**, with no rotation averaging (493-512).
- The pose is read **once**, before any motion (18-19).
- Target B is remembered on a 0.2 s timer, only with ≥3 detections **and the arm still**: moved <1 mm / 0.5° over 0.5 s (201-226). It must be ≤600 s old and tilted ≤10° (417-425).
- Tolerances:
  - Settle within 3 mm / 3° (`fr3_params.yaml:213-214`).
  - Open margin 20 mm, i.e. 10 mm per side (193).
  - Grasp epsilon 8 mm (198).
  - Arrive window 15 mm (203).
- These are base-frame consumers, so they inherit hand-eye error. The 8.14 mm validated scatter is close to the 10 mm-per-side margin.
- The cube is 4-fold symmetric: `grasp_tcp` chooses among 4 face yaws (`grip_logic.py:17-32`).

**Panel** (`tools/fr3/cell/ros_node.py`, `logic.py`):
- `/aruco/pose` freshness is judged by **arrival** time, ≤0.5 s (`ros_node.py:296-302, 327`; `core.py:92`).
- The pose is moved into the optical frame through the **latest** TF (331-343).
- ALIGN computes its errors in the camera frame against `[0, 0, target]` (`logic.py:735-748`). Tolerances: position 1/2/3/5 mm, default 2; rotation 1°; in-plane 0.5° (`core.py:36-50`).
- Chip text: `VISION <ms>` / `stale` / `no marker` (`logic.py:627-634`). Banners: NO MARKER / VISION STALE (521-528).
- Pose-jump check: the largest frame-to-frame position jump in the same frame_id over 1 s; above **10 mm** (a placeholder) it enlarges the camera view (`ros_node.py:792-801, 901-906`; `logic.py:751-758`; `settings.yaml:33`).
- TRACK entry requires ≤30 mm / 5° and ≤60 mm lead unless clamp is selected (`logic.py:389-403`). GRIP requires a fresh marker (408).
- `raw_age()` exists (893-895) but **nothing consumes it**, so ALIGN can act on KF predictions up to 0.3 s old.

**Parked mating_node:** fine tolerances 2 mm / 1°, 4 hold cycles, raw-topic arming gate (`fr3_params.yaml:48-53`; `mating_node.cpp:84-88, 215-218`).

**Frame convention the design must keep:**
- Object origin at the top-face centre, Z along the outward surface normal pointing toward the camera.
- A stable in-plane X axis, which TRACK's in-plane target needs (`plane_normal.py:178-179, 208-209`; `grip_logic.py:22-23`; `connector_pose.py:229-231`).

**What the contract is missing:**
1. **No per-message validity or covariance or source.** `/aruco/pose` mixes measurements with predictions (`cam_pub.py:543-565`). Consumers can only infer "measured" from the arrival of a separate topic, and the raw age is not matched by stamp.
2. **connector_pose's fallback publishes the prior as if it were a measurement** (254-257), and there is no `/connector/pose_raw`. Pointing tracking_node at `/connector/pose` would still gate on the marker's `/aruco/pose_raw`, which is a hidden marker dependency.
3. **connector_pose runs its Kalman filter in the optical frame**, with no `filter_frame` option. Eye-in-hand motion therefore reads as object motion, which is the failure `cam_pub.py:20-28` describes.
4. **Latent bug:** `connector_pose.marker_cb` ignores `header.frame_id` (209-212). With `filter_frame:=fr3_link0`, which is the FR3 launch default and the `vision_standalone.py:22` example, a base-frame pose gets used as T_cam_marker.
5. **Symmetry:** nothing declares which degrees of freedom are observable or ambiguous.
6. Timestamp latency is guessed rather than measured.

## 3. Decisions and constraints

- **Stated goals:**
  - The marker is scaffolding and an ICP prior; keep the contract marker-agnostic; the connector part is undecided (memory only).
  - `SETUP_AND_CALIBRATION.md:369-395`: point-to-plane ICP against the CAD model with the marker as prior; the template origin at the mate point makes the connector offsets zero; synthetic accuracy about 0.1 mm / 0.1° yaw / 0.7° roll-pitch.
  - For real mating, the in-plane target becomes the connector keyway (`core.py:46-47`).
- **Prior attempts:** connector_pose, teach_offsets and vision_standalone (commit 4c45609, 2026-07-06), synthetic only. There is **no global initialiser**: ICP needs the marker prior (`connector_pose.py:223-228`). An ArUco board option exists but was never used on the cell (`TODO.md:286-289`).
- **Accuracy implied by insertion** (the connector is undecided, so these are proxies):
  - `fine_pos_tol` 2 mm / 1° (`fr3_params.yaml:51-52`).
  - The static friction floor is F/k: about 2 mm at 3000 N/m, 2.3-4.3 mm at 1500 (`TRACKING_SPEC.md:60-69`).
  - Lateral self-alignment needs a chamfer force above 3.5 N (`TODO.md:477-478`).
  - The angular deadband is 3.4° at k_rot 10, "the one to watch for keyed connectors" (479-481).
  - Force thresholds: contact 8 N, lateral 12 N, stroke 40 mm, overdrive 10 mm (`fr3_params.yaml:40, 67-71, 104`).
- **Hard constraints:**
  - **Images stay off DDS.** Only poses cross, about 2 KB/s (`tools/fr3/README.md:36-58`); point clouds are "NEVER" enabled (`realsense_low_bw.yaml:28`); the panel subscribes to the debug image only on demand (`core.py:213-220`).
  - The robot NIC is excluded from DDS (`README.md:59-70`).
  - **One process per camera.** The FR3 launch's cam_pub owns the D405, so any depth estimator must be embedded in the same process (the `vision_standalone`/external pattern) or replace it.
  - **CPU and RT:**
    - The 1 kHz FCI loop runs on this host; PREEMPT_RT; the performance governor is set.
    - Comm reflexes on 09-24 were caused by an unoptimised controller build; P-core pinning did not help (`tools/fr3/cell/README.md:178-188`).
    - Decoding the 1 kHz robot state in Python costs 86% of a core (`TODO.md:442-447`).
    - Running on battery breaks the 1 ms deadline (`GUIDE.md:24-28`).
    - TRACKING_SPEC O3 (a 50 Hz stream versus FCI) is still open (357-361).
  - **Close range:**
    - The D405 minimum depth is **not recorded in the repo**. Intel's spec is about 7 cm; verify it.
    - TRACK standoff is 100 mm; ALIGN targets 80-300 mm (`core.py:34`).
    - At the grasp, the optical centre is about 46.55 − 25 ≈ **22 mm above the top face and 61 mm off-axis** (`handeye.yaml:18`, `fr3_params.yaml:194`). The object is invisible there, which is why GRIP reads the pose once.
    - Base-frame sightings taken while moving carry v·dt error, hence the arm-still gate (`grip_node.py:201-212`).
- **Operator rules:**
  - Every motion goes behind a button (`PROJECT_STATE.md:172`; `TRACKING_SPEC.md:271`).
  - Predictions never drive committed motion (Decision 5, 243-256).
  - Measure before tuning (`TODO.md:339-359`; memory). The two "control failures" were bad measurements.
- **Open questions already written down:**
  - End-to-end latency; V1-V6.
  - O1 (raise k_rot), O2 (Ki), O3 (the 50 Hz stream and FCI).
  - The KF velocity feed-forward (`PROJECT_STATE.md:150-157`).
  - Hand-eye's 1.57° residual is above the "well under 1°" target (`TODO.md:39-42`).
  - C8, a single writer for `equilibrium_pose` (`cell/README.md:174-177`).
  - Real STL export and a D405 tuning pass (`TODO.md:285`).

## 4. Machine

- **GPU:** NVIDIA RTX 2000 Ada Laptop, 8 GB, 35 W cap. Driver 590.48.01 (CUDA 13.1); toolkits 12.6 and 13.1 in `/usr/local`. Idle at inspection (10 MiB used, Xorg only).
- **CPU:** Intel Core Ultra 9 185H, 22 threads: 6 P-cores with HT at 4.8-5.1 GHz, 8 E-cores at 3.8, 2 LP-E cores at 2.5. Kernel 6.8.0-rt8+. Governor `performance` on all threads. **No isolcpus or nohz_full** on the kernel command line.
- **RAM:** 30 GiB, 21 GiB available; 17 GiB swap.
- **Python 3.10.12:**

| Package | Status |
|---|---|
| open3d | 0.18.0; `o3d.core.cuda.is_available()` returns True |
| torch / torchvision | 2.9.1+cu128 / 0.24.1; `cuda.is_available()` True; installed with `--user` |
| cupy-cuda12x | 14.0.1 |
| numba (+ numba-cuda) | 0.61.2 |
| onnxruntime + onnxruntime-gpu | both 1.23.2, but they conflict: only the CPU and Azure providers are exposed |
| opencv-python | 4.12.0; ArucoDetector present; no CUDA build |
| scipy / sklearn / numpy | 1.15.3 / 1.6.1 / 1.26.4 |
| pyrealsense2 | 2.58.4 |
| Not installed | Python PCL bindings (pcl, pclpy, python_pcl), trimesh, kornia, pytorch3d, TensorRT, teaser++, small_gicp, probreg |

- **C++ side:** libpcl-dev 1.12.1 and ros-humble-pcl-ros / pcl-conversions 2.4.5 are installed.

## State of the art

Sources: I used web search and fetch (links at the end), plus my own knowledge where marked [K]. I also measured a few things on this PC [M]: small read-only scripts in the scratchpad, with no repo edits and no ROS, run after checking that no franka, controller or camera process was running.

# 6-DoF pose of a small known object from an eye-in-hand D405: state of the art and a recommendation for this cell

## 0. What the repo and this machine already have
- **Code already in the repo, not yet used by the cell.** `roscam/roscam/connector_pose.py` and `roscam/roscam/icp.py` already do marker-seeded point-to-plane ICP in numpy/scipy. They have quality gates (inlier fraction, rms, jump), fall back to the marker pose, filter through PoseKF, and publish PoseStamped on `/connector/pose`. The cell does not use them: `tools/fr3/fr3_params.yaml:20` sets `pose_topic: /aruco/pose`. Depth is already captured in-process (plane_normal uses it to resolve the IPPE flip). `rs_capture.py` and `cam_pub.py` default to 640x480 at 15 fps.
- **GPU.** NVIDIA RTX 2000 Ada Generation Laptop, 8 GB, capped at 35 W, driver 590.48, CUDA 13.1 plus a 12.6 toolkit. It works on this PREEMPT_RT kernel (6.8.0-rt8).
- **CPU and memory.** Intel Core Ultra 9 185H (22 threads), 30 GB RAM.
- **Software.** Ubuntu 22.04 / Humble. open3d 0.18.0 with CUDA available, torch 2.9.1+cu128 with CUDA, onnxruntime-gpu 1.23, pyrealsense2 2.58, libpcl-dev 1.12, pcl_ros, realsense2_camera. No TensorRT, TEASER++, SAM or FoundationPose installed.

## 1. D405 at close range (this limits everything that uses depth)
- **Sensor.** 18 mm baseline passive RGB stereo, global shutter, ideal range 7–50 cm. Colour comes from the depth imager, so RGB and depth are pixel-matched.
- **No projector, and an IR one won't help.** The D405 has no pattern projector and its imagers filter out IR (librealsense #10351), so an external IR projector does nothing. Only a visible-light pattern would add texture.
- **Absolute accuracy** is under 2% at 50 cm (up to about 10 mm); one listing gives ±1.4% at 20 cm. That is a bias along the viewing ray, separate from noise.
- **Per-pixel noise from theory [K]:** σz ≈ z²·0.08 px / (f·b).

| Resolution | 10 cm | 20 cm | 30 cm | 50 cm |
|---|---|---|---|---|
| 1280 wide (f ≈ 650 px) | ≈0.07 mm | ≈0.27 mm | ≈0.6 mm | ≈1.7 mm |
| 640x480 (today) | about 1.5–1.7x worse | | | |

  RealSense forums call 848x480 the optimal D405 depth mode.
- **Where it fails:** flying pixels at silhouettes, holes on uniform black plastic, wrong depth on shiny metal pins.
- **Consequence for a connector.** A 10–20 mm part at 25 cm is only about 20–35 px across and mostly edge. Final registration of a connector will need about 10–15 cm working distance.

## 2. The approaches
Accuracy figures are for this range. "This PC" means estimated unless marked [M].

**A. Plane segmentation + ICP, tracking by refinement (seeded by the marker, then by the last pose)**
- **Measured on this PC [M].** Synthetic 55 mm cube at 25 cm, 0.3–0.5 mm depth noise, starting 5.8 mm / 7.2° off. These are ideal conditions: no flying pixels, no table.

| Method | Time per frame | Error after |
|---|---|---|
| Repo numpy point-to-plane | 14 ms | 0.2 mm / 0.07° |
| Repo numpy point-to-point | 30 ms | 1.3 mm / 0.14° |
| Open3D point-to-plane, CPU | 2.4 ms | 0.4 mm / 0.05° |
| Open3D tensor, CUDA | 6 ms | slower than CPU at 2k points; no GPU needed |

- **Pitfall found [M].** Calling Open3D with model = source and partial scene = target barely moves the pose (still 5.8 mm / 4.6°). Register scene to model, as the repo already does.
- **Top face only [M].** When the camera sees only the top face (camera-centred TRACK looking straight down), rotation error rises from 0.04° to about 1.2°. Lateral position and yaw then come only from edge points, which are exactly where the D405 is worst.
- **Expected real accuracy [K]:** about 0.5–2 mm and 0.5–2°.
- **Failure modes:** starting error outside the convergence basin (roughly over 1/3 of object size or over ~15°); table points left in the crop; holes on black or shiny parts; symmetry slip (harmless for a cube).
- **Possible gap in the existing node (untested).** `connector_pose.py` crops a 5 cm sphere and does no plane removal, and its inlier fraction counts table points. It may therefore keep falling back to the marker. Check this on a recording.
- **Maturity:** very high (Open3D, PCL).

**B. Coloured ICP (Park 2017).** Adds a photometric term that constrains motion along the surface. It helps only if the object is textured. A plain cube or a uniform black housing gains almost nothing [K].

**C. Global registration without a prior (FPFH+RANSAC, TEASER++)**
- Measured FPFH+RANSAC+ICP on the cube: 90–110 ms on CPU [M].
- It landed on a pose rotated 120°, which is one of the cube's 24 equivalent orientations; symmetry-aware error was 1.2°.
- On small objects with flat faces FPFH descriptors carry little information, so the result is inherently symmetric or ambiguous. It needs segmentation first.
- TEASER++ (C++/Python, build from source, no Humble package) runs in milliseconds and is worth it only when correspondences have high outlier rates.
- Use: initialising or re-acquiring the pose, not per-frame tracking.

**D. PPF (Drost / Vidal)**
- Depth-only; won BOP 2017 and 2019; overtaken by learned methods in 2020.
- About 0.1–1 s per image or more; weak on symmetric or tiny objects.
- Available in OpenCV-contrib `surface_matching` (not in opencv-python); HALCON is the commercial equivalent [K].
- Use: initialisation only.

**E. ICG / M3T (DLR, region + depth tracker)**
- Combines silhouette and depth; needs only the geometry.
- 1.3 ms per frame on one CPU core; beat se(3)-TrackNet on YCB-Video, OPT and Choi (2022).
- Needs a starting pose; C++, MIT licence, no ROS wrapper.
- Suits textureless or black parts that contrast with the background, because the silhouette carries the lateral and yaw information depth lacks.

**F. FoundationPose (NVIDIA, CVPR 2024)**
- **Data needs:** CAD mesh (OBJ/PLY) or reference views; no per-object training. Registration needs a mask; tracking refines from the last pose and needs no mask.
- **Accuracy:** YCB-Video ADD-S / ADD AUC 97.4 / 91.5. An industrial evaluation from a single RGB-D frame found median translation errors of 0.6–2.8 mm and median rotation error of about 2.8° on plane-symmetric parts.
- **Speed and memory:**

| Source | Registration | Tracking |
|---|---|---|
| Inference library, RTX PRO 6000, FP16 | 148 ms | 2 ms |
| Isaac ROS node, 720p | RTX 5090: 5.16 fps; RTX 5070: 3.32 fps | not listed |
| NGC TensorRT figures, RTX 4060 Ti (networks only) | 8.7 QPS | 1599 QPS |

  Isaac ROS peaks at about 7 GB in FP32 and recommends at least 8 GB.
- **Estimate for this 35 W, 8 GB GPU:** registration about 0.5–3 s, tracking plausibly 30 Hz or more, but it needs FP16 and a smaller batch (the library suggests 42–126). This must be measured.
- **Failure modes:** loses track under fast motion outside the refiner's range, long occlusion or re-appearance (RRTrack 2026, DynamicPose 2025); textureless symmetric parts; poor depth.
- **Humble path:** Isaac ROS 3.2 (Dec 2024) is the last Humble / 22.04 release. 4.x is Jazzy / Ubuntu 24.04 with CUDA 13 and driver 580+; 5.0 (21 Sep 2026) is Lyrical.
- **Easier path:** `nvidia-isaac/foundation-pose-inference-library`. Apache-2.0 code, Python/C API, no ROS needed, container-based, compute capability 7.5+ and driver 580+ (this GPU is Ada, driver 590). Weights come from HF `nvidia/foundationpose`; check their licence terms. The original NVlabs repo is under the NVIDIA Source Code License (non-commercial).
- **Symmetry:** Isaac ROS takes symmetry axes as parameters (e.g. `x_30`, `full`).

**G. MegaPose / GigaPose**
- New objects without training. MegaPose takes about 1.5 s per detection; GigaPose's coarse step takes 48 ms and 0.26 s per detection overall.
- BOP AR 57.9 for GigaPose with MegaPose refinement.
- Initialisers only, not 15–30 Hz trackers. Both need a 2D detection first.

**H. GDRNPP / CosyPose (seen objects, trained per object)**
- GDRNPP won BOP 2022; its fastest variant takes 0.23 s per image and it trains on BlenderProc PBR synthetic images.
- It is accurate once trained, but training per object costs a lot and is tight on 8 GB. That is worth it only once the connector is fixed and the part will be used long-term. CosyPose is the older version of this idea.
- **BOP trend:** the best unseen-object methods of 2024 are accurate but slow (FreeZeV2.1: 24.9 s per image; Co-op: 0.8 s). 2D detection is the main bottleneck.

**I. Segmentation front ends**
- **Plane RANSAC + Euclidean clustering / DBSCAN:** a few ms on CPU; enough for one object on a table.
- **SAM2.1 tiny:** 8.8 fps on V100. **EfficientTAM:** 20 fps. **Mobile SAM (Isaac):** 57.9 fps on RTX 5070.
- SAM needs a prompt; the marker or the depth-cluster centroid can supply it.
- When tracking, render the mask from the last pose instead of segmenting every frame.

**J. Symmetry**
- **Cube:** 24 equivalent orientations. Pick the one closest to the previous pose or the gripper's current yaw; any of them works for GRIP and PLACE.
- **Connector:** likely 2-fold symmetric apart from a key, and a 180° error means a jam. Resolve it once, using the marker, a key-feature check or operator confirmation on the panel, then rely on tracking continuity.
- Evaluate with ADD-S / MSSD.

**K. The kinematic prior (the biggest win for eye-in-hand)**
- The object is static in the base frame and the camera moves by known forward kinematics. Predict the object's camera-frame pose each frame from the arm's motion: T_cam(t)⁻¹·T_cam(t−1)·T_obj(t−1).
- This gives a better prior than DynamicPose's visual-inertial approach. It keeps ICP or FoundationPose inside their convergence range during fast arm motion.
- It needs camera and joint-state timestamps aligned.

## 3. Comparison

| Approach | Accuracy at 10–30 cm | Rate on this PC | Needs | Main failure | Maturity / Humble |
|---|---|---|---|---|---|
| ArUco (today) [K] | ~1–2 mm lateral, worse in depth, 1–3° | 15 Hz | marker | marker hidden, IPPE flip (fixed) | in use |
| Plane seg + p2plane ICP, marker/FK/last-pose prior | synth 0.2–0.7 mm / 0.05–1.2° [M]; real ~0.5–2 mm / 0.5–2° | 2.4 ms (Open3D) / 14 ms (repo) [M] | CAD/STL | convergence basin, top-face-only view, holes, table in crop | very high; Open3D/PCL installed |
| Coloured ICP | as above, better along surfaces if textured | ~ICP | textured model | untextured parts | high; Open3D |
| FPFH+RANSAC / TEASER++ (+ICP) | after ICP, same as ICP up to symmetry | 90–110 ms [M] | CAD, segmented cloud | weak features on small flat parts, symmetry | high; TEASER++ built from source |
| PPF | ~mm after ICP | 0.1–1 s+ | CAD | symmetric/tiny parts, clutter | high; OpenCV-contrib |
| ICG / M3T | near-SOTA tracking on YCB-V | 1.3 ms / core | CAD | needs start pose, needs contrast | research code, MIT, no ROS |
| FoundationPose tracking | ~1–3 mm / ~1–3° (lit.) | est. 30 Hz+, 8 GB tight | CAD mesh (texture helps) | fast motion, occlusion, symmetric textureless | Isaac ROS 3.2 (Humble) or inference lib |
| FoundationPose registration | same | est. 0.5–3 s | + mask | ambiguous / symmetric | same |
| MegaPose / GigaPose | lower than FoundationPose | 0.3–1.5 s+ | CAD + detection | slow | research |
| GDRNPP / CosyPose | best once trained | ~0.2 s | CAD + synthetic training per object | training cost, 8 GB | research |
| Depth clustering / SAM2 / EfficientTAM | mask only | ms / ~10–30 fps est. | prompt | clutter / needs prompt | high / research |

## 4. Recommendation for this cell

**Now, for the cube:** stay classical and build on `connector_pose.py`.
1. Seed from the marker the first time only. After that, each frame's prior is the last pose predicted forward by the arm's kinematics.
2. Crop around that prior and remove the table plane with RANSAC.
3. Run Open3D point-to-plane ICP, scene to model, at 2–3 ms (or keep the numpy version at 14 ms).
4. Recompute the gates on object points only.
5. Snap to the nearest of the cube's 24 symmetric orientations, then filter with PoseKF.
6. Publish on the same PoseStamped contract (a `/object/pose` + `/object/pose_raw` pair), so TRACK, GRIP and PLACE don't care where the pose came from.
7. Prefer oblique views showing 2–3 faces.
8. Move capture to 848x480 at 30 fps.
9. Before any robot run, log ICP against the marker at full rate and compare their bias and spread in a table.
10. Removing the marker altogether: start from depth clustering plus FPFH+RANSAC (measured ~0.1 s) and let the operator confirm the pose in the panel.

**Later, for the connector.** First run a depth-quality test on the actual part at 10, 20 and 30 cm.
- If depth is mostly valid with about 1 mm noise, keep the ICP path.
- If the part is black or shiny, move to FoundationPose tracking. Use the Apache-2.0 inference library in a container, or Isaac ROS 3.2 on Humble. Seed it with a marker- or depth-cluster-prompted mask, use FP16 with a small batch, and measure VRAM and Hz first.
- Or try ICG/M3T on the CPU.
- Resolve the connector's key orientation once with the operator in the loop. Let the impedance stroke absorb the remaining millimetres.
- Skip per-object trained methods until the part is fixed.
- Keep perception off the real-time cores, because this PC also runs the robot's 1 kHz control loop.

**Files:**
- Repo: /home/local/ISDADS/ses634/fabling/Robotic Connector Handling/src/roscam/roscam/connector_pose.py, .../roscam/roscam/icp.py, .../roscam/roscam/rs_capture.py, .../tools/fr3/fr3_params.yaml
- Benchmark scripts: /tmp/claude-783149324/-home-local-ISDADS-ses634-fabling-Robotic-Connector-Handling-src/aa8a5fb3-b648-4c07-830f-fd41ce302376/scratchpad/bench_icp.py, bench2.py, bench3.py

**Sources:**
- [isaac_ros_foundationpose](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html)
- [Isaac ROS performance](https://nvidia-isaac-ros.github.io/performance/index.html)
- [Isaac ROS releases](https://nvidia-isaac-ros.github.io/releases/index.html)
- [Isaac ROS 4.0 getting started](https://nvidia-isaac-ros.github.io/v/release-4.0/getting_started/index.html)
- [foundation-pose-inference-library](https://github.com/nvidia-isaac/foundation-pose-inference-library)
- [NGC FoundationPose](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/foundationpose)
- [NVlabs FoundationPose](https://github.com/NVlabs/FoundationPose)
- [FoundationPose paper](https://arxiv.org/abs/2312.08344)
- [FoundationPose industrial evaluation](https://www.researchsquare.com/article/rs-7992985/v1)
- [RRTrack](https://arxiv.org/html/2607.23669)
- [DynamicPose](https://arxiv.org/pdf/2508.11950)
- [BOP 2024](https://arxiv.org/abs/2504.02812)
- [BOP challenges](https://bop.felk.cvut.cz/challenges/)
- [GigaPose](https://arxiv.org/pdf/2311.14155)
- [GDRNPP](https://github.com/shanice-l/gdrnpp_bop2022)
- [BOP 2020 (PPF)](https://arxiv.org/pdf/2009.07378)
- [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)
- [ICG](https://arxiv.org/abs/2203.05334)
- [DLR 3DObjectTracking](https://github.com/DLR-RM/3DObjectTracking)
- [EfficientTAM](https://openaccess.thecvf.com/content/ICCV2025/papers/Xiong_Efficient_Track_Anything_ICCV_2025_paper.pdf)
- [TinySAM 2](https://arxiv.org/pdf/2605.18013)
- [D405 product page](https://www.realsenseai.com/products/stereo-depth-camera-d405/)
- [D405 on FRAMOS](https://framos.com/products/3d/3d-cameras/realsense-depth-camera-d405-starter-kit-26293/)
- [librealsense #10351](https://github.com/realsenseai/librealsense/issues/10351)
- [D405 optimal resolution thread](https://support.intelrealsense.com/hc/en-us/community/posts/18383924538515-Optimal-Depth-Resolution-for-D405)
- [Open3D ICP](https://www.open3d.org/docs/0.18.0/tutorial/t_pipelines/t_icp_registration.html)
- [B2TFPose](https://arxiv.org/abs/2609.06726)
