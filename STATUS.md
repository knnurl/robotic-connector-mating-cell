# Project Status — Robotic Connector Mating Cell

*Written 2026-07-12. Comprehensive snapshot of what exists, what is proven,
how to use it at the current stage, and what remains. Companion docs:
[SETUP_AND_CALIBRATION.md](SETUP_AND_CALIBRATION.md) (reference manual),
[TODO.md](TODO.md) (task list), [HANDOFF.md](HANDOFF.md) (original audit,
2026-07-05, now historical).*

---

## 1. One-paragraph state of the world

A vision-guided connector-mating cell: an eye-in-hand camera locates a
connector (ArUco marker, optionally refined by depth-ICP against CAD), and a
robot-agnostic C++ controller drives a phased mating sequence
(`WAIT_FOR_VISION → ALIGN_COARSE → ALIGN_FINE → INSERT → MATED`, with
`FAULT` and latched recovery) through MoveIt. The original target was a
Mitsubishi **RV-5AS**; the active development target is now a **Franka FR3**
on this PC (`~/franka_ros2_ws`, PREEMPT_RT). Over recent sessions the system
gained: time-correct transforms, a prediction-proof arming gate, a
Kalman-filtered vision path that can run **entirely outside ROS** (to protect
the FR3's 1 kHz control loop from image-bandwidth starvation), force-aware
insertion, a Cartesian-path stroke for Pilz-less robots, an opt-in
`moveit_servo` alignment mode, a **Cartesian-impedance insertion backend**
for compliant self-aligning mating, auto-taught connector offsets, multi-
marker board support, a pure unit-tested phase machine, and a consolidated
launch.

**Updated 2026-09-11 — the robot has now moved.** The FR3 ran under
closed-loop vision control on real hardware: FCI active, MoveIt +
ros2_control live, the arm driven from camera measurements via
`tools/fr3/align_gui.py` (a human-in-the-loop alignment panel added this
session). Camera-relative position alignment converges to ~1 mm, and the
marker-orientation measurement — previously unusable — is now trustworthy
after fixing two stacked defects in it (IPPE mirror-solution flipping at
62% of frames, and a 2.6 deg out-of-plane magnitude bias; both detailed in
TODO.md "Lessons"). The camera now runs at 90 fps with depth.

What is still NOT proven: **mating**. Hand-eye *rotation* is calibrated
empirically, but `handeye_xyz` (translation) remains the dry-run guess, so
connector offsets, standoff and insertion depth are all untaught. The
Cartesian-impedance backend has still never run on hardware. And the FCI
link currently shows ~4.7% packet loss, which has killed the stack twice
mid-motion and blocks reliable robot work until resolved. `handeye_calib`
for the translation is the next action.

---

## 2. Capability inventory (what exists now)

### Vision — `roscam` (Python)
- **`cam_pub`** — ArUco 6-DOF pose publisher. New-API + legacy fallback,
  `SOLVEPNP_IPPE_SQUARE`, reprojection-gated, Kalman-filtered (constant-
  velocity + small-angle, innovation gate, ≤0.3 s dropout prediction).
  Publishes `/aruco/pose` (filtered), `/aruco/pose_raw` (optical frame,
  always — hand-eye + arming depend on it), `/aruco/debug_image`.
  - **`filter_frame`** — run the KF in a fixed frame (re-express detections
    via TF at the image stamp) so eye-in-hand robot motion doesn't look
    like marker motion to the gate.
  - **`source: topic | realsense | external`** — `realsense` captures
    in-process via `pyrealsense2` (no image ever hits DDS), `external` is
    fed by an embedder.
  - **Multi-marker board** (`board_markers_x/y > 1`) — grid board, pose =
    board centre, any visible subset suffices (occlusion-robust).
- **`connector_pose`** — depth-ICP refinement of the connector pose against
  a CAD STL, marker pose as prior, quality-gated with graceful fallback.
  Same `source` switch (own in-process depth capture).
- **`vision_standalone`** — one process owns the camera and runs ArUco +
  depth-ICP together, publishing only poses. The complete "camera outside
  ROS, only end-data in" path.
- **`rs_capture`** — ROS-free RealSense wrapper with a hardware self-test
  CLI (`python3 -m roscam.rs_capture --seconds 5 --depth`).
- **`handeye_calib`** — eye-in-hand calibration (4 solvers, consistency
  ranking, prints a ready-to-paste static TF).
- **`teach_offsets`** — auto-teaches `connector_offset_*` + yaw by pairing
  `/aruco/pose_raw` with `/connector/pose` (replaces the caliper loop).

### Motion control — `melfa_rv5as_masterclass` (C++)
- **`move_l`** (`connector_mating_node`) — the controller. Subscription
  stores latest pose only; all planning/execution in a dedicated control
  thread. TF looked up at the **image timestamp**; staleness by header
  stamp; insertion armed only on fresh **raw** detections (predictions
  steer, never arm).
  - **Phase machine** extracted to [`mating_phase_machine.hpp`](melfa_rv5as_masterclass/include/melfa_rv5as_masterclass/mating_phase_machine.hpp)
    — pure, header-only, 14 gtests covering the whole transition matrix.
  - **Insertion backends** (`insert_backend`): `moveit` (default;
    `insert_planner: pipeline` = Pilz LIN, or `cartesian` =
    `computeCartesianPath` for Pilz-less robots) or `impedance` (see below).
  - **Force-aware stroke** (`wrench_topic`): axial reaction ⇒ seated
    (MATED) or jam (FAULT), lateral load ⇒ snag (FAULT); shared thresholds
    across backends.
  - **Alignment modes** (`align_mode`): `step` (default) or `servo`
    (publishes Cartesian twists for `moveit_servo`; INSERT stays discrete).
  - **Recovery**: `~/stop`, `~/pause`/`~/resume` (resume covers remaining
    stroke depth), `~/reset` (refused mid-interrupted-INSERT), `~/retract`
    (pull back along the stroke, then restart).
  - **State out**: `/mating/phase`, `/mating/paused`, `/mating/error_mm`,
    `/mating/error_deg`, `/diagnostics`.
- **`mating_geometry.hpp`** — all pose math (standoff goal, clamps,
  insertion/retract axis, servo twist, wrench axial/lateral split); 18
  gtests, ROS-free.
- **`plc_` / `hmi_`** — MELFA GPIO/gripper/safety demos, conditional build
  (only where `melfa_msgs` exists).
- **`cell.launch.py`** — consolidated: hand-eye TF + vision + controller
  (+ optional `connector_pose`) in one command.

### Compliant insertion — `fr3_mating_controllers` (C++, FR3-only)
- **`CartesianImpedanceStrokeController`** — `franka_ros2` ControllerInterface
  plugin: `τ = Jᵀ(K·Δx − D·J·q̇) + τ_nullspace + coriolis` at 1 kHz, `K`
  diagonal in the **tool frame** (soft lateral X/Y + roll/pitch = self-
  align, firm tool-Z = stroke drive), slew-limited equilibrium from
  `~/equilibrium_pose`, torque-rate saturation, `float_mode` commissioning
  switch. `move_l` switches it in for INSERT, streams the equilibrium along
  the tool axis, judges seating by the wrench, hands the arm back.

### FR3 integration — `tools/fr3/`
- `fr3_mating.launch.py` (cell side), `fr3_params.yaml` (safe OMPL +
  Cartesian stroke + force guard defaults, insertion off), `fr3_servo.yaml`,
  `cyclonedds_fr3.xml` (**interface isolation** — the bandwidth fix),
  `realsense_low_bw.yaml`, `fr3_preflight.sh` (RT/latency/DDS/bandwidth
  gate). Mock dry-run harness in `tools/dryrun_fr3/`.

### Operator UIs — `tools/gui/`
Foxglove layout, zero-install tkinter panel (YAML-configured buttons incl.
Retract), rqt recipe. All bind the state topics + Trigger services.

---

## 3. Verification matrix — what is and isn't proven

| Component | Status |
|---|---|
| C++ build, `-Wall -Wextra -Wpedantic` | ✅ clean |
| Geometry + phase-machine logic | ✅ **32 gtests** |
| Vision KF, fixed-frame, auto-teach math | ✅ **20 pytests** |
| Vision end-to-end (synthetic marker → pose) | ✅ 0.3 mm; board incl. half-occluded |
| Out-of-ROS capture switch (topic/external) | ✅ synthetic; ❌ real `pyrealsense2` |
| `fr3_mating_controllers` | ✅ compiles + plugin-exported vs `franka_ros2`; ❌ never run |
| **Controller vs live move_group (any robot)** | ❌ **never run** — the top risk |
| Marker-frame flip vs real TCP | ❌ first RViz run will show it |
| Hand-eye on the real cell | ❌ tool ready, not done |
| Connector offsets | ❌ **all zero** — must be taught (auto or manual) |
| Force-guard thresholds | ❌ need one manual mate to tune |
| Impedance stroke, servo align | ❌ real-arm commissioning only (fake HW can't) |
| `communication_constraints_violation` avoided end-to-end | ❌ unverified |

**Bottom line:** the code is mature and well-tested at the unit level; the
entire risk surface is *runtime behavior on hardware*, which is untouched.

---

## 4. How best to use the project at the current stage

You are pre-first-run. The goal now is to **close the verification gap
safely**, not to attempt a real mate. Recommended path:

### 4.1 If you have the FR3 (this PC) — the active path
1. **Preflight.** `eno1` on the FCI subnet, robot connected, then
   `tools/fr3/fr3_preflight.sh 172.16.0.3` until fully green (RT kernel,
   <1 ms ping, DDS isolation, no pointcloud topics). Export
   `CYCLONEDDS_URI=…/tools/fr3/cyclonedds_fr3.xml` in every terminal.
2. **Fake-hardware phase check (biggest single win).**
   `moveit.launch.py use_fake_hardware:=true` +
   `fr3_mating.launch.py use_fake_hardware:=true`. Watch the phase machine
   advance and motion directions in RViz. Insertion stays disabled. This is
   the first-ever live controller run and retires the top risk.
3. **Hand-eye.** `ros2 run roscam handeye_calib -p base_frame:=fr3_link0
   -p tcp_frame:=fr3_hand_tcp` (run `cam_pub` with `filter_frame:=''`
   during collection). Pass results via `handeye_xyz`/`handeye_quat`.
4. **Teach offsets.** Either auto (`connector_pose` with your STL +
   `fallback_to_prior:=false`, then `ros2 run roscam teach_offsets`) or
   manual (setup guide §2.4). Fill `tools/fr3/fr3_params.yaml`.
5. **Validation ladder** (setup guide §4): real HW, insertion **disabled**
   → confirm it hovers dead-centre, occlude the marker → confirm hold →
   only then enable insertion at `insert_speed: 0.02` with the **moveit**
   backend + force guard. Tune `contact_force_n` from a manual mate first.
6. **Compliant insertion (last).** Only after the force-guarded moveit
   stroke works, follow `fr3_mating_controllers/README.md` commissioning
   ladder (float → hold → setpoint → dispatched `insert_backend: impedance`).

### 4.2 If you have the RV-5AS
Same shape, on the robot PC (where `plc_`/`hmi_` build): copy repo,
`colcon build`, fake-hardware dry run via `move_l.launch.py` /
`cell.launch.py`, hand-eye, teach, ladder. `insert_backend` stays `moveit`
(no wrench source unless F/T hardware is fitted); Pilz LIN is available so
keep `insert_planner: pipeline`.

### 4.3 Bandwidth-safe vision (FR3, whenever the camera is live)
Prefer **not generating** image traffic on DDS: `vision_source:=realsense`
(or `ros2 run roscam vision_standalone` for marker+ICP on one camera).
Keep `cyclonedds_fr3.xml` isolation as the backstop. Verify with
`python3 -m roscam.rs_capture --seconds 5 --depth` first.

### 4.4 Golden rules (carried from the audit — do not regress)
- Never plan/execute in a subscription callback. Never bypass the step
  clamps. Every tunable lives in the params YAML, not code.
- INSERT is a *committed* stroke; marker occlusion by the gripper mid-stroke
  must not abort it. Fix the tool-Z-into-surface convention only in
  `mating_geometry::standoff_goal` (+ its tests), nowhere else.
- Keep `mating_geometry.hpp` and `mating_phase_machine.hpp` pure (no ROS)
  so they stay unit-testable.

---

## 5. Architecture snapshot

```
                 camera (USB, D405)
                        │
   ┌────────────────────┴─────────────────────┐   [vision_source:=realsense
   │  roscam: cam_pub (+ connector_pose ICP)   │    ⇒ in-process, no image
   │  Kalman filter, optional fixed-frame       │    topics on DDS]
   └────────────────────┬─────────────────────┘
        /aruco/pose[_raw] │ /connector/pose   (~2 KB/s — the only vision on DDS)
                          ▼
   ┌──────────────────────────────────────────┐
   │  move_l (connector_mating_node)           │
   │  ┌────────────────────────────────────┐   │   TF @ image stamp
   │  │ mating_phase_machine.hpp (pure)    │   │   wrench guard
   │  │ WAIT→COARSE→FINE→INSERT→MATED/FAULT │   │
   │  └────────────────────────────────────┘   │
   │  align: step | servo                       │
   │  insert: moveit(pipeline|cartesian) |      │
   │          impedance ──────────────────────┐ │
   └───────────────┬──────────────────────────┼─┘
        LIN/Cartesian goals │                  │ switch_controller +
                            ▼                  ▼ ~/equilibrium_pose
                     move_group (MoveIt)   fr3_mating_controllers
                            │              (1 kHz Jᵀ impedance, FR3)
                            ▼                  │
                       robot driver ◀──────────┘  (JTC ⇄ impedance, exclusive FCI)
```
State/services (`/mating/*`, `~/stop|pause|resume|reset|retract`) → GUIs.

---

## 6. Where the task list lives

The full prioritized, checkbox TODO — critical path, FR3 ladder, MELFA
ladder, improvements, code quality, housekeeping — is in
**[TODO.md](TODO.md)**. It is kept current; this document explains the
*why* and *how*, TODO.md tracks the *what next*.
