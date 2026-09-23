# Running the connector-mating cell on a Franka FR3

Scaffolding for the **real** FR3 (the mock dry-run harness is parked in
[melfa/parked/dryrun_fr3/](../../melfa/parked/dryrun_fr3/)). The mating stack itself is unchanged —
the controller is robot-agnostic — this directory supplies the FR3 wiring,
safe parameters, and the network/RT hardening the FR3 specifically needs.

| File | Purpose |
|---|---|
| `fr3_env.sh` | Source in every terminal: ROS 2 + `~/franka_ros2_ws` + this workspace, DDS pin, `FR3_ROBOT_IP`, `FR3_LOG_DIR`; defines `fr3_preflight` and `fr3_cell` |
| `fr3_preflight.sh` | Read-only checks: RT kernel / power / latency / Desk / DDS isolation / bandwidth / this workspace / camera. Exit code = FAIL count |
| `fr3_cell.launch.py` | Terminal 2: impedance controller spawned inactive + hand-eye TF + `cam_pub` (KF in `fr3_link0`) + `tracking_node` (idle until TRACK) + the panel |
| `cell_panel.py` | The operator panel: ALIGN, the IMPEDANCE ladder, TRACK. Every motion is a button press |
| `theme.py` | The panel's colour palette |
| `state_relay.py` | C++ `topic_tools throttle` child that relays the 1 kHz robot state to the panel at 50 Hz |
| `analyse_trace.py` | Summarises panel (`cell_*`) and tracking (`tracking_*`) traces; with no argument, the newest under `$FR3_LOG_DIR` |
| `sim/tracking_smoke.py` | No-robot smoke test: the real `tracking_node` in a fake cell on isolated DDS domain 87 (run by `tools/run_tests.sh`) |
| `fr3_params.yaml` | Node params (`tracking_node`, and the parked `mating_node`): `fr3_arm`/`fr3_hand_tcp`, OMPL, reduced speeds/depth, **insertion disabled by default** |
| `cyclonedds_fr3.xml` | DDS interface isolation + buffer tuning (the bandwidth fix) |
| `realsense_low_bw.yaml` | D405 config: colour-only, 640×480@15, **no pointcloud**. This 15 is also the `/aruco/pose` rate in topic mode — see "Continuous marker tracking" |
| `calib/` | `handeye.yaml`, the one hand-eye calibration (read by `fr3_cell.launch.py` and the panel), and the samples it was solved from |
| `setup/` | One-time PC setup: `99-realsense-no-suspend.rules` (stops the D405 USB-autosuspending; install steps in its header) |
| `test_*.py`, `conftest.py` | pytest suite: `python3 -m pytest tools/fr3` |

## Why ROS 2 (franka_ros2), not raw libfranka

The controller speaks MoveIt and TF only; `franka_ros2`'s hardware
interface already runs the 1 kHz FCI loop in its own real-time thread and
exposes the arm to MoveIt. Driving libfranka directly would mean rewriting
the phase machine against Cartesian impedance control — the right move
*later* for continuous servoing (see "Future"), not for getting the cell
running. The FCI stability problem you hit is not caused by ROS; it is
caused by what shares the network with FCI, and that is fixable (below).

## The bandwidth problem, and the actual fix

**Failure mode:** libfranka must complete a UDP round trip to the robot
every **1 ms**. When camera/pointcloud DDS traffic contends with that loop
— on the same NIC, or via kernel-level packet storms (fragment drops →
retransmits) — the deadline is missed and the arm stops with
`communication_constraints_violation`.

**Numbers** (why "just lower the FPS" isn't enough): raw colour VGA@30 ≈
26 MB/s, aligned depth ≈ 18 MB/s, an XYZRGB pointcloud ≈ 30–90 MB/s. One
Foxglove image panel over WiFi can saturate the link; a pointcloud
subscriber anywhere is worse.

**Defence in depth (all five, strongest first):**

0. **Don't create the traffic** — capture frames *outside* ROS and let
   only poses (~2 KB/s) cross into the graph:
   `ros2 run roscam vision_standalone` (owns the camera in-process, runs
   ArUco + optional depth-ICP on the same frames, publishes only
   `/aruco/pose[_raw]` + `/connector/pose` + a ≤5 Hz subscribe-gated
   debug image). Or per-node: `cam_pub`/`connector_pose` with
   `source:=realsense`. Needs `pip install pyrealsense2`; verify the
   camera path first with `python3 -m roscam.rs_capture --seconds 5`.
   This is `fr3_cell`'s default (`vision_source:=realsense`): no camera
   driver runs at all.
1. **Physical isolation** — the robot gets a dedicated wired NIC
   (here: `eno1`, e.g. PC `172.16.0.1/24` ↔ robot `172.16.0.2`). Nothing
   else on that subnet. Camera is USB (D405), so capture never touches
   any network.
2. **DDS interface pinning** — `cyclonedds_fr3.xml` lists the interfaces
   DDS may use (`lo`, optionally WiFi). The robot NIC is *not* listed, so
   no topic can ever ride that link. Export in **every** terminal:

   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file://$(pwd)/tools/fr3/cyclonedds_fr3.xml
   ```

   (You already run a tuned Cyclone config for the fr3_act project; this
   one adds the `<Interfaces>` isolation on top of the same buffer tuning.
   Only one `CYCLONEDDS_URI` can be active per process — use this one for
   the mating cell.)
3. **Publish less** — `realsense_low_bw.yaml`: colour-only 640×480@15,
   pointcloud off, depth off unless `connector_pose` is in use. The
   detector needs ~10 Hz of VGA colour; everything else is waste. The
   vision node already publishes `/aruco/debug_image` only while someone
   subscribes.
4. **RT hygiene** — PREEMPT_RT kernel (this PC: 6.8.0-rt8), `rtprio 99`
   in `/etc/security/limits.conf`, performance governor. `fr3_preflight.sh`
   checks all of it, plus ping max-latency to the robot and live
   pointcloud topics.

Remote GUIs: run Foxglove/rqt over the **WiFi** interface only (uncomment
it in `cyclonedds_fr3.xml`), subscribe to `/mating/*` scalars and
`/diagnostics` freely — but treat remote raw-image viewing as a
debugging-only act, never during mating.

## Launch order

Two terminals, `fr3_preflight` + driver in T1 and `fr3_cell` in T2:
[GUIDE.md section 2](../../GUIDE.md#2-bring-up), the only maintained copy.

## Control upgrades active in the FR3 profile

`fr3_params.yaml` enables two controller features the MELFA profile leaves
off, plus one opt-in:

- **Force-aware insertion** (`wrench_topic` = the FR3's estimated external
  wrench). The stroke stops on contact instead of running blind: axial
  reaction ≥ `contact_force_n` after ≥ `min_contact_depth_m` of travel
  = seated → MATED; contact earlier = obstruction → FAULT (`~/retract`);
  lateral load ≥ `max_lateral_force_n` anywhere = snag → FAULT. Tune the
  thresholds by watching the wrench topic during one manual mate — the
  estimate carries a few N of bias, so keep `contact_force_n` above that.
  A stroke that reaches full depth *without* contact force logs a warning
  (probably missed the connector).
- **Cartesian stroke planner** (`insert_planner: cartesian`): the FR3
  MoveIt config has no Pilz, so INSERT/retract use `computeCartesianPath`
  (straight interpolated path, TOTG-timed at `insert_speed`) instead of an
  OMPL plan that only promises the endpoints.
- **Compliant insertion** (opt-in, `insert_backend: impedance`): the
  stroke runs on a Cartesian-impedance torque controller
  ([fr3_mating_controllers](../../fr3_mating_controllers/README.md)) —
  soft lateral, firm axial, so the connector self-aligns under contact.
  `mating_node` switches controllers around the stroke and judges seating by
  the wrench. Commission via that package's ladder (float → hold →
  setpoint → dispatch), only after the force-guarded moveit stroke works.
- **Continuous marker tracking** (opt-in, off until asked;
  `mating_controller`'s `tracking_node`, specified in
  [TRACKING_SPEC.md](../../TRACKING_SPEC.md)): instead of stepping a MoveIt
  plan per cycle, the node streams `~/equilibrium_pose` at 50 Hz so the arm
  *follows* the marker on the impedance controller. The goal comes from the
  same `mating_geometry::standoff_goal` the stepped backend uses; a bounded
  integrator of the **measured** error pushes the equilibrium past that goal
  until the friction residual `F/k` closes, because the controller cannot
  know it is stuck and only vision can. **Unit-tested and reviewed, never run
  on the arm** — treat every number in this bullet as a prediction until the
  spec's V1–V6 have been done on the arm.
  - It comes up **idle** and is started from the panel's TRACK button
    (IMPEDANCE & TRACK tab) and stopped by STOP TRACKING in the panel header,
    reachable from every tab. Both call `/tracking_node/start_tracking` /
    `stop_tracking` (std_srvs/Trigger); `/tracking_node/status` reports what
    it is doing.
  - Starting it **snapshots the controller's live gains**, applies the
    `track_*` profile from `fr3_params.yaml` atomically, and stopping puts
    the snapshot back. It refuses to start if it cannot read what it would
    have to restore, if `float_mode` is on, or if the `fr3_hand_tcp` →
    controller-EE offset cannot be measured from TF and `o_t_ee` together.
  - The `track` profile raises the **now-live** `setpoint_slew_mps` /
    `setpoint_slew_rps` to 0.10 / 0.5 (`track_setpoint_slew_*`), because the
    slew, not the stiffness, is the speed limit — a 50 mm step took 1.05 s
    against a 0.96 s slew floor. `ConfigLimits` still caps them at
    0.25 m/s / 1.0 rad/s, and that bound is unchanged.
  - **Vision rate matters here.** `/aruco/pose` is one pose per captured
    frame, so in topic mode it is the 15 fps of `realsense_low_bw.yaml` and
    in `source:=realsense` mode it is `capture_fps`, which also defaults to
    15. The 50 Hz loop then repeats its last error for ~3 ticks at a time.
    For the 90 fps the D405 is capable of, use defence 0 with
    `-p capture_fps:=90` — never the driver profile, which would put six
    times the colour traffic on the graph.

## FR3-specific calibration notes

- **Hand-eye**: `ros2 run roscam handeye_calib --ros-args -p
  base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp`, then write the result
  into `calib/handeye.yaml` (the one copy, measured 2026-09-15; `fr3_cell`
  reads it on every launch). `fr3_cell handeye_xyz:="x y z"
  handeye_quat:="qx qy qz qw"` tries a candidate without editing it.
  Run the calibration with `filter_frame` left at `fr3_link0`? **No** —
  during hand-eye collection the TF chain being calibrated doesn't exist
  yet; run `cam_pub` standalone with `-p filter_frame:=''` for that step.
- **No Pilz on the FR3 MoveIt config** → `fr3_params.yaml` uses OMPL with
  reduced `insertion_depth_m`/speeds and one extra `align_hold_cycles`
  (setup guide §7). If mating quality demands a path-guaranteed stroke,
  the `computeCartesianPath` fallback is the planned fix — ask for it.
- Fake marker pose from the dry run (0.45, 0.10, 0.05 in `fr3_link0`) is
  known-reachable; place the real marker in a similar zone to start.

## Future (deliberate non-goals of this scaffolding)

- `computeCartesianPath` INSERT fallback for Pilz-less robots (FR3 is one).
- ~~Continuous tracking via `moveit_servo`~~ — settled: it runs on the
  impedance backend instead (above). The `moveit_servo` backend is archived
  at tag `pre-cleanup-2026-09-23`.
- A libfranka cartesian-impedance backend for compliant insertion — revisit
  after the stepped pipeline mates reliably.
- The parked mock dry run (`melfa/parked/dryrun_fr3/`) can serve as a
  no-hardware regression test of the phase machine once mating_node is
  revived (`melfa/parked/README.md`); it needs a working MoveIt build.
