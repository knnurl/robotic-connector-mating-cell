# Running the connector-mating cell on a Franka FR3

Scaffolding for the **real** FR3 (the mock dry-run harness lives in
[tools/dryrun_fr3/](../dryrun_fr3/)). The mating stack itself is unchanged —
the controller is robot-agnostic — this directory supplies the FR3 wiring,
safe parameters, and the network/RT hardening the FR3 specifically needs.

| File | Purpose |
|---|---|
| `fr3_params.yaml` | Controller params: `fr3_arm`/`fr3_hand_tcp`, OMPL, reduced speeds/depth, **insertion disabled by default** |
| `fr3_mating.launch.py` | Cell side: hand-eye TF + `cam_pub` (KF in `fr3_link0`) + `move_l` |
| `cyclonedds_fr3.xml` | DDS interface isolation + buffer tuning (the bandwidth fix) |
| `realsense_low_bw.yaml` | D405 config: colour-only, 640×480@15, **no pointcloud** |
| `fr3_preflight.sh` | RT-kernel / latency / DDS-isolation / bandwidth checks |

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
   In this mode, skip the camera-driver terminal (step 3 below) entirely.
1. **Physical isolation** — the robot gets a dedicated wired NIC
   (here: `eno1`, e.g. PC `172.16.0.1/24` ↔ robot `172.16.0.3`). Nothing
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

```bash
# 0. In EVERY terminal - step 2's above all: the controller manager must see
#    this workspace, or the impedance controller cannot be loaded.
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source tools/fr3/fr3_env.sh     # DDS isolation, FR3_ROBOT_IP, this workspace

# 1. Preflight (fix every FAIL)
tools/fr3/fr3_preflight.sh $FR3_ROBOT_IP

# 2. Robot driver + MoveIt (upstream, self-contained; add use_fake_hardware:=true for dry runs)
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP

# 3. Camera (low-bandwidth profile). SKIP this terminal when using the
#    out-of-ROS capture path (defence 0): pass vision_source:=realsense in
#    step 4 instead, or run `ros2 run roscam vision_standalone` for
#    marker+ICP on one camera.
ros2 launch realsense2_camera rs_launch.py config_file:="$PWD/tools/fr3/realsense_low_bw.yaml"

# 4. Cell: hand-eye TF + vision + controller (insertion disabled by default)
ros2 launch tools/fr3/fr3_mating.launch.py robot_ip:=$FR3_ROBOT_IP
# out-of-ROS capture variant (no camera driver, no image topics):
ros2 launch tools/fr3/fr3_mating.launch.py robot_ip:=$FR3_ROBOT_IP vision_source:=realsense
```

Then follow the standard validation ladder in
[SETUP_AND_CALIBRATION.md](../../SETUP_AND_CALIBRATION.md) §4: fake
hardware → real hardware insertion-disabled (occlusion hold test) → full
mate at reduced `insert_speed`.

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
  `move_l` switches controllers around the stroke and judges seating by
  the wrench. Commission via that package's ladder (float → hold →
  setpoint → dispatch), only after the force-guarded moveit stroke works.
- **Servo alignment** (opt-in, `align_mode: servo` + `fr3_servo.yaml`):
  ALIGN phases publish Cartesian twists for a moveit_servo node instead of
  stepped plan-execute cycles — smooth continuous tracking. Requires
  `sudo apt install ros-humble-moveit-servo` (not yet on this PC) and
  `control_period_s` below the servo command timeout; runtime-unverified
  until then. INSERT is unaffected — the committed-stroke semantics stay.

## FR3-specific calibration notes

- **Hand-eye**: `ros2 run roscam handeye_calib --ros-args -p
  base_frame:=fr3_link0 -p tcp_frame:=fr3_hand_tcp`, then pass the result:
  `ros2 launch tools/fr3/fr3_mating.launch.py handeye_xyz:="x y z"
  handeye_quat:="qx qy qz qw"`. The defaults are the dry-run **guess**.
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
- Continuous tracking via `moveit_servo`, or a libfranka
  cartesian-impedance backend for compliant insertion — revisit after the
  stepped pipeline mates reliably.
- The parked mock dry run (`tools/dryrun_fr3/`) still works for
  no-hardware regression tests of the phase machine.
