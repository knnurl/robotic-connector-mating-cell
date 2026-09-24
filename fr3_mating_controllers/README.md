# fr3_mating_controllers — compliant insertion backend

A `franka_ros2` ControllerInterface plugin implementing the Cartesian-
impedance mating stroke:

```
tau = J^T ( K (x_d − x) − D J·dq ) + tau_nullspace + coriolis      @ 1 kHz
```

with **K diagonal in the tool frame** — soft lateral X/Y and roll/pitch so
the connector self-aligns into its socket under contact, firm along tool Z
to drive the stroke. `D = 2ζ√K`, so tuning is one stiffness vector. That is
critical damping for a **1 kg** mass; the FR3's apparent Cartesian mass is a
few kg, so the effective damping ratio is about **0.5–0.7** (joint friction
adds a little). Expect one small overshoot when a push is released — not
oscillation.

Patterns ported from the `fr3_backend` testbed's proven joint-impedance loop:
equilibrium seeded at the current pose on activation (first cycle = zero
spring force), slew-limited setpoints (a topic jump becomes a bounded ramp,
never a reflex), torque-rate saturation (≤ 1 Nm/ms, the FCI discontinuity
limit).

**Bounds that hold whatever the gains are:**

- the commanded wrench is capped at `max_force_n` / `max_torque_nm`
  (30 N / 10 Nm). Stiffness × error alone is unbounded: a blocked arm with a
  slewing equilibrium would otherwise keep winding up force.
- joint torques are capped at `tau_max_nm` (never above the FR3 limits) and
  rate-limited.
- malformed setpoints are dropped: a non-finite position, or a quaternion
  whose norm is outside 0.9–1.1 (publish normalised quaternions; an all-zero
  one is the usual culprit). Non-finite state or torque commands zero.
- live gains must sit inside `GainLimits`
  ([impedance_detail.hpp](include/fr3_mating_controllers/impedance_detail.hpp));
  a set with any value out of range is rejected — all-or-nothing when sent
  atomically, as the panel does. `damping_ratio` 0 is an
  undamped spring and a negative value injects energy — the force cap bounds
  how hard the arm pushes, not whether it oscillates.
- configure-time parameters — `arm_id`, `max_force_n`, `max_torque_nm`,
  `tau_max_nm`, `tau_rate_limit` — are range-checked (configure fails) and
  cannot be changed while the controller is configured: clean it up, set them,
  configure again. The slew pair is no longer among them; it is live.
- `float_mode` is live, and switching it OFF re-seeds the equilibrium where
  the arm is now, discarding any setpoint published before — float → hold
  cannot snap the arm back.
- the 1 kHz loop never blocks on a lock a non-RT thread holds, and logs only on
  fault paths.

**Division of labour:** this controller tracks an equilibrium, compliantly
— nothing else. The phase machine in `mating_node` stays the brain: with
`insert_backend: impedance` it switches this controller in for INSERT,
streams the equilibrium along the tool axis (50 Hz, `~/equilibrium_pose`),
judges the outcome by the external wrench (seated / jam / snag — same
thresholds as the guarded MoveIt stroke), and hands the arm back to the
trajectory controller afterwards, whatever happened.

`~/equilibrium_pose` now has **three** possible publishers: `mating_node`'s
stroke ramp, the cell panel's (`tools/fr3/cell/`) hand-stepped setpoints, and
`mating_controller`'s `tracking_node` (below). `TargetHandoff` simply
overwrites under a mutex — there is **no arbitration**, so the last publisher
wins. That is harmless today, because `insert_backend` is `moveit` and
tracking is off until an operator starts it, but it has to be decided before
`insert_backend` flips to `impedance`.

**Continuous marker tracking** (`mating_controller/src/tracking_node.cpp`,
specified in [TRACKING_SPEC.md](../TRACKING_SPEC.md) §5) is the other client
of this controller. It streams `~/equilibrium_pose` at 50 Hz so the arm
follows the marker, and closes the `F/k` friction residual with a bounded
integrator of the **measured** error — the controller cannot know it is
stuck, only vision can, so that correction belongs outside it. What it asks
of this controller:

- `~/start_tracking` reads `k_pos_tool`, `k_rot_tool`, `damping_ratio`,
  `setpoint_slew_mps` and `setpoint_slew_rps` back from the controller,
  applies the `track_*` set from `tools/fr3/fr3_params.yaml` in **one**
  `set_parameters_atomically`, and `~/stop_tracking` restores the snapshot.
  It restores what was *in force*, not the shipped defaults, because the
  panel retunes gains live; and it refuses to start if it cannot read them.
- It refuses to start while `float_mode` is true: a free-floating arm must
  not be gain-stepped.
- It re-seeds the equilibrium at the measured pose and waits
  `tracking_settle_s` **before** the gain step, because `k_pos` 150 → 1500 is
  a tenfold multiplier on whatever equilibrium error already exists, and
  20 mm of it saturates the 30 N ceiling instantly.
- On stale vision it publishes the measured pose **once** and then stops.
  Merely ceasing to publish would not hold the arm: `slew_equilibrium` keeps
  stepping toward the last target for as long as `have_target_` is true.

**Compiles, never run, review still in flight.** Nothing below about the
track profile has been seen on hardware.

## Build and test

Builds only where the Franka stack exists (skips itself cleanly elsewhere):

```bash
source /opt/ros/humble/setup.sh
source ~/franka_ros2_ws/install/setup.sh
colcon build --packages-select fr3_mating_controllers
colcon test --packages-select fr3_mating_controllers && colcon test-result --verbose
```

The gtests cover the pure safety logic — gain limits and the setpoint handoff —
without a robot. The panel side is covered by `tools/fr3/cell/test_cell_pins.py`.

## Bring-up

The **controller manager** loads this plugin, so the terminal that runs
`moveit.launch.py` must have this workspace sourced — `tools/fr3/fr3_env.sh`
does it. Without it the spawner fails with "Loader for controller … not
found": safe, but the ladder cannot start.

```bash
# in every terminal:
source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash
source tools/fr3/fr3_env.sh
# driver + MoveIt, then spawn INACTIVE:
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=$FR3_ROBOT_IP
ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
    --inactive --param-file "$(pwd)/fr3_mating_controllers/config/cartesian_impedance_stroke.yaml"
# the commissioning rig:
python3 tools/fr3/cell/cell.py
```

`mating_node` activates/deactivates it around the stroke via
`/controller_manager/switch_controller` — never leave it active alongside
the trajectory controller (STRICT switching enforces this). Both claim the
effort interface, so a combined swap changes no franka command mode.

## Commissioning ladder (do not skip steps)

Run it with the cell panel (`tools/fr3/cell/`): one button per rung, the order
enforced. **Any libfranka reflex kills `ros2_control_node` on this cell**
(franka_hardware does not catch the exception). The robot stops — it is not
freed — and the stack must be relaunched and PRE-FLIGHT run again.

0. **Pre-flight.** Set the payload (camera + bracket only — the hand belongs to
   Desk's end-effector config; never count it twice, and enter 0 kg if Desk
   already includes the camera) and the collision reflex thresholds. Franka
   accepts both only with **no controller active**, so the panel releases
   `fr3_arm_controller`, sets them, checks each reply, and re-activates it on
   every path. The reflex trips at 40 N / 40 Nm Cartesian, about 10 N above
   this controller's 30 N force ceiling. The reflex watches libfranka's
   *estimated* external wrench, so that margin shrinks by the estimate's bias:
   PRE-FLIGHT reports |F ext| at rest, and if it reads more than a few N, fix
   the payload before FLOAT.
1. **RT-loop proof (`float_mode: true`)** — the arm must free-float smoothly
   under **gentle** hand pressure (under 10 N on the panel's |F ext|). Jitter,
   kicks, or a control success rate below ~99% are a network/RT problem
   (`tools/fr3/fr3_preflight.sh`), not a gain problem. Torque mode is the most
   bandwidth-sensitive FCI mode: the DDS interface isolation and out-of-ROS
   vision capture are prerequisites, not options. A slow sag means the payload
   is wrong.
2. **Hold test (`float_mode: false`)** — pressing HOLD must not move the arm.
   Push the TCP gently: lateral soft spring ~150 N/m (15 N ≈ 10 cm), tool Z
   visibly stiffer ~800 N/m (12 N ≈ 1.5 cm). One small overshoot on release is
   expected; more than a couple of visible cycles, buzz, or drift is not.
3. **Setpoint test** — step the equilibrium 1–2 cm **up** first; the arm should
   glide there at the slew limit and settle within a few mm. The panel refuses
   setpoints more than 30 mm below where HOLD started (the camera bracket hangs
   below the flange).
4. **Dispatch test** — `insert_backend: impedance` in
   `tools/fr3/fr3_params.yaml`, insertion disabled ladder first, then the
   full mate: expect `Seated by contact force at X mm depth`. Only after the
   force thresholds are tuned on the MoveIt stroke — they are shared.

## Tuning (`config/cartesian_impedance_stroke.yaml`)

| Knob | Default | Live? Range | Meaning |
|---|---|---|---|
| `k_pos_tool` | [150, 150, 800] N/m | live, 0–3000 | Lateral softness = self-alignment; Z = stroke drive. Preload at full lag = overdrive × k_z |
| `k_rot_tool` | [10, 10, 20] Nm/rad | live, 0–300 | Roll/pitch soft (align to socket plane); yaw firmer (keying) |
| `damping_ratio` | 1.0 | live, 0.1–2.0 | Stays 1.0: zero overshoot in 32 clean steps on 2026-09-22, because friction is itself a damper. The one open experiment is zeta 0.5 while walking `k_rot` above 90 ([TRACKING_SPEC.md](../TRACKING_SPEC.md) O1) — do not raise zeta above 1.0 |
| `nullspace_stiffness` | 5.0 | live, 0–50 | Elbow posture hold; raise if the arm drifts configuration mid-stroke |
| `float_mode` | false | live | Coriolis only; switching OFF re-seeds the equilibrium |
| `setpoint_slew_mps/rps` | 0.05 / 0.5 | live, ≤ 0.25 / ≤ 1.0 | Hard cap on equilibrium motion, and the actual speed limit — a 50 mm step took 1.05 s against a 0.96 s slew floor, so this is what to change to make tracking faster, not `k_pos_tool`. `tracking_node` raises it to 0.10 / 0.5 on `~/start_tracking`, in the same atomic set as the gains, and restores it on stop. Live means it can be raised while the arm is moving; `ConfigLimits` still refuses anything above 0.25 / 1.0 |
| `max_force_n` / `max_torque_nm` | 30 / 10 | configure, 1–100 / 0.5–30 | Ceiling on the commanded wrench. Keep well below the panel's 40 N reflex threshold (it watches the estimated wrench) |
| `tau_max_nm` | FR3 limits | configure, ≤ FR3 limits | Per-joint torque ceiling |
| `tau_rate_limit` | 1.0 Nm/cycle | configure, ≤ 1.0 | FCI torque-discontinuity guard — do not raise |

Stroke-side knobs live in `tools/fr3/fr3_params.yaml`:
`impedance_stroke_mps` (ramp speed), `impedance_overdrive_m` (preload
lead), `impedance_settle_s`, and the shared force thresholds
(`contact_force_n`, `max_lateral_force_n`, `min_contact_depth_m`).

Tracking-side knobs live there too, as `tracking_*` (loop rate, `Ki`, the
lead clamps, the deadbands, the timeouts) and `track_*` (the profile this
controller is asked to adopt: `[1500, 1500, 1500]` N/m, `[90, 90, 90]`
Nm/rad, ζ 1.0, slew 0.10 / 0.5). Three of them are **coupled and checked at
the tracking node's startup**, not here: the deadband must stay within
`[F/k, 3F/k]` of the stiffness in use, and `k_pos_tool × tracking_lead_max_m`
must stay under 15 N. Change the track stiffness without moving the other
two and the node refuses to start.

## Status

Compiled clean against `franka_ros2` (this workspace) and plugin-exported;
safety logic unit-tested. **Rungs 0–3 of the ladder ran on the real arm on
2026-09-16 and again on 2026-09-22**: float smooth, hold solid, setpoints
tracking to within the friction deadband. Rung 4, the dispatched stroke, waits on the force
thresholds. Fake hardware cannot verify the control law (the mock system does
not integrate torques) — commissioning happens on the real arm via the ladder
above.

The tracking client is a step behind that: `tracking_node` **compiles and
installs, has never run, and has had no adversarial review**. Its V1–V6 plan
is [TRACKING_SPEC.md](../TRACKING_SPEC.md) §7.
