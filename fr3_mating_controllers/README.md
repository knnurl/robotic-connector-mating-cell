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
- configure-time parameters are range-checked (configure fails) and cannot be
  changed while the controller is configured: clean it up, set them, configure
  again.
- `float_mode` is live, and switching it OFF re-seeds the equilibrium where
  the arm is now, discarding any setpoint published before — float → hold
  cannot snap the arm back.
- the 1 kHz loop never blocks on a lock a non-RT thread holds, and logs only on
  fault paths.

**Division of labour:** this controller tracks an equilibrium, compliantly
— nothing else. The phase machine in `move_l` stays the brain: with
`insert_backend: impedance` it switches this controller in for INSERT,
streams the equilibrium along the tool axis (50 Hz, `~/equilibrium_pose`),
judges the outcome by the external wrench (seated / jam / snag — same
thresholds as the guarded MoveIt stroke), and hands the arm back to the
trajectory controller afterwards, whatever happened.

## Build and test

Builds only where the Franka stack exists (skips itself cleanly elsewhere):

```bash
source /opt/ros/humble/setup.sh
source ~/franka_ros2_ws/install/setup.sh
colcon build --packages-select fr3_mating_controllers
colcon test --packages-select fr3_mating_controllers && colcon test-result --verbose
```

The gtests cover the pure safety logic — gain limits and the setpoint handoff —
without a robot. The panel side is covered by `tools/fr3/test_impedance_panel.py`.

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
python3 tools/fr3/impedance_panel.py
```

`move_l` activates/deactivates it around the stroke via
`/controller_manager/switch_controller` — never leave it active alongside
the trajectory controller (STRICT switching enforces this). Both claim the
effort interface, so a combined swap changes no franka command mode.

## Commissioning ladder (do not skip steps)

Run it with `tools/fr3/impedance_panel.py`: one button per rung, the order
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
| `damping_ratio` | 1.0 | live, 0.1–2.0 | Effective ~0.5–0.7 on the arm. Raise toward 1.4 if the hold test rings; drop toward 0.7 only if insertion is sluggish |
| `nullspace_stiffness` | 5.0 | live, 0–50 | Elbow posture hold; raise if the arm drifts configuration mid-stroke |
| `float_mode` | false | live | Coriolis only; switching OFF re-seeds the equilibrium |
| `setpoint_slew_mps/rps` | 0.05 / 0.5 | configure, ≤ 0.25 / ≤ 1.0 | Hard cap on equilibrium motion (safety net under the 50 Hz stream) |
| `max_force_n` / `max_torque_nm` | 30 / 10 | configure, 1–100 / 0.5–30 | Ceiling on the commanded wrench. Keep well below the panel's 40 N reflex threshold (it watches the estimated wrench) |
| `tau_max_nm` | FR3 limits | configure, ≤ FR3 limits | Per-joint torque ceiling |
| `tau_rate_limit` | 1.0 Nm/cycle | configure, ≤ 1.0 | FCI torque-discontinuity guard — do not raise |

Stroke-side knobs live in `tools/fr3/fr3_params.yaml`:
`impedance_stroke_mps` (ramp speed), `impedance_overdrive_m` (preload
lead), `impedance_settle_s`, and the shared force thresholds
(`contact_force_n`, `max_lateral_force_n`, `min_contact_depth_m`).

## Status

Compiled clean against `franka_ros2` (this workspace) and plugin-exported;
safety logic unit-tested; **never run against hardware**. Fake hardware
cannot verify the control law (the mock system does not integrate torques) —
commissioning happens on the real arm via the ladder above.
