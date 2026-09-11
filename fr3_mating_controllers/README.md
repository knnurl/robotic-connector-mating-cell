# fr3_mating_controllers — compliant insertion backend

A `franka_ros2` ControllerInterface plugin implementing the Cartesian-
impedance mating stroke:

```
tau = J^T ( K (x_d − x) − D J·dq ) + tau_nullspace + coriolis      @ 1 kHz
```

with **K diagonal in the tool frame** — soft lateral X/Y and roll/pitch so
the connector self-aligns into its socket under contact, firm along tool Z
to drive the stroke. `D = 2ζ√K` (critically damped by default), so tuning
is one stiffness vector. Patterns ported from the `fr3_backend` testbed's
proven joint-impedance loop: equilibrium seeded at the current pose on
activation (first cycle = zero spring force), slew-limited setpoints (a
topic jump becomes a bounded ramp, never a reflex), torque-rate saturation
(≤ 1 Nm/ms, the FCI discontinuity limit).

**Division of labour:** this controller tracks an equilibrium, compliantly
— nothing else. The phase machine in `move_l` stays the brain: with
`insert_backend: impedance` it switches this controller in for INSERT,
streams the equilibrium along the tool axis (50 Hz, `~/equilibrium_pose`),
judges the outcome by the external wrench (seated / jam / snag — same
thresholds as the guarded MoveIt stroke), and hands the arm back to the
trajectory controller afterwards, whatever happened.

## Build

Builds only where the Franka stack exists (skips itself cleanly elsewhere):

```bash
source /opt/ros/humble/setup.sh
source ~/franka_ros2_ws/install/setup.sh
colcon build --packages-select fr3_mating_controllers
```

## Bring-up

```bash
# FR3 driver + MoveIt up first (tools/fr3/README.md), then spawn INACTIVE:
ros2 run controller_manager spawner cartesian_impedance_stroke_controller \
    --inactive --param-file $(pwd)/fr3_mating_controllers/config/cartesian_impedance_stroke.yaml
```

`move_l` activates/deactivates it around the stroke via
`/controller_manager/switch_controller` — never leave it active alongside
the trajectory controller (STRICT switching enforces this).

## Commissioning ladder (do not skip steps)

1. **RT-loop proof (`float_mode: true`)** — activate manually; the arm must
   free-float smoothly under hand pressure. Jitter or reflexes here are a
   network/RT problem (`tools/fr3/fr3_preflight.sh`), not a gain problem.
   Torque mode is the most bandwidth-sensitive FCI mode: the DDS interface
   isolation and out-of-ROS vision capture are prerequisites, not options.
2. **Hold test (`float_mode: false`)** — activate at rest; push the TCP.
   Lateral: soft spring (~150 N/m), critically damped return. Tool Z:
   visibly stiffer (~800 N/m). No oscillation, no drift.
3. **Setpoint test** — publish a single `~/equilibrium_pose` 2 cm above the
   activation pose; the arm should glide there at the slew limit.
4. **Dispatch test** — `insert_backend: impedance` in
   `tools/fr3/fr3_params.yaml`, insertion disabled ladder first, then the
   full mate: expect `Seated by contact force at X mm depth`.

## Tuning (`config/cartesian_impedance_stroke.yaml`)

| Knob | Default | Meaning |
|---|---|---|
| `k_pos_tool` | [150, 150, 800] N/m | Lateral softness = self-alignment; Z = stroke drive. Preload at full lag = overdrive × k_z |
| `k_rot_tool` | [10, 10, 20] Nm/rad | Roll/pitch soft (align to socket plane); yaw firmer (keying) |
| `damping_ratio` | 1.0 | 0.7 if sluggish, >1.0 if contact chatter |
| `nullspace_stiffness` | 5.0 | Elbow posture hold; raise if the arm drifts configuration mid-stroke |
| `setpoint_slew_mps/rps` | 0.05 / 0.5 | Hard cap on equilibrium motion (safety net under the 50 Hz stream) |
| `tau_rate_limit` | 1.0 Nm/cycle | FCI torque-discontinuity guard — do not raise |

Stroke-side knobs live in `tools/fr3/fr3_params.yaml`:
`impedance_stroke_mps` (ramp speed), `impedance_overdrive_m` (preload
lead), `impedance_settle_s`, and the shared force thresholds
(`contact_force_n`, `max_lateral_force_n`, `min_contact_depth_m`).

## Status

Compiled clean against `franka_ros2` (this workspace) and plugin-exported;
**never run against hardware**. Fake hardware cannot verify it (the mock
system does not integrate torques) — commissioning happens on the real arm
via the ladder above.
