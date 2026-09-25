# Continuous marker tracking on the impedance backend — implementation spec

*Written 2026-09-22, against measurements taken on the real FR3 that day and
on 2026-09-16 (raw traces in `tools/fr3/logs/`). Companion docs:
[TODO.md](TODO.md) lesson 10 (the friction deadband),
[PROJECT_STATE.md](PROJECT_STATE.md) (where the project stands),
[fr3_mating_controllers/README.md](fr3_mating_controllers/README.md) (the
controller).*

> **AMENDED 2026-09-22, later the same day, after implementation.** This spec
> was written before any code existed. Four of its claims did not survive
> contact with the real pipeline - the vision rate (section 3), the gain
> profile mechanism (section 4 C1), the frame the error is computed in and
> what vision loss does (section 5) - and the pass criteria that depended on
> the first of those moved with them (sections 2, 6 and 7). Every change is
> marked **[AMENDED]** in place, next to the original text, so it is readable
> rather than silent. The
> implementation is
> [`mating_controller/src/tracking_node.cpp`](mating_controller/src/tracking_node.cpp)
> plus the pure law in
> [`tracking_law.hpp`](mating_controller/include/mating_controller/tracking_law.hpp).
> It compiles, the law is unit-tested
> ([`test_tracking_law.cpp`](mating_controller/test/test_tracking_law.cpp)),
> and it has **never been run** - on hardware or otherwise.

## 1. What this is for

Today alignment is **stepped**: `cell_panel` plans and executes a discrete
MoveIt motion per cycle, converging to ~1 mm. It works, and it is slow and
visibly jerky. The goal is **continuous** tracking: the arm follows the
marker smoothly, at a configurable speed, on the Cartesian-impedance
controller that already exists.

**Not in scope:** the insertion stroke (already specified and partly
commissioned), vision changes, and the servo/`moveit_servo` backend, which
stays parked.

## 2. The constraints this must live with (measured, not assumed)

Everything below is from the real arm. Do not re-derive it.

| Property | Value | Where from |
|---|---|---|
| Friction breakaway at the TCP | 3.5 - 6.5 N, pose dependent | 2026-09-16 and 09-22 |
| Friction about the wrist | ~0.6 Nm | 2026-09-16 |
| **Static deadband** | `F_friction / k` | confirmed 600 - 3000 N/m |
| deadband at k = 600 | 6.1 mm | 09-22 |
| deadband at k = 3000 | 1.8 mm | 09-22 |
| angular deadband at k_rot = 10 | 3.4 deg | 09-16 |
| angular deadband at k_rot = 90 | ~0.2 - 0.4 deg | 09-22 |
| **Translational stiffness ceiling** | 3000 N/m (configured max) | - |
| **Rotational stiffness ceiling** | ~90 Nm/rad - noise and vibration above | 09-22, by ear |
| Overshoot at zeta 1.0 | **none**, 0 of 32 clean steps | 09-22 |
| 50 mm step, time to 90% | 1.05 s, against a 0.96 s slew floor | 09-22 |
| RT success while tracking | median 1.000, min 0.900 | 09-22, 22 min |
| Resting wrench-estimate bias | 0.8 - 1.0 N | pre-flight, 09-22 |

Three consequences drive the whole design:

1. **The arm cannot be made to reach its setpoint by raising gains.** The
   residual is `F/k` and both k ceilings are now known. About **2 mm and
   0.4 deg** is the controller's static accuracy floor.

   > **[AMENDED]** 2 mm is the floor at the 3000 N/m *ceiling*, not at the
   > stiffness tracking ships with. Section 4's `track` profile is 1500 N/m,
   > where the residual is `3.5 - 6.5 N / 1500` = **2.3 - 4.3 mm**.
   > `tracking_deadband_m` is therefore 5 mm and the integrator freezes
   > inside half of it (2.5 mm). Section 7's V1 and V2 inherit this: their
   > "<= 2 mm" criteria are unreachable as shipped, and are amended there.
2. **Tracking speed is slew-limited, not stiffness-limited.** A 50 mm step
   took 1.05 s against a 0.96 s floor imposed by `setpoint_slew_mps` = 0.05.
   The spring is not the bottleneck; the rate cap is.
3. **Damping needs no change.** Zero overshoot in 32 clean steps - friction
   is itself a damper. Keep `damping_ratio` at 1.0.

## 3. Architecture

```
  cam_pub (KF, filter_frame=fr3_link0)
        | /aruco/pose  (~15 Hz as launched - see below; pose only)
        v
  tracking node  ---- bounded integral of the MEASURED error ---->
        | ~/equilibrium_pose  (PoseStamped, base frame, 50 Hz)
        v
  CartesianImpedanceStrokeController   (slew-limits, springs, 1 kHz)
        v
  fr3 (effort interface)
```

> **[AMENDED] `/aruco/pose` arrives at ~15 Hz as launched, not ~90 Hz.**
> `cam_pub` publishes one pose per frame - there is no timer - so the pose
> rate *is* the capture rate, and nothing in the launch path asks for 90:
>
> - `source: realsense` (the out-of-ROS path) takes `capture_fps`, which
>   **defaults to 15** in `roscam/roscam/cam_pub.py` and is not set by
>   `tools/fr3/fr3_mating.launch.py`.
> - `source: topic` (the default) takes whatever the driver publishes, and
>   `tools/fr3/realsense_low_bw.yaml` pins
>   `rgb_camera.color_profile: 640x480x15`.
>
> 90 fps is real - the D405 measured **89.9 fps at 640x480 colour+depth with
> 0.2 ms jitter** on 2026-09-11 (TODO.md) - but only over the out-of-ROS
> path, and only when asked for. To get it today:
>
> ```bash
> ros2 run roscam vision_standalone --ros-args \
>     -p capture_fps:=90 -p filter_frame:=fr3_link0
> ```
>
> `fr3_mating.launch.py` exposes `vision_source` but **not** `capture_fps`, so
> launching with `vision_source:=realsense` alone still captures at 15; adding
> a `capture_fps` launch argument to its `_vision()` is the small change that
> would close that. Raising the *driver* profile instead is the wrong fix:
> 640x480 colour at 15 fps is already ~13 MB/s of DDS traffic, and six times
> that is exactly what defence 0 in
> [tools/fr3/README.md](tools/fr3/README.md) exists to keep off the graph.
>
> The design survives at 15 Hz - the 50 Hz loop sees a new marker pose every
> ~3 ticks and repeats the last error in between, and `vision_timeout_s`
> (0.6 s) is nine frame intervals from tripping - but the loop bandwidth is
> set by vision, so V3's phase lag must be read against the capture rate
> actually in use, and recorded with it.

**Decision 1 - impedance, not `moveit_servo`.** Streaming into the
effort-mode trajectory controller stalled the arm outright (lesson 6), and
`franka_hardware` 2.0.2 crashes on cross-mode controller switches (lesson
7). Impedance is effort-to-effort with the arm controller, so a swap needs
no command-mode change at all. This also reuses the slew limiter, the wrench
ceiling and the torque-rate guard already in the controller.

**Decision 2 - the friction residual is closed by an outer integrator, not
inside the controller.** The controller cannot know it is stuck; only vision
sees the true error. So the tracking node pushes the equilibrium *past* the
target until the measured error closes. This is exactly what
`impedance_overdrive_m` (10 mm) already does for the stroke, generalised.
Rejected alternatives: dither injection and joint friction feedforward -
both are real work and both spend the stability margin found on 09-22.

## 4. Controller changes required

Two, both small, both in `fr3_mating_controllers`.

**C1 - named gain profiles.** Tracking wants stiff, mating wants soft; one
gain set cannot serve both (lesson 10). Add to the controller yaml:

```yaml
profiles:
  track: {k_pos_tool: [1500, 1500, 1500], k_rot_tool: [90, 90, 90],
          damping_ratio: 1.0}
  mate:  {k_pos_tool: [150, 150, 800],    k_rot_tool: [10, 10, 20],
          damping_ratio: 1.0}
```

Applied atomically on phase transition, through the existing
`set_parameters_atomically` path. The existing `GainLimits` range checks
still apply and still reject a set as a whole.

- `track` is isotropic on purpose: an anisotropic K in the tool frame
  deflects the commanded force away from the commanded direction whenever
  the tool is tilted (measured 18.5 deg median on 09-16). For tracking,
  motion should follow the command.
- `k_rot` 90 is the measured ceiling, not a preference. See open question
  O1 before raising it.

> **[AMENDED] C1 shipped with no `mate` profile and no `profiles:` block.**
> There is no `profiles:` map in `cartesian_impedance_stroke.yaml`. Instead
> the five `track_*` keys (`track_k_pos_tool`, `track_k_rot_tool`,
> `track_damping_ratio`, `track_setpoint_slew_mps`,
> `track_setpoint_slew_rps`) live in `tools/fr3/fr3_params.yaml`,
> `tracking_gain_profile` names the prefix, and `~/start_tracking`
> **reads the controller's live values** for those five parameters, holds
> them, applies the `track` set atomically, and `~/stop_tracking` puts the
> snapshot back.
>
> This is better than the named pair above, not merely different.
> `cell_panel.py` (IMPEDANCE & TRACK tab) retunes gains live, so "restore the `mate` profile"
> would restore something the operator never had, while a snapshot restores
> what was actually in force. It also deletes a two-file drift: a `mate`
> block would have duplicated the controller yaml's own defaults with
> nothing pinning the two together. And if the snapshot cannot be read,
> `~/start_tracking` **refuses** - the node does not change what it cannot
> put back.
>
> Two consequences. There is no `mate` profile to name: mating gains are
> simply whatever the controller is configured with. And the switch happens
> on the operator's service call, **not** "on phase transition" -
> `mating_node` does not call it.

**C2 - make `setpoint_slew_mps` / `setpoint_slew_rps` live.** They are
configure-time today, which means changing profile needs a cleanup and
reconfigure cycle. Since the slew is the actual speed limit (finding 2),
tracking needs it at 0.10 - 0.25 m/s while the stroke needs 0.005. Keep the
existing range checks (`ConfigLimits`: <= 0.25 m/s, <= 1.0 rad/s) - they
stay the hard safety bound. Remove both from `kConfigureOnlyParams`.

> **[DONE as written.]** Both are live, `ConfigLimits` is unchanged, and
> `tracking_node` raises them in the same atomic set as the gains and
> restores them on stop. One consequence the spec did not call out: because
> they are live, the equilibrium speed cap can now be raised *while the arm
> is moving*. `ConfigLimits` 0.25 m/s, `max_force_n` 30 N and the 1 Nm/ms
> torque-rate limiter are what stand under that.

## 5. The tracking node

**Inputs:** `/aruco/pose` (filtered marker pose), TF, the current TCP pose
from `/franka_robot_state_broadcaster/robot_state`.
**Output:** `~/equilibrium_pose` at **50 Hz**, base frame.

Per cycle:

1. Compute the goal TCP pose from the marker pose, using
   `mating_geometry::standoff_goal` - the same function the stepped backend
   uses, so the two agree by construction.
2. `error = goal - measured_TCP` (position and rotation vector).
3. `lead += Ki * error * dt`, clamped to `lead_max`.
4. Publish `equilibrium = goal + lead`.

> **[AMENDED] step 2 is an EE-frame error, not a TCP one.**
> `mating_geometry::standoff_goal` produces a goal for `EEF_FRAME_ID`
> (`fr3_hand_tcp`), but the controller seeds and measures its equilibrium at
> `franka::Frame::kEndEffector` - that is the frame it actually compares
> against, and the frame `~/equilibrium_pose` is read in. Nothing in this
> cell asserts the two coincide, and it carries a custom D405 wrist mount.
> So `~/start_tracking` **measures** the fixed offset `T_tcp_ee` from TF and
> `o_t_ee` sampled together, accepts it only when three consecutive samples
> agree, logs it at INFO, and **refuses to start** if it cannot be
> established. `tracking_law::goal_in_ee` applies it, and steps 2-4 then all
> happen in the EE frame. A wrong offset would be a fixed error the 10 mm
> lead clamp cannot absorb, so the lead would simply sit saturated -
> measuring it is what makes that a startup refusal instead of a surprise.

**Decision 3 - the orientation target comes from vision, never from the
measured pose.** `cell_panel.setpoint_step` republishes the measured
quaternion, which is right for hand-stepping but would ratchet the angular
deadband into the target on every cycle when tracking.

**Decision 4 - anti-windup by construction.** `lead` is clamped to
`lead_max`, so the worst-case extra force is `k * lead_max`. At k = 1500 and
`lead_max` = 10 mm that is **15 N**: below the controller's 30 N ceiling and
well below the 40 N reflex. Freeze the integrator while `|error|` is inside
half the deadband, or it will hunt across the stiction band.

**Decision 5 - vision loss holds, it does not coast.** On a stale pose
(`vision_timeout_s`), stop publishing and let the equilibrium sit where it
is. The arm holds compliantly. Do not extrapolate; predicted poses must
never drive committed motion (an existing rule of this project).

> **[AMENDED] "stop publishing" does not stop the arm.** The controller's
> `slew_equilibrium` keeps stepping toward the last target every cycle for as
> long as `have_target_` is true, so merely ceasing to publish lets the
> equilibrium walk out the whole outstanding lag rather than holding. The
> implemented behaviour is **publish the measured pose once, then stop**:
> one message converges the slew to a genuine hold where the arm is, and the
> latch keeps it from being re-sent every tick. The integrator `lead_` is
> deliberately *not* reset, so a brief occlusion does not lose it and
> reacquire does not lurch. The no-extrapolation rule is unchanged.

All of `Ki`, `lead_max`, the publish rate and the profile name are
parameters in the params yaml, not constants in code.

## 6. Safety bounds (all already exist - do not bypass)

| Bound | Value | Enforced by |
|---|---|---|
| Commanded wrench | 30 N / 10 Nm | controller `max_force_n` |
| Equilibrium rate | <= 0.25 m/s | controller slew limit |
| Per-joint torque | FR3 spec | controller `tau_max_nm` |
| Torque rate | 1 Nm/ms | controller and `franka_hardware` |
| Integrator lead | `k * lead_max` <= 15 N | tracking node clamp |
| Collision reflex | 40 N / 40 Nm | set by PRE-FLIGHT |
| Human in the loop | every motion behind a button | operator panel |
| Equilibrium lead from the arm | <= 60 mm | **[AMENDED]** tracking node `publish_veto` |
| Equilibrium Z floor | 100 mm above the base, absolute in `fr3_link0` (`tracking_z_floor_m`) | **[AMENDED]** tracking node `publish_veto` |

> **[AMENDED]** The last two rows are bounds the tracking node adds. Both are
> copies of limits `cell_panel.py` (IMPEDANCE & TRACK tab) already enforces on this same topic
> (`MAX_LEAD_MM`, `FLOOR_Z_MM`); a 50 Hz stream must not be allowed
> to bypass what a hand-pressed button obeys. 2026-09-23: the floor was
> "30 mm below the start pose"; it is now one absolute floor for the whole
> cell, because a relative one refused every goal below where TRACK began.
>
> The "human in the loop" row is **not yet satisfied for tracking.** The node
> is gated - it publishes nothing until `~/start_tracking` - but the START /
> STOP TRACKING buttons on `cell_panel.py` (IMPEDANCE & TRACK tab) are not built, so today the
> only way to start it is a `ros2 service call`. That is a deliberate human
> act, not the panel gate this project's standard asks for.

## 7. Verification plan (in order, each with a pass criterion)

| # | Test | Method | Pass |
|---|---|---|---|
| V1 | Static accuracy | fixed marker, tracking on, 60 s | steady error **<= 5 mm** *[AMENDED, was 2 mm]*, <= 0.5 deg |
| V2 | Step | move the marker 20 mm, hands off | no overshoot; settles **<= 5 mm** *[AMENDED, was 2 mm]* |
| V3 | Sine | marker at 0.1 / 0.2 / 0.5 Hz | amplitude ratio and phase lag recorded; lag gives end-to-end latency |
| V4 | Vision loss | occlude the marker mid-track | arm holds, no lurch on reacquire |
| V5 | Profile switch | `~/stop_tracking` mid-run: `track` -> the snapshot *[AMENDED: there is no `mate` profile — section 4]* | no torque step, no reflex |
| V6 | Endurance | 15 min continuous | RT success >= 0.99, no reflex |

V3 also closes the open "measure true end-to-end latency" item in TODO.md.

> **[AMENDED] V1 and V2 could not have passed as written.** Both numbers came
> from the `k = 3000` row of section 2, but the shipped `track` profile is
> `k_pos_tool` **1500**, so the deadband `F/k` is **2.3 - 4.3 mm**, not 1.8,
> and `tracking_deadband_m` is set to 5 mm to match. Two things then bound
> the steady error, and both are above 2 mm:
>
> - the integrator **stops correcting** once `|error|` falls inside half the
>   deadband, i.e. at **2.5 mm** (Decision 4 - the freeze is what stops it
>   hunting across the stiction band);
> - once frozen, Coulomb stick lets the arm rest anywhere within `F/k` of
>   where the spring wants it, which is up to **4.3 mm**.
>
> 5 mm is therefore the criterion that the shipped configuration can be
> asked for. Where in 2.5 - 5 mm it actually lands is a stiction limit cycle
> and is not predictable from here - V1 is the measurement that settles it.
> The angular criterion is untouched: 0.6 Nm / 90 Nm/rad is 0.38 deg with a
> 0.2 deg freeze band, so <= 0.5 deg stands.
>
> To chase 2 mm instead, three values must move in **one** edit, because
> `tracking_law::validate_config` refuses a half-done change at startup:
> `track_k_pos_tool` to 3000, `tracking_lead_max_m` to 0.005 (the 15 N lead
> bound is `k * lead_max`), and `tracking_deadband_m` to 0.003 (it must stay
> within `[F/k, 3F/k]`, and a 1.5 mm freeze band is what leaves room under
> 2 mm). The prediction below is the cheaper thing to try first.

**Prediction worth checking at V1:** stiction is worst at zero velocity, so
a continuously moving equilibrium should show a *smaller* effective deadband
than the 4.3 mm worst case computed above. If V1 beats it, that is why.
*[AMENDED: the comparison was against 2 mm, the k = 3000 number.]*

## 8. Open questions

- **O1 - is 90 Nm/rad really the rotational ceiling?** The buzz above 90 was
  heard, not measured: the operator trace is relayed at 50 Hz and cannot see
  anything above 25 Hz. `D = 2*zeta*sqrt(K)` at k_rot 90 is ~19 Nms/rad
  multiplying velocity noise, so **try `damping_ratio` 0.5 and walk k_rot up
  again**. There is headroom - overshoot was zero. Diagnosing it properly
  needs 1 kHz data, not the 50 Hz relay.
  **[ANSWERED 2026-09-23, 1 kHz]** The buzz is real and it is the damping.
  During TRACK at k_rot 90 / zeta 1.0 a 40 Hz mode grew (x2 every 0.2 s)
  once the wrist started moving: 99% of the motion was EE rotation (J7 and
  J5 moved most), J1/J4 carried 4-5 Nm rms of reaction torque, and at 40 Hz
  the damping torque (D_rot * omega ~ 7 Nm) was ~50x the spring's. Its
  amplitude sat at the torque-rate limit (1 Nm/ms / (2 pi 40 Hz) = 4 Nm); the
  same gains at HOLD with position kicks stayed quiet, because a still wrist
  sits in stiction. Now: `track_damping_ratio` 0.5, and tracking_node stops
  itself above 3.5 Nm rms of >20 Hz joint torque (normal work peaked at
  0.15 Nm over 24 min of recording; 0.5, then 1.0, until 2026-09-24, when
  fast TRACK's stop-start on 15 fps goal steps and catch-ups read 0.2-1.0 Nm
  without a buzz, then 2.5; the real buzz crosses 3.5 0.76 s after 0.5). Since
  then the node glides the goal between camera frames (`GoalGlide`,
  `tracking_goal_glide`), so a fast slew follows the marker continuously
  instead of stop-starting each frame. Walking k_rot up again is still open.
- **O2 - what `Ki` is stable?** The outer loop closes around a spring with a
  deadband; too fast and it hunts. Start low (lead reaches `lead_max` in
  ~2 s) and raise it at V1.
- **O3 - does the 50 Hz equilibrium stream destabilise the FCI loop?** On
  2026-09-16 a `communication_constraints_violation` arrived 19 s after the
  panel's 1 kHz relay attached. Later crashes were traced to the laptop
  running on battery, which demotes but does not clear the suspicion.
  Watch RT success during V6.

## 9. Out of scope, deliberately

Inertia shaping (no `Lambda` term - the path curves under anisotropic
apparent mass, which is a transient and harmless at tracking speeds),
friction feedforward, dither, and any change to the vision pipeline.
