# FR3 Cell Control

The operator panel for the FR3 cell: align (position control), the impedance
ladder and TRACK (torque control) on one page. It moves nothing on its own:
every motion is a button press. It does not replace the hardware E-stop or
the enabling device.

```bash
source tools/fr3/fr3_env.sh
# real cell: terminal 1 as usual (fr3_preflight && moveit.launch.py), then
fr3_cell                   # terminal 2: controller spawn, TF, vision, tracking_node, this panel
# no robot at all: the mock cell and the panel on the isolated DDS domain 88
fr3_cell mock:=true
```

The settings drawer is kept across restarts in
`~/.config/fr3_cell/settings.yaml` (`settings_mock.yaml` for the mock).
It is saved on every change, and the start-up log lists every restored value
that differs from its default. Delete the file to reset. Three things start at
their safe defaults on every launch instead:
- both speed sliders (motion 20 %, TRACK 10 %);
- the robot-state gate, which is ON;
- the gains, which the controller itself keeps.

The panel runs as the ROS node `cell_panel`. It refuses to start while
another `cell_panel` is up, and exits without touching the controllers, so
two panels never drive one arm.

Needs PySide6 (`pip install --user PySide6`, tested with 6.11). The
screenshot check also needs Pillow. It replaced the Tk `tools/fr3/cell_panel.py`
on 2026-09-24; its ROS layer and exit handoff carried over unchanged
(`ros_node.py`, `core.py`).

## Layout
- **Top:** a status bar with the same seven chips in every state (FRANKA ·
  MOVEIT · RT · CONTROLLER · PRE-FLIGHT · VISION · GATE), then CALIB and REC.
- **Banner:** below the status bar, showing the one highest-priority fault
  (`logic.banner`), or READY and the mode.
- **Left column:**
  - camera thumbnail: click to enlarge; it enlarges by itself on marker
    loss, stale frames or a pose jump;
  - telemetry;
  - force bar against this session's PRE-FLIGHT thresholds;
  - worst joint;
  - plots: convergence in position mode, force and lead in torque mode.
- **Right column:**
  - the mode, and the speed slider. In position mode it sets MoveIt scaling
    for the next planned move. In torque mode it sets the controller's
    `setpoint_slew_mps/rps`, and above 25 % it needs a confirm;
  - POSITION section: align steps, AUTO-CONVERGE, saved poses;
  - TORQUE section: PRE-FLIGHT (the entry gate), FLOAT, HOLD, SETPOINT,
    hold HERE, TRACK, the gain preset slider, RELEASE.
- **Settings drawer:** target and safety, the workspace box, motion steps,
  TRACK entry and marker loss, numeric gains, TEACH poses, calibration,
  camera, and fault injection in mock mode.
- **Bottom:** a stop bar that never moves (STOP NOW, PAUSE, stop after the
  current move) and a 3-line log that expands on demand. Esc is STOP NOW
  from anywhere in the window.

Enable rules, banner priority, the single blue next step and the chips are
pure functions in `logic.py`. They are the panel spec's rules ANDed with
every refusal the Tk panel made.

## Files

| file | what |
|---|---|
| `cell.py` | entry point, `LiveBackend` (window ↔ ROS), exit handoff |
| `core.py` | constants, geometry, `run_resumable`, the exit handoff and shutdown order - no ROS, no Qt |
| `ros_node.py` | `CellNode`: MoveIt moves, switches, parameters, the 50 Hz relay, controller poll, readback, recovery, heartbeat |
| `logic.py` | pure decisions: `enable`, `banner`, `next_step`, `chips`, slider and preset maps |
| `actions.py` | the commands: `Cell.run` (pending → done), the stops, the watchers |
| `view.py`, `palette.py`, `choices.py` | the Qt window; IEC 60073 tokens and the three text sizes; the selector values |
| `config/` | `settings.yaml` (thresholds, PLACEHOLDERs marked), `gain_presets.yaml`, `poses.yaml` |
| `persist.py` | the drawer, saved on change and restored at start |
| `mock_cell.py`, `isolate.py` | fake cell with fault injection; the domain-88 re-exec |
| `mock_smoke.py` | headless: the real node + commands against the mock (44 checks) |
| `visual.py`, `baselines/` | five scenes captured offscreen, diffed against the baselines |

## Tests
`python3 -m pytest tools/fr3/cell` from `src/` (also part of
`tools/run_tests.sh`'s pytest step):
- `test_cell_logic.py`: enable, banner priority and the four required
  cases, plus presets and the slider maps;
- `test_cell_pins.py`: the copies match `core.py`, and the cell's shared
  numbers match the controller header and yaml, `fr3_params.yaml`, the
  tracking law and the hand-eye file; ROS-side code never imports Qt;
- `test_cell_gate.py`: the robot-state gate, PAUSE and the ALIGN torque
  interlock;
- `test_cell_exit.py`: the exit handoff and the shutdown order;
- `test_cell_view.py`: Esc, blocked presses, pending, one blue step,
  confirms;
- `test_cell_persist.py`: the drawer survives a restart, and the speeds and
  gate do not;
- `test_cell_integration.py`: runs `visual.py` and `mock_smoke.py`.

After an intended look change, run `python3 tools/fr3/cell/visual.py --update`.
The baselines depend on the installed fonts (Lato, DejaVu Sans Mono).

## Controller-side TODOs
The panel does what the GUI side can. These need the controller side:

- **C1 Heartbeat watchdog.** The panel publishes `/cell_panel/heartbeat`
  (std_msgs/Header, 10 Hz, from the GUI thread). Nothing consumes it yet.
  After about 0.5 s of silence while tracking, tracking_node should stop
  tracking and hold where the arm is, with gains restored. It should hold,
  not release: a controller cannot switch itself out, and holding is the
  lower-energy state. The impedance controller already holds when no
  setpoints arrive.
- **C2 Marker-loss policy in the node.** tracking_node only holds on stale
  vision (`vision_timeout_s` 0.6 s / raw 0.25 s) and resumes by itself.
  The panel's stop/release-after-N-ms policy runs in the GUI, so it only works
  while the GUI is alive. Proposed parameters:
  `tracking_marker_loss_policy` and `tracking_marker_loss_ms`.
- **C3 Pause during TRACK.** There is no pause interface, so PAUSE while
  tracking has to stop tracking. The node already has an internal
  "holding" state; `~/pause` and `~/resume` would expose it.
- **C4 PRE-FLIGHT readback.** franka_hardware cannot report the current
  collision thresholds or FCI load. The panel infers validity: it is voided by any
  robot-state gap over 2 s and is unknown at startup. Needed: a latched
  topic or a get service in franka_hardware.
- **C5 Reflex survival.** On this cell a reflex kills ros2_control_node,
  which also hosts `/action_server/error_recovery`, so RECOVER usually finds
  no server. The fix is for franka_hardware to catch the control exception;
  that is a franka_ros2 change.
- **C6 Workspace box in the node.** tracking_node enforces only
  `tracking_z_floor_m`. The panel's X/Y/Z-max box is GUI-side while tracking.
  Proposed parameters: `tracking_box_min_m` and `tracking_box_max_m`.
- **C7 Controller status output.** The impedance controller publishes no
  equilibrium and no commanded wrench. Outside TRACK, the spring lead can
  only come from the GUI's own last setpoint. Needed: `~/state` with the
  equilibrium, the commanded wrench and the gains in force.
- **C9 TRACK speed at START.** tracking_node reads its `track_*` profile,
  including `track_setpoint_slew_mps` (100 mm/s), once at construction. It
  applies that profile at START and starts streaming before it replies.
  The panel's TRACK SPEED can therefore only override it right after the reply, a
  gap of tens of milliseconds at 100 mm/s. If the node re-read the profile
  in `start_session()`, the panel could set `track_setpoint_slew_mps/rps` on the
  node before START, and the speed would land atomically with the gains.
- **C8 Setpoint arbitration.** `~/equilibrium_pose` has three possible
  publishers and the last one wins (controller README). The panel refuses its
  own setpoints while tracking, but only the controller can enforce a
  single writer.
- **C10 Driver executor above the control loop.** In `ros2_control_node`
  the 1 kHz update thread runs at FIFO 50, but the main thread (the ROS
  executor, which serves every service call) and its DDS threads run at
  FIFO 99, most likely inherited from libfranka raising the priority of the
  thread that constructs the robot. Every service call into the driver
  therefore competes with the control loop. On 2026-09-24 the RT success
  rate dipped about four times as often with the panel polling at 2 Hz as
  under the Tk panel, and three `communication_constraints_violation`
  reflexes followed. The panel now polls rarely (every 5 s and 10 s,
  re-reading after commands and transitions). The real fix belongs in the
  driver: run the executor and DDS threads below the control loop.
