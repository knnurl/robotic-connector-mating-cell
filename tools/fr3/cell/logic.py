"""FR3 Cell Control - the decisions, as pure functions of one Snapshot.

What the operator may press (enable), what the one banner says (banner),
which single button is blue (next_step), what the status bar shows (chips),
and the small numeric maps behind the sliders. No ROS and no Qt here: every
rule is a function of a Snap and the Settings, so it is unit tested without a
robot or a display (test_cell_logic.py).

The rules are the panel spec's, ANDed with every refusal the Tk panel
(retired 2026-09-24) made, so this is never looser than what ran on the arm.
"""

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np
import yaml

ARM = 'fr3_arm_controller'
IMP = 'cartesian_impedance_stroke_controller'

# franka_msgs/FrankaRobotState robot_mode (pinned to core's copy by a test)
MODE_OTHER, MODE_IDLE, MODE_MOVE, MODE_GUIDING = 0, 1, 2, 3
MODE_REFLEX, MODE_USER_STOPPED, MODE_RECOVERY = 4, 5, 6
ROBOT_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX',
               5: 'USER_STOPPED', 6: 'ERROR_RECOVERY'}
CHIP_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX',
              5: 'USER STOP', 6: 'RECOVERY'}     # short: the status bar is one line
GATE_REASONS = {0: 'robot mode OTHER', 1: 'robot IDLE - no control loop',
                3: 'hand-guiding', 5: 'robot USER_STOPPED (user stop)',
                6: 'automatic error recovery'}

# Indicator levels. IEC 60073 hues, one meaning each (palette.py):
# normal = grey (ISA-101: healthy and idle things are quiet), active = green
# (a process running and healthy), warn = amber, fault = red. Blue is not a
# level: it marks the one next step, see next_step().
NORMAL, ACTIVE, WARN, FAULT = 'normal', 'active', 'warn', 'fault'

TRACK_LIVE_STATES = ('starting', 'tracking', 'holding', 'stopping')

# Every control the window has. enable() answers for each of them.
POSITION_CONTROLS = ('translate', 'level', 'inplane', 'auto_converge')
TORQUE_CONTROLS = ('preflight', 'float', 'hold', 'setpoint_minus',
                   'setpoint_plus', 'hold_here', 'track', 'release',
                   'apply_gains', 'preset', 'speed', 'track_speed')
ALWAYS = ('stop_now', 'pause', 'stop_after', 'record')


@dataclass(frozen=True)
class Settings:
    """Thresholds, from config/settings.yaml. Values marked there as
    placeholders are guesses for the user to tune on the cell."""
    state_stale_s: float = 0.3          # core.ROBOT_STATE_STALE_S
    driver_down_s: float = 2.0          # core.DRIVER_DOWN_S; also voids PRE-FLIGHT
    pose_stale_s: float = 0.5           # core.POSE_STALE_S
    image_stale_s: float = 1.5
    track_status_stale_s: float = 1.0   # core.TRACK_STATUS_STALE_S
    rt_ok: float = 0.99
    rt_down: float = 0.95
    stationary_rad_s: float = 0.02
    rot_tol_deg: float = 1.0            # core.ROT_TOL_DEG
    inplane_tol_deg: float = 0.5        # core.INPLANE_TOL_DEG
    track_entry_mm: float = 30.0
    track_entry_deg: float = 5.0
    marker_loss_policy: str = 'hold'
    marker_loss_ms: float = 1000.0
    pose_jump_mm: float = 10.0
    speed_default_pct: float = 20.0
    track_speed_default_pct: float = 10.0
    speed_confirm_pct: float = 25.0
    position_ceiling_pct: float = 20.0  # core.CEIL_SPEED_PCT
    slew_max_mps: float = 0.25          # impedance_detail.hpp ConfigLimits
    slew_max_rps: float = 1.0
    preset_max_force_n: float = 5.0
    preset_max_lead_mm: float = 5.0
    push_limit_n: float = 10.0          # core.PUSH_LIMIT_N
    controller_max_force_n: float = 30.0
    lead_warn_mm: float = 20.0
    joint_warn_pct: float = 90.0
    box_x: tuple = (0.20, 0.80)
    box_y: tuple = (-0.45, 0.45)
    box_z_max: float = 0.80
    pose_confirm_mm: float = 50.0


MARKER_LOSS_POLICIES = ('hold', 'stop', 'release')


def load_settings(path):
    """Settings from a yaml file; an unknown key is an error, not ignored."""
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    names = {f.name for f in dataclasses.fields(Settings)}
    bad = sorted(set(raw) - names)
    if bad:
        raise ValueError(f'{path}: unknown settings {bad}')
    for k, v in raw.items():
        if isinstance(v, list):
            raw[k] = tuple(float(x) for x in v)
    s = Settings(**raw)
    if s.marker_loss_policy not in MARKER_LOSS_POLICIES:
        raise ValueError(f'marker_loss_policy must be one of {MARKER_LOSS_POLICIES}')
    return s


@dataclass
class Snap:
    """Everything the decisions read, as plain values at one instant.

    Built on the GUI thread (actions.Cell.snapshot) from what the ROS
    callbacks stored; the view and these functions only ever see this.
    """
    # robot state (the 50 Hz relay)
    state_age: float = None             # s since the last robot state; None = never
    robot_mode: int = None
    rt_rate: float = None               # control_command_success_rate, 0..1
    tcp: tuple = None                   # (x, y, z) m in fr3_link0
    force: tuple = None                 # o_f_ext_hat_k force, N
    dq_max: float = None                # largest |joint velocity|, rad/s
    joint_pct: tuple = None             # (joint number, % of half-range)
    errors: tuple = ()                  # current_errors flags that are set
    last_errors: tuple = ()             # last_motion_errors flags that are set
    # controller manager: {name: state}, None when it has not answered lately
    controllers: dict = None
    floating: bool = None               # float_mode as last known; None unknown
    params_ok: bool = False             # controller parameter service reachable
    moveit_up: bool = False
    recover_ready: bool = False         # franka error-recovery server reachable
    # vision
    marker_age: float = None            # s since /aruco/pose; None = never
    marker: dict = None                 # marker_errors(), when fresh
    marker_why: str = None
    image_age: float = None
    camera_on: bool = True
    jump_mm: float = 0.0                # largest frame-to-frame jump, last 1 s
    calib: str = 'waiting'              # loaded | waiting | missing | error
    # tracking node
    track_age: float = None
    track: dict = field(default_factory=dict)
    track_node_up: bool = False
    # this GUI session
    busy: str = None                    # command in flight
    preflight: str = 'unknown'          # done | needed | unknown
    thresholds: dict = None             # what this session's PRE-FLIGHT sent
    torque_attempted: bool = False
    tracking: bool = False              # the GUI believes the node streams
    user_paused: bool = False
    gate_on: bool = True
    z_floor_set: bool = True
    inplane_target: float = None        # None = 'off'
    setpoint_lead_mm: float = None
    failure: tuple = None               # (command, message) until acknowledged
    stopped: bool = False
    gains_pending: bool = False
    gains_valid: bool = True
    poses: dict = field(default_factory=dict)   # name -> taught
    recording: float = None             # s since the rosbag started, or None
    over_lead_policy: str = 'hold'


@dataclass(frozen=True)
class Enable:
    ok: bool
    why: str = ''


@dataclass(frozen=True)
class Banner:
    key: str
    level: str
    title: str
    detail: str
    action: str = None                  # None | 'recover' | 'dismiss'


@dataclass(frozen=True)
class Chip:
    label: str
    value: str
    level: str


# ---------------------------------------------------------------- derived

def fresh(s, st):
    return s.state_age is not None and s.state_age <= st.state_stale_s


def robot_move(s, st):
    return fresh(s, st) and s.robot_mode == MODE_MOVE


def robot_error(s, st):
    """The robot's own fault: REFLEX, or error flags set right now."""
    return fresh(s, st) and (s.robot_mode == MODE_REFLEX or bool(s.errors))


def controller(s):
    """'arm' | 'impedance' | 'none' | 'both', or None when unknown."""
    if s.controllers is None:
        return None
    arm = s.controllers.get(ARM) == 'active'
    imp = s.controllers.get(IMP) == 'active'
    if arm and imp:
        return 'both'
    return 'arm' if arm else 'impedance' if imp else 'none'


def mode(s):
    """POSITION (arm controller) | TORQUE (impedance) | NONE | CONFLICT |
    UNKNOWN - from the controller manager, never from what the GUI pressed."""
    return {'arm': 'POSITION', 'impedance': 'TORQUE', 'none': 'NONE',
            'both': 'CONFLICT', None: 'UNKNOWN'}[controller(s)]


def imp_loaded(s):
    return s.controllers is not None and IMP in s.controllers


def holding(s):
    """Impedance active and known not to float."""
    return controller(s) == 'impedance' and s.floating is False


def marker_fresh(s, st):
    return (s.marker is not None and s.marker_age is not None
            and s.marker_age <= st.pose_stale_s)


def track_live(s, st):
    """The node's own word that it drives (or is about to), when fresh."""
    return (s.track_age is not None and s.track_age <= st.track_status_stale_s
            and s.track.get('state') in TRACK_LIVE_STATES)


def tracking(s, st):
    return s.tracking or track_live(s, st)


def lead_mm(s, st):
    """Spring lead: the node's while it tracks, else the GUI's own setpoint."""
    if tracking(s, st):
        try:
            v = float(s.track.get('lead_mm', 'nan'))
            return None if math.isnan(v) else v
        except ValueError:
            return None
    return s.setpoint_lead_mm


def force_n(s):
    return None if s.force is None else float(np.linalg.norm(s.force))


def gate_block(s, st):
    """None if the robot may move now, else (reason, fatal) - the Tk panel's
    robot_block. REFLEX is fatal whatever the gate toggle says."""
    if fresh(s, st) and s.robot_mode == MODE_REFLEX:
        return 'robot in REFLEX - run error recovery', True
    if s.user_paused:
        return 'paused by operator', False
    if not s.gate_on:
        return None
    if not fresh(s, st):
        return 'no robot state - is franka_ros2 up?', False
    if s.robot_mode == MODE_MOVE:
        return None
    return GATE_REASONS.get(s.robot_mode, f'robot mode {s.robot_mode}'), False


def under_load(s, st):
    """Why the arm is not in free space, or None. Presets switch only there."""
    f = force_n(s)
    if f is not None and f > st.preset_max_force_n:
        return (f'|F| ext {f:.1f} N > {st.preset_max_force_n:g} N - '
                'switch gains in free space')
    lead = lead_mm(s, st)
    if lead is not None and lead > st.preset_max_lead_mm:
        return (f'spring lead {lead:.1f} mm > {st.preset_max_lead_mm:g} mm - '
                'let the arm settle first')
    return None


def in_box(p, st, floor_m=None):
    """None if base-frame point p is inside the workspace box, else why."""
    x, y, z = p
    if not st.box_x[0] <= x <= st.box_x[1]:
        return f'x {x*1000:.0f} mm outside [{st.box_x[0]*1000:.0f}, {st.box_x[1]*1000:.0f}]'
    if not st.box_y[0] <= y <= st.box_y[1]:
        return f'y {y*1000:.0f} mm outside [{st.box_y[0]*1000:.0f}, {st.box_y[1]*1000:.0f}]'
    if z > st.box_z_max:
        return f'z {z*1000:.0f} mm above the box top {st.box_z_max*1000:.0f}'
    if floor_m is not None and z < floor_m:
        return f'z {z*1000:.0f} mm below the floor {floor_m*1000:.0f}'
    return None


# ---------------------------------------------------------------- enable

def _first(checks):
    for ok, why in checks:
        if not ok:
            return Enable(False, why)
    return Enable(True, '')


def enable(s, st):
    """{control: Enable} for every control. State-driven: what may be pressed
    now follows from the snapshot alone, never from the order of presses."""
    ctl = controller(s)
    imp_on, arm_on = ctl == 'impedance', ctl == 'arm'
    trk = tracking(s, st)
    rm = ROBOT_MODES.get(s.robot_mode, s.robot_mode)
    idle = (s.busy is None, f'waiting for {s.busy}')
    state = (fresh(s, st), 'no robot state - is the driver (terminal 1) up?')
    err = (not robot_error(s, st), 'robot error active - RECOVER first')
    move = (robot_move(s, st), f'robot is {rm}, not MOVE')
    arm = (arm_on, 'impedance holds the arm - RELEASE first' if imp_on
           else 'the controller manager does not answer' if ctl is None
           else f'{ARM} is not active')
    blk = gate_block(s, st)
    gate = (blk is None, f'gate: {blk[0]}' if blk else '')
    marker = (marker_fresh(s, st), s.marker_why or 'marker not visible or stale')
    en = {}

    align = [idle, state, err, (s.moveit_up, 'MoveIt is down'), arm, gate,
             (s.calib == 'loaded', f'no calibration ({s.calib})'), marker]
    en['translate'] = _first(align)
    en['level'] = _first(align)
    en['inplane'] = _first(align + [(s.inplane_target is not None,
                                     'in-plane target is off')])
    en['auto_converge'] = _first(align + [(s.z_floor_set, 'no Z floor set')])
    for name, taught in s.poses.items():
        en[f'goto:{name}'] = _first([idle, state, err, (s.moveit_up, 'MoveIt is down'),
                                     arm, gate, (taught, f'"{name}" is not taught yet '
                                                 '- TEACH it in settings')])

    stationary = s.dq_max is not None and s.dq_max <= st.stationary_rad_s
    en['preflight'] = _first([
        idle, state, err, (ctl is not None, 'the controller manager does not answer'),
        (not imp_on, 'impedance is active - RELEASE first'), arm, move,
        (stationary, 'the arm is moving' if s.dq_max is not None
         else 'no joint velocities yet')])

    rt = (s.rt_rate is not None and s.rt_rate >= st.rt_down,
          'RT link below {:.0f}%'.format(st.rt_down * 100))
    ladder = [idle, state, err, (s.preflight == 'done', 'run PRE-FLIGHT first'
                                 + (' (not known for this session)'
                                    if s.preflight == 'unknown' else '')),
              rt, move, (imp_loaded(s), f'{IMP} is not loaded - is fr3_cell up?'),
              (not trk, 'tracking - end TRACK first')]
    en['float'] = _first(ladder + [(not (imp_on and s.floating), 'already floating')])
    en['hold'] = _first(ladder + [(not holding(s),
                                   'already holding - hold HERE re-seeds')])

    torque = [idle, state, err, (imp_on, 'impedance controller is not active'),
              (s.floating is False, 'floating - press HOLD first'),
              (not trk, 'the tracking node owns the equilibrium')]
    en['setpoint_minus'] = _first(torque + [move])
    en['setpoint_plus'] = en['setpoint_minus']
    en['hold_here'] = _first(torque)

    if trk:
        en['track'] = Enable(True, '')          # the button is END TRACK now
    else:
        m = s.marker or {}
        e_mm, tilt = m.get('err_mm'), m.get('tilt_deg')
        en['track'] = _first(torque + [
            move, (s.track_node_up, 'the tracking node is not running'), marker,
            (e_mm is not None and e_mm <= st.track_entry_mm,
             f'marker error {e_mm or 0:.0f} mm > TRACK entry {st.track_entry_mm:g} mm'
             ' - ALIGN closer first'),
            (tilt is not None and tilt <= st.track_entry_deg,
             f'tilt {tilt or 0:.1f} deg > TRACK entry {st.track_entry_deg:g} deg')])

    # RELEASE stays pressable while the controller manager is silent: the Tk panel's
    # RELEASE always was, and a flaky poll must never strand the operator.
    en['release'] = _first([idle, (imp_on or ctl is None,
                                   'impedance controller is not active')])
    load = under_load(s, st)
    gains = [idle, (s.params_ok, 'controller parameters unavailable'),
             (not trk, 'the tracking node owns the gains until TRACK ends'),
             (load is None, load or '')]
    en['preset'] = _first(gains)
    en['apply_gains'] = _first(gains + [(s.gains_valid, 'a gain is out of range'),
                                        (s.gains_pending, 'nothing to apply')])
    en['speed'] = _first([(not (imp_on and trk),
                           'during TRACK the TRACK SPEED slider sets the limit')])
    # Before TRACK it only stores the value; while tracking it writes live.
    en['track_speed'] = _first([(not trk or s.params_ok,
                                 'controller parameters unavailable')])
    en['recover'] = _first([idle, (s.recover_ready, 'error-recovery server not '
                                   'available - relaunch terminal 1'),
                            (robot_error(s, st), 'no robot error to recover from')])
    en['reload_calib'] = _first([idle])
    en['auto_floor'] = _first([idle, marker, (s.tcp is not None, 'no TCP pose')])
    en['teach'] = _first([idle, (s.tcp is not None, 'no TCP pose')])
    for name in ALWAYS:
        en[name] = Enable(True, '')
    return en


# ---------------------------------------------------------------- banner

def banner(s, st):
    """The one highest-priority fault, or READY with the mode.

    Order (spec): no robot state, MoveIt down, RT down, controller mismatch,
    reflex/error, [command failed], PRE-FLIGHT needed, gate blocked, vision.
    MoveIt counts only while position control is possible, RT only while
    the robot runs a control loop; a failed command stays until the next
    success or DISMISS (HMI rule: failures surface here, not only in the log).
    """
    ctl = controller(s)
    busy_pre = s.busy == 'preflight'      # PRE-FLIGHT idles the arm on purpose
    if not fresh(s, st):
        if (s.state_age is None or s.state_age > st.driver_down_s) and ctl is None:
            return Banner('driver_down', FAULT, 'NO ROBOT STATE - DRIVER DOWN',
                          'robot state stopped and the controller manager is gone '
                          '- relaunch terminal 1, then PRE-FLIGHT')
        return Banner('no_state', FAULT, 'NO ROBOT STATE',
                      'no robot state for {} - is franka_ros2 (terminal 1) up?'.format(
                          'ever' if s.state_age is None else f'{s.state_age:.1f} s'))
    if not s.moveit_up and ctl in ('arm', 'none', None):
        return Banner('moveit', WARN, 'MOVEIT DOWN',
                      'compute_cartesian_path / execute_trajectory unavailable - '
                      'position moves are blocked')
    if (s.robot_mode == MODE_MOVE and s.rt_rate is not None
            and s.rt_rate < st.rt_down):
        return Banner('rt', WARN, f'RT DEGRADED  {s.rt_rate*100:.1f}%',
                      'control success rate is low: a network/RT problem, not a '
                      'gain problem (tools/fr3/fr3_preflight.sh)')
    mismatch = None
    if ctl is None:
        mismatch = 'the controller manager does not answer'
    elif ctl == 'both':
        mismatch = 'both controllers report active'
    elif ctl == 'none' and not busy_pre:
        mismatch = 'no controller holds the arm'
    elif tracking(s, st) and ctl != 'impedance':
        mismatch = 'tracking is live but the impedance controller is not active'
    elif s.tracking and not track_live(s, st):
        mismatch = 'no status from tracking_node - is it still running? end TRACK'
    elif s.torque_attempted and not imp_loaded(s):
        mismatch = f'{IMP} is not loaded - is fr3_cell (terminal 2) up?'
    if mismatch:
        return Banner('controller', WARN, 'CONTROLLER MISMATCH', mismatch)
    if robot_error(s, st):
        names = ', '.join(s.errors) or 'reflex'
        return Banner('reflex', FAULT, 'ROBOT ERROR  -  REFLEX'
                      if s.robot_mode == MODE_REFLEX else 'ROBOT ERROR', names
                      + ('  -  press RECOVER' if s.recover_ready
                         else '  -  no recovery server: relaunch terminal 1'),
                      'recover' if s.recover_ready else None)
    if s.failure:
        cmd, msg = s.failure
        return Banner('failed', WARN, f'{cmd.upper()} FAILED', msg, 'dismiss')
    if s.preflight != 'done' and (ctl == 'impedance' or s.torque_attempted):
        return Banner('preflight', WARN, 'PRE-FLIGHT NEEDED',
                      'impedance is active without a PRE-FLIGHT this session - the '
                      'reflex thresholds are unknown: RELEASE, then PRE-FLIGHT'
                      if ctl == 'impedance' else
                      'torque mode needs PRE-FLIGHT: it sets the collision reflex '
                      'thresholds and zeroes the FCI payload')
    blk = gate_block(s, st)
    if blk is not None and not busy_pre:
        return Banner('gate', WARN, 'PAUSED' if s.user_paused else 'GATE BLOCKED',
                      blk[0] + ('  -  RESUME continues' if s.user_paused else ''))
    if s.robot_mode != MODE_MOVE and not busy_pre:
        return Banner('gate', WARN, 'ROBOT NOT IN MOVE',
                      f'robot is {ROBOT_MODES.get(s.robot_mode, s.robot_mode)}')
    vision_needed = ctl == 'arm' or tracking(s, st) or holding(s)
    if vision_needed and not marker_fresh(s, st):
        if s.marker_age is None:
            text = s.marker_why or 'no marker pose yet - is cam_pub running?'
            return Banner('vision', WARN, 'NO MARKER', text)
        return Banner('vision', WARN, 'VISION STALE',
                      f'last marker pose {s.marker_age*1000:.0f} ms ago'
                      + (f' - {s.marker_why}' if s.marker_why else ''))
    tr = s.track if track_live(s, st) else {}
    if tr.get('state') == 'holding' and tr.get('reason'):
        return Banner('track', WARN, 'TRACK HOLDING', tr['reason']
                      + '  -  the node follows again once clear')
    return Banner('ready', NORMAL, 'READY', ready_detail(s, st))


def ready_detail(s, st):
    ctl = controller(s)
    m = s.marker
    if ctl == 'arm':
        what = f'aligning ({s.busy})' if s.busy else 'arm controller'
        tail = (f'   |e| {m["err_mm"]:.1f} mm   tilt {m["tilt_deg"]:.2f} deg'
                if m else '')
        return f'POSITION: {what}{tail}' + ('   (stopped by operator)'
                                             if s.stopped else '')
    if ctl == 'impedance':
        if tracking(s, st):
            t = s.track

            def v(k):
                x = t.get(k, '')
                return '—' if x in ('', 'nan', '-nan') else x
            return (f'TORQUE: tracking   error {v("pos_err_mm")} mm / {v("rot_err_deg")} deg'
                    f'   lead {v("lead_mm")} mm   over lead: {v("policy")}')
        return 'TORQUE: impedance, ' + ('floating' if s.floating else 'holding')
    return 'no controller active'


# ---------------------------------------------------------------- next step

def next_step(s, st, en):
    """The one control drawn blue: the next mandatory step, or None.

    Align is optional, so it is never blue; the nominal path is
    PRE-FLIGHT -> HOLD -> TRACK, with RECOVER or RELEASE first when the
    state demands it."""
    if not fresh(s, st) or s.busy:
        return None
    if robot_error(s, st):
        return 'recover' if en['recover'].ok else None
    ctl = controller(s)
    if ctl == 'impedance' and s.preflight != 'done':
        return 'release' if en['release'].ok else None
    if ctl == 'arm':
        if s.preflight != 'done':
            return 'preflight' if en['preflight'].ok else None
        return 'hold' if en['hold'].ok else None
    if ctl == 'impedance' and not tracking(s, st):
        if s.floating:
            return 'hold' if en['hold'].ok else None
        return 'track' if en['track'].ok else None
    return None


# ---------------------------------------------------------------- chips

def chips(s, st):
    """FRANKA | MOVEIT | RT | CONTROLLER | PRE-FLIGHT | VISION | GATE - the
    same seven, in the same place, whatever the mode."""
    ctl = controller(s)
    if s.state_age is None:
        franka = Chip('FRANKA', 'no state', FAULT)
    elif not fresh(s, st):
        franka = Chip('FRANKA', f'stale {s.state_age:.1f} s', FAULT)
    else:
        name = CHIP_MODES.get(s.robot_mode, str(s.robot_mode))
        lvl = (NORMAL if s.robot_mode == MODE_MOVE else
               FAULT if s.robot_mode in (MODE_OTHER, MODE_REFLEX) else WARN)
        franka = Chip('FRANKA', name + (' +err' if s.errors else ''),
                      FAULT if s.errors else lvl)
    moveit = Chip('MOVEIT', 'ready' if s.moveit_up else 'down',
                  NORMAL if s.moveit_up else WARN)
    if robot_move(s, st) and s.rt_rate is not None:
        r = s.rt_rate
        rt = Chip('RT', f'{r*100:.1f}%', NORMAL if r >= st.rt_ok
                  else WARN if r >= st.rt_down else FAULT)
    else:
        rt = Chip('RT', '--', NORMAL)
    if ctl is None:
        con = Chip('CONTROLLER', 'no answer', WARN)
    elif ctl == 'both':
        con = Chip('CONTROLLER', 'both active', FAULT)
    elif ctl == 'none':
        con = Chip('CONTROLLER', 'none (PRE-FLIGHT)' if s.busy == 'preflight'
                   else 'none active', NORMAL if s.busy == 'preflight' else WARN)
    elif ctl == 'arm':
        con = Chip('CONTROLLER', ARM, NORMAL)
    else:
        sub = ('tracking' if tracking(s, st) else 'float' if s.floating
               else 'hold' if s.floating is False else '?')
        con = Chip('CONTROLLER', f'impedance · {sub}', NORMAL)
    relevant = ctl == 'impedance' or s.torque_attempted
    pre = Chip('PRE-FLIGHT', s.preflight,
               NORMAL if s.preflight == 'done' or not relevant else WARN)
    if not s.camera_on and s.marker_age is None:
        vis = Chip('VISION', 'off', NORMAL)
    elif s.marker_age is None:
        vis = Chip('VISION', 'no marker', WARN)
    elif marker_fresh(s, st):
        vis = Chip('VISION', f'{s.marker_age*1000:.0f} ms', NORMAL)
    else:
        vis = Chip('VISION', f'stale {s.marker_age*1000:.0f} ms', WARN)
    blk = gate_block(s, st)
    if blk is not None and blk[1]:
        gate = Chip('GATE', 'reflex', FAULT)
    elif s.user_paused:
        gate = Chip('GATE', 'paused', WARN)
    elif not s.gate_on:
        gate = Chip('GATE', 'off', WARN)
    elif blk is None:
        gate = Chip('GATE', 'open', NORMAL)
    else:
        gate = Chip('GATE', 'blocked', WARN)
    return [franka, moveit, rt, con, pre, vis, gate]


# ---------------------------------------------------------------- sliders

def clamp_pct(pct):
    return max(1.0, min(100.0, float(pct)))


def speed_position(pct, st):
    """Slider % -> MoveIt scaling for the NEXT planned move. the Tk panel's retiming
    stretches time by 1/v, so velocity scales by v and acceleration by v^2."""
    v = clamp_pct(pct) / 100.0 * st.position_ceiling_pct / 100.0
    return {'velocity_scale': v, 'accel_scale': v * v, 'slowdown': 1.0 / v}


def speed_torque(pct, st):
    """Slider % -> the controller's equilibrium slew caps (m/s, rad/s)."""
    f = clamp_pct(pct) / 100.0
    return {'setpoint_slew_mps': max(0.001, f * st.slew_max_mps),
            'setpoint_slew_rps': max(0.001, f * st.slew_max_rps)}


def speed_needs_confirm(pct, st):
    return clamp_pct(pct) > st.speed_confirm_pct


def speed_text(pct, m, st):
    """The physical value under the slider."""
    if m == 'TORQUE':
        t = speed_torque(pct, st)
        return (f'{t["setpoint_slew_mps"]*1000:.0f} mm/s   '
                f'{math.degrees(t["setpoint_slew_rps"]):.1f} deg/s  equilibrium slew')
    p = speed_position(pct, st)
    return (f'next move: velocity x{p["velocity_scale"]:.3f}   '
            f'accel x{p["accel_scale"]:.4f}')


# ---------------------------------------------------------------- gains

GAIN_KEYS = ('k_xy', 'k_z', 'k_rp', 'k_yaw', 'zeta')


def validate_presets(presets, limits, zeta_max=1.0):
    """Problems with a preset list, [] when usable. Ordered soft -> stiff:
    every gain must move one way only across the stops."""
    problems = []
    if len(presets) < 2:
        problems.append('need at least two presets')
    names = [p.get('name') for p in presets]
    if len(set(names)) != len(names) or not all(names):
        problems.append('preset names must be present and unique')
    for p in presets:
        for k in GAIN_KEYS:
            v = p.get(k)
            lo, hi = limits[k]
            if not isinstance(v, (int, float)) or not lo <= v <= hi:
                problems.append(f'{p.get("name")}: {k}={v} outside [{lo:g}, {hi:g}]')
        if isinstance(p.get('zeta'), (int, float)) and p['zeta'] > zeta_max:
            problems.append(f'{p.get("name")}: zeta above {zeta_max:g} '
                            '(controller README: do not raise it above 1.0)')
    if problems:
        return problems
    for k in GAIN_KEYS:
        d = np.diff([float(p[k]) for p in presets])
        if (d > 0).any() and (d < 0).any():
            problems.append(f'{k} is not monotonic across the stops')
    return problems


def preset_index(values, presets, tol=1e-9):
    """Index of the preset equal to values, or None ('Custom')."""
    for i, p in enumerate(presets):
        if all(abs(float(values[k]) - float(p[k])) <= tol for k in GAIN_KEYS):
            return i
    return None


def gain_problem(values, limits):
    for k in GAIN_KEYS:
        v = values.get(k)
        lo, hi = limits[k]
        if v is None or not math.isfinite(v) or not lo <= v <= hi:
            return f'{k} must be within [{lo:g}, {hi:g}]'
    return None


# ---------------------------------------------------------------- vision

def marker_errors(p, R, target_m, inplane_target, pos_tol_m, st):
    """the Tk panel's ALIGN readout, from a camera-frame marker measurement."""
    mz = R[:, 2]
    tilt = float(np.degrees(np.arccos(np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
    err = p - np.array([0.0, 0.0, target_m])
    en = float(np.linalg.norm(err))
    ip = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    ip_err = (0.0 if inplane_target is None
              else abs((ip - inplane_target + 180.0) % 360.0 - 180.0))
    return {'dist_mm': float(p[2] * 1000), 'lat_mm': float(np.hypot(p[0], p[1]) * 1000),
            'tilt_deg': tilt, 'ip_deg': ip, 'ip_err_deg': ip_err,
            'err_mm': en * 1000, 'err_xyz_mm': (err * 1000).tolist(),
            'ok': en <= pos_tol_m and tilt <= st.rot_tol_deg
            and ip_err <= st.inplane_tol_deg}


def vision_event(s, st):
    """Why the camera view should enlarge by itself, or None."""
    if s.marker_age is not None and s.marker_age > st.pose_stale_s:
        return f'marker lost ({s.marker_age*1000:.0f} ms)'
    if s.camera_on and s.image_age is not None and s.image_age > st.image_stale_s:
        return f'camera frames stale ({s.image_age:.1f} s)'
    if s.jump_mm > st.pose_jump_mm:
        return f'marker pose jumped {s.jump_mm:.0f} mm between frames'
    return None


def marker_loss_action(policy, is_tracking, marker_age, loss_ms):
    """'stop' | 'release' | None. 'hold' is the tracking node's own
    behaviour, so it never needs the GUI (TODO C2 moves the rest there)."""
    if policy == 'hold' or not is_tracking:
        return None
    if marker_age is None or marker_age * 1000.0 > loss_ms:
        return policy
    return None


def joint_proximity(q, limits):
    """(joint number, % of half-range from centre to the nearer limit) for
    the worst joint, or None."""
    if not q or not limits:
        return None
    worst = None
    for i, (qi, (lo, hi)) in enumerate(zip(q, limits)):
        half = (hi - lo) / 2.0
        if half <= 0:
            continue
        pct = abs(qi - (lo + hi) / 2.0) / half * 100.0
        if worst is None or pct > worst[1]:
            worst = (i + 1, pct)
    return worst
