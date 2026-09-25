"""The panel's commands: the ALIGN and ladder actions, without Tk.

Ported from AlignPane / LadderPane of the Tk panel (cell_panel.py, retired
2026-09-24), which read their settings from Tk variables and wrote to Tk
widgets from worker threads. Here a command reads one frozen Params (written
by the GUI thread only), returns (ok, message), and reports through emit() -
which the window turns into a queued Qt signal. The refusals, the order of
the controller switches and the trace records are the Tk panel's, so
analyse_trace.py still reads the logs.

Cell.run() is the one way a command starts: it marks the command pending
at once, runs it off the GUI thread, and emits 'done' with the outcome. The
stops (stop_now, pause, stop_after) never go through run(): a stop that
waits for a busy flag is not a stop.
"""

import collections
import concurrent.futures
import ctypes
import datetime
import json
import math
import pathlib
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, replace

import numpy as np
import yaml

import logic
import core

HERE = pathlib.Path(__file__).resolve().parent
POSES_PATH = HERE / 'config' / 'poses.yaml'
TORQUE_ATTEMPT_S = 30.0      # the banner remembers a refused torque press this long
IMP_RELOAD_GRACE_S = 15.0    # impedance controller missing this long after a driver restart: reload it
LADDER_FLOOR_M = core.FLOOR_Z_MM / 1000.0


@dataclass(frozen=True)
class Params:
    """The operator's selections at one instant. The GUI thread replaces the
    whole object; workers take one reference and never see it half-written."""
    target_m: float = float(core.TARGET_MM_DEFAULT) / 1000.0
    tol_m: float = float(core.POS_TOL_MM_DEFAULT) / 1000.0
    inplane_target: float = float(core.INPLANE_TARGET_DEFAULT)   # None = off
    step_m: float = float(core.STEP_MM_DEFAULT) / 1000.0
    rot_deg: float = float(core.ROT_DEG_DEFAULT)
    floor_m: float = core.FLOOR_Z_MM / 1000.0                     # ALIGN floor; None = unset
    speed_pct: float = 20.0
    track_speed_pct: float = 10.0
    track_fast: bool = False          # FAST: live only while tracking, off at every START/end
    track_blind: bool = False         # drawer: TRACK may start without the marker; off at launch
    grip_cube_mm: float = 55.0        # drawer: the cube GRIP grips
    grip_force_n: float = 20.0        # drawer: the Hand's grasp force
    setpoint_mm: float = float(core.SETPOINT_MM_DEFAULT)
    axis: str = core.AXIS_CHOICES[0]
    over_lead: str = core.OVER_LEAD_DEFAULT
    marker_loss: str = 'hold'
    marker_loss_ms: float = 1000.0


def load_poses(path=None):
    try:
        doc = yaml.safe_load((path or POSES_PATH).read_text()) or {}
    except FileNotFoundError:
        doc = {}
    return {k: v for k, v in doc.items()}


class Recorder:
    """ros2 bag record, auto-named beside the cell traces. Not the 1 kHz
    robot state (state_recorder does that, and it loads the robot NIC) and
    not the images."""

    TOPICS = [core.ROBOT_STATE_RELAY, '/joint_states', '/aruco/pose',
              '/aruco/pose_raw', core.TRACK_STATUS_TOPIC, core.EQUILIBRIUM_TOPIC,
              f'/{core.IMPEDANCE_CONTROLLER}/transition_event',
              '/trajectory_execution_event', '/cell_panel/heartbeat',
              '/tf', '/tf_static', '/aruco/target_pose_raw', core.GRIP_STATUS_TOPIC]

    def __init__(self):
        self.proc, self.t0, self.path = None, None, None
        # PR_SET_PDEATHSIG fires when the THREAD that forked the child exits,
        # not the process. REC runs on a short-lived action thread, so forking
        # there sent the bag SIGINT at once; fork from this one, which lives
        # as long as the panel.
        self._spawner = concurrent.futures.ThreadPoolExecutor(1, 'bag_spawner')

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def elapsed(self):
        return time.monotonic() - self.t0 if self.running() else None

    def start(self):
        if self.running():
            return False, 'already recording'
        folder = core.trace_dir()
        folder.mkdir(parents=True, exist_ok=True)
        self.path = folder / ('bag_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        libc = ctypes.CDLL('libc.so.6', use_errno=True)
        # SIGINT on parent death: the bag is finalised even if the GUI dies.
        self.proc = self._spawner.submit(
            subprocess.Popen, ['ros2', 'bag', 'record', '-o', str(self.path)] + self.TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=lambda: libc.prctl(1, signal.SIGINT)).result()
        self.t0 = time.monotonic()
        return True, f'recording -> {self.path}'

    def stop(self, timeout_s=10.0):
        if not self.running():
            self.proc = None
            return False, 'not recording'
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None
        return True, f'bag closed: {self.path}'


class Cell:
    """The session: GUI-side state plus every command."""

    def __init__(self, node, settings, emit=None):
        self.n = node
        self.st = settings
        self.emit = emit or (lambda *a: None)
        self.params = Params(speed_pct=settings.speed_default_pct,
                             track_speed_pct=settings.track_speed_default_pct,
                             grip_cube_mm=settings.grip_cube_mm,
                             grip_force_n=settings.grip_force_n,
                             marker_loss=settings.marker_loss_policy,
                             marker_loss_ms=settings.marker_loss_ms)
        # run state
        self.busy = None
        self.abort = False
        self.stopped = False
        self.user_paused = False
        self.gate_on = core.GATE_DEFAULT
        self.paused = False
        self.pause_reason = ''
        self.gate_fault = None
        self.last_outcome = None
        self.failure = None
        self.camera_on = True
        # torque side
        self.preflight = 'unknown'   # nothing can read the thresholds back (TODO C4)
        self.thresholds = None
        self.arm_released = False    # exit_handoff() reads it
        self.floating = None
        self.setpoint = None
        self.tracking = False
        self._tracking_at = float('-inf')
        self._track_seen = None
        self.torque_attempted_at = float('-inf')
        self.pending_gains = None    # the view's numeric fields
        self.session_gains = None    # last applied (or restored): written at activation
        self._loss_fired = False
        self._box_fired = False
        self._imp_active = False
        self._driver_gap = False
        self.reload_impedance = False    # cell.py: True on the real cell, never the mock
        self._imp_missing_since = None
        self._spawner = None
        # align side
        self.R = None
        self.calib_meta = {}
        self.calib_state = 'waiting'
        self.hist = collections.deque(maxlen=900)    # (t, |e| mm, tilt, ip err)
        self.fhist = collections.deque(maxlen=900)   # (t, |F| N, lead mm)
        self._mode_log = collections.deque(maxlen=64)
        self._mode_name = '?'
        # output
        self.tracef = None
        self.tracepath = None
        self._trace_lock = threading.Lock()
        self.recorder = Recorder()
        node.on_mode_change = self._on_mode_change
        node.on_sample = self._sample
        self._read_calib_meta()

    # ------------------------------------------------------------ plumbing

    def say(self, msg):
        self.emit('log', msg)

    def run(self, name, fn, *a):
        """Start one command off the GUI thread. False if another is running."""
        if self.busy:
            self.say(f'REFUSING {name}: waiting for {self.busy}')
            return False
        self.busy = name
        self.emit('pending', name)

        def work():
            ok, msg = False, ''
            try:
                r = fn(*a)
                ok, msg = r if isinstance(r, tuple) else (r is not False, '')
            except Exception as e:                          # noqa: BLE001
                ok, msg = False, f'error: {e}'
                self.say(f'ERROR in {name}: {e}')
            finally:
                self.busy = None
                self.user_paused = False
                if ok:
                    self.failure = None
                elif msg:
                    self.failure = (name, msg)
                self.emit('done', name, ok, msg)
        threading.Thread(target=work, daemon=True).start()
        return True

    def _thread(self, name, fn):
        """A stop-path command: never queued behind busy, never refused."""
        self.emit('pending', name)

        def work():
            ok, msg = False, ''
            try:
                ok, msg = fn()
            except Exception as e:                          # noqa: BLE001
                msg = f'error: {e}'
            finally:
                if not ok and msg:
                    self.failure = (name, msg)      # a failed stop belongs in the banner
                self.emit('done', name, ok, msg)
        threading.Thread(target=work, daemon=True).start()

    def trace(self, rec):
        """One JSONL trace per session, the Tk panel's format. Any thread."""
        with self._trace_lock:
            if self.tracef is None:
                return
            rec['t'] = time.time()
            rec.setdefault('tab', self._mode_name)
            try:
                self.tracef.write(json.dumps(rec, default=float) + '\n')
                self.tracef.flush()
            except Exception as e:                          # noqa: BLE001
                print(f'cell: trace write failed: {e}', file=sys.stderr)

    def open_trace(self, header=None):
        with self._trace_lock:
            if self.tracef is not None:
                return
            folder = core.trace_dir()
            folder.mkdir(parents=True, exist_ok=True)
            self.tracepath = folder / ('cell_' + datetime.datetime.now().strftime(
                '%Y%m%d_%H%M%S') + '.jsonl')
            self.tracef = self.tracepath.open('w')
        self.trace({'rec': 'session_start', **(header or {})})
        self.say(f'trace -> {self.tracepath}')

    def close_trace(self, outcome=None):
        self.trace({'rec': 'session_end', 'outcome': outcome})
        with self._trace_lock:
            if self.tracef is not None:
                self.tracef.close()
                self.tracef = None

    def controller(self):
        return logic.controller(logic.Snap(controllers=self.n.controller_states()))

    def floating_now(self):
        """float_mode as the controller reports it, else as last written."""
        p = self.n.applied_params()
        if p is not None and 'float_mode' in p:
            return bool(p['float_mode'])
        return self.floating

    def slowdown(self, p):
        return logic.speed_position(p.speed_pct, self.st)['slowdown']

    # ------------------------------------------------------------ snapshot

    def snapshot(self):
        """GUI thread, 10 Hz: everything the decisions need, as plain values."""
        n, st, p = self.n, self.st, self.params
        now = time.monotonic()
        s = logic.Snap()
        state = n.state()
        if state is not None:
            pos, _quat, force, mode, rate, age = state
            s.state_age, s.robot_mode, s.rt_rate = age, mode, rate
            s.tcp, s.force = tuple(pos), tuple(force)
        extra = n.extra()
        limits, _src = n.joint_limits()
        if extra is not None:
            q, dq, errs, last = extra
            s.dq_max = max((abs(v) for v in dq[:7]), default=None)
            s.errors, s.last_errors = tuple(errs), tuple(last)
            s.joint_pct = logic.joint_proximity(q[:7], limits)
        s.controllers = n.controller_states()
        ctl = logic.controller(s)
        self._imp_active = ctl == 'impedance'
        s.floating = self.floating_now()
        s.params_ok = n.get_params_cli.service_is_ready()
        s.moveit_up = n.cart.service_is_ready() and n.exec_ac.server_is_ready()
        s.recover_ready = n.recover_ac.server_is_ready()
        m = n.marker()
        s.marker_age = n.marker_age()
        if m is not None:
            s.marker = logic.marker_errors(m[0], m[1], p.target_m, p.inplane_target,
                                           p.tol_m, st)
            self.hist.append((now, s.marker['err_mm'], s.marker['tilt_deg'],
                              s.marker['ip_err_deg']))
        s.marker_why = n.marker_why
        s.image_age = n.image_age()
        s.camera_on = self.camera_on
        s.jump_mm = n.jump_mm()
        s.calib = self.calib_state
        ts = n.track_status()
        if ts is not None:
            s.track, s.track_age = dict(ts[0]), ts[1]
        s.track_node_up = n.track_start_cli.service_is_ready()
        s.grip_node_up = n.grip_cli.service_is_ready()
        s.grip = n.grip_status() or {}
        try:
            s.grip_target_age = max(0.0, time.time() - float(s.grip['target_t']))
        except (KeyError, TypeError, ValueError):
            s.grip_target_age = None
        s.grip_cube_mm = p.grip_cube_mm
        s.busy = self.busy
        s.preflight = self.preflight
        s.thresholds = self.thresholds
        s.torque_attempted = now - self.torque_attempted_at < TORQUE_ATTEMPT_S
        s.tracking = self.tracking
        s.user_paused = self.user_paused
        s.gate_on = self.gate_on
        s.z_floor_set = p.floor_m is not None
        s.inplane_target = p.inplane_target
        if self.setpoint is not None and state is not None and ctl == 'impedance':
            s.setpoint_lead_mm = float(np.linalg.norm(self.setpoint[0] - state[0])) * 1000
        s.failure = self.failure
        s.stopped = self.stopped
        applied = self.applied_gains()
        if self.pending_gains is not None:
            s.gains_valid = logic.gain_problem(self.pending_gains, core.GAIN_LIMITS) is None
            s.gains_pending = applied is None or any(
                abs(self.pending_gains[k] - applied[k]) > 1e-9 for k in logic.GAIN_KEYS)
        s.poses = {k: v is not None for k, v in load_poses_cached().items()}
        s.recording = self.recorder.elapsed()
        s.over_lead_policy = p.over_lead
        s.track_fast = p.track_fast
        s.track_blind = p.track_blind
        if state is not None:
            lead = logic.lead_mm(s, st)
            self.fhist.append((now, logic.force_n(s), lead or 0.0))
        self._mode_name = logic.mode(s)
        return s

    def applied_gains(self):
        """The five gains as the controller runs them, or None."""
        p = self.n.applied_params()
        if p is None or p.get('k_pos_tool') is None or p.get('k_rot_tool') is None:
            return None
        kp, kr = p['k_pos_tool'], p['k_rot_tool']
        return {'k_xy': kp[0], 'k_z': kp[2], 'k_rp': kr[0], 'k_yaw': kr[2],
                'zeta': p.get('damping_ratio')}

    # ------------------------------------------------------------ tick

    def tick(self, s):
        """GUI thread, after each snapshot: the watchers the Tk panel ran."""
        self._drain_mode_log()
        self._follow_track_status()
        self._watch_driver(s)
        self._watch_impedance(s)
        self._watch_marker_loss(s)
        self._watch_box(s)

    def _watch_driver(self, s):
        """A robot-state gap longer than driver_down_s voids PRE-FLIGHT:
        a restarted driver is a new libfranka connection with default
        thresholds, and nothing can read them back (TODO C4)."""
        gap = s.state_age is None or s.state_age > self.st.driver_down_s
        if gap and not self._driver_gap:
            self._driver_gap = True
            self.n.refresh_now()                  # is the controller manager gone too?
            if self.preflight == 'done':
                self.preflight = 'needed'
                self.thresholds = None
                self.say('robot state silent for over '
                         f'{self.st.driver_down_s:g} s - PRE-FLIGHT is void (a '
                         'restarted driver starts from default thresholds). Run '
                         'it again before torque mode.')
                self.trace({'rec': 'driver_down'})
            self.setpoint = None
        elif not gap:
            self._driver_gap = False

    def _watch_impedance(self, s):
        """fr3_cell's spawner loads the impedance controller once, at launch,
        so a restarted driver (T1) comes up without it. Load it again the
        same way - inactive, nothing moves - instead of needing fr3_cell
        restarted. The grace lets the launch's own spawner, which retries
        every 10 s, get there first."""
        if self._spawner is not None and self._spawner.poll() is not None:
            code, self._spawner = self._spawner.returncode, None
            self.say(f'{core.IMPEDANCE_CONTROLLER}: '
                     + ('loaded again, inactive - FLOAT/HOLD available' if code == 0 else
                        f'spawner FAILED (exit {code}) - retrying in {IMP_RELOAD_GRACE_S:g} s'))
            self.trace({'rec': 'reload_impedance_done', 'exit': code})
            self.n.refresh_now()
        if (not self.reload_impedance or s.controllers is None
                or core.IMPEDANCE_CONTROLLER in s.controllers):
            self._imp_missing_since = None
            return
        now = time.monotonic()
        if self._imp_missing_since is None:
            self._imp_missing_since = now
        if self._spawner is not None or now - self._imp_missing_since < IMP_RELOAD_GRACE_S:
            return
        self._imp_missing_since = now                 # a retry waits a full grace again
        self.say(f'{core.IMPEDANCE_CONTROLLER} is not loaded (a restarted driver?) - '
                 'loading it, inactive')
        self.trace({'rec': 'reload_impedance'})
        self._spawner = subprocess.Popen(
            ['ros2', 'run', 'controller_manager', 'spawner', core.IMPEDANCE_CONTROLLER,
             '--inactive', '--param-file', str(core.IMPEDANCE_PARAMS)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _watch_marker_loss(self, s):
        live = logic.tracking(s, self.st)
        if not live:
            self._loss_fired = False
            return
        p = self.params
        act = logic.marker_loss_action(p.marker_loss, live, s.marker_age,
                                       p.marker_loss_ms)
        if act is None or self._loss_fired:
            return
        self._loss_fired = True
        self.say(f'MARKER LOSS: stale over {p.marker_loss_ms:.0f} ms while '
                 f'tracking - {act.upper()} (GUI-side policy, TODO C2)')
        self.trace({'rec': 'marker_loss', 'action': act,
                    'marker_age_s': s.marker_age})
        if act == 'release' and self.run('release', self.release):
            return
        self._thread('stop_tracking', self.stop_tracking)

    def _watch_box(self, s):
        """tracking_node holds at the box itself (handed over at START).
        This only reports a TCP that got outside anyway - an overshoot, or a
        box edited mid-run, which the node takes at the next START."""
        if not logic.tracking(s, self.st) or s.tcp is None:
            self._box_fired = False
            return
        why = logic.in_box(s.tcp, self.st)
        if why and not self._box_fired:
            self._box_fired = True
            self.say(f'WORKSPACE: TCP {why} while tracking - tracking_node holds at the box '
                     '(a box edit applies at the next START)')
            self.trace({'rec': 'box_warn', 'why': why})
        elif not why:
            self._box_fired = False

    # ------------------------------------------------------------ stops

    def stop_now(self):
        """STOP NOW / Esc. Instant MoveIt halt, then whatever holds the
        arm stops too: tracking ends, FLOAT turns into a hold, a gliding
        setpoint is pinned where the arm is. It never releases the arm."""
        self.abort = True
        self.stopped = True
        self.user_paused = False
        self.n.stop_now()

        self.n.grip_stop()                  # a running GRIP/PLACE holds where the arm is

        def work():
            msgs, ok = ['MoveIt halted'], True
            ctl = self.controller()
            if self.tracking or self._track_state() in logic.TRACK_LIVE_STATES:
                t_ok, t_msg = self.stop_tracking()
                ok = ok and t_ok
                msgs.append(f'tracking: {t_msg}')
            elif ctl == 'impedance':
                if self.floating_now():
                    f_ok, f_msg = self.n.set_params({'float_mode': False})
                    if f_ok:
                        self.floating = False
                    ok = ok and f_ok
                    msgs.append('float off - holding where the arm is' if f_ok
                                else f'float off FAILED: {f_msg}')
                else:
                    st = self.n.state()
                    if st is not None:
                        self.setpoint = (st[0], st[1])
                        self.n.publish_equilibrium(st[0], st[1])
                        msgs.append('equilibrium pinned where the arm is')
            self.say('*** STOP NOW - ' + '; '.join(msgs) + ' ***')
            self.trace({'rec': 'stop_now', 'msgs': msgs})
            return ok, '; '.join(msgs)
        self._thread('stop_now', work)

    def pause(self):
        """PAUSE / RESUME. Position: the Tk panel's pause (halt, keep the run, resume
        from rest). Torque: pin the equilibrium where the arm is; while
        tracking there is no pause interface (TODO C3), so it stops tracking."""
        if self.busy in ('grip', 'place', 'place_b'):
            self.n.grip_stop()
            self.say(f'PAUSE during {self.busy.upper()}: grip_node has no pause - stopped it; '
                     'it holds where the arm is. Press it again to redo it from the start.')
            self.trace({'rec': 'pause_grip', 'busy': self.busy})
            return
        if self.controller() == 'impedance':
            if self.tracking or self._track_state() in logic.TRACK_LIVE_STATES:
                self.say('PAUSE while tracking: tracking_node has no pause '
                         '(TODO C3) - stopping tracking instead')
                self._thread('pause', self.stop_tracking)
            else:
                def pin():
                    st = self.n.state()
                    if st is None:
                        return False, 'no robot state'
                    self.setpoint = (st[0], st[1])
                    self.n.publish_equilibrium(st[0], st[1])
                    self.trace({'rec': 'pause_pin', 'anchor': st[0].tolist()})
                    return True, 'equilibrium pinned where the arm is'
                self._thread('pause', pin)
            return
        if not self.busy:
            self.say('PAUSE: nothing is running')
            return
        self.user_paused = not self.user_paused
        if self.user_paused:
            self.n.interrupt()
            self.say('PAUSE - motion halted, run kept. RESUME continues, STOP NOW '
                     'ends it.')
        else:
            self.say('RESUME pressed')
        self.trace({'rec': 'pause' if self.user_paused else 'resume'})

    def stop_after(self):
        """Graceful: let the current move finish, start nothing after it."""
        if self.tracking or self._track_state() in logic.TRACK_LIVE_STATES:
            self.say('stop after current move: ending tracking')
            self._thread('stop_after', self.stop_tracking)
            return
        if not self.busy:
            self.say('stop after current move: nothing is running (a setpoint '
                     'glide always runs to its end)')
            return
        if self.busy in ('grip', 'place', 'place_b'):
            self.say(f'stop after current move: {self.busy.upper()} is one move and runs to '
                     'its end - STOP NOW stops it now')
            return
        self.abort = True
        self.stopped = True
        self.say('STOP (graceful) - cancelling goal; current move may finish.'
                 if self.n.stop() else 'STOP (graceful) - loop will not continue.')
        self.trace({'rec': 'stop_graceful'})

    # ------------------------------------------------------------ gate

    def robot_block(self):
        rm = self.n.robot_mode()
        fresh = rm is not None and rm[1] <= core.ROBOT_STATE_STALE_S
        if fresh and rm[0] == core.MODE_REFLEX:
            return 'robot in REFLEX - run error recovery', True
        if self.user_paused:
            return 'paused by operator', False
        if not self.gate_on:
            return None
        if not fresh:
            return 'no robot state - is franka_ros2 up?', False
        if rm[0] == core.MODE_MOVE:
            return None
        return core.GATE_REASONS.get(rm[0], f'robot mode {rm[0]}'), False

    def wait_gate(self):
        """the Tk panel's wait_gate: block while the robot may not move."""
        blk = self.robot_block()
        if blk is None:
            return not self.abort
        if blk[1]:
            return self._gate_fault(blk[0])
        t0, held = time.time(), None
        self.paused, self.pause_reason = True, blk[0]
        self.say(f'PAUSED - {blk[0]}. '
                 + ('press RESUME to continue' if self.user_paused
                    else 'resumes once the robot is back in MOVE')
                 + '; STOP NOW ends the run.')
        self.trace({'rec': 'gate_pause', 'reason': blk[0]})
        try:
            while not self.abort:
                blk = self.robot_block()
                if blk is not None and blk[1]:
                    return self._gate_fault(blk[0])
                if blk is not None:
                    held, self.pause_reason = None, blk[0]
                elif held is None:
                    held = time.monotonic()
                elif time.monotonic() - held >= core.GATE_RESUME_HOLD_S:
                    break
                time.sleep(0.02)
        finally:
            self.paused = False
        if self.abort:
            return False
        self.load_calib(quiet=True)
        dt = time.time() - t0
        self.say(f'RESUMED after {dt:.1f} s')
        self.trace({'rec': 'gate_resume', 'paused_s': dt})
        return True

    def _gate_fault(self, reason):
        self.gate_fault = 'robot_reflex'
        self.say(f'ABORT: {reason}')
        self.trace({'rec': 'gate_fault', 'reason': reason})
        return False

    def _on_mode_change(self, prev, mode):
        """Spin thread. Halts motion the moment the robot leaves MOVE."""
        if (self.busy and prev == core.MODE_MOVE
                and (self.gate_on or mode == core.MODE_REFLEX)):
            self.n.halt()
        self._mode_log.append((time.time(), prev, mode))

    def _drain_mode_log(self):
        while self._mode_log:
            t, prev, mode = self._mode_log.popleft()
            stamp = datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')
            self.say(f'robot mode {core.ROBOT_MODES.get(prev, prev)} -> '
                     f'{core.ROBOT_MODES.get(mode, mode)}  ({stamp[:-3]})')
            self.trace({'rec': 'robot_mode', 'prev': prev, 'mode': mode})

    def set_gate(self, on):
        if self.busy:
            return False
        self.gate_on = bool(on)
        self.say('robot-state gate ON - motion only while the robot reports MOVE'
                 if self.gate_on else 'robot-state gate OFF - motion no longer '
                 'waits for robot MOVE (a REFLEX still ends a run)')
        return True

    def _resume_hook(self):
        return self.wait_gate

    # ------------------------------------------------------------ calibration

    def _read_calib_meta(self):
        if not core.CALIB_PATH.exists():
            self.calib_meta, self.calib_state = {}, 'missing'
            return
        try:
            self.calib_meta = yaml.safe_load(core.CALIB_PATH.read_text()) or {}
        except Exception:                                   # noqa: BLE001
            self.calib_meta, self.calib_state = {}, 'error'

    def try_autoload(self):
        """GUI timer, every 2 s until loaded: TF may simply not be up yet."""
        if self.R is not None or not core.CALIB_PATH.exists() or self.busy:
            return
        try:
            self.n.tcp_pose()
        except Exception:                                   # noqa: BLE001
            self.calib_state = 'waiting'
            return
        self.load_calib(quiet=True)

    def load_calib(self, quiet=False):
        """Rebuild R_cam_base for the CURRENT arm pose."""
        self._read_calib_meta()
        if not core.CALIB_PATH.exists():
            return False, f'no saved calibration at {core.CALIB_PATH.name}'
        try:
            R_cam_tcp = core.q2R(*self.calib_meta['quat_xyzw']).T
            _, R_base_tcp = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.calib_state = 'waiting' if 'does not exist' in str(e) else 'error'
            return False, f'calibration load failed: {e}'
        self.R = R_cam_tcp @ R_base_tcp.T
        self.calib_state = 'loaded'
        if not quiet:
            self.say(f'calibration loaded ({core.CALIB_PATH.name}), re-based to the '
                     'current pose')
        return True, 'calibration loaded'

    def auto_floor(self):
        """predict the end pose from the current measurement, floor below."""
        m = self.n.marker()
        if m is None:
            return False, 'auto floor: marker not visible'
        pos, _ = self.n.tcp_pose()
        descent = float(m[0][2]) - self.params.target_m
        floor = pos[2] - max(descent, 0.0) - core.FLOOR_MARGIN_M
        self.emit('floor_mm', floor * 1000)
        msg = (f'auto floor: TCP z now {pos[2]*1000:.1f} mm, expected descent '
               f'{descent*1000:.1f} mm -> floor {floor*1000:.1f} mm')
        self.say(msg)
        return True, msg

    # ------------------------------------------------------------ ALIGN

    def _box_refusal(self, goal):
        return logic.in_box(goal, self.st)

    def translate(self, quiet=False):
        p = self.params
        if self.R is None:
            return False, 'no calibration loaded - reload it in settings'
        m = self.n.marker()
        if m is None:
            return False, 'marker not visible'
        err = m[0] - np.array([0, 0, p.target_m])
        d = self.R.T @ err
        nrm = np.linalg.norm(d)
        if nrm < 1e-4:
            self.say('already within 0.1 mm - nothing to do')
            return True, 'already there'
        if nrm > p.step_m:
            d *= p.step_m / nrm
        pos, _ = self.n.tcp_pose()
        why = self._box_refusal(pos + d)
        if why:
            return False, f'BLOCKED by the workspace box: goal {why}'
        if not quiet:
            self.say(f'translate: base delta [{d[0]*1000:+.2f} {d[1]*1000:+.2f} '
                     f'{d[2]*1000:+.2f}] mm (|e|={nrm*1000:.2f} mm)')
        ok, msg = self.n.move(d, z_floor=p.floor_m, slowdown=self.slowdown(p),
                              resume=self._resume_hook())
        self.say(f'  -> {msg}')
        self.trace({'rec': 'translate', 'ok': ok, 'msg': msg,
                    'err_cam_mm': (err * 1000).tolist(),
                    'err_norm_mm': float(nrm * 1000),
                    'cmd_d_base_mm': (d * 1000).tolist(),
                    'marker_pos_cam': m[0].tolist(), **self.n.last_cmd})
        return ok, msg

    def level(self, quiet=False):
        p = self.params
        if self.R is None:
            return False, 'no calibration loaded - reload it in settings'
        m = self.n.marker()
        if m is None:
            return False, 'marker not visible'
        mz = m[1][:, 2]
        tgt = np.array([0, 0, -1.0]) if mz[2] < 0 else np.array([0, 0, 1.0])
        ang = np.arccos(np.clip(mz @ tgt, -1, 1))
        if np.degrees(ang) < 0.15:
            self.say('already square within 0.15 deg')
            return True, 'already square'
        axis = np.cross(mz, tgt)
        if np.linalg.norm(axis) < 1e-8:
            return False, 'degenerate rotation axis'
        tilt_before = float(np.degrees(np.arccos(np.clip(abs(mz @ np.array([0, 0, 1.0])),
                                                         -1, 1))))
        full_ang = float(np.degrees(ang))
        ang = min(ang, np.radians(min(p.rot_deg, core.CEIL_ROT_DEG)))
        Rc = core.axis_angle_R(axis, ang).T
        D = self.R.T @ Rc @ self.R
        if not quiet:
            self.say(f'level: rotating {np.degrees(ang):.2f} deg')
        ok, msg = self.n.move(np.zeros(3), R_delta=D, z_floor=p.floor_m,
                              slowdown=self.slowdown(p), resume=self._resume_hook())
        self.say(f'  -> {msg}')
        rec = {'rec': 'level', 'ok': ok, 'msg': msg, 'tilt_before_deg': tilt_before,
               'full_correction_deg': full_ang, 'clamped_cmd_deg': float(np.degrees(ang)),
               'marker_normal_cam': mz.tolist(),
               'axis_cam': (axis / np.linalg.norm(axis)).tolist(),
               'R_delta_base_quat': core.R2q(D).tolist(), **self.n.last_cmd}
        time.sleep(0.6)
        m2 = self.n.marker()
        if m2 is not None:
            mz2 = m2[1][:, 2]
            rec['tilt_after_deg'] = float(np.degrees(np.arccos(
                np.clip(abs(mz2 @ np.array([0, 0, 1.0])), -1, 1))))
            rec['tilt_change_deg'] = rec['tilt_after_deg'] - tilt_before
        self.trace(rec)
        if ok:
            self.R = Rc.T @ self.R
        return ok, msg

    def inplane(self, quiet=False):
        p = self.params
        tgt = p.inplane_target
        if tgt is None:
            return True, 'in-plane target is off'
        if self.R is None:
            return False, 'no calibration loaded - reload it in settings'
        m = self.n.marker()
        if m is None:
            return False, 'marker not visible'
        cur = core.inplane_angle(m[1])
        if cur is None:
            return False, 'in-plane angle unavailable'
        ang = core.inplane_correction(cur, tgt, min(p.rot_deg, core.CEIL_ROT_DEG))
        if abs(ang) < 1e-3:
            return True, 'in-plane already on target'
        Rc = core.axis_angle_R(np.array([0.0, 0.0, 1.0]), np.radians(ang))
        D = self.R.T @ Rc @ self.R
        if not quiet:
            self.say(f'in-plane: {cur:+.2f} -> {tgt:+.0f} deg, rotating {ang:+.2f} deg')
        ok, msg = self.n.move(np.zeros(3), R_delta=D, z_floor=p.floor_m,
                              slowdown=self.slowdown(p), resume=self._resume_hook())
        self.say(f'  -> {msg}')
        rec = {'rec': 'inplane', 'ok': ok, 'msg': msg, 'inplane_before_deg': cur,
               'target_deg': tgt, 'cmd_deg': float(ang), **self.n.last_cmd}
        time.sleep(0.4)
        m2 = self.n.marker()
        if m2 is not None:
            a2 = core.inplane_angle(m2[1])
            if a2 is not None:
                rec['inplane_after_deg'] = a2
                rec['inplane_change_deg'] = core.wrap_deg(a2 - cur)
        self.trace(rec)
        if ok:
            self.R = Rc.T @ self.R
        return ok, msg

    def start_align(self, name):
        """What a position button press runs: the Tk panel's go_impl prologue."""
        self.stopped = False
        self.abort = False
        self.gate_fault = None
        self.user_paused = False
        blk = self.robot_block()
        if blk is not None:
            return False, f'REFUSING: {blk[0]}'
        # The poller's answer, at most CM_POLL_S old; silence is no answer.
        ctl = self.controller()
        if ctl == 'impedance' or self.tracking:
            return False, f'{core.IMPEDANCE_CONTROLLER} holds the arm - RELEASE first'
        if ctl != 'arm':
            return False, f'{core.ARM_CONTROLLER} is not active ({ctl or "no answer"})'
        fn = {'translate': self.translate, 'level': self.level,
              'inplane': self.inplane, 'auto_converge': self.auto_converge}[name]
        return fn()

    def auto_converge(self):
        p = self.params
        if self.R is None:
            return False, 'no calibration loaded - reload it in settings'
        if p.floor_m is None:
            return False, 'REFUSING: set a TCP Z floor first'
        m0 = self.n.marker()
        if m0 is None:
            return False, 'marker not visible'
        self.last_outcome = None
        err0 = float(np.linalg.norm(m0[0] - np.array([0, 0, p.target_m])))
        cap = int(min(core.MAX_ITERS_ABS, max(60, 3 * err0 / p.step_m + 30)))
        self.say(f'=== AUTO-CONVERGE to {p.target_m*1000:g} mm, tol {p.tol_m*1000:g} mm / '
                 f'{core.ROT_TOL_DEG:g} deg, cap {cap} iters, speed {p.speed_pct:.0f}% ===')
        self.open_trace({'target_mm': p.target_m * 1000, 'pos_tol_mm': p.tol_m * 1000,
                         'rot_tol_deg': core.ROT_TOL_DEG, 'step_mm': p.step_m * 1000,
                         'rot_step_deg': p.rot_deg, 'speed_pct': p.speed_pct,
                         'slowdown': self.slowdown(p), 'z_floor_mm': p.floor_m * 1000,
                         'R_cam_base': self.R.tolist(), 'cap': cap,
                         'inplane_target': ('off' if p.inplane_target is None
                                            else p.inplane_target),
                         'gate': self.gate_on, 'controller': core.ARM_CONTROLLER})
        outcome = 'exception'
        try:
            outcome = self._converge_loop(p, cap)
        finally:
            self.last_outcome = outcome
            self.close_trace(outcome)
            self.say(f'trace written: {self.tracepath}')
        if outcome == 'converged':
            return True, 'converged'
        if outcome == 'stopped':
            return True, 'stopped by operator'
        return False, f'run ended: {outcome.replace("_", " ")}'

    def _converge_loop(self, p, cap):
        """the Tk panel's loop body, on the Params taken at the start of the run."""
        phase, prev, stale = None, np.inf, 0
        seen_edges = self.n.gate_edges
        tgt_ip = p.inplane_target
        for it in range(1, cap + 1):
            if self.abort:
                self.say('ABORTED by STOP')
                return 'stopped'
            if not self.wait_gate():
                return self.gate_fault or 'stopped'
            if self.n.gate_edges != seen_edges:
                seen_edges, phase = self.n.gate_edges, None
            m = self.n.marker()
            if m is None:
                self.say('ABORT: marker lost / stale')
                return 'marker_lost'
            low = self.n.lowest_link(core.FLOOR_LINKS)
            if p.floor_m is not None and low is not None and low[1] < p.floor_m:
                self.say(f'ABORT: Z FLOOR - {low[0]} at {low[1]*1000:.1f} mm < '
                         f'{p.floor_m*1000:.1f} mm')
                return 'z_floor'
            err = float(np.linalg.norm(m[0] - np.array([0, 0, p.target_m])))
            mz = m[1][:, 2]
            tilt = float(np.degrees(np.arccos(np.clip(abs(mz @ np.array([0, 0, 1.0])),
                                                      -1, 1))))
            ip = core.inplane_angle(m[1])
            ip_err = (0.0 if tgt_ip is None or ip is None
                      else abs(core.wrap_deg(ip - tgt_ip)))
            ipmsg = '' if tgt_ip is None else f'  in-plane={ip:+6.2f} deg'
            self.say(f'[{it:02d}] |e|={err*1000:7.2f} mm  tilt={tilt:5.2f} deg{ipmsg}')
            self.trace({'rec': 'iter', 'it': it, 'err_mm': err * 1000, 'tilt_deg': tilt,
                        'marker_pos_cam': m[0].tolist(), 'marker_normal_cam': mz.tolist(),
                        'marker_quat': core.R2q(m[1]).tolist(), 'inplane_deg': ip,
                        'inplane_err_deg': ip_err, 'joints': self.n.joints()})
            if err <= p.tol_m and tilt <= core.ROT_TOL_DEG and ip_err <= core.INPLANE_TOL_DEG:
                self.say(f'=== CONVERGED: {err*1000:.2f} mm, {tilt:.2f} deg in {it} '
                         'iterations ===')
                return 'converged'
            cur = 't' if err > p.tol_m else 'r' if tilt > core.ROT_TOL_DEG else 'p'
            if cur != phase:
                phase, prev, stale = cur, np.inf, 0
            metric = {'t': err, 'r': tilt, 'p': ip_err}[cur]
            if metric < prev - (1e-5 if cur == 't' else 1e-3):
                stale = 0
            else:
                stale += 1
                if stale >= core.NO_PROGRESS_LIMIT:
                    self.say(f'ABORT: no progress in {stale} iterations of phase '
                             f'"{cur}" (reduce the step size)')
                    return f'no_progress_{cur}'
            prev = metric
            ok, _msg = {'t': self.translate, 'r': self.level,
                        'p': self.inplane}[cur](quiet=True)
            if not ok:
                if self.gate_fault:
                    return self.gate_fault
                if self.abort:
                    return 'stopped'
                self.say('ABORT: step failed (see message above)')
                return 'step_failed'
            time.sleep(0.5)
        self.say(f'ABORT: hit iteration cap ({cap})')
        return 'iter_cap'

    # ------------------------------------------------------------ saved poses

    def pose_distance(self, name):
        """(mm, deg) from the TCP to a taught pose, or None."""
        pose = load_poses().get(name)
        if not pose:
            return None
        try:
            pos, Rt = self.n.tcp_pose()
        except Exception:                                   # noqa: BLE001
            return None
        Rg = core.q2R(*pose['quat_xyzw'])
        ang = np.degrees(np.arccos(np.clip((np.trace(Rg @ Rt.T) - 1) / 2, -1, 1)))
        return float(np.linalg.norm(np.array(pose['xyz']) - pos) * 1000), float(ang)

    def goto_pose(self, name):
        """A taught TCP pose through ALIGN's straight-line path and floor."""
        p = self.params
        pose = load_poses().get(name)
        if not pose:
            return False, f'"{name}" is not taught'
        blk = self.robot_block()
        if blk is not None:
            return False, f'REFUSING: {blk[0]}'
        if self.controller() != 'arm':
            return False, f'{core.ARM_CONTROLLER} is not active'
        goal = np.array(pose['xyz'], dtype=float)
        why = logic.in_box(goal, self.st, p.floor_m)
        if why:
            return False, f'BLOCKED: {name} {why}'
        self.stopped = self.abort = False
        pos, Rt = self.n.tcp_pose()
        D = core.q2R(*pose['quat_xyzw']) @ Rt.T
        self.say(f'go to {name}: {np.linalg.norm(goal - pos)*1000:.0f} mm')
        ok, msg = self.n.move(goal - pos, R_delta=D, z_floor=p.floor_m,
                              slowdown=self.slowdown(p), resume=self._resume_hook())
        self.say(f'  -> {msg}')
        self.trace({'rec': 'goto_pose', 'name': name, 'ok': ok, 'msg': msg,
                    **self.n.last_cmd})
        if ok:
            self.load_calib(quiet=True)
        return ok, msg

    def teach(self, name):
        pos, Rt = self.n.tcp_pose()
        doc = load_poses()
        doc[name] = {'xyz': [round(float(v), 5) for v in pos],
                     'quat_xyzw': [round(float(v), 6) for v in core.R2q(Rt)],
                     'taught': datetime.datetime.now().isoformat(timespec='seconds')}
        header = POSES_PATH.read_text().split('\n')
        comments = '\n'.join(ln for ln in header if ln.startswith('#'))
        POSES_PATH.write_text(comments + '\n' + yaml.safe_dump(doc, sort_keys=False))
        _POSE_CACHE['at'] = 0.0
        self.trace({'rec': 'teach', 'name': name, **doc[name]})
        return True, f'taught {name} at z {pos[2]*1000:.0f} mm'

    # ------------------------------------------------------------ ladder

    def blocked(self):
        st = self.n.state()
        if st is None or st[5] > core.STATE_STALE_S:
            return 'no robot state (is franka_ros2 running?)'
        if st[3] == core.MODE_REFLEX:
            return 'robot in REFLEX - run error recovery'
        if st[3] != core.MODE_MOVE:
            return f'robot is {core.ROBOT_MODES.get(st[3], st[3])}, not MOVE'
        return None

    def _wait_mode(self, modes, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            st = self.n.state()
            if st is not None and st[5] <= core.STATE_STALE_S and st[3] in modes:
                return True
            time.sleep(0.05)
        return False

    def note_torque_attempt(self):
        """A torque press refused for want of PRE-FLIGHT: the banner owes
        the operator 'PRE-FLIGHT NEEDED' (it only says so on the way in)."""
        self.torque_attempted_at = time.monotonic()

    def preflight_run(self):
        """the Tk panel's PRE-FLIGHT: zero the FCI payload and set the reflex
        thresholds with the robot idle, the arm controller restored on every
        path."""
        self.preflight = 'needed' if self.preflight == 'done' else self.preflight
        states = self.n.controllers()
        if states is None:
            return False, 'the controller manager is not answering'
        if states.get(core.IMPEDANCE_CONTROLLER) == 'active':
            return False, 'RELEASE the impedance controller before PRE-FLIGHT'
        if states.get(core.ARM_CONTROLLER) != 'active':
            return False, f'{core.ARM_CONTROLLER} is not active'
        why = self.blocked()
        if why is not None:
            return False, why
        extra = self.n.extra()
        if extra is not None and max(abs(v) for v in extra[1][:7]) > self.st.stationary_rad_s:
            return False, 'the arm is moving - wait until it is still'
        self.open_trace()
        self.say(f'PRE-FLIGHT: releasing {core.ARM_CONTROLLER} so the robot goes idle')
        load_ok = collision_ok = restored = False
        load_msg = collision_msg = ''
        self.arm_released = True
        try:
            ok, msg = self.n.switch([], [core.ARM_CONTROLLER])
            if not ok:
                return False, f'could not release {core.ARM_CONTROLLER} ({msg})'
            if not self._wait_mode({core.MODE_IDLE}, core.PREFLIGHT_MODE_TIMEOUT_S):
                return False, 'the robot never reported IDLE'
            load_ok, load_msg = self.n.set_load(0.0, [0.0] * 3, [0.0] * 3)
            self.say('  FCI payload zeroed: ' + ('set' if load_ok else f'FAILED ({load_msg})'))
            collision_ok, collision_msg = self.n.set_collision_behavior()
            self.say(f'  collision reflex at {core.COLLISION_WRENCH[0]:.0f} N / '
                     f'{core.COLLISION_WRENCH[3]:.0f} Nm, contact flag at '
                     f'{core.CONTACT_WRENCH[0]:.0f} N: '
                     + ('set' if collision_ok else f'FAILED ({collision_msg})'))
        finally:
            restored = self._restore_arm_controller()
            done = bool(load_ok and collision_ok and restored)
            self.preflight = 'done' if done else 'needed'
            self.thresholds = ({'contact_n': core.CONTACT_WRENCH[0],
                                'reflex_n': core.COLLISION_WRENCH[0]} if done else None)
            self.trace({'rec': 'preflight', 'ok': done, 'mass_kg': 0.0,
                        'load_ok': load_ok, 'collision_ok': collision_ok,
                        'restored': restored,
                        'contact_torque_nm': core.CONTACT_TORQUE_NM,
                        'collision_torque_nm': core.COLLISION_TORQUE_NM,
                        'contact_wrench': core.CONTACT_WRENCH,
                        'collision_wrench': core.COLLISION_WRENCH})
        if self.preflight != 'done':
            return False, ('PRE-FLIGHT failed: ' + (load_msg if not load_ok else
                           collision_msg if not collision_ok else
                           'arm controller not restored'))
        st = self.n.state()
        rest = float('nan') if st is None else float(np.linalg.norm(st[2]))
        warn = (' - that bias eats into the reflex margin: check the payload'
                if not rest <= core.REST_FORCE_WARN_N else '')
        self.say(f'PRE-FLIGHT done. |F ext| at rest {rest:.1f} N{warn}. Keep hand '
                 f'pushes under {core.PUSH_LIMIT_N:.0f} N.')
        self.trace({'rec': 'preflight_rest_force', 'force_n': rest})
        return True, f'PRE-FLIGHT done, |F ext| at rest {rest:.1f} N'

    def _restore_arm_controller(self):
        states = self.n.controllers()
        if states is not None and states.get(core.ARM_CONTROLLER) == 'active':
            back, back_msg = True, 'already active'
        else:
            back, back_msg = self.n.switch([core.ARM_CONTROLLER], [])
        restored = back and self._wait_mode({core.MODE_MOVE}, core.PREFLIGHT_MODE_TIMEOUT_S)
        if restored:
            self.arm_released = False
            self.say(f'  {core.ARM_CONTROLLER} active again')
        else:
            self.say(f'*** {core.ARM_CONTROLLER} did NOT come back '
                     f'({back_msg if not back else "robot not in MOVE"}) - relaunch '
                     'the stack before anything else ***')
        self.n.refresh_now()
        return restored

    def _activate(self, float_mode):
        if self.preflight != 'done':
            self.note_torque_attempt()
            return False, 'run PRE-FLIGHT first - reflex thresholds must be set for this session'
        why = self.blocked()
        if why is not None:
            return False, why
        states = self.n.controllers()
        if states is None:
            return False, 'the controller manager is not answering'
        if states.get(core.IMPEDANCE_CONTROLLER) is None:
            return False, (f'{core.IMPEDANCE_CONTROLLER} is not loaded - after a driver '
                           f'restart the panel loads it within ~{IMP_RELOAD_GRACE_S + 5:g} s')
        values = {'float_mode': bool(float_mode)}
        g = self.session_gains
        restore = (states.get(core.IMPEDANCE_CONTROLLER) != 'active' and g is not None
                   and logic.gain_problem(g, core.GAIN_LIMITS) is None)
        if restore:
            # A relaunch reloads the yaml gains; put the operator's back in
            # the same atomic set, before activation seeds the equilibrium
            # where the arm is (zero spring force, so the change is free).
            values.update({'k_pos_tool': [g['k_xy'], g['k_xy'], g['k_z']],
                           'k_rot_tool': [g['k_rp'], g['k_rp'], g['k_yaw']],
                           'damping_ratio': g['zeta']})
        ok, msg = self.n.set_params(values)
        if not ok:
            return False, f'could not set float_mode{" and gains" if restore else ""}: {msg}'
        if restore:
            self.say(f'gains restored for this activation: k_pos [{g["k_xy"]:.0f} '
                     f'{g["k_xy"]:.0f} {g["k_z"]:.0f}] N/m, k_rot [{g["k_rp"]:.0f} '
                     f'{g["k_rp"]:.0f} {g["k_yaw"]:.0f}] Nm/rad, zeta {g["zeta"]:.2f}')
        self.floating = bool(float_mode)
        if states.get(core.IMPEDANCE_CONTROLLER) == 'active':
            return True, 'already active'
        ok, msg = self.n.switch([core.IMPEDANCE_CONTROLLER], [core.ARM_CONTROLLER])
        self.n.refresh_now()
        if not ok:
            return False, f'could not activate {core.IMPEDANCE_CONTROLLER}: {msg}'
        self.setpoint = None
        self.say(f'{core.IMPEDANCE_CONTROLLER} ACTIVE - the arm is compliant now')
        self.trace({'rec': 'activate', 'float_mode': self.floating})
        # Never inherit a slew from an earlier session: write this session's,
        # on every activation. Above the confirm threshold only the threshold
        # goes out; the window then offers CONFIRM for the rest.
        pct = self.params.speed_pct
        if logic.speed_needs_confirm(pct, self.st):
            pct = self.st.speed_confirm_pct
            self.say(f'speed capped at {pct:.0f}% on activation - CONFIRM the slider '
                     f'value ({self.params.speed_pct:.0f}%) to go faster')
        ok, msg = self.write_speed(pct)
        if not ok:
            self.say(f'speed limit NOT written ({msg}) - the controller keeps its previous '
                     'slew')
        return True, 'activated'

    def float_on(self):
        if self.tracking:
            return False, 'FLOAT while tracking would free the arm under it'
        ok, msg = self._activate(True)
        if not ok:
            return ok, msg
        self.say('FLOAT: move the arm gently by hand - smooth, no buzz, no kicks. '
                 f'Keep |F| ext under {core.PUSH_LIMIT_N:.0f} N.')
        self.trace({'rec': 'float_on'})
        return True, 'floating'

    def hold_on(self):
        ctl = self.controller()
        if ctl == 'impedance' and self.floating_now() is False:
            return True, 'already holding - hold HERE re-seeds the equilibrium'
        if ctl != 'impedance':
            ok, msg = self._activate(False)
            if not ok:
                return ok, msg
        else:
            ok, msg = self.n.set_params({'float_mode': False})
            if not ok:
                return False, f'float_mode not cleared: {msg}'
            self.floating = False
        self.setpoint = None
        self.say('HOLD: the controller re-seeded its equilibrium where the arm is '
                 f'now. Setpoint floor {core.FLOOR_Z_MM:.0f} mm above the base.')
        self.trace({'rec': 'hold_on', 'gains': self.applied_gains(),
                    'z_floor': LADDER_FLOOR_M})
        return True, 'holding'

    def setpoint_step(self, sign):
        p = self.params
        if self.tracking:
            return False, 'the tracking node owns the equilibrium'
        if self.controller() != 'impedance' or self.floating_now() is not False:
            return False, 'SETPOINT needs the controller holding (HOLD first)'
        why = self.blocked()
        st = self.n.state()
        if why is not None or st is None:
            return False, why or 'no robot state'
        pos, quat = st[0], st[1]
        step = min(float(p.setpoint_mm), 50.0) / 1000.0 * sign
        if p.axis.startswith('tool Z'):
            d = core.q2R(*quat)[:, 2] * step
        else:
            d = np.zeros(3)
            d[{'base X': 0, 'base Y': 1}.get(p.axis, 2)] = step
        anchor = (self.setpoint[0] if self.setpoint is not None else pos) + d
        if anchor[2] < LADDER_FLOOR_M and d[2] < 0.0:
            return False, (f'that puts the equilibrium at z {anchor[2]*1000:.0f} mm, below '
                           f'the floor {LADDER_FLOOR_M*1000:.0f} mm')
        why = logic.in_box(anchor, self.st)
        if why and not (self.setpoint is None and logic.in_box(pos, self.st)):
            return False, f'equilibrium would leave the workspace box: {why}'
        lead = float(np.linalg.norm(anchor - pos))
        if lead * 1000.0 > core.MAX_LEAD_MM:
            return False, (f'equilibrium {lead*1000:.0f} mm from the arm (cap '
                           f'{core.MAX_LEAD_MM:.0f} mm) - wait for the arm to catch up')
        self.setpoint = (anchor, quat)
        self.n.publish_equilibrium(anchor, quat)
        self.say(f'setpoint {step*1000:+.0f} mm along {p.axis} - lead now '
                 f'{lead*1000:.1f} mm; the arm glides at the slew limit')
        self.trace({'rec': 'setpoint', 'axis': p.axis, 'step_mm': step * 1000,
                    'anchor': anchor.tolist(), 'lead_mm': lead * 1000})
        return True, f'setpoint {step*1000:+.0f} mm'

    def hold_here(self):
        if self.tracking:
            return False, 'the tracking node owns the equilibrium'
        if self.controller() != 'impedance' or self.floating_now() is not False:
            return False, 'nothing to re-seed: the controller is not holding'
        st = self.n.state()
        if st is None:
            return False, 'no robot state'
        self.setpoint = (st[0], st[1])
        self.n.publish_equilibrium(st[0], st[1])
        self.trace({'rec': 'hold_here', 'anchor': st[0].tolist()})
        return True, 'equilibrium re-seeded at the current pose'

    # ------------------------------------------------------------ tracking

    def _track_state(self):
        got = self.n.track_status()
        if got is None or got[1] > core.TRACK_STATUS_STALE_S:
            return None
        return got[0].get('state')

    def _set_tracking(self, on):
        self.tracking = on
        self.params = replace(self.params, track_fast=False)
        self.setpoint = None
        self._tracking_at = time.monotonic()

    def _follow_track_status(self):
        """the node's latched status is the truth about tracking."""
        got = self.n.track_status()
        if got is None or got[1] > core.TRACK_STATUS_STALE_S:
            return
        f, age = got
        key = (f.get('state'), f.get('reason'), f.get('policy'))
        if key != self._track_seen:
            self._track_seen = key
            self.trace({'rec': 'track_status', **f})
        live = f.get('state') in logic.TRACK_LIVE_STATES
        if (live != self.tracking and not self.busy
                and time.monotonic() - age > self._tracking_at):
            if live:
                self.floating = False
                self.say(f'{core.TRACKING_NODE} is already tracking - adopted')
            else:
                self.say(f'{core.TRACKING_NODE} stopped tracking: '
                         f'{f.get("reason") or f.get("message")}')
            self._set_tracking(live)

    def start_tracking(self):
        p = self.params
        why = self.blocked()
        if why is not None:
            return False, why
        if self.controller() != 'impedance' or self.floating_now() is not False:
            return False, 'TRACK needs the controller holding - HOLD first'
        e = {'err_mm': None, 'tilt_deg': None}
        if not p.track_blind:             # blind: the node holds until it sees the marker
            m = self.n.marker()
            if m is None:
                return False, 'marker not visible'
            e = logic.marker_errors(m[0], m[1], p.target_m, p.inplane_target, p.tol_m,
                                    self.st)
            if p.over_lead != 'clamp' and e['err_mm'] > self.st.track_max_lead_mm:
                return False, (f'marker error {e["err_mm"]:.0f} mm is beyond the tracking '
                               f'node\'s {self.st.track_max_lead_mm:g} mm lead cap - with over '
                               f'lead \'{p.over_lead}\' it would not move: ALIGN closer, or set '
                               'over lead to clamp')
            if (e['err_mm'] > self.st.track_entry_mm
                    or e['tilt_deg'] > self.st.track_entry_deg):
                return False, (f'marker error {e["err_mm"]:.0f} mm / {e["tilt_deg"]:.1f} deg '
                               f'is above the TRACK entry ({self.st.track_entry_mm:g} mm / '
                               f'{self.st.track_entry_deg:g} deg)')
        goal = {'tracking_standoff_m': p.target_m,
                'tracking_inplane_hold': p.inplane_target is None,
                'tracking_over_lead_policy': p.over_lead,
                # the drawer box: the node holds at it (read at START only)
                'tracking_box_x_m': list(self.st.box_x),
                'tracking_box_y_m': list(self.st.box_y),
                'tracking_box_z_max_m': float(self.st.box_z_max)}
        if p.inplane_target is not None:
            goal['tracking_inplane_deg'] = float(p.inplane_target)
        ok, msg = self.n.set_tracking_params(goal)
        self.trace({'rec': 'track_goal', 'ok': ok, 'msg': msg, **goal,
                    'entry_err_mm': e['err_mm'], 'entry_tilt_deg': e['tilt_deg'],
                    'entry_limit_mm': self.st.track_entry_mm,
                    'entry_limit_deg': self.st.track_entry_deg,
                    'track_speed_pct': p.track_speed_pct, 'track_blind': p.track_blind})
        if not ok:
            return False, f'could not hand the goal to {core.TRACKING_NODE} ({msg})'
        self.open_trace()
        ok, msg = self.n.call_trigger(self.n.track_start_cli)
        self.trace({'rec': 'track_start', 'ok': ok, 'msg': msg})
        if not ok:
            return False, f'tracking NOT started: {msg}'
        self._set_tracking(True)
        self.n.refresh_now()
        # The node has just put its own profile slew (100 mm/s) in force and
        # is already streaming: override it with TRACK SPEED at once. It
        # restores the pre-TRACK slew at STOP. Re-reading the profile at
        # START would close this gap on the node side (TODO C9).
        s_ok, s_msg = self.write_speed(p.track_speed_pct)
        if not s_ok:
            t_ok, t_msg = self.stop_tracking()
            return False, (f'TRACK SPEED not applied ({s_msg}) - tracking stopped rather '
                           f'than run at the node\'s profile speed ({t_msg})')
        self.say(f'TRACKING: {msg} at {p.track_speed_pct:.0f}% - the arm follows the '
                 'marker. STOP NOW or END TRACK ends it.')
        return True, msg

    def grip(self):
        """GRIP: grip_node reads the cube from the marker, opens, glides above
        it, descends, grasps and lifts - on the impedance controller, with the
        operator's gains. Blocks until it is done; STOP NOW stops it."""
        p = self.params
        self.abort = self.stopped = False
        why = self.blocked()
        if why is not None:
            return False, why
        if self.tracking:
            return False, 'end TRACK first - GRIP and TRACK both drive the equilibrium'
        # grip_node moves the equilibrium: a later SETPOINT +/- must re-anchor
        # at the arm, not step from wherever the panel last put it.
        self.setpoint = None
        ok, msg = self.n.set_grip_params({
            'grip_cube_m': p.grip_cube_mm / 1000.0, 'grip_force_n': float(p.grip_force_n),
            **self._grip_box()})
        if not ok:
            return False, f'could not hand the cube size to {core.GRIP_NODE} ({msg})'
        self.open_trace()
        self.say(f'GRIP: {p.grip_cube_mm:.0f} mm cube, {p.grip_force_n:.0f} N - STOP NOW holds '
                 'where the arm is')
        if self.abort:                    # STOP NOW while the parameters went out
            return False, 'GRIP not started: STOP NOW'
        ok, msg = self.n.call_trigger(self.n.grip_cli, core.GRIP_CALL_TIMEOUT_S, 'grip_node')
        self.setpoint = None
        self.trace({'rec': 'grip', 'ok': ok, 'msg': msg, 'cube_mm': p.grip_cube_mm,
                    'force_n': p.grip_force_n})
        self.say(f'GRIP: {msg}')
        return ok, msg

    def _grip_box(self):
        """The drawer's box as grip_node's parameters: sent before EVERY
        sequence, so a box narrowed after GRIP binds PLACE too."""
        return {'tracking_box_x_m': list(self.st.box_x), 'tracking_box_y_m': list(self.st.box_y),
                'tracking_box_z_max_m': float(self.st.box_z_max)}

    def place_at(self):
        """PLACE AT B: carry the held cube to target marker B and set it down."""
        self.abort = self.stopped = False
        why = self.blocked()
        if why is not None:
            return False, why
        if self.tracking:
            return False, 'end TRACK first - PLACE AT B and TRACK both drive the equilibrium'
        ok, msg = self.n.set_grip_params(self._grip_box())
        if not ok:
            return False, f'could not hand the workspace box to {core.GRIP_NODE} ({msg})'
        if self.abort:
            return False, 'PLACE AT B not started: STOP NOW'
        self.setpoint = None
        ok, msg = self.n.call_trigger(self.n.place_at_cli, core.GRIP_CALL_TIMEOUT_S, 'grip_node')
        self.setpoint = None
        self.trace({'rec': 'place_at', 'ok': ok, 'msg': msg})
        self.say(f'PLACE AT B: {msg}')
        return ok, msg

    def place(self):
        self.abort = self.stopped = False
        why = self.blocked()
        if why is not None:
            return False, why
        if self.tracking:
            return False, 'end TRACK first - PLACE and TRACK both drive the equilibrium'
        ok, msg = self.n.set_grip_params(self._grip_box())
        if not ok:
            return False, f'could not hand the workspace box to {core.GRIP_NODE} ({msg})'
        if self.abort:
            return False, 'PLACE not started: STOP NOW'
        self.setpoint = None
        ok, msg = self.n.call_trigger(self.n.place_cli, core.GRIP_CALL_TIMEOUT_S, 'grip_node')
        self.setpoint = None
        self.trace({'rec': 'place', 'ok': ok, 'msg': msg})
        self.say(f'PLACE: {msg}')
        return ok, msg

    def stop_tracking(self):
        """Never refuses."""
        ok, msg = self.n.call_trigger(self.n.track_stop_cli)
        self._set_tracking(False)
        self.n.refresh_now()                      # the node put its snapshot back
        self.say(f'STOP TRACKING: {msg}')
        self.trace({'rec': 'track_stop', 'ok': ok, 'msg': msg})
        return ok, msg

    def write_policy(self, v):
        ok, msg = self.n.set_tracking_params({'tracking_over_lead_policy': v},
                                             atomic=False)
        self.say(f'over-lead policy -> {v}' + ('' if ok else f': NOT written ({msg})'))
        self.trace({'rec': 'track_policy', 'policy': v, 'ok': ok, 'msg': msg})
        return ok, msg

    # ------------------------------------------------------------ release etc.

    def release(self):
        if not self.n.cm_reachable():
            return False, 'the controller manager is not answering - press RELEASE again'
        states = self.n.controllers()
        if states is None:
            return False, 'the controller manager did not answer - press RELEASE again'
        if states.get(core.IMPEDANCE_CONTROLLER) != 'active':
            self.setpoint = None
            return True, 'impedance controller is not active'
        t_ok, t_msg = self.n.call_trigger(self.n.track_stop_cli)
        if t_ok:
            self.say(f'tracking stopped first: {t_msg}')
        self._set_tracking(False)
        ok, msg = self.n.switch([core.ARM_CONTROLLER], [core.IMPEDANCE_CONTROLLER])
        self.n.refresh_now()
        if not ok:
            self.trace({'rec': 'release', 'ok': False, 'msg': msg})
            return False, (f'RELEASE FAILED ({msg}) - the arm is still on the impedance '
                           'controller; use the robot E-stop if it is not behaving')
        self.setpoint = None
        self.say(f'released - {core.ARM_CONTROLLER} holds the arm again')
        self.trace({'rec': 'release', 'ok': True})
        self.close_trace()
        self.load_calib(quiet=True)
        return True, 'released'

    def apply_gains(self, g):
        if self.tracking:
            return False, 'the tracking node owns the gains until TRACK ends'
        err = logic.gain_problem(g, core.GAIN_LIMITS)
        if err is not None:
            return False, err
        ok, msg = self.n.set_params({'k_pos_tool': [g['k_xy'], g['k_xy'], g['k_z']],
                                     'k_rot_tool': [g['k_rp'], g['k_rp'], g['k_yaw']],
                                     'damping_ratio': g['zeta']})
        self.n.refresh_now()
        if ok:
            self.session_gains = dict(g)
            self.emit('gains_applied', dict(g))
        self.say(f'gains {"applied" if ok else "REFUSED"}: k_pos [{g["k_xy"]:.0f} '
                 f'{g["k_xy"]:.0f} {g["k_z"]:.0f}] N/m, k_rot [{g["k_rp"]:.0f} '
                 f'{g["k_rp"]:.0f} {g["k_yaw"]:.0f}] Nm/rad, zeta {g["zeta"]:.2f}'
                 + ('' if ok else f' ({msg})'))
        self.trace({'rec': 'gains', 'ok': ok, **g})
        return ok, msg

    def write_speed(self, pct):
        """Torque mode: the controller's own slew caps - no second limiter."""
        vals = logic.speed_torque(pct, self.st)
        ok, msg = self.n.set_params(vals)
        self.n.refresh_now()
        self.say(f'speed {pct:.0f}% -> slew {vals["setpoint_slew_mps"]*1000:.0f} mm/s, '
                 f'{math.degrees(vals["setpoint_slew_rps"]):.1f} deg/s'
                 + ('' if ok else f': REFUSED ({msg})'))
        self.trace({'rec': 'speed', 'pct': pct, 'ok': ok, **vals})
        return ok, msg

    def track_pct(self):
        p = self.params
        return self.st.track_fast_pct if p.track_fast else p.track_speed_pct

    def set_track_fast(self, on):
        """FAST: track_fast_pct while tracking; off goes back to TRACK SPEED.
        The click is the confirm - a labelled switch, not a slider drag."""
        if not self.tracking:
            self.params = replace(self.params, track_fast=False)
            return False, 'FAST works only while tracking'
        self.params = replace(self.params, track_fast=on)
        ok, msg = self.write_speed(self.track_pct())
        if not ok:
            self.params = replace(self.params, track_fast=False)
        return ok, msg

    def recover(self):
        ok, msg = self.n.recover()
        self.say(f'ERROR RECOVERY: {msg}')
        self.trace({'rec': 'recover', 'ok': ok, 'msg': msg})
        return ok, msg

    def toggle_record(self):
        stopping = self.recorder.running()
        ok, msg = self.recorder.stop() if stopping else self.recorder.start()
        self.say(msg)
        # The camera's own frames, recorded inside vision_standalone (images
        # never go on DDS); cam_pub has no record_dir and says so.
        v_ok, v_msg = self.n.set_vision_record('' if stopping else
                                               str(core.trace_dir() / 'vision'))
        if not stopping:
            self.say('camera frames: ' + (f'recording -> {core.trace_dir() / "vision"}' if v_ok
                                          else f'NOT recorded ({v_msg}) - launch with '
                                               'vision_source:=standalone to record them'))
        return ok, msg

    # ------------------------------------------------------------ 50 Hz sample

    def _sample(self, s):
        """Spin thread, every relayed state (50 Hz), while a trace is open and
        impedance holds the arm - the Tk panel's hold-test record."""
        if self.tracef is None or not self._imp_active:
            return
        anchor = self.setpoint
        self.trace({'rec': 'sample', **s,
                    'anchor': None if anchor is None else list(anchor[0]),
                    'floating': self.floating})


_POSE_CACHE = {'at': 0.0, 'poses': {}}


def load_poses_cached(max_age_s=2.0):
    """poses.yaml, re-read at most every two seconds (the snapshot is 10 Hz)."""
    now = time.monotonic()
    if now - _POSE_CACHE['at'] > max_age_s:
        _POSE_CACHE['poses'] = load_poses()
        _POSE_CACHE['at'] = now
    return _POSE_CACHE['poses']

