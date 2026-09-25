"""Robot-state gate and PAUSE decisions, pinned without a robot.

This logic only really runs when a run is interrupted mid-motion, which is
exactly when a logic slip is least welcome. So:

  * run_resumable retries ONLY failures caused by an interruption
  * robot_block fails closed, and a REFLEX ends a run even with the gate off
  * wait_gate needs an unbroken all-clear before resuming; STOP always wins
  * the mode callback counts MOVE exits and halts motion from the spin thread
  * ALIGN never moves while the impedance controller or the tracking node
    holds the arm

Ported from the Tk panel's test_enable_gate.py (retired 2026-09-24): the
same cases, against actions.Cell and core.run_resumable.
"""

import collections
import threading
import time
import types

import pytest

import actions
import core

MOVE, IDLE, REFLEX, USER_STOPPED = 2, 1, 4, 5


class _Node:
    def __init__(self, mode=MOVE, age=0.0):
        self.mode, self.age = mode, age
        self.gate_edges = 0
        self.halts = 0

    def robot_mode(self):
        return None if self.mode is None else (self.mode, self.age)

    def halt(self):
        self.halts += 1


def _gui(node, gate_on=True, busy=True):
    g = types.SimpleNamespace(
        n=node, gate_on=gate_on, abort=False, paused=False, pause_reason='',
        gate_fault=None, user_paused=False, busy=busy, logs=[], traces=[], reloads=0,
        _mode_log=collections.deque())
    g.say = g.logs.append
    g.trace = g.traces.append

    def load_calib(quiet=False):
        g.reloads += 1
    g.load_calib = load_calib
    for name in ('robot_block', 'wait_gate', '_gate_fault', '_on_mode_change',
                 '_resume_hook'):
        setattr(g, name, types.MethodType(getattr(actions.Cell, name), g))
    return g


def _later(delay_s, fn):
    t = threading.Timer(delay_s, fn)
    t.start()
    return t


@pytest.fixture
def fast_hold(monkeypatch):
    monkeypatch.setattr(core, 'GATE_RESUME_HOLD_S', 0.15)
    return 0.15


# ---- run_resumable ---------------------------------------------------------

def test_mode_constants_match_franka_msgs():
    """franka_msgs/FrankaRobotState: MOVE=2, REFLEX=4, USER_STOPPED=5."""
    assert (core.MODE_MOVE, core.MODE_REFLEX, core.MODE_USER_STOPPED) == (2, 4, 5)


def test_success_runs_once():
    calls = []
    ok, _ = core.run_resumable(lambda: calls.append(1) or (True, 'executed'),
                               lambda: 0, lambda: True)
    assert ok and len(calls) == 1


def test_real_failure_is_never_retried():
    """A planning failure with nothing interrupting must not loop forever."""
    calls = []

    def attempt():
        calls.append(1)
        assert len(calls) <= 2, 'retried a failure the robot did not cause'
        return False, 'path only 40% complete'
    ok, msg = core.run_resumable(attempt, lambda: 0, lambda: True)
    assert not ok and msg == 'path only 40% complete' and len(calls) == 1


def test_interrupted_attempt_resumes_and_finishes():
    edges, calls = [0], []

    def attempt():
        calls.append(1)
        if len(calls) == 1:
            edges[0] += 1               # robot left MOVE during this attempt
            return False, 'execution error code -7'
        return True, 'executed'
    ok, _ = core.run_resumable(attempt, lambda: edges[0], lambda: True)
    assert ok and len(calls) == 2


def test_stop_during_pause_gives_up_without_moving():
    edges, calls, answers = [0], [], iter([True, False])

    def attempt():
        calls.append(1)
        edges[0] += 1
        return False, 'execution error code -7'
    ok, _ = core.run_resumable(attempt, lambda: edges[0], lambda: next(answers))
    assert not ok and len(calls) == 1


def test_no_resume_hook_means_no_retry():
    edges, calls = [0], []

    def attempt():
        calls.append(1)
        edges[0] += 1
        return False, 'execution error code -7'
    ok, _ = core.run_resumable(attempt, lambda: edges[0], None)
    assert not ok and len(calls) == 1


def test_blocked_at_start_and_stopped_never_attempts():
    calls = []
    ok, _ = core.run_resumable(lambda: calls.append(1) or (True, ''), lambda: 0,
                               lambda: False)
    assert not ok and not calls


# ---- robot_block -------------------------------------------------------------

@pytest.mark.parametrize('mode,age,gate_on,expect', [
    (MOVE, 0.0, True, None),
    (USER_STOPPED, 0.0, True, 'pause'),
    (IDLE, 0.0, True, 'pause'),
    (None, 0.0, True, 'pause'),          # no state yet: fail closed
    (MOVE, 5.0, True, 'pause'),          # stale MOVE is not MOVE
    (USER_STOPPED, 0.0, False, None),    # gate off: user stop not gated
    (None, 0.0, False, None),
    (REFLEX, 0.0, True, 'fatal'),
    (REFLEX, 0.0, False, 'fatal'),       # a reflex ends a run regardless
    (REFLEX, 5.0, False, None),          # stale: unknown, not asserted
])
def test_robot_block_table(mode, age, gate_on, expect):
    blk = _gui(_Node(mode, age), gate_on).robot_block()
    got = None if blk is None else ('fatal' if blk[1] else 'pause')
    assert got == expect, blk


def test_user_stop_reason_names_the_mode():
    """Not 'enabling device': measured 2026-09-15, it is invisible over FCI."""
    blk = _gui(_Node(USER_STOPPED)).robot_block()
    assert 'USER_STOPPED' in blk[0] and 'enabling' not in blk[0]


# ---- wait_gate ------------------------------------------------------------------

def test_open_gate_returns_at_once():
    g = _gui(_Node(MOVE))
    assert g.wait_gate() is True
    assert not g.logs and g.reloads == 0


def test_open_gate_still_honours_stop():
    g = _gui(_Node(MOVE))
    g.abort = True
    assert g.wait_gate() is False


def test_resumes_only_after_an_unbroken_hold(fast_hold):
    node = _Node(USER_STOPPED)
    g = _gui(node)
    seen_paused = []
    _later(0.10, lambda: seen_paused.append(g.paused))
    _later(0.20, lambda: setattr(node, 'mode', MOVE))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    assert time.monotonic() - t0 >= 0.20 + fast_hold - 0.03
    assert seen_paused == [True] and g.paused is False
    assert g.reloads == 1, 'R must be re-based from TF on resume'
    assert [t['rec'] for t in g.traces] == ['gate_pause', 'gate_resume']


def test_flicker_does_not_resume(fast_hold):
    """Held 50 ms then released again: the hold timer must restart."""
    node = _Node(USER_STOPPED)
    g = _gui(node)
    _later(0.10, lambda: setattr(node, 'mode', MOVE))
    _later(0.15, lambda: setattr(node, 'mode', USER_STOPPED))
    _later(0.40, lambda: setattr(node, 'mode', MOVE))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    assert time.monotonic() - t0 >= 0.40 + fast_hold - 0.03


def test_stop_during_pause_ends_the_wait(fast_hold):
    g = _gui(_Node(USER_STOPPED))
    _later(0.10, lambda: setattr(g, 'abort', True))
    assert g.wait_gate() is False
    assert g.paused is False and g.reloads == 0 and g.gate_fault is None


def test_reflex_during_pause_is_a_fault(fast_hold):
    node = _Node(USER_STOPPED)
    g = _gui(node)
    _later(0.10, lambda: setattr(node, 'mode', REFLEX))
    assert g.wait_gate() is False
    assert g.gate_fault == 'robot_reflex' and g.paused is False


def test_reflex_at_start_is_a_fault_without_pausing():
    g = _gui(_Node(REFLEX))
    assert g.wait_gate() is False
    assert g.gate_fault == 'robot_reflex'
    assert 'gate_pause' not in [t['rec'] for t in g.traces]


def test_resume_hook_exists_with_gate_off():
    """PAUSE must be able to resume a Cartesian step with the gate off too."""
    assert _gui(_Node(), gate_on=True)._resume_hook() is not None
    assert _gui(_Node(), gate_on=False)._resume_hook() is not None


# ---- operator PAUSE ------------------------------------------------------------

@pytest.mark.parametrize('gate_on', [True, False])
def test_operator_pause_blocks_whatever_the_gate(gate_on):
    g = _gui(_Node(MOVE), gate_on=gate_on)
    g.user_paused = True
    assert g.robot_block() == ('paused by operator', False)


def test_reflex_outranks_operator_pause():
    g = _gui(_Node(REFLEX))
    g.user_paused = True
    assert g.robot_block()[1] is True


def test_resume_button_continues_after_hold(fast_hold):
    g = _gui(_Node(MOVE), gate_on=False)
    g.user_paused = True
    _later(0.10, lambda: setattr(g, 'user_paused', False))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    assert time.monotonic() - t0 >= 0.10 + fast_hold - 0.03
    assert 'RESUME' in g.logs[0]


# ---- mode callback -------------------------------------------------------------------

@pytest.mark.parametrize('busy,gate_on,prev,mode,halts', [
    (True, True, MOVE, USER_STOPPED, 1),
    (True, True, MOVE, REFLEX, 1),
    (False, True, MOVE, USER_STOPPED, 0),   # nothing running, nothing to halt
    (True, False, MOVE, USER_STOPPED, 0),   # gate off: device ignored
    (True, False, MOVE, REFLEX, 1),         # ...but a reflex still halts
    (True, True, USER_STOPPED, MOVE, 0),    # re-held: resume is wait_gate's job
    (True, True, None, USER_STOPPED, 0),
])
def test_on_mode_change_halts(busy, gate_on, prev, mode, halts):
    node = _Node()
    g = _gui(node, gate_on=gate_on, busy=busy)
    g._on_mode_change(prev, mode)
    assert node.halts == halts
    assert list(g._mode_log)[-1][1:] == (prev, mode)


def _node_class():
    pytest.importorskip('rclpy')
    pytest.importorskip('franka_msgs')
    import ros_node
    return ros_node.CellNodeBase


def test_interrupt_counts_before_halting():
    """Counted BEFORE halt, or the halted step would look like a real failure
    and end the run instead of resuming."""
    base = _node_class()
    seen = []
    fake = types.SimpleNamespace(_lock=threading.Lock(), gate_edges=0)
    fake.halt = lambda: seen.append(fake.gate_edges)
    base.interrupt(fake)
    assert fake.gate_edges == 1 and seen == [1]


def test_mode_cb_counts_move_exits_and_reports_changes():
    base = _node_class()
    fake = types.SimpleNamespace(_lock=threading.Lock(), _mode=None, gate_edges=0,
                                 on_mode_change=None)
    changes = []
    fake.on_mode_change = lambda prev, mode: changes.append((prev, mode))
    for mode in (MOVE, MOVE, USER_STOPPED, USER_STOPPED, MOVE, REFLEX, IDLE):
        base._mode_cb(fake, types.SimpleNamespace(robot_mode=mode))
    assert fake.gate_edges == 2
    assert changes == [(None, MOVE), (MOVE, USER_STOPPED), (USER_STOPPED, MOVE),
                       (MOVE, REFLEX), (REFLEX, IDLE)]


# ---- torque interlock --------------------------------------------------------------
# ALIGN plans on fr3_arm_controller. While the impedance controller holds the
# arm, and above all while the tracking node streams its equilibrium, an
# ALIGN move would fight them. The poller's answer decides, never memory.

ARM_ONLY = {'fr3_arm_controller': 'active',
            'cartesian_impedance_stroke_controller': 'inactive'}


@pytest.mark.parametrize('ctrl,tracking,moves', [
    (ARM_ONLY, False, True),
    ({'fr3_arm_controller': 'active'}, False, True),     # impedance not loaded
    (ARM_ONLY, True, False),
    ({'fr3_arm_controller': 'inactive',
      'cartesian_impedance_stroke_controller': 'active'}, False, False),
    (None, False, False),                                # cannot rule it out
])
def test_align_never_moves_under_impedance_or_tracking(ctrl, tracking, moves):
    node = _Node(MOVE)
    node.controller_states = lambda: ctrl
    g = _gui(node, busy=False)
    g.tracking = tracking
    ran = []
    g.translate = lambda: ran.append(True) or (True, 'executed')
    g.level = g.inplane = g.auto_converge = g.translate
    g.controller = types.MethodType(actions.Cell.controller, g)
    ok, msg = actions.Cell.start_align(g, 'translate')
    assert bool(ran) is moves and ok is moves, msg


# ---------------------------------------------------------------- driver restart

def _reload_rig(monkeypatch, reload_impedance=True):
    clock = [100.0]
    monkeypatch.setattr(actions.time, 'monotonic', lambda: clock[0])
    started = []

    class FakeSpawner:
        def __init__(self, args, **_kw):
            started.append(args)
            self.returncode = None

        def poll(self):
            return self.returncode

    monkeypatch.setattr(actions.subprocess, 'Popen', FakeSpawner)
    said = []
    cell = types.SimpleNamespace(reload_impedance=reload_impedance, _imp_missing_since=None,
                                 _spawner=None, say=said.append, trace=lambda r: None,
                                 n=types.SimpleNamespace(refresh_now=lambda: None))

    def watch(controllers):
        actions.Cell._watch_impedance(cell, types.SimpleNamespace(controllers=controllers))
    return clock, started, said, cell, watch


def test_a_restarted_driver_gets_the_impedance_controller_back(monkeypatch):
    """2026-09-24: T1 restarted, and FLOAT/HOLD stayed blocked until fr3_cell
    was restarted too - its spawner had run once, at launch."""
    clock, started, said, cell, watch = _reload_rig(monkeypatch)
    arm_only = {core.ARM_CONTROLLER: 'active'}
    watch(arm_only)
    clock[0] += actions.IMP_RELOAD_GRACE_S - 1
    watch(arm_only)
    assert started == []                        # the launch's own spawner gets its grace
    clock[0] += 2
    watch(arm_only)
    assert len(started) == 1
    assert '--inactive' in started[0] and str(core.IMPEDANCE_PARAMS) in started[0]
    clock[0] += 60
    watch(arm_only)
    assert len(started) == 1                    # one spawner at a time
    cell._spawner.returncode = 0
    watch({**arm_only, core.IMPEDANCE_CONTROLLER: 'inactive'})
    assert 'loaded again' in said[-1] and cell._spawner is None


def test_a_failed_reload_retries_after_a_full_grace(monkeypatch):
    clock, started, said, cell, watch = _reload_rig(monkeypatch)
    arm_only = {core.ARM_CONTROLLER: 'active'}
    watch(arm_only)
    clock[0] += actions.IMP_RELOAD_GRACE_S + 1
    watch(arm_only)
    cell._spawner.returncode = 1
    watch(arm_only)
    assert 'FAILED' in said[-1] and len(started) == 1
    clock[0] += actions.IMP_RELOAD_GRACE_S + 1
    watch(arm_only)
    assert len(started) == 2


def test_no_reload_in_the_mock_or_while_the_driver_is_down(monkeypatch):
    clock, started, _, _, watch = _reload_rig(monkeypatch, reload_impedance=False)
    watch({core.ARM_CONTROLLER: 'active'})
    clock[0] += 100
    watch({core.ARM_CONTROLLER: 'active'})
    clock, started2, _, _, watch2 = _reload_rig(monkeypatch)
    watch2(None)                                # no controller manager at all
    clock[0] += 100
    watch2(None)
    assert started == [] and started2 == []


def test_a_box_excursion_while_tracking_warns_and_never_stops(monkeypatch):
    """The node holds at the box now; the GUI-side stop is gone (TODO C6)."""
    said, threads = [], []
    cell = types.SimpleNamespace(st=actions.logic.Settings(), _box_fired=False, say=said.append,
                                 trace=lambda r: None,
                                 _thread=lambda *a: threads.append(a))
    monkeypatch.setattr(actions.logic, 'tracking', lambda s, st: True)
    actions.Cell._watch_box(cell, types.SimpleNamespace(tcp=(0.95, 0.0, 0.4)))
    assert threads == [] and 'holds at the box' in said[-1]
    actions.Cell._watch_box(cell, types.SimpleNamespace(tcp=(0.95, 0.0, 0.4)))
    assert len(said) == 1                                  # once per excursion


# ------------------------------------------------ the object pose contract

def _align_cell(raw_age):
    """Enough of a Cell for one ALIGN step: a visible, filtered marker pose;
    the raw detection behind it raw_age seconds old (None = never)."""
    import numpy as np
    n = types.SimpleNamespace(marker=lambda: (np.array([0.0, 0.0, 0.2]), np.eye(3), 0.0, 'cam'),
                              raw_age=lambda: raw_age)
    cell = types.SimpleNamespace(n=n, R=np.eye(3), st=actions.logic.Settings(),
                                 params=types.SimpleNamespace(target_m=0.1, inplane_target=90.0))
    cell._marker_for_motion = lambda: actions.Cell._marker_for_motion(cell)
    return cell


def test_align_steps_refuse_a_pose_with_no_fresh_raw_detection_behind_it():
    """Predictions never drive committed motion: ALIGN checks it again at
    the move, not only in the view's enable."""
    for age in (None, core.RAW_MAX_AGE_S + 0.1):
        cell = _align_cell(age)
        for step in (actions.Cell.translate, actions.Cell.level, actions.Cell.inplane):
            ok, msg = step(cell)
            assert not ok and 'raw pose' in msg, (step.__name__, msg)
    m, why = actions.Cell._marker_for_motion(_align_cell(0.05))
    assert m is not None and why is None


def test_the_pose_source_is_refused_while_tracking_or_gripping():
    set_to = []
    n = types.SimpleNamespace(grip_status=lambda: {'state': 'idle'},
                              set_pose_source=lambda s: (set_to.append(s) or (True, 'ok')))
    cell = types.SimpleNamespace(n=n, tracking=False, say=lambda m: None, trace=lambda r: None)
    assert actions.Cell.set_pose_source(cell, 'marker')[0] and set_to == ['marker']
    assert not actions.Cell.set_pose_source(cell, 'nonsense')[0]
    cell.tracking = True
    assert not actions.Cell.set_pose_source(cell, 'marker')[0]
    cell.tracking = False
    n.grip_status = lambda: {'state': 'idle', 'holding': 'true'}
    assert not actions.Cell.set_pose_source(cell, 'marker')[0]
    assert set_to == ['marker']                       # nothing sent after the first
