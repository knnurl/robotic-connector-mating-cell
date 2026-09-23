#!/usr/bin/env python3
"""Robot-state gate and PAUSE decisions, pinned without ROS or a robot.

This logic only really runs when a run is interrupted mid-motion, which is
exactly when a logic slip is least welcome. So:

  * run_resumable retries ONLY failures caused by an interruption
  * robot_block fails closed, and a REFLEX ends a run even with the gate off
  * wait_gate needs an unbroken all-clear before resuming; STOP always wins
  * the mode callback counts MOVE exits and halts motion from the spin thread

    python3 -m pytest tools/fr3/test_enable_gate.py -q
"""

import collections
import threading
import time
import types

import pytest

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


def _gui(ag, node, gate_on=True, busy=True):
    g = types.SimpleNamespace(
        n=node, gate_on=gate_on, abort=False, paused=False, pause_reason='',
        gate_fault=None, user_paused=False, busy=busy, logs=[], traces=[], reloads=0,
        _mode_log=collections.deque())
    g.say = g.logs.append
    g.trace = g.traces.append
    g.set_status = lambda *a, **k: None

    def load_calib(quiet=False):
        g.reloads += 1
    g.load_calib = load_calib
    for name in ('robot_block', 'wait_gate', '_gate_fault',
                 '_on_mode_change', '_resume_hook', '_resume_hint'):
        setattr(g, name, types.MethodType(getattr(ag.AlignPane, name), g))
    return g


def _later(delay_s, fn):
    t = threading.Timer(delay_s, fn)
    t.start()
    return t


@pytest.fixture
def fast_hold(ag, monkeypatch):
    monkeypatch.setattr(ag, 'GATE_RESUME_HOLD_S', 0.15)
    return 0.15


# ---- run_resumable ---------------------------------------------------------

def test_mode_constants_match_franka_msgs(ag):
    """franka_msgs/FrankaRobotState: MOVE=2, REFLEX=4, USER_STOPPED=5."""
    assert (ag.MODE_MOVE, ag.MODE_REFLEX, ag.MODE_USER_STOPPED) == (2, 4, 5)


def test_success_runs_once(ag):
    calls = []
    ok, _ = ag.run_resumable(lambda: calls.append(1) or (True, 'executed'),
                             lambda: 0, lambda: True)
    assert ok and len(calls) == 1


def test_real_failure_is_never_retried(ag):
    """A planning failure with nothing interrupting must not loop forever."""
    calls = []

    def attempt():
        calls.append(1)
        # fail fast instead of hanging if retrying ever creeps back in
        assert len(calls) <= 2, 'retried a failure the robot did not cause'
        return False, 'path only 40% complete'
    ok, msg = ag.run_resumable(attempt, lambda: 0, lambda: True)
    assert not ok and msg == 'path only 40% complete' and len(calls) == 1


def test_interrupted_attempt_resumes_and_finishes(ag):
    edges, calls = [0], []

    def attempt():
        calls.append(1)
        if len(calls) == 1:
            edges[0] += 1               # robot left MOVE during this attempt
            return False, 'execution error code -7'
        return True, 'executed'
    ok, _ = ag.run_resumable(attempt, lambda: edges[0], lambda: True)
    assert ok and len(calls) == 2


def test_stop_during_pause_gives_up_without_moving(ag):
    edges, calls, answers = [0], [], iter([True, False])

    def attempt():
        calls.append(1)
        edges[0] += 1
        return False, 'execution error code -7'
    ok, _ = ag.run_resumable(attempt, lambda: edges[0], lambda: next(answers))
    assert not ok and len(calls) == 1


def test_no_resume_hook_means_no_retry(ag):
    edges, calls = [0], []

    def attempt():
        calls.append(1)
        edges[0] += 1
        return False, 'execution error code -7'
    ok, _ = ag.run_resumable(attempt, lambda: edges[0], None)
    assert not ok and len(calls) == 1


def test_blocked_at_start_and_stopped_never_attempts(ag):
    calls = []
    ok, _ = ag.run_resumable(lambda: calls.append(1) or (True, ''),
                             lambda: 0, lambda: False)
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
def test_robot_block_table(ag, mode, age, gate_on, expect):
    blk = _gui(ag, _Node(mode, age), gate_on).robot_block()
    got = None if blk is None else ('fatal' if blk[1] else 'pause')
    assert got == expect, blk


def test_user_stop_reason_names_the_mode(ag):
    """Not 'enabling device': measured 2026-09-15, it is invisible over FCI."""
    blk = _gui(ag, _Node(USER_STOPPED)).robot_block()
    assert 'USER_STOPPED' in blk[0] and 'enabling' not in blk[0]


# ---- wait_gate ------------------------------------------------------------------

def test_open_gate_returns_at_once(ag):
    g = _gui(ag, _Node(MOVE))
    assert g.wait_gate() is True
    assert not g.logs and g.reloads == 0


def test_open_gate_still_honours_stop(ag):
    g = _gui(ag, _Node(MOVE))
    g.abort = True
    assert g.wait_gate() is False


def test_resumes_only_after_an_unbroken_hold(ag, fast_hold):
    node = _Node(USER_STOPPED)
    g = _gui(ag, node)
    seen_paused = []
    _later(0.10, lambda: seen_paused.append(g.paused))
    _later(0.20, lambda: setattr(node, 'mode', MOVE))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    elapsed = time.monotonic() - t0
    assert elapsed >= 0.20 + fast_hold - 0.03, elapsed
    assert seen_paused == [True] and g.paused is False
    assert g.reloads == 1, 'R must be re-based from TF on resume'
    assert [t['rec'] for t in g.traces] == ['gate_pause', 'gate_resume']


def test_flicker_does_not_resume(ag, fast_hold):
    """Held 50 ms then released again: the hold timer must restart."""
    node = _Node(USER_STOPPED)
    g = _gui(ag, node)
    _later(0.10, lambda: setattr(node, 'mode', MOVE))
    _later(0.15, lambda: setattr(node, 'mode', USER_STOPPED))
    _later(0.40, lambda: setattr(node, 'mode', MOVE))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    assert time.monotonic() - t0 >= 0.40 + fast_hold - 0.03


def test_stop_during_pause_ends_the_wait(ag, fast_hold):
    g = _gui(ag, _Node(USER_STOPPED))
    _later(0.10, lambda: setattr(g, 'abort', True))
    assert g.wait_gate() is False
    assert g.paused is False and g.reloads == 0 and g.gate_fault is None


def test_reflex_during_pause_is_a_fault(ag, fast_hold):
    node = _Node(USER_STOPPED)
    g = _gui(ag, node)
    _later(0.10, lambda: setattr(node, 'mode', REFLEX))
    assert g.wait_gate() is False
    assert g.gate_fault == 'robot_reflex' and g.paused is False


def test_reflex_at_start_is_a_fault_without_pausing(ag):
    g = _gui(ag, _Node(REFLEX))
    assert g.wait_gate() is False
    assert g.gate_fault == 'robot_reflex'
    assert 'gate_pause' not in [t['rec'] for t in g.traces]


def test_resume_hook_exists_with_gate_off(ag):
    """PAUSE must be able to resume a Cartesian step with the gate off too."""
    assert _gui(ag, _Node(), gate_on=True)._resume_hook() is not None
    assert _gui(ag, _Node(), gate_on=False)._resume_hook() is not None


# ---- operator PAUSE ------------------------------------------------------------

@pytest.mark.parametrize('gate_on', [True, False])
def test_operator_pause_blocks_whatever_the_gate(ag, gate_on):
    g = _gui(ag, _Node(MOVE), gate_on=gate_on)
    g.user_paused = True
    assert g.robot_block() == ('paused by operator', False)


def test_reflex_outranks_operator_pause(ag):
    g = _gui(ag, _Node(REFLEX))
    g.user_paused = True
    assert g.robot_block()[1] is True


def test_resume_button_continues_after_hold(ag, fast_hold):
    g = _gui(ag, _Node(MOVE), gate_on=False)
    g.user_paused = True
    _later(0.10, lambda: setattr(g, 'user_paused', False))
    t0 = time.monotonic()
    assert g.wait_gate() is True
    assert time.monotonic() - t0 >= 0.10 + fast_hold - 0.03
    assert 'RESUME' in g.logs[0]


def test_interrupt_counts_before_halting(ag):
    """Counted BEFORE halt, or the halted step would look like a real failure
    and end the run instead of resuming."""
    seen = []
    fake = types.SimpleNamespace(_lock=threading.Lock(), gate_edges=0)
    fake.halt = lambda: seen.append(fake.gate_edges)
    ag.CellNode.interrupt(fake)
    assert fake.gate_edges == 1 and seen == [1]


def test_paused_cartesian_step_resumes(ag, fast_hold):
    """PAUSE mid-move -> execution fails -> RESUME -> retried to the goal."""
    fake = types.SimpleNamespace(_lock=threading.Lock(), gate_edges=0)
    fake.halt = lambda: None
    g = _gui(ag, _Node(MOVE), gate_on=False)
    calls = []

    def attempt():
        calls.append(1)
        if len(calls) == 1:
            g.user_paused = True
            ag.CellNode.interrupt(fake)
            _later(0.10, lambda: setattr(g, 'user_paused', False))
            return False, 'execution error code -7'
        return True, 'executed'
    ok, _ = ag.run_resumable(attempt, lambda: fake.gate_edges,
                             g._resume_hook())
    assert ok and len(calls) == 2


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
def test_on_mode_change_halts(ag, busy, gate_on, prev, mode, halts):
    node = _Node()
    g = _gui(ag, node, gate_on=gate_on, busy=busy)
    g._on_mode_change(prev, mode)
    assert node.halts == halts
    assert list(g._mode_log)[-1][1:] == (prev, mode)


def test_mode_cb_counts_move_exits_and_reports_changes(ag):
    fake = types.SimpleNamespace(_lock=threading.Lock(), _mode=None,
                                 gate_edges=0, on_mode_change=None)
    changes = []
    fake.on_mode_change = lambda prev, mode: changes.append((prev, mode))
    for mode in (MOVE, MOVE, USER_STOPPED, USER_STOPPED, MOVE, REFLEX, IDLE):
        ag.CellNode._mode_cb(fake, types.SimpleNamespace(robot_mode=mode))
    assert fake.gate_edges == 2
    assert changes == [(None, MOVE), (MOVE, USER_STOPPED),
                       (USER_STOPPED, MOVE), (MOVE, REFLEX), (REFLEX, IDLE)]
