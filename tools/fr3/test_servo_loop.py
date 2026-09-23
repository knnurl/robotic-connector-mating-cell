#!/usr/bin/env python3
"""The whole servo run, against a fake robot: controller swap, aborts, gate.

servo_converge is the one path in this GUI that streams motion continuously,
so the ordering around it is what keeps the cell safe:

  * the arm is handed to the position controller BEFORE servo starts, and
    handed back on every exit - converged, aborted, stalled or crashed
  * a failed hand-over refuses the run instead of streaming into the effort
    trajectory controller (which is what stalled the arm on 2026-09-15)
  * a failed hand-BACK is reported loudly, because planned moves then fail

The real loop runs here; only the node is fake. Rates are shortened so the
tests take about a second.

    python3 -m pytest tools/fr3/test_servo_loop.py -q
"""

import threading
import types

import numpy as np
import pytest


class _Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class FakeNode:
    """Stands in for AlignNode: records what the loop asked the robot to do."""

    def __init__(self, marker_fn, lowest=('fr3_link3', 0.40)):
        self.base = 'fr3_link0'
        self.servo_start, self.servo_stop = 'start', 'stop'
        self.marker_fn = marker_fn
        self.lowest = lowest
        self.mode = 2              # robot_mode MOVE
        self.ctrl = 'arm'
        self.switch_ok = True
        self.restore_ok = True
        self.calls = []            # ordered: switches, servo start/stop
        self.twists = []
        self.i = 0

    def marker(self):
        m = self.marker_fn(self.i)
        self.i += 1
        return m

    def robot_mode(self):
        return None if self.mode is None else (self.mode, 0.0)

    def lowest_link(self, links):
        return self.lowest

    def switch_controllers(self, activate, deactivate, mode):
        ok = self.switch_ok if mode == 'servo' else self.restore_ok
        self.calls.append(('switch', mode, ok))
        self.ctrl = mode if ok else None
        return ok, ('switched' if ok else 'controller_manager refused')

    def call_servo(self, which, timeout_s=5.0):
        self.calls.append(('servo', which))
        return True, 'ok'

    def publish_twist(self, lin, ang, frame):
        self.twists.append((np.asarray(lin), np.asarray(ang)))

    def zero_twist(self, n=3):
        self.twists.append(('zero', n))

    def interrupt(self):
        pass

    def halt(self):
        pass


def _gui(ag, node, profile='conservative', converge='hold'):
    g = types.SimpleNamespace(
        n=node, R=np.eye(3), abort=False, gate_on=False, user_paused=False,
        busy=True, paused=False, pause_reason='', gate_fault=None,
        logs=[], traces=[],
        v_servo=_Var(profile), v_converge=_Var(converge), v_inplane=_Var('off'))
    g.say = g.logs.append
    g.trace = g.traces.append
    g.set_status = lambda *a, **k: None
    g.target_m = lambda: 0.100
    g.pos_tol_m = lambda: 0.002
    g.z_floor = lambda: 0.050
    g.inplane_target = lambda: None
    g.load_calib = lambda quiet=False: None
    g._clamp = ag.AlignPane._clamp
    for name in ('servo_converge', '_servo_error', 'robot_block', 'wait_gate',
                 '_gate_fault', '_resume_hint'):
        setattr(g, name, types.MethodType(getattr(ag.AlignPane, name), g))
    return g


@pytest.fixture(autouse=True)
def fast(ag, monkeypatch):
    """Shorten the loop so a run takes ~1 s instead of ~10."""
    monkeypatch.setattr(ag, 'SERVO_RATE_HZ', 200.0)
    monkeypatch.setattr(ag, 'SERVO_SETTLE_S', 0.0)
    monkeypatch.setattr(ag, 'SERVO_STALL_S', 0.5)
    monkeypatch.setattr(ag, 'GATE_RESUME_HOLD_S', 0.05)


def marker_converging(i):
    """Marker walking to the 100 mm target, square on (no tilt)."""
    z = 0.150 - 0.050 * min(i / 60.0, 1.0)
    return np.array([0.0, 0.0, z]), np.eye(3)


def marker_stuck(i):
    return np.array([0.0, 0.0, 0.150]), np.eye(3)


def switches(node):
    return [(c[1], c[2]) for c in node.calls if c[0] == 'switch']


def test_converged_run_swaps_to_position_controller_and_back(ag):
    node = FakeNode(marker_converging)
    out = _gui(ag, node).servo_converge()
    assert out == 'converged'
    assert switches(node) == [('servo', True), ('arm', True)]
    assert node.ctrl == 'arm'
    # the hand-over must come before servo is allowed to start
    order = [c[0] for c in node.calls]
    assert order.index('switch') < order.index('servo')


def test_failed_handover_refuses_without_starting_servo(ag):
    node = FakeNode(marker_converging)
    node.switch_ok = False
    g = _gui(ag, node)
    out = g.servo_converge()
    assert out == 'controller_switch_failed'
    assert not [c for c in node.calls if c[0] == 'servo'], 'servo was started'
    assert not node.twists, 'streamed into the wrong controller'
    assert any('REFUSING' in m for m in g.logs)


def test_stalled_arm_aborts_and_hands_back(ag):
    node = FakeNode(marker_stuck)
    g = _gui(ag, node)
    out = g.servo_converge()
    assert out == 'servo_stalled'
    assert switches(node)[-1] == ('arm', True)
    assert any('not following' in m for m in g.logs)


def test_marker_loss_aborts_and_hands_back(ag):
    node = FakeNode(lambda i: marker_converging(i) if i < 20 else None)
    out = _gui(ag, node).servo_converge()
    assert out == 'marker_lost'
    assert switches(node)[-1] == ('arm', True)


def test_floor_breach_aborts_and_hands_back(ag):
    node = FakeNode(marker_converging, lowest=('fr3_link3', 0.020))
    g = _gui(ag, node)
    out = g.servo_converge()
    assert out == 'refused_below_floor', 'must refuse before any streaming'
    assert not node.twists and not switches(node)


def test_floor_breach_during_the_run_aborts_and_hands_back(ag):
    node = FakeNode(marker_converging)

    def sink(links, node=node):
        return ('fr3_link3', 0.40 if node.i < 20 else 0.020)
    node.lowest_link = sink
    out = _gui(ag, node).servo_converge()
    assert out == 'z_floor'
    assert switches(node)[-1] == ('arm', True)


def test_stop_hands_back(ag):
    node = FakeNode(marker_converging)
    g = _gui(ag, node)
    g.abort = True
    out = g.servo_converge()
    assert out == 'stopped'
    assert switches(node)[-1] == ('arm', True)


def test_failed_hand_back_is_reported_loudly(ag):
    node = FakeNode(marker_converging)
    node.restore_ok = False
    g = _gui(ag, node)
    assert g.servo_converge() == 'converged'
    assert node.ctrl is None, 'unknown controller state must not read as arm'
    assert any('NOT RESTORED' in m for m in g.logs)


def test_user_stop_mid_run_pauses_then_resumes_and_converges(ag):
    """Robot leaves MOVE, comes back: the run waits, then finishes - it does
    not abort, and it does not keep streaming while the robot is stopped."""
    node = FakeNode(marker_converging)

    def marker(i, node=node):
        if i == 20:                      # robot stops, and is released later
            node.mode = 5                # USER_STOPPED
            threading.Timer(0.2, lambda: setattr(node, 'mode', 2)).start()
        return marker_converging(i)
    node.marker_fn = marker
    g = _gui(ag, node)
    g.gate_on = True
    out = g.servo_converge()
    assert out == 'converged'
    assert any('PAUSED' in m for m in g.logs)
    assert any('RESUMED' in m for m in g.logs)
    assert switches(node)[-1] == ('arm', True)
    assert [t['rec'] for t in g.traces if t['rec'].startswith('gate')] == [
        'gate_pause', 'gate_resume']


# ---- the switch itself ----------------------------------------------------------

def _node_with_switch(ag, results):
    """AlignNode.switch_controllers over a recording _switch_once."""
    node = types.SimpleNamespace(ctrl='arm', switch_ctrl=types.SimpleNamespace(
        wait_for_service=lambda timeout_sec=0: True), sent=[])
    answers = iter(results)

    def once(activate, deactivate):
        node.sent.append((tuple(activate), tuple(deactivate)))
        return next(answers)
    node._switch_once = once
    node.switch_controllers = types.MethodType(ag.CellNode.switch_controllers,
                                               node)
    return node


def test_switch_releases_before_it_claims(ag, monkeypatch):
    """Never both in one call: franka_hardware 2.0.2 starts the new command
    mode and then stops the old one in the same pass, which killed
    ros2_control_node on the way back to the arm controller (2026-09-15)."""
    monkeypatch.setattr(ag, 'CTRL_SWITCH_SETTLE_S', 0.0)
    node = _node_with_switch(ag, [(True, 'switched'), (True, 'switched')])
    ok, _ = node.switch_controllers([ag.SERVO_CONTROLLER], [ag.ARM_CONTROLLER],
                                    'servo')
    assert ok and node.ctrl == 'servo'
    assert node.sent == [((), (ag.ARM_CONTROLLER,)),
                         ((ag.SERVO_CONTROLLER,), ())]


def test_failed_release_never_claims_the_new_controller(ag, monkeypatch):
    monkeypatch.setattr(ag, 'CTRL_SWITCH_SETTLE_S', 0.0)
    node = _node_with_switch(ag, [(False, 'refused')])
    ok, msg = node.switch_controllers([ag.ARM_CONTROLLER],
                                      [ag.SERVO_CONTROLLER], 'arm')
    assert not ok and node.ctrl is None
    assert len(node.sent) == 1, 'claimed a controller after a failed release'
    assert ag.SERVO_CONTROLLER in msg


def test_run_ends_with_a_zero_twist(ag):
    node = FakeNode(marker_converging)
    _gui(ag, node).servo_converge()
    assert node.twists[-1][0] == 'zero'
