"""The exit handoff: whatever ends the panel, the arm is never left on the
impedance controller, and teardown never races the spin thread.

Ported from the Tk panel's test_cell_panel.py (retired 2026-09-24): the
helpers moved to core.py unchanged, and so did these cases.
"""

import threading
import types

import pytest

import core

MOVE, IDLE = 2, 1
ARM, IMP = core.ARM_CONTROLLER, core.IMPEDANCE_CONTROLLER


class FakeNode:
    """A controller manager + robot that behave like the real pair."""

    def __init__(self, loaded='inactive', cm_ready=True):
        self.cm_ready = cm_ready
        self.mode = MOVE
        self.ctrl = {ARM: 'active'}
        if loaded is not None:
            self.ctrl[IMP] = loaded
            if loaded == 'active':
                self.ctrl[ARM] = 'inactive'
        self.calls = []
        self.track_stop_cli = 'stop'

    def controllers(self, timeout_s=3.0):
        return dict(self.ctrl) if self.cm_ready else None

    def switch(self, activate, deactivate, timeout_s=10.0):
        self.calls.append(('switch', tuple(activate), tuple(deactivate)))
        for c in deactivate:
            self.ctrl[c] = 'inactive'
        for c in activate:
            self.ctrl[c] = 'active'
        return True, 'switched'

    def call_trigger(self, cli, timeout_s=6.0):
        self.calls.append(('trigger', cli))
        return True, 'stopped'


def test_exit_helper():
    active = FakeNode(loaded='active')
    assert core.release_if_active(active, say=lambda m: None) is True
    assert active.ctrl[IMP] == 'inactive'
    idle = FakeNode(loaded='inactive')
    assert core.release_if_active(idle, say=lambda m: None) is True
    assert not idle.calls
    down = FakeNode(cm_ready=False)
    assert core.release_if_active(down, say=lambda m: None) is None


def test_exit_release_stops_tracking_only_when_impedance_is_live():
    live = FakeNode(loaded='active')
    assert core.release_if_active(live, say=lambda m: None) is True
    assert live.calls[0] == ('trigger', 'stop')          # the stream first
    idle = FakeNode(loaded='inactive')
    assert core.release_if_active(idle, say=lambda m: None) is True
    assert not idle.calls


def test_exit_waits_for_a_running_action_then_restores_the_arm_controller():
    node = FakeNode()
    node.ctrl[ARM] = 'inactive'              # PRE-FLIGHT released it
    panel = types.SimpleNamespace(busy=True, arm_released=True)
    threading.Timer(0.1, lambda: setattr(panel, 'busy', False)).start()
    assert core.exit_handoff(node, panel, say=lambda m: None, wait_s=2.0) is True
    assert node.ctrl[ARM] == 'active'
    assert ('switch', (ARM,), ()) in node.calls


def test_exit_leaves_the_arm_controller_alone_unless_preflight_released_it():
    node = FakeNode()
    node.ctrl[ARM] = 'inactive'              # someone else's decision
    panel = types.SimpleNamespace(busy=False, arm_released=False)
    core.exit_handoff(node, panel, say=lambda m: None, wait_s=0.1)
    assert not node.calls


def test_exit_hands_back_live_impedance_and_warns_when_it_cannot_ask():
    live = FakeNode(loaded='active')
    core.exit_handoff(live, None, say=lambda m: None)
    assert live.ctrl[IMP] == 'inactive'
    msgs = []
    core.exit_handoff(FakeNode(cm_ready=False), None, say=msgs.append)
    assert any('could not ask' in m for m in msgs)


def _fake_rclpy(monkeypatch, shutdown):
    import sys
    monkeypatch.setitem(sys.modules, 'rclpy', types.SimpleNamespace(shutdown=shutdown))


def test_shutdown_stops_the_spin_before_dropping_the_context(monkeypatch):
    """A spin thread still running when rclpy.shutdown() lands ABORTS the
    process - inside the exit path that hands the arm back. Observed
    2026-09-16 on an ordinary window close. The order is what makes it not a
    race."""
    order = []

    class Executor:
        def shutdown(self):
            order.append(('executor.shutdown', None))

    class Spin:
        def join(self, timeout=None):
            order.append(('spin.join', timeout))

    node = FakeNode()
    node.close = lambda: order.append(('node.close', None))
    node.destroy_node = lambda: order.append(('node.destroy_node', None))
    monkeypatch.setattr(core, 'exit_handoff', lambda *a, **k: order.append(('exit_handoff', None)))
    _fake_rclpy(monkeypatch, lambda: order.append(('rclpy.shutdown', None)))
    core.shutdown_ros(node, None, Executor(), Spin(), say=lambda m: None)
    assert [n for n, _ in order] == ['exit_handoff', 'node.close', 'executor.shutdown',
                                     'spin.join', 'node.destroy_node', 'rclpy.shutdown']
    assert dict(order)['spin.join'] == core.SPIN_JOIN_S


def test_shutdown_hands_the_arm_back_even_if_the_teardown_throws(monkeypatch):
    handed = []

    class Executor:
        def shutdown(self):
            raise RuntimeError('executor already gone')

    class Spin:
        def join(self, timeout=None):
            pass

    node = FakeNode()
    node.close = lambda: None
    node.destroy_node = lambda: None
    monkeypatch.setattr(core, 'exit_handoff', lambda *a, **k: handed.append(True))
    _fake_rclpy(monkeypatch, lambda: None)
    with pytest.raises(RuntimeError):
        core.shutdown_ros(node, None, Executor(), Spin(), say=lambda m: None)
    assert handed == [True]
