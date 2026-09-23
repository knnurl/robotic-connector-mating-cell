#!/usr/bin/env python3
"""Commissioning-panel decisions, without ROS, Tk or a robot.

The panel drives a torque controller that has never touched hardware, so the
rules it enforces are worth pinning:

  * nothing activates before PRE-FLIGHT, unless the robot is in MOVE, or
    unless the controller is loaded; float_mode is set BEFORE activation
  * PRE-FLIGHT releases the arm controller, sets payload and thresholds, and
    restores the arm controller on every path; the reflex threshold stays
    above what the controller itself can push with
  * gains outside the controller's limits never leave the panel, and those
    limits are the SAME numbers the C++ enforces
  * the equilibrium is a spring anchor: steps accumulate on the anchor, sit
    no further than MAX_LEAD_MM from the arm, and never below the Z floor
  * whether impedance is live is asked of the controller manager - on
    RELEASE, on close and at exit - so a stale panel cannot strand the arm
  * tracking is started and stopped by the operator, STOP TRACKING is never
    greyed out, and the shipped tracking numbers stay inside the limits the
    C++ enforces

    python3 -m pytest tools/fr3/test_cell_panel.py -q
"""

import pathlib
import re
import threading
import time
import types

import numpy as np
import pytest

MOVE, IDLE, REFLEX = 2, 1, 4
ARM = 'fr3_arm_controller'
IMP = 'cartesian_impedance_stroke_controller'
POS = np.array([0.40, 0.0, 0.30])
QUAT_DOWN = np.array([1.0, 0.0, 0.0, 0.0])   # 180 deg about X: tool Z is -Z
SRC = pathlib.Path(__file__).resolve().parents[2]


class _Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class _Button:
    """Enough of a tk.Button for go()'s disable list and for the show/hide
    paint_tracking() does."""

    def __init__(self):
        self.state = 'normal'
        self.text = ''
        self.packed = False

    def config(self, state=None, text=None, **_kw):
        if state is not None:
            self.state = state
        if text is not None:
            self.text = text

    def pack(self, **_kw):
        self.packed = True

    def pack_forget(self):
        self.packed = False

    def __getitem__(self, key):
        return self.text if key == 'text' else self.state


class FakeNode:
    """A controller manager + robot that behave like the real pair."""

    def __init__(self, mode=MOVE, pos=POS, quat=(0.0, 0.0, 0.0, 1.0),
                 age=0.0, loaded='inactive', has_state=True, cm_ready=True,
                 goes_idle=True):
        self.mode, self.pos, self.quat, self.age = mode, np.asarray(pos), \
            np.asarray(quat, dtype=float), age
        self.has_state, self.cm_ready, self.goes_idle = \
            has_state, cm_ready, goes_idle
        self.ctrl = {ARM: 'active'}
        if loaded is not None:
            self.ctrl[IMP] = loaded
            if loaded == 'active':
                self.ctrl[ARM] = 'inactive'
        self.calls, self.published = [], []
        self.track_start_cli, self.track_stop_cli = 'start', 'stop'
        self.trigger_ok = True
        self.param_ok = self.switch_ok = True
        self.load_ok = self.collision_ok = True
        self.load_args = None
        self.force = np.zeros(3)

    def state(self):
        if not self.has_state:
            return None
        return (self.pos, self.quat, self.force, self.mode, 1.0, self.age)

    def cm_reachable(self):
        return self.cm_ready

    def controllers(self, timeout_s=3.0):
        return dict(self.ctrl) if self.cm_ready else None

    def set_params(self, values):
        self.calls.append(('params', dict(values)))
        return self.param_ok, 'applied' if self.param_ok else 'refused'

    def switch(self, activate, deactivate, timeout_s=10.0):
        self.calls.append(('switch', tuple(activate), tuple(deactivate)))
        if not self.switch_ok:
            return False, 'refused'
        for c in deactivate:
            self.ctrl[c] = 'inactive'
        for c in activate:
            self.ctrl[c] = 'active'
        if self.goes_idle:
            live = any(self.ctrl.get(c) == 'active' for c in (ARM, IMP))
            self.mode = MOVE if live else IDLE
        return True, 'switched'

    def set_load(self, mass, com_m, inertia_diag):
        self.calls.append(('load',))
        self.load_args = (mass, list(com_m), list(inertia_diag))
        return self.load_ok, 'ok' if self.load_ok else 'command exception'

    def set_collision_behavior(self):
        self.calls.append(('collision',))
        return self.collision_ok, 'ok' if self.collision_ok else 'rejected'

    def publish_equilibrium(self, pos, quat):
        self.published.append((np.asarray(pos), np.asarray(quat)))

    def call_trigger(self, cli, timeout_s=6.0):
        self.calls.append(('trigger', cli))
        return self.trigger_ok, ('tracking' if self.trigger_ok
                                 else 'the tracking node is not running')


GAINS_OK = {'k_xy': '150', 'k_z': '800', 'k_rp': '10', 'k_yaw': '20',
            'zeta': '1.0'}


def _panel(ip, node, active=False, floating=False, step='10',
           axis='base Z (up)', preflight_ok=True, gains=None):
    p = types.SimpleNamespace(
        n=node, active=active, floating=floating, setpoint=None, busy=False,
        tracef=None, preflight_ok=preflight_ok, driver_down_logged=False,
        arm_released=False, after_calls=[], tracking=False,
        z_floor=(POS[2] - 0.030) if active else None,
        logs=[], traces=[], destroyed=[], v_step=_Var(step), v_axis=_Var(axis),
        tune={k: _Var(v) for k, v in (gains or GAINS_OK).items()})
    p.root = types.SimpleNamespace(
        destroy=lambda: p.destroyed.append(True),
        after=lambda ms, fn, *a: p.after_calls.append((ms, fn)))
    p.say = p.logs.append
    p.trace = p.traces.append
    p.set_status = lambda *a, **k: None
    p.open_trace = lambda: None
    p.close_trace = lambda: None
    # The merge split these: the task methods live on LadderPane, the window
    # chrome on CellPanel. The stand-in plays both, as the real panel does
    # through the pane's delegation to its shell.
    for name in ('blocked', '_activate', 'float_on', 'hold_on',
                 'setpoint_step', 'hold_here', 'release', 'banner_state',
                 'preflight', '_wait_mode', 'gains', 'apply_gains',
                 '_clear_run_state', '_driver_down', '_state_age',
                 '_sample', '_restore_arm_controller', 'go_impl',
                 'start_tracking', 'stop_tracking', 'refresh', 'pill_states',
                 'paint_tracking'):
        setattr(p, name, types.MethodType(getattr(ip.LadderPane, name), p))
    for name in ('on_close', 'tick', '_on_signal', '_update_image',
                 'toggle_camera'):
        setattr(p, name, types.MethodType(getattr(ip.CellPanel, name), p))
    p.go = p.go_impl
    p._banner_state = p.banner_state
    p._refresh = p.refresh
    p._pill_states = p.pill_states
    p.panes = (p,)
    p.active_pane = lambda: p
    p.ladder = p
    for name in ('b_pre', 'b_float', 'b_hold', 'b_minus', 'b_plus', 'b_here',
                 'b_track', 'b_track_stop', 'b_release'):
        setattr(p, name, _Button())
    p.gain_error = ip.LadderPane.gain_error
    return p


def said(p, text):
    return any(text in m for m in p.logs)


@pytest.fixture(autouse=True)
def fast(ip, monkeypatch):
    monkeypatch.setattr(ip, 'PREFLIGHT_MODE_TIMEOUT_S', 0.3)


# ---- gate -----------------------------------------------------------------

@pytest.mark.parametrize('kwargs,blocked', [
    (dict(), False),
    (dict(mode=IDLE), True),
    (dict(mode=REFLEX), True),
    (dict(has_state=False), True),
    (dict(age=5.0), True),                    # stale state is no state
])
def test_blocked_table(ip, kwargs, blocked):
    assert (_panel(ip, FakeNode(**kwargs)).blocked() is not None) == blocked


def test_reflex_names_error_recovery(ip):
    assert 'REFLEX' in _panel(ip, FakeNode(mode=REFLEX)).blocked()


# ---- pre-flight -------------------------------------------------------------

def test_ladder_is_locked_until_preflight(ip):
    node = FakeNode()
    p = _panel(ip, node, preflight_ok=False)
    assert p._activate(True) is False
    assert not node.calls, 'touched the robot before pre-flight'
    assert said(p, 'PRE-FLIGHT')


def test_preflight_releases_sets_both_and_restores_in_order(ip):
    node = FakeNode()
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert node.calls == [('switch', (), (ARM,)), ('load',), ('collision',),
                          ('switch', (ARM,), ())]
    assert p.preflight_ok is True
    # The payload now lives in Desk; PRE-FLIGHT only zeroes the FCI load.
    mass, com, inertia = node.load_args
    assert mass == 0.0
    assert list(com) == [0.0, 0.0, 0.0] and list(inertia) == [0.0, 0.0, 0.0]
    assert node.ctrl[ARM] == 'active' and node.mode == MOVE


def test_reflex_threshold_sits_above_the_controller_force_ceiling(ip):
    """Below the ceiling, the controller's own capped push would trip the
    reflex and kill the driver. And the ceiling must be the yaml's."""
    yaml = (SRC / 'fr3_mating_controllers' / 'config'
            / 'cartesian_impedance_stroke.yaml').read_text()
    ceiling = float(re.search(r'max_force_n:\s*([0-9.]+)', yaml).group(1))
    assert ceiling == ip.CONTROLLER_MAX_FORCE_N
    assert min(ip.COLLISION_WRENCH[:3]) > ceiling
    assert all(lo <= hi for lo, hi in zip(ip.CONTACT_WRENCH,
                                          ip.COLLISION_WRENCH))
    assert all(lo <= hi for lo, hi in zip(ip.CONTACT_TORQUE_NM,
                                          ip.COLLISION_TORQUE_NM))
    assert ip.PUSH_LIMIT_N < min(ip.CONTACT_WRENCH[:3])


def test_preflight_refuses_while_impedance_is_active(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert not node.calls


def test_preflight_refuses_when_arm_controller_is_not_active(ip):
    node = FakeNode()
    node.ctrl[ARM] = 'inactive'
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert not node.calls


def test_failed_set_load_still_restores_the_arm_controller(ip):
    node = FakeNode()
    node.load_ok = False
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert node.calls[-1] == ('switch', (ARM,), ())
    assert p.preflight_ok is False and node.ctrl[ARM] == 'active'


def test_robot_that_never_idles_restores_without_setting_anything(ip):
    node = FakeNode(goes_idle=False)
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    kinds = [c[0] for c in node.calls]
    assert 'load' not in kinds and 'collision' not in kinds
    assert node.calls[-1] == ('switch', (ARM,), ())
    assert p.preflight_ok is False


def test_unrestored_arm_controller_fails_preflight_loudly(ip):
    node = FakeNode()
    p = _panel(ip, node, preflight_ok=False)
    real_switch = node.switch

    def switch(activate, deactivate, timeout_s=10.0):
        if activate:                           # the restore
            node.calls.append(('switch', tuple(activate), tuple(deactivate)))
            return False, 'controller_manager refused'
        return real_switch(activate, deactivate)
    node.switch = switch
    p.preflight()
    assert p.preflight_ok is False
    assert said(p, 'did NOT come back')


# ---- gains ------------------------------------------------------------------

def test_gain_limits_are_the_controllers_limits(ip):
    header = (SRC / 'fr3_mating_controllers' / 'include'
              / 'fr3_mating_controllers' / 'impedance_detail.hpp').read_text()

    def const(name):
        return float(re.search(rf'{name}\s*=\s*([0-9.]+)', header).group(1))
    assert ip.GAIN_LIMITS['k_xy'] == (0.0, const('kPosMax'))
    assert ip.GAIN_LIMITS['k_z'] == (0.0, const('kPosMax'))
    assert ip.GAIN_LIMITS['k_rp'] == (0.0, const('kRotMax'))
    assert ip.GAIN_LIMITS['k_yaw'] == (0.0, const('kRotMax'))
    assert ip.GAIN_LIMITS['zeta'] == (const('zetaMin'), const('zetaMax'))


@pytest.mark.parametrize('bad', [{'zeta': '0'}, {'zeta': '-1'},
                                 {'zeta': 'nan'}, {'k_z': '80000'},
                                 {'k_rp': '-5'}])
def test_out_of_range_gains_never_reach_the_controller(ip, bad):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, gains={**GAINS_OK, **bad})
    p.apply_gains()
    assert not node.calls
    assert said(p, 'REFUSING')


def test_boundary_gains_are_applied(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True,
               gains={'k_xy': '0', 'k_z': '3000', 'k_rp': '300',
                      'k_yaw': '0', 'zeta': '0.1'})
    p.apply_gains()
    assert node.calls and node.calls[0][0] == 'params'


# ---- activation -------------------------------------------------------------

def test_activation_sets_float_mode_before_switching(ip):
    """on_activate reads float_mode; setting it after would come up holding."""
    node = FakeNode()
    p = _panel(ip, node)
    assert p._activate(True) is True
    assert [c[0] for c in node.calls] == ['params', 'switch']
    assert node.calls[0][1] == {'float_mode': True}
    assert node.calls[1][1:] == ((IMP,), (ARM,))
    assert p.active and p.floating


def test_refuses_when_controller_not_loaded(ip):
    node = FakeNode(loaded=None)
    p = _panel(ip, node)
    assert p._activate(True) is False
    assert not node.calls
    assert said(p, 'REFUSING')


def test_refuses_when_robot_not_in_move(ip):
    node = FakeNode(mode=IDLE)
    p = _panel(ip, node)
    assert p._activate(True) is False
    assert not node.calls


def test_failed_switch_leaves_panel_inactive(ip):
    node = FakeNode()
    node.switch_ok = False
    p = _panel(ip, node)
    assert p._activate(False) is False
    assert not p.active and not p.floating


def test_restarted_panel_adopts_a_live_controller(ip):
    """Found already active: the panel must mark it, or RELEASE would refuse."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=False)
    assert p._activate(False) is True
    assert p.active is True
    assert [c[0] for c in node.calls] == ['params']


def test_float_press_while_already_active_switches_mode_only(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, floating=False)
    p.float_on()
    assert [c[0] for c in node.calls] == ['params']
    assert node.calls[0][1] == {'float_mode': True} and p.floating


def test_hold_on_sets_a_floor_under_the_current_pose(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, floating=True)
    p.z_floor = None
    p.hold_on()
    assert [c[0] for c in node.calls] == ['params']
    assert node.calls[0][1] == {'float_mode': False}
    assert p.floating is False and p.setpoint is None
    assert p.z_floor == pytest.approx(POS[2] - ip.FLOOR_BELOW_HOLD_MM / 1000)


# ---- setpoints ----------------------------------------------------------------

def test_setpoint_needs_a_holding_controller(ip):
    for active, floating in ((False, False), (True, True)):
        node = FakeNode()
        p = _panel(ip, node, active=active, floating=floating)
        p.setpoint_step(1.0)
        assert not node.published


def test_setpoint_needs_a_floor(ip):
    node = FakeNode()
    p = _panel(ip, node, active=True)
    p.z_floor = None
    p.setpoint_step(1.0)
    assert not node.published


def test_steps_accumulate_on_the_anchor_not_the_arm(ip):
    """The arm lags the anchor; stepping from the ARM would silently halve
    every step once the spring has any lead."""
    node = FakeNode()
    p = _panel(ip, node, active=True)
    p.setpoint_step(1.0)
    p.setpoint_step(1.0)
    assert len(node.published) == 2
    assert node.published[-1][0][2] == pytest.approx(POS[2] + 0.020)


def test_lead_cap_refuses_and_keeps_the_last_anchor(ip):
    node = FakeNode()
    p = _panel(ip, node, active=True, step='50')
    p.setpoint_step(1.0)                      # 50 mm: allowed
    anchor = p.setpoint[0].copy()
    p.setpoint_step(1.0)                      # would be 100 mm: refused
    assert len(node.published) == 1
    assert np.allclose(p.setpoint[0], anchor)
    assert said(p, 'REFUSING')


def test_floor_refuses_a_step_down_through_it(ip):
    node = FakeNode()
    p = _panel(ip, node, active=True, step='50')
    p.setpoint_step(-1.0)                     # 50 mm down, floor is 30 mm
    assert not node.published
    assert said(p, 'below the floor')


def test_floor_also_catches_tool_z_pointing_down(ip):
    node = FakeNode(quat=QUAT_DOWN)
    p = _panel(ip, node, active=True, step='50', axis='tool Z (stroke)')
    p.setpoint_step(1.0)                      # +tool Z is DOWN here
    assert not node.published


def test_small_step_down_above_the_floor_is_allowed(ip):
    node = FakeNode(quat=QUAT_DOWN)
    p = _panel(ip, node, active=True, axis='tool Z (stroke)')
    p.setpoint_step(1.0)
    assert node.published[-1][0][2] == pytest.approx(POS[2] - 0.010)


def test_hold_here_zeroes_the_lead(ip):
    node = FakeNode()
    p = _panel(ip, node, active=True)
    p.setpoint_step(1.0)
    p.hold_here()
    assert np.allclose(node.published[-1][0], POS)
    assert np.allclose(p.setpoint[0], POS)


# ---- release, close, exit ------------------------------------------------------

def test_release_hands_back_and_clears_state(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, floating=True)
    p.release()
    assert node.calls[-1] == ('switch', (ARM,), (IMP,))
    assert not p.active and not p.floating and p.setpoint is None
    assert p.z_floor is None


def test_failed_release_stays_loud_and_active(ip):
    node = FakeNode(loaded='active')
    node.switch_ok = False
    p = _panel(ip, node, active=True)
    p.release()
    assert p.active, 'claimed the arm was handed back when it was not'
    assert said(p, 'RELEASE FAILED') and said(p, 'E-stop')


def test_release_with_the_driver_dead_says_so_not_e_stop(ip):
    node = FakeNode(loaded='active', cm_ready=False, has_state=False)
    p = _panel(ip, node, active=True)
    p.release()
    assert said(p, 'DRIVER DOWN')
    assert not said(p, 'E-stop')
    assert not p.active and p.preflight_ok is False
    assert not node.calls


def test_release_corrects_a_stale_panel_without_switching(ip):
    node = FakeNode(loaded='inactive')
    p = _panel(ip, node, active=True)
    p.release()
    assert not node.calls
    assert not p.active and said(p, 'corrected')


def test_release_asks_the_controller_manager_not_the_panel(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=False)          # panel restarted, forgot
    p.release()
    assert node.calls[-1] == ('switch', (ARM,), (IMP,))


def test_close_refuses_while_an_action_runs(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.busy = True
    p.on_close()
    assert not p.destroyed and not node.calls


def test_close_releases_by_controller_manager_state(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=False)
    p.on_close()
    assert node.calls[-1] == ('switch', (ARM,), (IMP,))
    assert p.destroyed


def test_close_stays_open_if_the_release_fails(ip):
    node = FakeNode(loaded='active')
    node.switch_ok = False
    p = _panel(ip, node, active=True)
    p.on_close()
    assert not p.destroyed and p.busy is False


def test_exit_helper(ip):
    active = FakeNode(loaded='active')
    assert ip.release_if_active(active, say=lambda m: None) is True
    assert active.ctrl[IMP] == 'inactive'
    idle = FakeNode(loaded='inactive')
    assert ip.release_if_active(idle, say=lambda m: None) is True
    assert not idle.calls
    down = FakeNode(cm_ready=False)
    assert ip.release_if_active(down, say=lambda m: None) is None


def test_banner_warns_when_the_robot_leaves_move_mid_run(ip):
    node = FakeNode(mode=REFLEX)
    p = _panel(ip, node, active=True)
    state, _, sub = p._banner_state()
    assert state == 'CHECK THE ROBOT' and 'RELEASE' in sub


def test_banner_says_driver_down_only_when_the_controller_manager_is_gone(ip):
    """A stuck state relay with a live controller manager is NOT a dead driver:
    relaunching then would kill a live impedance controller."""
    dead = FakeNode(has_state=False, cm_ready=False)
    assert _panel(ip, dead, active=True)._banner_state()[0] == 'DRIVER DOWN'
    relay_stuck = FakeNode(has_state=False, cm_ready=True)
    state, _, sub = _panel(ip, relay_stuck, active=True)._banner_state()
    assert state == 'NO ROBOT STATE' and 'RELEASE' in sub


# ---- trace --------------------------------------------------------------------

def test_samples_are_traced_only_while_active_with_joints_and_anchor(ip):
    node = FakeNode()
    p = _panel(ip, node, active=False)
    p.tracef = object()
    s = {'pos': [0.4, 0.0, 0.3], 'q': [0.1] * 7, 'mode': MOVE}
    p._sample(s)
    assert not p.traces
    p.active = True
    p.setpoint = (np.array([0.4, 0.0, 0.31]), np.array([0, 0, 0, 1.0]))
    p._sample(s)
    rec = p.traces[-1]
    assert rec['rec'] == 'sample' and rec['q'] == [0.1] * 7
    assert rec['anchor'] == pytest.approx([0.4, 0.0, 0.31])


def test_trace_is_safe_across_threads(ip):
    """The spin thread samples at 50 Hz while workers log events."""
    import io
    p = types.SimpleNamespace(tracef=io.StringIO(),
                              _trace_lock=threading.Lock())
    trace = types.MethodType(ip.CellPanel.trace, p)
    threads = [threading.Thread(target=lambda: [trace({'rec': 'x', 'i': i})
                                                for i in range(500)])
               for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    lines = p.tracef.getvalue().splitlines()
    assert len(lines) == 2000
    import json
    assert all(json.loads(line)['rec'] == 'x' for line in lines)


# ---- review round 2: close, HOLD, restore, exit, live view ---------------------------

def test_close_stays_open_when_the_controller_manager_is_silent(ip):
    """Driver alive (fresh state) but no answer: impedance may be live."""
    node = FakeNode(loaded='active', cm_ready=False)
    p = _panel(ip, node, active=True)
    p.on_close()
    assert not p.destroyed and p.busy is False
    assert said(p, 'cannot confirm')


def test_close_is_allowed_when_the_driver_is_dead(ip):
    node = FakeNode(loaded='active', cm_ready=False, has_state=False)
    p = _panel(ip, node, active=True)
    p.on_close()
    assert p.destroyed and p.busy is False


def test_pressing_hold_again_changes_nothing(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, floating=False)
    anchor = (POS + [0, 0, 0.01], np.array([0, 0, 0, 1.0]))
    p.setpoint, floor = anchor, p.z_floor
    node.pos = POS - [0, 0, 0.02]            # arm lower than where HOLD started
    p.hold_on()
    assert not node.calls
    assert p.z_floor == floor and p.setpoint is anchor
    assert said(p, 'already holding') and not said(p, 're-seeded')


def test_float_then_hold_never_lowers_the_floor(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True, floating=True)
    floor = p.z_floor
    node.pos = POS - [0, 0, 0.05]            # floated 50 mm down by hand
    p.hold_on()
    assert p.z_floor == floor
    p.floating = True
    node.pos = POS + [0, 0, 0.10]            # floated up: the floor may rise
    p.hold_on()
    assert p.z_floor == pytest.approx(node.pos[2] - ip.FLOOR_BELOW_HOLD_MM / 1000)


def test_stepping_up_from_below_the_floor_is_allowed(ip):
    node = FakeNode()
    p = _panel(ip, node, active=True)
    p.z_floor = POS[2] + 0.020               # the arm sits below the floor
    p.setpoint_step(1.0)
    assert len(node.published) == 1, 'refused a step that moves the anchor UP'
    p.setpoint_step(-1.0)                    # back down, still below the floor
    assert len(node.published) == 1, 'allowed a step that lowers it below the floor'
    assert said(p, 'below the floor')


def test_restore_is_judged_by_the_controller_manager_not_the_release_call(ip):
    """Release times out on our side but completes in the controller manager:
    the arm controller must still be brought back."""
    node = FakeNode()
    real = node.switch

    def switch(activate, deactivate, timeout_s=10.0):
        ok, msg = real(activate, deactivate)
        return (False, 'switch_controller timed out') if deactivate else (ok, msg)
    node.switch = switch
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert node.ctrl[ARM] == 'active'
    assert ('switch', (ARM,), ()) in node.calls
    assert p.preflight_ok is False and p.arm_released is False


def test_restore_that_never_reaches_move_fails_preflight(ip):
    node = FakeNode()
    real = node.switch

    def switch(activate, deactivate, timeout_s=10.0):
        ok, msg = real(activate, deactivate)
        if activate:
            node.mode = IDLE                 # controller claims active, robot idles
        return ok, msg
    node.switch = switch
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert p.preflight_ok is False
    assert said(p, 'robot not in MOVE')
    assert p.arm_released is True


def test_preflight_reports_the_resting_force_bias(ip):
    node = FakeNode()
    node.force = np.array([0.0, 0.0, 8.0])
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert p.preflight_ok is True
    assert said(p, 'at rest 8.0 N') and said(p, 'eats into the reflex margin')


def test_exit_waits_for_a_running_action_then_restores_the_arm_controller(ip):
    node = FakeNode()
    node.ctrl[ARM] = 'inactive'              # PRE-FLIGHT released it
    panel = types.SimpleNamespace(busy=True, arm_released=True)
    threading.Timer(0.1, lambda: setattr(panel, 'busy', False)).start()
    msgs = []
    assert ip.exit_handoff(node, panel, say=msgs.append, wait_s=2.0) is True
    assert node.ctrl[ARM] == 'active'
    assert ('switch', (ARM,), ()) in node.calls


def test_exit_leaves_the_arm_controller_alone_unless_preflight_released_it(ip):
    node = FakeNode()
    node.ctrl[ARM] = 'inactive'              # someone else's decision
    panel = types.SimpleNamespace(busy=False, arm_released=False)
    ip.exit_handoff(node, panel, say=lambda m: None, wait_s=0.1)
    assert not node.calls


def test_exit_hands_back_live_impedance_and_warns_when_it_cannot_ask(ip):
    live = FakeNode(loaded='active')
    ip.exit_handoff(live, None, say=lambda m: None)
    assert live.ctrl[IMP] == 'inactive'
    msgs = []
    ip.exit_handoff(FakeNode(cm_ready=False), None, say=msgs.append)
    assert any('could not ask' in m for m in msgs)


def test_live_view_keeps_running_after_an_error(ip):
    node = FakeNode()
    p = _panel(ip, node)

    def boom():
        raise RuntimeError('draw failed')
    p._refresh = boom
    p.tick()
    assert (150, p.tick) in p.after_calls


def test_ctrl_c_takes_the_guarded_close_path(ip):
    p = _panel(ip, FakeNode())
    p._on_signal(2, None)
    assert p.after_calls == [(0, p.on_close)]


class _SlowWriter:
    """Writes one character at a time, yielding in between - unlike StringIO,
    concurrent writers without a lock interleave here."""

    def __init__(self):
        self.chars = []

    def write(self, text):
        for ch in text:
            self.chars.append(ch)
            time.sleep(0)

    def flush(self):
        pass


def test_trace_lock_keeps_concurrent_records_whole(ip):
    import json
    p = types.SimpleNamespace(tracef=_SlowWriter(), _trace_lock=threading.Lock())
    trace = types.MethodType(ip.CellPanel.trace, p)
    threads = [threading.Thread(target=lambda: [trace({'rec': 'x', 'i': i})
                                                for i in range(200)])
               for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    lines = ''.join(p.tracef.chars).splitlines()
    assert len(lines) == 800
    assert all(json.loads(line)['rec'] == 'x' for line in lines)


def test_shipped_yaml_is_inside_the_controllers_limits(ip):
    """A yaml edit outside these makes configure fail on the day."""
    import yaml
    header = (SRC / 'fr3_mating_controllers' / 'include'
              / 'fr3_mating_controllers' / 'impedance_detail.hpp').read_text()

    def const(name):
        return float(re.search(rf'{name}\s*=\s*([0-9.]+)', header).group(1))
    spec = [float(v) for v in re.search(r'kTauSpecNm\{([^}]*)\}', header)
            .group(1).split(',')]
    cfg = yaml.safe_load((SRC / 'fr3_mating_controllers' / 'config'
                          / 'cartesian_impedance_stroke.yaml').read_text())
    prm = cfg['cartesian_impedance_stroke_controller']['ros__parameters']
    assert all(0 <= k <= const('kPosMax') for k in prm['k_pos_tool'])
    assert all(0 <= k <= const('kRotMax') for k in prm['k_rot_tool'])
    assert const('zetaMin') <= prm['damping_ratio'] <= const('zetaMax')
    assert 0 <= prm['nullspace_stiffness'] <= const('nullspaceMax')
    assert const('maxForceMin') <= prm['max_force_n'] <= const('maxForceMax')
    assert const('maxTorqueMin') <= prm['max_torque_nm'] <= const('maxTorqueMax')
    assert const('tauRateMin') <= prm['tau_rate_limit'] <= const('tauRateMax')
    assert const('slewMpsMin') <= prm['setpoint_slew_mps'] <= const('slewMpsMax')
    assert const('slewRpsMin') <= prm['setpoint_slew_rps'] <= const('slewRpsMax')
    assert len(prm['tau_max_nm']) == 7
    assert all(const('tauMaxMin') <= t <= s_ for t, s_ in zip(prm['tau_max_nm'], spec))


def test_shutdown_stops_the_spin_before_dropping_the_context(ip, monkeypatch):
    """A spin thread still running when rclpy.shutdown() lands ABORTS the
    process ("terminate called without an active exception") - and it aborts
    inside the exit path that hands the arm back. Observed 2026-09-16 on an
    ordinary window close, harmless only because the race fell the right way.
    The order below is what makes it not a race."""
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
    monkeypatch.setattr(ip, 'exit_handoff',
                        lambda *a, **k: order.append(('exit_handoff', None)))
    monkeypatch.setattr(ip.rclpy, 'shutdown',
                        lambda: order.append(('rclpy.shutdown', None)),
                        raising=False)

    ip.shutdown_ros(node, None, Executor(), Spin(), say=lambda m: None)

    assert [n for n, _ in order] == [
        'exit_handoff', 'node.close', 'executor.shutdown', 'spin.join',
        'node.destroy_node', 'rclpy.shutdown']
    assert dict(order)['spin.join'] == ip.SPIN_JOIN_S


def test_shutdown_hands_the_arm_back_even_if_the_teardown_throws(ip,
                                                                 monkeypatch):
    """The handoff runs first and its result is never lost to a later error."""
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
    monkeypatch.setattr(ip, 'exit_handoff',
                        lambda *a, **k: handed.append(True))
    monkeypatch.setattr(ip.rclpy, 'shutdown', lambda: None, raising=False)

    with pytest.raises(RuntimeError):
        ip.shutdown_ros(node, None, Executor(), Spin(), say=lambda m: None)
    assert handed == [True]


# ---- tracking ---------------------------------------------------------------
# The shipped tracking numbers (tools/fr3/fr3_params.yaml) must stay inside
# the limits the C++ enforces, for the same reason the impedance yaml must:
# a value outside them is a refusal on the day, at the arm, with the operator
# waiting. The constants are SCRAPED from the headers so a limit moves in one
# place - which means they must stay bare decimal literals there.
IMPEDANCE_HPP = ('fr3_mating_controllers/include/fr3_mating_controllers'
                 '/impedance_detail.hpp')
TRACKING_HPP = 'mating_controller/include/mating_controller/tracking_law.hpp'


def _consts(rel_path):
    """A const(name) -> float over one C++ header."""
    header = (SRC / rel_path).read_text()

    def const(name):
        return float(re.search(rf'{name}\s*=\s*([0-9.]+)', header).group(1))
    return const


def _shipped():
    """The tracking parameters as the node will actually receive them."""
    import yaml
    cfg = yaml.safe_load((SRC / 'tools' / 'fr3'
                          / 'fr3_params.yaml').read_text())
    return cfg['/**']['ros__parameters']


def _controller_yaml():
    import yaml
    return yaml.safe_load((SRC / 'fr3_mating_controllers' / 'config'
                           / 'cartesian_impedance_stroke.yaml').read_text()
                          )['cartesian_impedance_stroke_controller'][
                              'ros__parameters']


def test_track_profile_is_inside_the_controllers_gain_limits(ip):
    """The track profile is applied atomically at ~/start_tracking; outside
    GainLimits the controller rejects the whole set and tracking does not
    start."""
    const = _consts(IMPEDANCE_HPP)
    prm = _shipped()
    k_pos, k_rot = prm['track_k_pos_tool'], prm['track_k_rot_tool']
    assert all(0 <= k <= const('kPosMax') for k in k_pos)
    assert all(0 <= k <= const('kRotMax') for k in k_rot)
    assert len(set(k_pos)) == 1 and len(set(k_rot)) == 1, (
        'the track profile must be isotropic: an anisotropic tool-frame K '
        'deflects the commanded force off the commanded direction at the '
        'measured 18.5 deg median tilt (TRACKING_SPEC.md section 4)')
    assert const('zetaMin') <= prm['track_damping_ratio'] <= 1.0, (
        'zeta above 1.0 is not covered by the measurement this profile '
        'rests on (zero overshoot in 32 clean steps, 2026-09-22). '
        'TRACKING_SPEC.md O1 asks for zeta 0.5 while walking k_rot up, '
        'which this range allows; to go above 1.0, re-measure and move '
        'this bound with the evidence.')


def test_track_slew_is_inside_the_controllers_slew_limits(ip):
    """Slew is live now, but ConfigLimits is still the hard bound: the live
    path validates against the same numbers the configure path does."""
    const = _consts(IMPEDANCE_HPP)
    prm = _shipped()
    assert (const('slewMpsMin') <= prm['track_setpoint_slew_mps']
            <= const('slewMpsMax'))
    assert (const('slewRpsMin') <= prm['track_setpoint_slew_rps']
            <= const('slewRpsMax'))


def test_tracking_ceilings_match_the_controller_yaml(ip):
    """The node validates its lead against ceilings it is TOLD; if those
    drift from the controller's own, it validates against fiction."""
    prm, ctrl = _shipped(), _controller_yaml()
    assert prm['tracking_max_force_n'] == ctrl['max_force_n']
    assert prm['tracking_max_force_n'] == ip.CONTROLLER_MAX_FORCE_N
    assert prm['tracking_max_torque_nm'] == ctrl['max_torque_nm']


def test_tracking_lead_force_stays_under_every_ceiling(ip):
    """The integrator's whole anti-windup is the clamp: worst case the lead
    adds k * lead_max, and that must stay under the controller's ceiling and
    well under the reflex that kills the driver."""
    const = _consts(TRACKING_HPP)
    prm = _shipped()
    lead_n = max(prm['track_k_pos_tool']) * prm['tracking_lead_max_m']
    assert lead_n <= const('kLeadForceMaxN')
    assert const('kLeadForceMaxN') < ip.CONTROLLER_MAX_FORCE_N
    assert const('kLeadForceMaxN') < min(ip.COLLISION_WRENCH[:3])
    lead_nm = max(prm['track_k_rot_tool']) * prm['tracking_lead_max_rad']
    assert lead_nm < prm['tracking_max_torque_nm']


def test_tracking_deadbands_match_the_track_stiffness(ip):
    """The deadband is F_friction / k, so it belongs to the stiffness in use.
    This catches a deadband copied from another k - the 1.8 mm measured at
    k = 3000 is wrong here - and a half-done edit when O1 moves k_rot."""
    const = _consts(TRACKING_HPP)
    prm = _shipped()
    band_m = const('kFrictionBreakawayN') / max(prm['track_k_pos_tool'])
    assert band_m <= prm['tracking_deadband_m'] <= 3.0 * band_m
    band_rad = const('kFrictionBreakawayNm') / max(prm['track_k_rot_tool'])
    assert band_rad <= prm['tracking_deadband_rad'] <= 3.0 * band_rad


def test_tracking_lead_cap_and_floor_match_the_panel(ip):
    """A 50 Hz stream must not be looser than the hand-stepped path: same
    equilibrium-lead cap, same floor under where the run started."""
    prm = _shipped()
    assert prm['tracking_max_lead_m'] * 1000 <= ip.MAX_LEAD_MM
    assert (prm['tracking_floor_below_start_m'] * 1000
            == pytest.approx(ip.FLOOR_BELOW_HOLD_MM))


def test_the_panel_calls_the_services_the_node_actually_offers(ip):
    """Both buttons are dead - silently, as "the tracking node is not
    running" - if the node's name or its two service names drift."""
    node_cpp = (SRC / 'mating_controller' / 'src'
                / 'tracking_node.cpp').read_text()
    name = re.search(r'Node\("([a-z_]+)"', node_cpp).group(1)
    assert name == ip.TRACKING_NODE
    assert '"~/start_tracking"' in node_cpp
    assert '"~/stop_tracking"' in node_cpp
    assert ip.TRACK_START_SRV == f'/{name}/start_tracking'
    assert ip.TRACK_STOP_SRV == f'/{name}/stop_tracking'


def test_stop_tracking_is_never_disabled_by_a_running_action(ip):
    """A stop control that greys out while the arm is moving is not a stop
    control. STOP TRACKING is outside go(): still clickable, still answered,
    while another action holds the panel busy."""
    node = FakeNode()
    p = _panel(ip, node, active=True)
    running = threading.Event()
    p.go(lambda: running.wait(2.0))
    try:
        assert p.busy is True
        assert p.b_track['state'] == 'disabled'
        assert p.b_release['state'] == 'disabled'
        assert p.b_track_stop['state'] == 'normal'
        p.stop_tracking()
        assert ('trigger', 'stop') in node.calls
    finally:
        running.set()


@pytest.mark.parametrize('node_kwargs,panel_kwargs', [
    (dict(mode=REFLEX), dict(active=True)),       # robot cannot be driven
    (dict(), dict(active=True, floating=True)),   # free-floating: no gain step
    (dict(), dict(active=False)),                 # not holding, no Z floor
])
def test_start_tracking_refuses_before_touching_the_tracking_node(
        ip, node_kwargs, panel_kwargs):
    node = FakeNode(**node_kwargs)
    p = _panel(ip, node, **panel_kwargs)
    p.start_tracking()
    assert not node.calls, 'asked the node to start anyway'
    assert said(p, 'REFUSING')


def test_release_stops_tracking_before_handing_the_arm_back(ip):
    """An orphaned tracker resumes autonomous motion at the next activation,
    so RELEASE must stop the 50 Hz stream, and stop it BEFORE the switch."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node)
    p.release()
    order = [c for c in node.calls if c[0] in ('trigger', 'switch')]
    assert order[0] == ('trigger', 'stop'), order
    assert any(c[0] == 'switch' for c in order), order
    assert order.index(('trigger', 'stop')) < \
        next(i for i, c in enumerate(order) if c[0] == 'switch')


def test_release_does_not_touch_a_dead_driver(ip):
    """No stop call when there is nothing alive to stop - the node self-halts
    on the controller leaving ACTIVE, and a dead driver has no services."""
    node = FakeNode(loaded='active', cm_ready=False, age=9.0)
    p = _panel(ip, node)
    p.release()
    assert not node.calls


def test_exit_release_stops_tracking_only_when_impedance_is_live(ip):
    live = FakeNode(loaded='active')
    idle = FakeNode(loaded='inactive')
    assert ip.release_if_active(live, say=lambda m: None) is True
    assert ('trigger', 'stop') in live.calls
    assert ip.release_if_active(idle, say=lambda m: None) is True
    assert not idle.calls


def test_track_timeout_exceeds_the_nodes_own_worst_case_start(ip):
    """The panel must never report 'not started' while the arm is tracking.
    The node's start makes three parameter round-trips, each bounded by
    wait_for_service(1 s) + tracking_profile_timeout_s, plus tool-offset
    sampling and the settle."""
    prm = _shipped()
    worst = 3 * (1.0 + prm['tracking_profile_timeout_s']) \
        + prm['tracking_settle_s'] + 1.0
    assert ip.TRACK_CALL_TIMEOUT_S >= worst, (
        f'TRACK_CALL_TIMEOUT_S {ip.TRACK_CALL_TIMEOUT_S} s is below the '
        f'node\'s {worst} s worst-case start')


def test_camera_toggle_creates_and_destroys_the_subscription(ip):
    """Turning the view off must take the frames OFF THE WIRE, not just stop
    drawing them: /aruco/debug_image is subscribe-gated at the publisher, so
    only destroying the subscription stops it being encoded. (The default is
    ON - ALIGN cannot work blind, and that is what the proven 2026-09-11/15
    alignment runs used.)"""
    calls = []

    class CamNode(FakeNode):
        def camera(self, on):
            calls.append(on)
            return on

    node = CamNode()
    p = _panel(ip, node)
    p.v_cam = _Var(False)
    p.photo = None
    p.image_label = types.SimpleNamespace(config=lambda **k: None)
    p.toggle_camera = types.MethodType(ip.LadderPane.toggle_camera, p)
    p._update_image = types.MethodType(ip.CellPanel._update_image, p)

    # nothing subscribes until the box is ticked
    p._update_image()
    assert calls == []

    p.v_cam = _Var(True)
    p.toggle_camera()
    assert calls == [True]
    assert p.traces[-1] == {'rec': 'camera', 'on': True}

    p.v_cam = _Var(False)
    p.toggle_camera()
    assert calls == [True, False]
    assert p.traces[-1] == {'rec': 'camera', 'on': False}


def test_preflight_zeroes_the_fci_payload_so_desk_is_the_only_source(ip):
    """The payload lives in Desk's end-effector profile. PRE-FLIGHT sends a
    ZERO load so a value left by an earlier session can never be added on
    top of it - the double-count the old entry fields invited."""
    node = FakeNode()
    p = _panel(ip, node, preflight_ok=False)
    p.preflight()
    assert ('load',) in node.calls, node.calls
    mass, com, inertia = node.load_args
    assert mass == 0.0, f'PRE-FLIGHT set a non-zero FCI load: {mass} kg'
    assert list(com) == [0.0, 0.0, 0.0] and list(inertia) == [0.0, 0.0, 0.0]
    assert p.traces[0]['mass_kg'] == 0.0


def test_stop_tracking_is_only_offered_while_tracking_runs(ip):
    """A permanently visible red STOP for an idle feature is noise, and it
    trains the operator to read red as decoration. It appears when the node
    starts streaming and goes away when it stops."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.paint_tracking()
    assert not p.b_track_stop.packed, 'STOP offered before tracking started'
    assert p.b_track.state == 'normal'

    p.start_tracking()
    assert p.tracking and p.b_track_stop.packed
    # the node owns the equilibrium now; hand-stepping it would fight the stream
    assert p.b_track.state == 'disabled'
    assert all(b.state == 'disabled'
               for b in (p.b_minus, p.b_plus, p.b_here))

    p.stop_tracking()
    assert not p.tracking and not p.b_track_stop.packed
    assert p.b_track.state == 'normal'
    assert all(b.state == 'normal' for b in (p.b_minus, p.b_plus, p.b_here))


def test_release_takes_the_stop_button_away_too(ip):
    """RELEASE stops the stream, so the button that stops it must go."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    assert p.b_track_stop.packed
    p.release()
    assert not p.tracking and not p.b_track_stop.packed
