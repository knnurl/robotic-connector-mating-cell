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
  * while the node tracks, nothing on the ladder can step the equilibrium,
    float the arm or change the gains it will restore; START sends the ALIGN
    goal first, in one
    atomic call; the node's latched status drives the banner, and a panel
    started mid-run adopts the tracker
  * ALIGN reads the marker in the camera optical frame, whatever frame
    /aruco/pose was published in
  * traces go to $FR3_LOG_DIR/YYYY-MM-DD when fr3_env.sh set it

    python3 -m pytest tools/fr3/test_cell_panel.py -q
"""

import datetime
import inspect
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

    def set(self, v):
        self.v = v


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
        self.trigger_threads = []             # threads call_trigger ran on
        self.track_start_cli, self.track_stop_cli = 'start', 'stop'
        self.trigger_ok = True
        self.param_ok = self.switch_ok = True
        self.load_ok = self.collision_ok = True
        self.load_args = None
        self.force = np.zeros(3)
        self.status = None                    # (fields, age_s) as track_status()
        self.track_params_ok = True

    def track_status(self):
        return self.status

    def set_tracking_params(self, values, atomic=True):
        self.calls.append(('track_params', dict(values), atomic))
        return self.track_params_ok, ('applied' if self.track_params_ok
                                      else 'the tracking node is not running')

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
        self.trigger_threads.append(threading.get_ident())
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
        arm_released=False, after_calls=[], tracking=False, _track_seen=None,
        _tracking_at=float('-inf'), _painted=None,
        z_floor=(POS[2] - 0.030) if active else None,
        logs=[], traces=[], destroyed=[], v_step=_Var(step), v_axis=_Var(axis),
        v_policy=_Var('hold'),
        tune={k: _Var(v) for k, v in (gains or GAINS_OK).items()})
    # ALIGN's goal, which START hands to the tracking node, read the way
    # ALIGN reads it: millimetres and degrees in the entry fields
    p.align = types.SimpleNamespace(v_target=_Var('100'), v_inplane=_Var('90'))
    for name in ('target_m', 'inplane_target'):
        setattr(p.align, name,
                types.MethodType(getattr(ip.AlignPane, name), p.align))
    p.root = types.SimpleNamespace(
        destroy=lambda: p.destroyed.append(True),
        after=lambda ms, fn, *a: p.after_calls.append((ms, fn) + a))
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
                 'paint_tracking', '_run_finished', '_refuse_while_tracking',
                 'track_banner', '_follow_track_status', 'write_policy',
                 'buttons', '_set_tracking', '_track_state'):
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
                 'b_track', 'b_track_stop', 'b_release', 'b_gains',
                 'track_ind'):
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
    The node's start makes four service round-trips (ListControllers, the
    float_mode read, the gains read, the profile apply), each bounded by
    wait_for_service(1 s) + tracking_profile_timeout_s, plus the settle and
    ~1 s of tool-offset sampling - and the panel keeps 5 s in hand."""
    prm = _shipped()
    worst = 4 * (1.0 + prm['tracking_profile_timeout_s']) \
        + prm['tracking_settle_s'] + 1.0
    assert ip.TRACK_CALL_TIMEOUT_S >= worst + 5.0, (
        f'TRACK_CALL_TIMEOUT_S {ip.TRACK_CALL_TIMEOUT_S} s leaves less than '
        f'5 s over the node\'s {worst} s worst-case start')


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
    run_after(p)
    assert p.tracking and p.b_track_stop.packed
    # the node owns the equilibrium now; hand-stepping it would fight the stream
    assert p.b_track.state == 'disabled'
    assert all(b.state == 'disabled'
               for b in (p.b_minus, p.b_plus, p.b_here))

    p.stop_tracking()
    run_after(p)
    assert not p.tracking and not p.b_track_stop.packed
    assert p.b_track.state == 'normal'
    assert all(b.state == 'normal' for b in (p.b_minus, p.b_plus, p.b_here))


def test_release_takes_the_stop_button_away_too(ip):
    """RELEASE stops the stream, so the button that stops it must go."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    run_after(p)
    assert p.b_track_stop.packed
    p.release()
    run_after(p)
    assert not p.tracking and not p.b_track_stop.packed


# ---- tracking: the ladder while the node drives --------------------------------

QUIET = ('b_float', 'b_track', 'b_minus', 'b_plus', 'b_here', 'b_gains')


def wait_idle(p, timeout_s=2.0):
    """Until go()'s worker is done AND has queued its Tk-thread cleanup -
    it clears busy a moment before it schedules that, and the action may
    have queued a repaint of its own before either."""
    def queued():
        return any(getattr(c[1], '__name__', '') == '_run_finished'
                   for c in p.after_calls)
    end = time.monotonic() + timeout_s
    while (p.busy or not queued()) and time.monotonic() < end:
        time.sleep(0.01)
    assert not p.busy and queued()


def run_after(p):
    """Run what a worker scheduled on the Tk thread, as mainloop would."""
    calls, p.after_calls[:] = list(p.after_calls), []
    for _ms, fn, *a in calls:
        fn(*a)


def test_track_pressed_through_go_leaves_the_ladder_quiet(ip):
    """The real wiring, button -> go() -> start_tracking. go()'s cleanup used
    to re-enable every button AFTER paint_tracking had greyed them, so the
    operator could step the equilibrium under a running stream."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.paint_tracking()
    p.go(p.start_tracking)
    wait_idle(p)
    run_after(p)
    assert p.tracking and p.b_track_stop.packed
    assert {n: getattr(p, n).state for n in QUIET} == \
        dict.fromkeys(QUIET, 'disabled')
    assert all(getattr(p, n).state == 'normal'
               for n in ('b_pre', 'b_hold', 'b_release'))


def test_an_action_while_tracking_does_not_wake_the_ladder(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    run_after(p)
    p.go(p.hold_on)
    wait_idle(p)
    run_after(p)
    assert all(getattr(p, n).state == 'disabled' for n in QUIET)
    p.stop_tracking()
    p.go(p.hold_on)
    wait_idle(p)
    run_after(p)
    assert all(getattr(p, n).state == 'normal' for n in QUIET)


@pytest.mark.parametrize('action', ['setpoint_step', 'hold_here',
                                    'apply_gains', 'float_on'])
def test_hand_steps_and_gains_refuse_while_tracking(ip, action):
    """Greying is not a guard. The node owns the equilibrium, and on stop it
    puts back the gains it snapshotted at START: a hand step would fight the
    stream, and gains applied now would be silently undone. FLOAT would
    free the arm under the node's stream - and the TRACKING banner would
    hide FLOATING."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.tracking = True
    getattr(p, action)(*((1.0,) if action == 'setpoint_step' else ()))
    assert not node.published and not node.calls
    assert said(p, 'REFUSING') and said(p, 'STOP TRACKING')


def test_start_sends_the_align_goal_in_one_atomic_call_first(ip):
    """TRACK holds the camera where ALIGN would leave it, so the node needs
    ALIGN's goal - all of it or none of it - before it starts."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.v_policy = _Var('clamp')
    p.start_tracking()
    assert node.calls == [
        ('track_params', {'tracking_standoff_m': 0.100,
                          'tracking_inplane_deg': 90.0,
                          'tracking_inplane_hold': False,
                          'tracking_over_lead_policy': 'clamp'}, True),
        ('trigger', 'start')]
    assert p.tracking


def test_inplane_off_asks_the_node_to_hold_what_it_sees(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.align.v_inplane = _Var('off')
    p.start_tracking()
    sent = node.calls[0][1]
    assert sent['tracking_inplane_hold'] is True
    assert 'tracking_inplane_deg' not in sent


def test_start_refuses_when_the_goal_cannot_be_written(ip):
    node = FakeNode(loaded='active')
    node.track_params_ok = False
    p = _panel(ip, node, active=True)
    p.start_tracking()
    assert ('trigger', 'start') not in node.calls
    assert not p.tracking and said(p, 'REFUSING')


def test_goal_parameters_are_typed_as_the_yaml_declares_them(ip, monkeypatch):
    """The node declares its parameters from fr3_params.yaml, which fixes
    each one's type: a bool sent as a double is rejected, and with it the
    whole atomic set, so START refuses on the day."""
    monkeypatch.setattr(ip, 'Parameter', types.SimpleNamespace)
    monkeypatch.setattr(ip, 'ParameterValue', types.SimpleNamespace)
    monkeypatch.setattr(ip, 'ParameterType', types.SimpleNamespace(
        PARAMETER_BOOL='bool', PARAMETER_DOUBLE='double',
        PARAMETER_STRING='string', PARAMETER_DOUBLE_ARRAY='double[]'))
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    shipped = _shipped()
    for name, v in node.calls[0][1].items():
        want = {bool: 'bool', float: 'double', str: 'string'}[
            type(shipped[name])]
        assert ip._param_msg(name, v).value.type == want, name


def test_policy_dropdown_writes_the_node_live(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.tracking = True
    p.v_policy = _Var('stop')
    p.write_policy()
    assert node.calls == [('track_params',
                           {'tracking_over_lead_policy': 'stop'}, False)]
    assert p.traces[-1]['rec'] == 'track_policy'


def test_the_policy_choice_is_read_on_the_tk_thread_written_off_it(
        ip, monkeypatch):
    """The dropdown's handler reads the choice where Tk variables may be
    read, then leaves the blocking parameter call to a worker."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p._on_policy_selected = types.MethodType(
        ip.LadderPane._on_policy_selected, p)
    made = []

    class Thread:                             # runs only when the test says
        def __init__(self, target, args=(), daemon=None):
            self.target, self.args, self.daemon = target, args, daemon
            self.started = False
            made.append(self)

        def start(self):
            self.started = True
    monkeypatch.setattr(ip, 'threading', types.SimpleNamespace(Thread=Thread))
    p.v_policy.set('clamp')
    p._on_policy_selected(None)
    assert not node.calls, 'blocked the Tk thread on the parameter call'
    (t,) = made
    assert t.started and t.daemon
    p.v_policy.set('stop')                    # a later choice is a later event
    t.target(*t.args)
    assert node.calls == [('track_params',
                           {'tracking_over_lead_policy': 'clamp'}, False)]


def test_the_dropdown_is_wired_to_the_policy_handler(ip, monkeypatch):
    from unittest import mock
    panel, _, ttk = _built(ip, monkeypatch, FakeNode(loaded='active'))
    assert (mock.call('<<ComboboxSelected>>', panel.ladder._on_policy_selected)
            in ttk.Combobox.return_value.bind.call_args_list)


def test_a_refused_policy_write_shows_what_the_node_has(ip):
    """Left as chosen, the dropdown would claim a policy the node does not
    run. With no word from the node there is nothing to show instead, and
    the choice stands: START sends it with the goal."""
    node = FakeNode(loaded='active')
    node.track_params_ok = False
    node.status = status('tracking', policy='hold')
    p = _panel(ip, node, active=True)
    p.v_policy.set('stop')
    p.write_policy('stop')
    assert p.v_policy.get() == 'stop', 'set a Tk variable off the Tk thread'
    run_after(p)
    assert p.v_policy.get() == 'hold' and said(p, 'NOT written')
    node.status = status('tracking', policy='hold', age=5.0)   # gone silent
    p.v_policy.set('clamp')
    p.write_policy('clamp')
    run_after(p)
    assert p.v_policy.get() == 'clamp'


def test_panel_defaults_are_the_nodes_defaults(ip):
    prm = _shipped()
    assert ip.OVER_LEAD_CHOICES == ['hold', 'stop', 'clamp']
    assert ip.OVER_LEAD_DEFAULT == prm['tracking_over_lead_policy'] == 'hold'
    assert prm['tracking_standoff_m'] * 1000 == \
        pytest.approx(float(ip.TARGET_MM_DEFAULT))
    assert prm['tracking_inplane_deg'] == float(ip.INPLANE_TARGET_DEFAULT)


def status(state, reason='', level=0, policy='hold', age=0.1, **values):
    """A /tracking_node/status as CellNode.track_status() hands it over."""
    f = {'name': 'tracking_node', 'level': level, 'message': '',
         'state': state, 'reason': reason, 'policy': policy,
         'pos_err_mm': '1.2', 'rot_err_deg': '0.3', 'lead_mm': '4.0',
         'lead_deg': '0.5', 'marker_age_s': '0.03', 'raw_age_s': '0.05',
         'standoff_m': '0.1', 'inplane_deg': '90'}
    f.update(values)
    return f, age


def test_the_nodes_status_drives_the_banner(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    node.status = status('tracking')
    assert p.banner_state()[0] == 'TRACKING'
    node.status = status('holding', 'no fresh raw detection', level=1)
    assert p.banner_state()[0] == 'HOLDING - no fresh raw detection'
    node.status = status('idle', 'over lead', level=1)
    assert p.banner_state()[0] == 'STOPPED - over lead'
    p.floating = True                        # ...but never hides FLOATING
    assert p.banner_state()[0] == 'FLOATING'
    p.floating = False
    node.status = status('idle')             # an operator stop: nothing to add
    assert p.banner_state()[0] == 'HOLDING'  # the ladder's own
    node.status, node.mode = status('tracking'), REFLEX
    assert p.banner_state()[0] == 'CHECK THE ROBOT'


@pytest.mark.parametrize('state', ['tracking', 'holding'])
def test_a_panel_started_mid_run_adopts_the_tracker(ip, state):
    """Restarted while the node streams: without adopting it, STOP TRACKING
    is hidden and TRACK is offered for a tracker that is already running.
    HOLDING is the node armed and waiting - it moves again by itself."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node)                      # fresh panel: knows nothing
    node.status = status(state, policy='clamp')
    p._follow_track_status()
    assert p.tracking and p.b_track_stop.packed and p.track_ind.packed
    assert p.v_policy.get() == 'clamp'
    assert all(getattr(p, n).state == 'disabled' for n in QUIET)


@pytest.mark.parametrize('state', ['holding', 'stopping'])
def test_the_panel_leaves_tracking_only_when_the_node_is_idle(ip, state):
    """HOLDING resumes by itself. STOPPING is the node putting back the gains
    it snapshotted: waking the ladder then lets 'apply gains' land just
    before the node overwrites them."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    run_after(p)
    time.sleep(0.01)
    node.status = status(state, 'over lead', level=1, age=0.0)
    p._follow_track_status()
    assert p.tracking and p.b_track_stop.packed
    assert all(getattr(p, n).state == 'disabled' for n in QUIET)
    node.status = status('idle', 'over lead', level=1, age=0.0)
    p._follow_track_status()
    assert not p.tracking and not p.b_track_stop.packed


def test_an_adopted_tracker_leaves_the_panel_holding_after_stop(ip):
    """The node tracks only on an ACTIVE controller that is not floating, so
    that is what an adopting panel knows once the tracker is gone - not
    PRE-FLIGHT NEEDED over an arm the impedance controller holds. TRACK
    still wants the Z floor that only HOLD sets."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, preflight_ok=False)  # restarted: knows nothing
    node.status = status('tracking')
    p._follow_track_status()
    assert p.tracking and p.active and not p.floating
    p.stop_tracking()
    node.status = status('idle', age=0.0)
    p._follow_track_status()
    assert p.banner_state()[0] == 'HOLDING'
    assert {k: v for k, v, _ in p.pill_states()}['CONTROL'] == 'impedance'
    p.start_tracking()
    assert ('trigger', 'start') not in node.calls and said(p, 'no Z floor')
    p.hold_on()
    assert not [c for c in node.calls if c[0] in ('params', 'switch')]
    assert p.z_floor == pytest.approx(POS[2] - ip.FLOOR_BELOW_HOLD_MM / 1000)
    p.start_tracking()
    assert node.calls[-1] == ('trigger', 'start')


def test_a_tracking_session_forgets_the_hand_stepped_anchor(ip):
    """The node owns the equilibrium while it tracks and re-seeds it where
    the arm is on stop. Stepping on from the anchor the panel kept from
    before would jump by however far the node moved the arm."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.setpoint_step(1.0)                      # anchor 10 mm above POS
    p.start_tracking()
    node.pos = POS + [0.040, 0.0, 0.0]        # the node carried the arm 40 mm
    p.stop_tracking()
    p.setpoint_step(1.0)
    assert node.published[-1][0] == pytest.approx(node.pos + [0.0, 0.0, 0.010])
    assert 'lead now 10.0 mm' in p.logs[-1]


def test_tracking_is_painted_on_the_tk_thread_only(ip):
    """START, STOP and RELEASE run on workers and Tk is not thread-safe:
    the header is repainted by a callback the Tk thread runs."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    painted = []
    paint = p.paint_tracking
    p.paint_tracking = lambda: painted.append(threading.get_ident()) or paint()
    for action in (p.start_tracking, p.stop_tracking):
        t = threading.Thread(target=action)
        t.start()
        t.join()
    assert not painted, 'painted Tk widgets from a worker thread'
    run_after(p)
    assert painted and set(painted) == {threading.get_ident()}
    assert not p.b_track_stop.packed


def test_a_stop_the_node_decides_drops_the_panel_out_of_tracking(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    time.sleep(0.01)
    node.status = status('idle', 'over lead', level=1, age=0.0)
    p._follow_track_status()
    assert not p.tracking
    assert not p.b_track_stop.packed and not p.track_ind.packed
    assert said(p, 'over lead')


def test_a_status_already_in_flight_cannot_undo_a_press(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.start_tracking()
    p.stop_tracking()
    node.status = status('tracking', age=0.2)   # sent before the stop landed
    p._follow_track_status()
    assert not p.tracking and not p.b_track_stop.packed


def test_status_never_flips_tracking_mid_action(ip):
    """START runs in go(); the node reports 'tracking' before it returns."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.busy = True
    node.status = status('tracking', age=0.0)
    p._follow_track_status()
    assert not p.tracking and not said(p, 'adopted')


def test_a_silent_node_is_not_believed(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    node.status = status('tracking', age=5.0)
    p._follow_track_status()
    assert not p.tracking
    p.tracking = True
    assert p.banner_state()[0] == 'TRACKING?'


def test_each_status_change_is_traced_once(ip):
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    for st in (status('idle'), status('idle', pos_err_mm='9'),
               status('tracking'), status('holding', 'over lead', level=1),
               status('holding', 'over lead', level=1, lead_mm='7')):
        node.status = st
        p._follow_track_status()
    recs = [t for t in p.traces if t['rec'] == 'track_status']
    assert [(r['state'], r['reason']) for r in recs] == [
        ('idle', ''), ('tracking', ''), ('holding', 'over lead')]


def test_stop_tracking_is_offered_while_the_node_arms(ip):
    """START takes seconds (tool offset, gain profile, settle) and the arm
    may move at its end: a STOP pressed meanwhile must be reachable."""
    node = FakeNode(loaded='active')
    p = _panel(ip, node, active=True)
    p.busy = True                             # the START call is in flight
    node.status = status('starting', age=0.0)
    p._follow_track_status()
    assert p.b_track_stop.packed and p.b_track_stop.state == 'normal'
    # ...while the ladder stays quiet: busy is not tracking, but no less busy
    assert all(getattr(p, n).state == 'disabled' for n in QUIET)


def test_stop_tracking_lives_in_the_shared_header(ip):
    """The node drives the arm whichever tab is on top."""
    shell = inspect.getsource(ip.CellPanel.__init__)
    assert 'self.b_track_stop =' in shell and 'self.track_ind =' in shell
    assert 'self.b_track_stop =' not in inspect.getsource(
        ip.LadderPane._build_ladder)


def _built(ip, monkeypatch, node):
    """The real CellPanel, built on a mocked Tk: the wiring as shipped.
    Returns the panel, every tk.Button made (with its kwargs) and ttk."""
    from unittest import mock
    tk, buttons = mock.MagicMock(), []

    def button(*_a, **kw):
        b = mock.MagicMock()
        b.kw = kw
        buttons.append(b)
        return b
    tk.Button = button
    ttk = mock.MagicMock()
    for name, mod in (('tk', tk), ('ttk', ttk), ('tkfont', mock.MagicMock()),
                      ('scrolledtext', mock.MagicMock())):
        monkeypatch.setattr(ip, name, mod)
    node.camera, node.relay_error = (lambda on: on), None
    panel = ip.CellPanel(node)
    panel.tabs.index.return_value = 0         # the ALIGN tab is on top
    return panel, buttons, ttk


def _until(cond, timeout_s=2.0):
    end = time.monotonic() + timeout_s
    while not cond() and time.monotonic() < end:
        time.sleep(0.01)
    return cond()


def test_header_stop_tracking_is_never_refused_and_never_blocks(
        ip, monkeypatch):
    """Pressed on the ALIGN tab while an action holds the panel busy.
    Through go() the press is dropped (busy) or handed to ALIGN, whose
    torque interlock refuses STOP itself; on the Tk thread the Trigger
    freezes the window for up to TRACK_CALL_TIMEOUT_S."""
    node = FakeNode(loaded='active')
    panel, buttons, _ = _built(ip, monkeypatch, node)
    (stop,) = [b for b in buttons if b.kw.get('text') == 'STOP TRACKING']
    assert stop.kw['command'] == panel.header_stop_tracking
    panel.ladder.tracking = True
    panel.busy = True
    assert panel.active() is panel.align
    stop.kw['command']()
    assert _until(lambda: ('trigger', 'stop') in node.calls), \
        'STOP TRACKING was refused'
    assert threading.get_ident() not in node.trigger_threads, \
        'the Trigger ran on the Tk thread'


def _align_stop(ip, ladder):
    a = types.SimpleNamespace(n=types.SimpleNamespace(stop_now=lambda: True),
                              ladder=ladder, abort=False, stopped=False,
                              logs=[])
    a.say = a.logs.append
    a.set_status = lambda *_a, **_k: None
    a.trace = lambda _r: None
    ip.AlignPane.stop_now(a)
    return a


@pytest.mark.parametrize('tracking,state', [
    (True, 'tracking'), (True, 'holding'),
    (False, 'starting'),                      # the panel's START in flight
])
def test_align_stop_now_stops_the_tracker_too(ip, tracking, state):
    """STOP NOW is the biggest red button on the screen. ALIGN's own halt
    does not reach the tracking node, so under a live tracker it would
    have said 'halting motion' while the arm kept following the marker."""
    node = FakeNode(loaded='active')
    ladder = _panel(ip, node, active=True)
    ladder.tracking = tracking
    ladder.stop_tracking_now = types.MethodType(
        ip.LadderPane.stop_tracking_now, ladder)
    node.status = status(state, age=0.0)
    a = _align_stop(ip, ladder)
    assert _until(lambda: ('trigger', 'stop') in node.calls)
    assert threading.get_ident() not in node.trigger_threads
    assert not any('halting motion' in m for m in a.logs)
    assert any('tracking node' in m for m in a.logs)


def test_align_stop_now_leaves_an_idle_tracker_alone(ip):
    node = FakeNode(loaded='active')
    ladder = _panel(ip, node, active=True)
    node.status = status('idle', age=0.0)
    a = _align_stop(ip, ladder)
    time.sleep(0.05)
    assert not node.calls
    assert any('STOP NOW' in m for m in a.logs)


def _param_node(ip, monkeypatch, ready=True, ok=True, reason=''):
    """CellNode.set_tracking_params over two fake clients, one per service:
    each answers in its own service's response shape."""
    monkeypatch.setattr(ip, 'Parameter', types.SimpleNamespace)
    monkeypatch.setattr(ip, 'ParameterValue', types.SimpleNamespace)
    monkeypatch.setattr(ip, 'ParameterType', types.SimpleNamespace(
        PARAMETER_BOOL='bool', PARAMETER_DOUBLE='double',
        PARAMETER_STRING='string', PARAMETER_DOUBLE_ARRAY='double[]'))
    for srv in ('SetParametersAtomically', 'SetParameters'):
        monkeypatch.setattr(ip, srv, types.SimpleNamespace(
            Request=lambda: types.SimpleNamespace(parameters=[])))

    class Cli:
        def __init__(self, name, response):
            self.srv_name, self.response, self.sent = name, response, []

        def service_is_ready(self):
            return ready

        def call_async(self, req):
            self.sent.append(req)
            return types.SimpleNamespace(done=lambda: True,
                                         result=lambda: self.response)
    one = types.SimpleNamespace(successful=ok, reason=reason)
    n = types.SimpleNamespace(
        track_params_cli=Cli(ip.TRACK_PARAMS_SRV,
                             types.SimpleNamespace(result=one)),
        track_param_cli=Cli(ip.TRACK_PARAM_SRV,
                            types.SimpleNamespace(results=[one])),
        _wait=ip.CellNode._wait)
    return n


@pytest.mark.parametrize('atomic', [True, False])
@pytest.mark.parametrize('ok,reason', [(True, ''), (False, 'bad standoff')])
def test_tracking_params_are_answered_by_the_right_service(
        ip, monkeypatch, atomic, ok, reason):
    """START's goal is all-or-nothing (set_parameters_atomically, one
    result); the live policy is one plain set_parameters (a result per
    parameter). A refusal is the node's reason, never 'applied'."""
    n = _param_node(ip, monkeypatch, ok=ok, reason=reason)
    got = ip.CellNode.set_tracking_params(
        n, {'tracking_over_lead_policy': 'stop'}, atomic=atomic)
    assert got == ((True, 'applied') if ok else (False, reason))
    used, idle = ((n.track_params_cli, n.track_param_cli) if atomic
                  else (n.track_param_cli, n.track_params_cli))
    assert len(used.sent) == 1 and not idle.sent
    assert [q.name for q in used.sent[0].parameters] == \
        ['tracking_over_lead_policy']


@pytest.mark.parametrize('atomic', [True, False])
def test_tracking_params_to_an_absent_node_are_refused(ip, monkeypatch,
                                                       atomic):
    n = _param_node(ip, monkeypatch, ready=False)
    ok, msg = ip.CellNode.set_tracking_params(
        n, {'tracking_over_lead_policy': 'stop'}, atomic=atomic)
    assert ok is False and 'not running' in msg
    assert not n.track_params_cli.sent and not n.track_param_cli.sent


def test_the_panel_listens_to_the_nodes_latched_status(ip, monkeypatch):
    """The node latches its status (transient_local) so a panel started
    mid-run gets its last word. A VOLATILE subscription never matches that
    for the latched sample, and a wrong name hears nothing: either way the
    banner and adoption go silent."""
    node_cpp = (SRC / 'mating_controller' / 'src'
                / 'tracking_node.cpp').read_text()
    assert re.search(r'"~/status",\s*rclcpp::QoS\(1\)\s*\.reliable\(\)'
                     r'\s*\.transient_local\(\)', node_cpp)
    assert ip.TRACK_STATUS_TOPIC == f'/{ip.TRACKING_NODE}/status'
    subs = []

    class RosNode:                            # just what CellNode() calls
        def __init__(self, _name):
            pass

        def declare_parameter(self, name, value):
            pass

        def get_parameter(self, name):
            return types.SimpleNamespace(value=None)

        def create_subscription(self, *a):
            subs.append(a)

        def create_client(self, *_a):
            return None

        def create_publisher(self, *_a):
            return None

    class Probe(ip.CellNode, RosNode):
        pass
    monkeypatch.setattr(ip, 'TransformListener', lambda *_a: None)
    monkeypatch.setattr(ip, 'ActionClient', lambda *_a: None)
    monkeypatch.setattr(ip, 'start_throttle', lambda *_a, **_k: None)
    monkeypatch.setattr(ip, 'QoSProfile', lambda **k: k)
    monkeypatch.setattr(ip, 'ReliabilityPolicy', types.SimpleNamespace(
        RELIABLE='reliable', BEST_EFFORT='best_effort'))
    monkeypatch.setattr(ip, 'DurabilityPolicy', types.SimpleNamespace(
        TRANSIENT_LOCAL='transient_local', VOLATILE='volatile'))
    n = Probe()
    (sub,) = [s for s in subs if s[1] == ip.TRACK_STATUS_TOPIC]
    assert sub[0] is ip.DiagnosticStatus and sub[2] == n._track_status_cb
    assert sub[3] == {'depth': 1, 'reliability': 'reliable',
                      'durability': 'transient_local'}


def test_status_callback_reads_the_byte_level(ip):
    """diagnostic_msgs' level is a `byte`, which rclpy hands over as bytes."""
    fake = types.SimpleNamespace(_lock=threading.Lock(), _track_status=None)

    def kv(k, v):
        return types.SimpleNamespace(key=k, value=v)
    ip.CellNode._track_status_cb(fake, types.SimpleNamespace(
        level=b'\x01', name='tracking_node', message='holding: over lead',
        values=[kv('state', 'holding'), kv('reason', 'over lead')]))
    f, age = ip.CellNode.track_status(fake)
    assert f['level'] == 1 and f['state'] == 'holding'
    assert f['reason'] == 'over lead' and 0.0 <= age < 1.0


# ---- ALIGN's marker frame -----------------------------------------------------
# fr3_mating.launch.py filters in fr3_link0 and cam_pub publishes /aruco/pose
# in THAT frame, while every ALIGN error is a camera-frame error (the marker
# [0, 0, standoff] ahead). Read as camera-frame, a base pose is off by the
# whole reach of the arm.
CAM = 'camera_color_optical_frame'


@pytest.fixture
def tf_time(ip, monkeypatch):
    monkeypatch.setattr(ip.rclpy, 'time', types.SimpleNamespace(Time=int),
                        raising=False)


def _xyz(v):
    return types.SimpleNamespace(x=v[0], y=v[1], z=v[2])


def _xyzw(q):
    return types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])


def _marker_node(tf):
    """CellNode's marker path on a fake whose TF buffer knows camera <- any
    frame only when tf is given."""
    n = types.SimpleNamespace(_lock=threading.Lock(), _pose=None, cam=CAM,
                              marker_why=None, warned=[], lookups=[], tf=tf)
    n.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=10**9))
    n.get_logger = lambda: types.SimpleNamespace(warning=n.warned.append)

    def lookup(target, source, _stamp):
        n.lookups.append((target, source))
        if n.tf is None:
            raise RuntimeError(f'"{source}" passed to lookupTransform '
                               'argument source_frame does not exist')
        return types.SimpleNamespace(transform=types.SimpleNamespace(
            translation=_xyz(n.tf[0]), rotation=_xyzw(n.tf[1])))
    n.tf_buf = types.SimpleNamespace(lookup_transform=lookup)
    return n


def _pose(ip, n, frame, p, q):
    ip.CellNode._cb(n, types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame),
        pose=types.SimpleNamespace(position=_xyz(p), orientation=_xyzw(q))))
    return ip.CellNode.marker(n)


def test_align_reads_a_camera_frame_pose_as_is(ip, tf_time):
    n = _marker_node(None)
    p, R = _pose(ip, n, CAM, [0.01, 0.02, 0.30], [0.0, 0.0, 0.0, 1.0])
    assert p == pytest.approx([0.01, 0.02, 0.30]) and np.allclose(R, np.eye(3))
    assert not n.lookups


def test_align_re_expresses_a_base_frame_pose_in_the_camera_frame(ip,
                                                                   tf_time):
    R_base_cam = ip.q2R(1.0, 0.0, 0.0, 0.0)      # looking straight down
    c = np.array([0.4, 0.0, 0.5])                # from 0.5 m up
    R_cb = R_base_cam.T
    n = _marker_node((-R_cb @ c, ip.R2q(R_cb)))  # TF camera <- base
    R_base_marker = ip.axis_angle_R(np.array([0.0, 0.0, 1.0]),
                                    np.radians(30.0))
    p, R = _pose(ip, n, 'fr3_link0', [0.4, 0.02, 0.1],
                 ip.R2q(R_base_marker))
    assert n.lookups == [(CAM, 'fr3_link0')]
    assert p == pytest.approx([0.0, -0.02, 0.4], abs=1e-9)
    assert np.allclose(R, R_cb @ R_base_marker, atol=1e-9)


def test_align_without_tf_sees_no_marker_and_says_why_once(ip, tf_time):
    n = _marker_node(None)
    assert _pose(ip, n, 'fr3_link0', [0.4, 0.0, 0.1],
                 [0.0, 0.0, 0.0, 1.0]) is None
    assert all(ip.CellNode.marker(n) is None for _ in range(20))
    assert len(n.warned) == 1 and 'fr3_link0' in n.warned[0]
    assert 'fr3_link0' in n.marker_why


def test_align_forgets_why_once_the_tf_is_back_and_warns_again(ip,
                                                               tf_time):
    """The reason is shown on the ALIGN tab: a stale one over a visible
    marker is a lie, and a second outage deserves its own warning."""
    n = _marker_node(None)
    assert _pose(ip, n, 'fr3_link0', [0.4, 0.0, 0.1],
                 [0.0, 0.0, 0.0, 1.0]) is None
    n.tf = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    assert ip.CellNode.marker(n) is not None and n.marker_why is None
    n.tf = None
    assert ip.CellNode.marker(n) is None
    assert len(n.warned) == 2


# ---- trace location -----------------------------------------------------------

def _tracer(ip):
    p = types.SimpleNamespace(tracef=None, tracepath=None, logs=[],
                              _trace_lock=threading.Lock())
    p.say = p.logs.append
    for name in ('trace', 'open_trace', 'close_trace'):
        setattr(p, name, types.MethodType(getattr(ip.CellPanel, name), p))
    return p


def test_traces_go_to_fr3_log_dir_by_day(ip, tmp_path, monkeypatch):
    """Next to the tracking node's own logs, outside the repo."""
    monkeypatch.setenv('FR3_LOG_DIR', str(tmp_path / 'runs'))
    monkeypatch.setattr(ip, 'LOG_DIR', tmp_path / 'logs')   # never the repo
    p = _tracer(ip)
    p.open_trace()
    p.close_trace()
    day = tmp_path / 'runs' / datetime.date.today().isoformat()
    assert p.tracepath.parent == day and p.tracepath.exists()
    assert said(p, str(p.tracepath))


def test_traces_stay_in_tools_fr3_logs_without_it(ip, tmp_path, monkeypatch):
    monkeypatch.delenv('FR3_LOG_DIR', raising=False)
    monkeypatch.setattr(ip, 'LOG_DIR', tmp_path / 'logs')
    p = _tracer(ip)
    p.open_trace()
    p.close_trace()
    assert p.tracepath.parent == tmp_path / 'logs'


def test_log_dir_is_defined_once(ip):
    src = pathlib.Path(ip.__file__).read_text()
    assert len(re.findall(r'^LOG_DIR\s*=', src, re.M)) == 1
