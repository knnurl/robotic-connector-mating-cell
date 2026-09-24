"""The panel decisions: enable logic, banner priority, next step, chips and the
slider/preset maps. Pure functions of a Snap - no ROS, robot or display."""

import dataclasses
import pathlib

import pytest
import yaml

import logic as L

HERE = pathlib.Path(__file__).resolve().parent
ST = L.load_settings(HERE / 'config' / 'settings.yaml')
LIMITS = {'k_xy': (0.0, 3000.0), 'k_z': (0.0, 3000.0), 'k_rp': (0.0, 300.0),
          'k_yaw': (0.0, 300.0), 'zeta': (0.1, 2.0)}
GOOD_MARKER = {'dist_mm': 101.0, 'lat_mm': 0.8, 'tilt_deg': 0.3, 'ip_deg': 90.1,
               'ip_err_deg': 0.1, 'err_mm': 1.2, 'err_xyz_mm': [0.5, 0.6, 1.0],
               'ok': True}


def position(**kw):
    """A healthy cell on the arm controller, marker in view, nothing wrong."""
    s = L.Snap(state_age=0.02, robot_mode=L.MODE_MOVE, rt_rate=0.999,
               tcp=(0.45, 0.0, 0.30), force=(0.5, 0.2, -1.0), dq_max=0.001,
               controllers={L.ARM: 'active', L.IMP: 'inactive'}, floating=False,
               params_ok=True, moveit_up=True, recover_ready=True,
               marker_age=0.05, marker=dict(GOOD_MARKER), image_age=0.2,
               calib='loaded', track_age=0.1, track={'state': 'idle'},
               track_node_up=True, inplane_target=90.0,
               poses={'home': True, 'pre_align': False})
    return dataclasses.replace(s, **kw)


def torque(**kw):
    """Impedance active and holding, PRE-FLIGHT done this session."""
    base = dict(controllers={L.ARM: 'inactive', L.IMP: 'active'},
                floating=False, preflight='done', setpoint_lead_mm=0.0)
    base.update(kw)
    return position(**base)


def tracking(**kw):
    base = dict(tracking=True, track={'state': 'tracking', 'lead_mm': '1.2',
                                      'pos_err_mm': '2.0', 'rot_err_deg': '0.3',
                                      'policy': 'hold'})
    base.update(kw)
    return torque(**base)


# ---------------------------------------------------------------- required

def test_no_robot_state_and_no_marker_blames_the_robot_state():
    """Spec case 1: the Tk panel's ALIGN said NO MARKER while there was no robot state
    at all; the root cause must win."""
    s = position(state_age=None, robot_mode=None, marker_age=None, marker=None,
                 controllers=None)
    b = L.banner(s, ST)
    assert b.key == 'driver_down' and b.level == L.FAULT
    s = dataclasses.replace(s, controllers={L.ARM: 'active'}, state_age=0.8)
    assert L.banner(s, ST).key == 'no_state'


def test_torque_mode_with_stale_marker():
    """Spec case 2: holding on impedance, the marker goes stale."""
    s = torque(marker_age=0.9, marker=None)
    b = L.banner(s, ST)
    assert b.key == 'vision' and b.title == 'VISION STALE' and b.level == L.WARN
    en = L.enable(s, ST)
    assert not en['track'].ok and 'marker' in en['track'].why
    assert en['hold_here'].ok and en['release'].ok        # still in control
    # while tracking: the same banner, and END TRACK stays pressable
    s = tracking(marker_age=0.9, marker=None)
    assert L.banner(s, ST).key == 'vision'
    assert L.enable(s, ST)['track'].ok


def test_track_entry_refused_above_the_threshold():
    """Spec case 3: align is optional, so TRACK may be asked for far away."""
    far = dict(GOOD_MARKER, err_mm=ST.track_entry_mm + 12.0, ok=False)
    en = L.enable(torque(marker=far), ST)
    assert not en['track'].ok and 'TRACK entry' in en['track'].why
    tilted = dict(GOOD_MARKER, tilt_deg=ST.track_entry_deg + 1.0)
    assert 'tilt' in L.enable(torque(marker=tilted), ST)['track'].why
    assert L.enable(torque(), ST)['track'].ok


def test_preset_change_blocked_under_load():
    """Spec case 4: presets switch only in free space."""
    en = L.enable(torque(force=(0.0, 0.0, ST.preset_max_force_n + 3.0)), ST)
    assert not en['preset'].ok and '|F| ext' in en['preset'].why
    assert not en['apply_gains'].ok
    en = L.enable(torque(setpoint_lead_mm=ST.preset_max_lead_mm + 1.0), ST)
    assert not en['preset'].ok and 'spring lead' in en['preset'].why
    assert L.enable(torque(), ST)['preset'].ok


# ---------------------------------------------------------------- banner order

@pytest.mark.parametrize('snap, key', [
    (position(moveit_up=False, rt_rate=0.5), 'moveit'),
    (position(rt_rate=0.90, controllers=None), 'rt'),
    (position(controllers=None, robot_mode=L.MODE_REFLEX), 'controller'),
    (position(robot_mode=L.MODE_REFLEX, failure=('translate', 'x')), 'reflex'),
    (position(failure=('translate', 'boom'), torque_attempted=True), 'failed'),
    (position(torque_attempted=True, user_paused=True), 'preflight'),
    (position(user_paused=True, marker_age=None, marker=None), 'gate'),
    (position(marker_age=None, marker=None), 'vision'),
    (position(), 'ready'),
])
def test_banner_priority(snap, key):
    assert L.banner(snap, ST).key == key


def test_moveit_down_is_no_fault_while_torque_holds():
    s = torque(moveit_up=False)
    assert L.banner(s, ST).key == 'ready'
    assert L.chips(s, ST)[1].level == L.WARN        # still visible in the bar


def test_impedance_without_preflight_is_flagged():
    s = torque(preflight='unknown')
    b = L.banner(s, ST)
    assert b.key == 'preflight' and 'RELEASE' in b.detail
    assert L.next_step(s, ST, L.enable(s, ST)) == 'release'


def test_reflex_offers_recover_only_with_a_server():
    s = position(robot_mode=L.MODE_REFLEX, errors=('cartesian_reflex',))
    b = L.banner(s, ST)
    assert b.key == 'reflex' and b.action == 'recover' and 'cartesian_reflex' in b.detail
    assert L.next_step(s, ST, L.enable(s, ST)) == 'recover'
    s = dataclasses.replace(s, recover_ready=False)
    assert L.banner(s, ST).action is None and 'relaunch' in L.banner(s, ST).detail


def test_preflight_idling_the_arm_is_not_a_mismatch():
    s = position(controllers={L.ARM: 'inactive', L.IMP: 'inactive'},
                 robot_mode=L.MODE_IDLE, busy='preflight')
    assert L.banner(s, ST).key == 'ready'
    s = dataclasses.replace(s, busy=None)
    assert L.banner(s, ST).key == 'controller'


def test_tracking_node_silent_while_tracking_is_a_mismatch():
    s = tracking(track_age=5.0)
    assert L.banner(s, ST).key == 'controller'


# ---------------------------------------------------------------- enable

def test_stop_controls_are_always_enabled():
    for s in (L.Snap(), position(busy='auto_converge'), tracking(busy='x')):
        en = L.enable(s, ST)
        assert all(en[c].ok for c in ('stop_now', 'pause', 'stop_after'))


def test_every_control_has_an_answer_and_disabled_ones_say_why():
    for s in (L.Snap(), position(), torque(), tracking(), position(busy='x')):
        en = L.enable(s, ST)
        for name in L.POSITION_CONTROLS + L.TORQUE_CONTROLS + L.ALWAYS:
            assert name in en
            assert en[name].ok or en[name].why, name


def test_align_needs_state_moveit_arm_gate_marker():
    assert L.enable(position(), ST)['translate'].ok
    for kw, word in ((dict(state_age=None), 'robot state'),
                     (dict(moveit_up=False), 'MoveIt'),
                     (dict(controllers={L.ARM: 'inactive', L.IMP: 'active'}), 'RELEASE'),
                     (dict(robot_mode=L.MODE_USER_STOPPED), 'gate'),
                     (dict(calib='waiting'), 'calibration'),
                     (dict(marker=None), 'marker')):
        en = L.enable(position(**kw), ST)['translate']
        assert not en.ok and word in en.why, (kw, en)


def test_auto_converge_also_needs_the_floor():
    en = L.enable(position(z_floor_set=False), ST)
    assert en['translate'].ok and not en['auto_converge'].ok


def test_preflight_needs_the_arm_stationary():
    assert L.enable(position(), ST)['preflight'].ok
    en = L.enable(position(dq_max=0.3), ST)['preflight']
    assert not en.ok and 'moving' in en.why
    assert not L.enable(torque(), ST)['preflight'].ok


def test_float_hold_need_preflight_and_rt():
    assert not L.enable(position(), ST)['hold'].ok          # preflight unknown
    s = position(preflight='done')
    assert L.enable(s, ST)['hold'].ok and L.enable(s, ST)['float'].ok
    en = L.enable(position(preflight='done', rt_rate=0.9), ST)['hold']
    assert not en.ok and 'RT' in en.why
    assert not L.enable(torque(), ST)['hold'].ok             # already holding
    assert L.enable(torque(floating=True), ST)['hold'].ok


def test_setpoints_need_impedance_holding_and_no_tracking():
    assert L.enable(torque(), ST)['setpoint_plus'].ok
    assert not L.enable(position(preflight='done'), ST)['setpoint_plus'].ok
    assert not L.enable(torque(floating=True), ST)['hold_here'].ok
    assert not L.enable(tracking(), ST)['setpoint_minus'].ok


def test_release_needs_impedance_or_an_unknown_controller():
    assert not L.enable(position(), ST)['release'].ok
    assert L.enable(torque(), ST)['release'].ok
    assert L.enable(torque(controllers=None), ST)['release'].ok


def test_busy_greys_everything_but_the_stops():
    en = L.enable(torque(busy='hold_on'), ST)
    assert not en['release'].ok and en['release'].why == 'waiting for hold_on'
    assert en['stop_now'].ok


def test_gains_locked_while_tracking():
    en = L.enable(tracking(), ST)
    assert not en['preset'].ok and 'tracking' in en['preset'].why
    assert not en['speed'].ok


def test_track_speed_stays_live_while_tracking():
    en = L.enable(tracking(), ST)
    assert en['track_speed'].ok and not en['speed'].ok
    assert 'TRACK SPEED' in en['speed'].why
    en = L.enable(tracking(params_ok=False), ST)
    assert not en['track_speed'].ok
    assert L.enable(position(), ST)['track_speed'].ok     # stored for the next START


def test_saved_pose_needs_teaching():
    en = L.enable(position(), ST)
    assert en['goto:home'].ok and not en['goto:pre_align'].ok


# ---------------------------------------------------------------- next step

def test_exactly_one_next_step_along_the_nominal_path():
    def nxt(s):
        return L.next_step(s, ST, L.enable(s, ST))
    assert nxt(position()) == 'preflight'
    assert nxt(position(preflight='done')) == 'hold'
    assert nxt(torque(floating=True)) == 'hold'
    assert nxt(torque()) == 'track'
    assert nxt(tracking()) is None
    assert nxt(position(busy='preflight')) is None


# ---------------------------------------------------------------- chips

def test_status_bar_is_the_same_seven_everywhere():
    for s in (L.Snap(), position(), torque(), tracking()):
        assert [c.label for c in L.chips(s, ST)] == [
            'FRANKA', 'MOVEIT', 'RT', 'CONTROLLER', 'PRE-FLIGHT', 'VISION', 'GATE']


def test_chips_quiet_when_healthy():
    """ISA-101: nothing is coloured while all is well."""
    assert all(c.level == L.NORMAL for c in L.chips(position(), ST))


# ---------------------------------------------------------------- sliders

def test_speed_maps():
    p = L.speed_position(100, ST)
    assert p['velocity_scale'] == pytest.approx(ST.position_ceiling_pct / 100)
    assert p['accel_scale'] == pytest.approx(p['velocity_scale'] ** 2)
    assert L.speed_position(25, ST)['slowdown'] == pytest.approx(20.0)  # the Tk panel's 5 % default
    t = L.speed_torque(20, ST)
    assert t['setpoint_slew_mps'] == pytest.approx(0.05)
    assert L.speed_torque(100, ST)['setpoint_slew_rps'] == pytest.approx(ST.slew_max_rps)
    assert L.speed_torque(0, ST)['setpoint_slew_mps'] >= 0.001
    assert not L.speed_needs_confirm(ST.speed_default_pct, ST)
    assert L.speed_needs_confirm(ST.speed_confirm_pct + 1, ST)


def test_shipped_presets_are_valid_and_contain_commission():
    doc = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())
    presets = doc['presets']
    assert L.validate_presets(presets, LIMITS) == []
    com = {'k_xy': 150.0, 'k_z': 800.0, 'k_rp': 10.0, 'k_yaw': 20.0, 'zeta': 1.0}
    i = L.preset_index(com, presets)
    assert i is not None and presets[i]['placeholder'] is False
    assert [p['name'] for p in presets if not p['placeholder']] == ['commission']


def test_non_monotonic_presets_are_refused():
    p = [dict(name='a', k_xy=100, k_z=500, k_rp=5, k_yaw=10, zeta=1.0),
         dict(name='b', k_xy=300, k_z=400, k_rp=10, k_yaw=20, zeta=1.0),
         dict(name='c', k_xy=600, k_z=900, k_rp=20, k_yaw=30, zeta=1.0)]
    assert any('k_z' in x for x in L.validate_presets(p, LIMITS))
    p[1]['k_z'] = 700
    assert L.validate_presets(p, LIMITS) == []
    p[2]['zeta'] = 1.5
    assert any('zeta' in x for x in L.validate_presets(p, LIMITS))


def test_custom_values_have_no_preset():
    doc = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())
    v = {'k_xy': 151.0, 'k_z': 800.0, 'k_rp': 10.0, 'k_yaw': 20.0, 'zeta': 1.0}
    assert L.preset_index(v, doc['presets']) is None


# ---------------------------------------------------------------- vision etc.

def test_marker_loss_policy():
    assert L.marker_loss_action('hold', True, None, 500) is None
    assert L.marker_loss_action('stop', False, None, 500) is None
    assert L.marker_loss_action('stop', True, 0.3, 500) is None
    assert L.marker_loss_action('stop', True, 0.6, 500) == 'stop'
    assert L.marker_loss_action('release', True, None, 500) == 'release'


def test_vision_events():
    assert L.vision_event(position(), ST) is None
    assert 'lost' in L.vision_event(position(marker_age=2.0), ST)
    assert 'stale' in L.vision_event(position(image_age=9.0), ST)
    assert 'jumped' in L.vision_event(position(jump_mm=ST.pose_jump_mm + 5), ST)


def test_joint_proximity_names_the_worst_joint():
    limits = [(-2.0, 2.0), (-1.0, 1.0)]
    j, pct = L.joint_proximity([0.5, 0.95], limits)
    assert j == 2 and pct == pytest.approx(95.0)
    assert L.joint_proximity(None, limits) is None


def test_workspace_box():
    assert L.in_box((0.45, 0.0, 0.3), ST, 0.1) is None
    assert 'x' in L.in_box((0.95, 0.0, 0.3), ST)
    assert 'floor' in L.in_box((0.45, 0.0, 0.05), ST, 0.1)


def test_settings_reject_unknown_keys(tmp_path):
    f = tmp_path / 's.yaml'
    f.write_text('no_such_threshold: 1\n')
    with pytest.raises(ValueError):
        L.load_settings(f)
