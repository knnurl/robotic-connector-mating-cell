"""Numbers that live in more than one place, pinned so they cannot drift:
the panel's copies (choices.py, logic.py, config/settings.yaml) against
core.py, and the cell's shared numbers against the controller header, the
controller yaml, fr3_params.yaml and the tracking law. Plus the threading
rule, checked structurally: ROS-side code never imports Qt, and the window
never imports rclpy.

The cross-file pins were test_cell_panel.py's until the Tk panel retired
(2026-09-24); the C++ headers point here. Constants are SCRAPED from the
headers, so they must stay bare decimal literals there.
"""

import ast
import importlib.util
import os
import pathlib
import re
import types

import numpy as np
import pytest
import yaml

import actions
import choices as C
import core
import logic as L

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parents[2]
ST = L.load_settings(HERE / 'config' / 'settings.yaml')
IMPEDANCE_HPP = ('fr3_mating_controllers/include/fr3_mating_controllers'
                 '/impedance_detail.hpp')
TRACKING_HPP = 'mating_controller/include/mating_controller/tracking_law.hpp'
HANDEYE = SRC / 'tools' / 'fr3' / 'calib' / 'handeye.yaml'


def _consts(rel_path):
    """A const(name) -> float over one C++ header."""
    header = (SRC / rel_path).read_text()

    def const(name):
        return float(re.search(rf'{name}\s*=\s*([0-9.]+)', header).group(1))
    return const


def _shipped():
    """The tracking parameters as the node will actually receive them."""
    cfg = yaml.safe_load((SRC / 'tools' / 'fr3' / 'fr3_params.yaml').read_text())
    return cfg['/**']['ros__parameters']


def _controller_yaml():
    return yaml.safe_load((SRC / 'fr3_mating_controllers' / 'config'
                           / 'cartesian_impedance_stroke.yaml').read_text()
                          )['cartesian_impedance_stroke_controller']['ros__parameters']


# ---------------------------------------------------------------- copies

def test_choices_match_core():
    assert C.STEP_MM == core.STEP_MM_CHOICES and C.STEP_MM_DEFAULT == core.STEP_MM_DEFAULT
    assert C.ROT_DEG == core.ROT_DEG_CHOICES and C.ROT_DEG_DEFAULT == core.ROT_DEG_DEFAULT
    assert C.TARGET_MM == core.TARGET_MM_CHOICES
    assert C.TARGET_MM_DEFAULT == core.TARGET_MM_DEFAULT
    assert C.POS_TOL_MM == core.POS_TOL_MM_CHOICES
    assert C.POS_TOL_MM_DEFAULT == core.POS_TOL_MM_DEFAULT
    assert C.INPLANE == core.INPLANE_TARGET_CHOICES
    assert C.INPLANE_DEFAULT == core.INPLANE_TARGET_DEFAULT
    assert C.FLOOR_MM_DEFAULT == core.FLOOR_MM_DEFAULT
    assert C.SETPOINT_MM == core.SETPOINT_MM_CHOICES
    assert C.SETPOINT_MM_DEFAULT == core.SETPOINT_MM_DEFAULT
    assert C.AXES == core.AXIS_CHOICES
    assert C.OVER_LEAD == core.OVER_LEAD_CHOICES
    assert C.OVER_LEAD_DEFAULT == core.OVER_LEAD_DEFAULT
    assert C.POSE_SOURCES == list(core.POSE_SOURCES)
    assert C.POSE_SOURCE_DEFAULT in core.POSE_SOURCES
    assert C.GAIN_LIMITS == core.GAIN_LIMITS
    assert C.MAX_LEAD_MM == core.MAX_LEAD_MM
    assert C.CONTACT_N == core.CONTACT_WRENCH[0] and C.REFLEX_N == core.COLLISION_WRENCH[0]


def test_logic_constants_match_core():
    assert (L.ARM, L.IMP) == (core.ARM_CONTROLLER, core.IMPEDANCE_CONTROLLER)
    assert L.ROBOT_MODES == core.ROBOT_MODES and L.GATE_REASONS == core.GATE_REASONS
    assert (L.MODE_IDLE, L.MODE_MOVE, L.MODE_REFLEX, L.MODE_USER_STOPPED) == (
        core.MODE_IDLE, core.MODE_MOVE, core.MODE_REFLEX, core.MODE_USER_STOPPED)
    assert L.TRACK_LIVE_STATES == core.TRACK_LIVE_STATES


def test_settings_copied_from_core_match():
    assert ST.state_stale_s == core.ROBOT_STATE_STALE_S
    assert ST.driver_down_s == core.DRIVER_DOWN_S
    assert ST.pose_stale_s == core.POSE_STALE_S
    assert ST.raw_max_age_s == core.RAW_MAX_AGE_S
    assert ST.track_status_stale_s == core.TRACK_STATUS_STALE_S
    assert ST.rot_tol_deg == core.ROT_TOL_DEG and ST.inplane_tol_deg == core.INPLANE_TOL_DEG
    assert ST.position_ceiling_pct == core.CEIL_SPEED_PCT
    assert ST.push_limit_n == core.PUSH_LIMIT_N
    assert ST.controller_max_force_n == core.CONTROLLER_MAX_FORCE_N


def test_controller_launches_no_stiffer_than_commission():
    """The yaml gains hold the arm from launch until the panel writes a
    preset or the saved gains; they must not be stiffer than commission."""
    doc = yaml.safe_load((HERE / 'config' / 'gain_presets.yaml').read_text())
    com = next(p for p in doc['presets'] if p['name'] == 'commission')
    ctrl = _controller_yaml()
    assert all(c <= p for c, p in zip(ctrl['k_pos_tool'], [com['k_xy'], com['k_xy'], com['k_z']]))
    assert all(c <= p for c, p in zip(ctrl['k_rot_tool'], [com['k_rp'], com['k_rp'], com['k_yaw']]))


# ---------------------------------------------------------------- controller

def test_reflex_threshold_sits_above_the_controller_force_ceiling():
    """Below the ceiling, the controller's own capped push would trip the
    reflex and kill the driver. And the ceiling must be the yaml's."""
    assert _controller_yaml()['max_force_n'] == core.CONTROLLER_MAX_FORCE_N
    assert min(core.COLLISION_WRENCH[:3]) > core.CONTROLLER_MAX_FORCE_N
    assert all(lo <= hi for lo, hi in zip(core.CONTACT_WRENCH, core.COLLISION_WRENCH))
    assert all(lo <= hi for lo, hi in zip(core.CONTACT_TORQUE_NM, core.COLLISION_TORQUE_NM))
    assert core.PUSH_LIMIT_N < min(core.CONTACT_WRENCH[:3])


def test_gain_limits_are_the_controllers_limits():
    const = _consts(IMPEDANCE_HPP)
    assert core.GAIN_LIMITS['k_xy'] == (0.0, const('kPosMax'))
    assert core.GAIN_LIMITS['k_z'] == (0.0, const('kPosMax'))
    assert core.GAIN_LIMITS['k_rp'] == (0.0, const('kRotMax'))
    assert core.GAIN_LIMITS['k_yaw'] == (0.0, const('kRotMax'))
    assert core.GAIN_LIMITS['zeta'] == (const('zetaMin'), const('zetaMax'))


def test_speed_slider_ceilings_are_the_controllers_slew_limits():
    """Both speed sliders map onto ConfigLimits; above them the controller
    refuses the whole set."""
    const = _consts(IMPEDANCE_HPP)
    assert ST.slew_max_mps == const('slewMpsMax')
    assert ST.slew_max_rps == const('slewRpsMax')
    assert L.speed_torque(1, ST)['setpoint_slew_mps'] >= const('slewMpsMin')


def test_shipped_yaml_is_inside_the_controllers_limits():
    """A yaml edit outside these makes configure fail on the day."""
    const = _consts(IMPEDANCE_HPP)
    header = (SRC / IMPEDANCE_HPP).read_text()
    spec = [float(v) for v in re.search(r'kTauSpecNm\{([^}]*)\}', header)
            .group(1).split(',')]
    prm = _controller_yaml()
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


# ---------------------------------------------------------------- tracking

def test_track_profile_is_inside_the_controllers_gain_limits():
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
        'zeta above 1.0 is not covered by the measurement this profile rests on '
        '(zero overshoot in 32 clean steps, 2026-09-22); re-measure before moving it')


def test_track_slew_is_inside_the_controllers_slew_limits():
    const = _consts(IMPEDANCE_HPP)
    prm = _shipped()
    assert const('slewMpsMin') <= prm['track_setpoint_slew_mps'] <= const('slewMpsMax')
    assert const('slewRpsMin') <= prm['track_setpoint_slew_rps'] <= const('slewRpsMax')


def test_tracking_ceilings_match_the_controller_yaml():
    """The node validates its lead against ceilings it is TOLD; if those
    drift from the controller's own, it validates against fiction."""
    prm, ctrl = _shipped(), _controller_yaml()
    assert prm['tracking_max_force_n'] == ctrl['max_force_n']
    assert prm['tracking_max_force_n'] == core.CONTROLLER_MAX_FORCE_N
    assert prm['tracking_max_torque_nm'] == ctrl['max_torque_nm']


def test_tracking_lead_force_stays_under_every_ceiling():
    """The integrator's whole anti-windup is the clamp: worst case the lead
    adds k * lead_max, and that must stay under the controller's ceiling and
    well under the reflex that kills the driver."""
    const = _consts(TRACKING_HPP)
    prm = _shipped()
    lead_n = max(prm['track_k_pos_tool']) * prm['tracking_lead_max_m']
    assert lead_n <= const('kLeadForceMaxN')
    assert const('kLeadForceMaxN') < core.CONTROLLER_MAX_FORCE_N
    assert const('kLeadForceMaxN') < min(core.COLLISION_WRENCH[:3])
    lead_nm = max(prm['track_k_rot_tool']) * prm['tracking_lead_max_rad']
    assert lead_nm < prm['tracking_max_torque_nm']


def test_tracking_deadbands_match_the_track_stiffness():
    """The deadband is F_friction / k, so it belongs to the stiffness in use."""
    const = _consts(TRACKING_HPP)
    prm = _shipped()
    band_m = const('kFrictionBreakawayN') / max(prm['track_k_pos_tool'])
    assert band_m <= prm['tracking_deadband_m'] <= 3.0 * band_m
    band_rad = const('kFrictionBreakawayNm') / max(prm['track_k_rot_tool'])
    assert band_rad <= prm['tracking_deadband_rad'] <= 3.0 * band_rad


def test_tracking_lead_cap_and_floor_match_the_panel():
    """A 50 Hz stream must not be looser than the hand-stepped path: same
    equilibrium-lead cap, and ONE Z floor for the cell - ALIGN's default,
    the ladder's and the tracking node's - absolute in the base frame."""
    prm = _shipped()
    assert prm['tracking_max_lead_m'] * 1000 <= core.MAX_LEAD_MM
    assert prm['tracking_z_floor_m'] * 1000 == pytest.approx(core.FLOOR_Z_MM)
    assert float(core.FLOOR_MM_DEFAULT) == pytest.approx(core.FLOOR_Z_MM)
    assert core.FLOOR_Z_MM == pytest.approx(100.0)


def test_track_goal_defaults_match_the_node():
    prm = _shipped()
    assert prm['tracking_standoff_m'] * 1000 == pytest.approx(float(core.TARGET_MM_DEFAULT))
    assert prm['tracking_inplane_deg'] == pytest.approx(float(core.INPLANE_TARGET_DEFAULT))


def test_track_timeout_exceeds_the_nodes_own_worst_case_start():
    """The panel must never report 'not started' while the arm is tracking:
    four service round-trips, the settle and ~1 s of tool-offset sampling,
    with 5 s in hand."""
    prm = _shipped()
    worst = 4 * (1.0 + prm['tracking_profile_timeout_s']) + prm['tracking_settle_s'] + 1.0
    assert core.TRACK_CALL_TIMEOUT_S >= worst + 5.0


class _TrackNode:
    """Just enough node for Cell.start_tracking to reach the goal write."""

    def __init__(self):
        self.goal = None

    def state(self):
        return (np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3), core.MODE_MOVE, 1.0, 0.0)

    def controller_states(self):
        return {core.ARM_CONTROLLER: 'inactive', core.IMPEDANCE_CONTROLLER: 'active'}

    def applied_params(self):
        return {'float_mode': False}

    def marker(self):
        return np.array([0.0, 0.0, 0.10]), np.diag([1.0, -1.0, -1.0])

    def set_tracking_params(self, values, atomic=True):
        self.goal = dict(values)
        return False, 'stop here'


def test_goal_parameters_are_typed_as_the_yaml_declares_them():
    """The node declares its parameters from fr3_params.yaml, which fixes
    each one's type: a bool sent as a double is rejected, and with it the
    whole atomic set, so START refuses on the day."""
    node = _TrackNode()
    cell = types.SimpleNamespace(n=node, st=ST, params=actions.Params(), trace=lambda r: None,
                                 floating=False)
    for name in ('blocked', 'controller', 'floating_now'):
        setattr(cell, name, types.MethodType(getattr(actions.Cell, name), cell))
    ok, _msg = actions.Cell.start_tracking(cell)
    assert not ok and node.goal
    shipped = _shipped()
    for name, v in node.goal.items():
        assert type(v) is type(shipped[name]), (name, type(v), type(shipped[name]))


# ---------------------------------------------------------------- hand-eye

def _R_from_quat(q):
    """Rodrigues from the quaternion's axis and angle - deliberately not
    core.q2R, so the test does not grade its own homework."""
    q = np.asarray(q, dtype=float) / np.linalg.norm(q)
    ang = 2.0 * np.arctan2(np.linalg.norm(q[:3]), q[3])
    x, y, z = q[:3] / np.linalg.norm(q[:3])
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)


def test_align_rotation_is_the_handeye_quaternion_transposed():
    """quat_xyzw is TCP -> optical, i.e. R_tcp_cam, so ALIGN needs its
    transpose. The wrong way round is 180 deg out about the optical axis on
    this ~90 deg mount: every lateral step would go the wrong way."""
    assert core.CALIB_PATH == HANDEYE
    meta = yaml.safe_load(HANDEYE.read_text())
    R_base_tcp = _R_from_quat([0.3, -0.2, 0.1, 0.9])
    cell = types.SimpleNamespace(R=None, calib_meta={}, calib_state='waiting',
                                 say=lambda m: None,
                                 n=types.SimpleNamespace(tcp_pose=lambda: (np.zeros(3),
                                                                           R_base_tcp)))
    cell._read_calib_meta = types.MethodType(actions.Cell._read_calib_meta, cell)
    ok, msg = actions.Cell.load_calib(cell)
    assert ok and cell.calib_state == 'loaded', msg
    assert np.allclose(cell.R, _R_from_quat(meta['quat_xyzw']).T @ R_base_tcp.T, atol=1e-12)


def test_the_cell_launch_reads_the_same_handeye_file():
    """The TF must come from calib/handeye.yaml, with no second copy of the
    numbers in the launch."""
    src = (SRC / 'tools' / 'fr3' / 'fr3_cell.launch.py').read_text()
    assert re.search(r"""['"]calib['"]\s*[,/]\s*['"]handeye\.yaml['"]""", src)
    assert 'yaml.safe_load' in src
    meta = yaml.safe_load(HANDEYE.read_text())
    for v in meta['xyz'] + meta['quat_xyzw']:
        assert f'{v:.6f}'.rstrip('0') not in src, f'{v} is hard-coded'


def test_the_cell_launch_starts_this_panel():
    src = (SRC / 'tools' / 'fr3' / 'fr3_cell.launch.py').read_text()
    assert "'cell', 'cell.py'" in src and 'cell_panel.py' not in src


# ---------------------------------------------------------------- threading rule

def _imports(name, top_level=False):
    tree = ast.parse((HERE / name).read_text())
    mods = set()
    for node in (tree.body if top_level else ast.walk(tree)):
        if isinstance(node, ast.Import):
            mods |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module.split('.')[0])
    return mods


def test_ros_side_never_touches_qt():
    """ROS callbacks must never touch widgets: the modules they live in
    cannot even import Qt. The window hears from them only via post()."""
    for name in ('ros_node.py', 'actions.py', 'core.py', 'logic.py', 'mock_cell.py'):
        assert not _imports(name) & {'PySide6', 'PyQt5', 'tkinter'}, name


def test_window_and_decisions_never_touch_ros():
    for name in ('view.py', 'logic.py', 'palette.py', 'choices.py', 'visual.py',
                 'actions.py', 'core.py', 'persist.py'):
        # module level only: core.shutdown_ros imports rclpy when it runs
        assert not _imports(name, top_level=True) & {'rclpy', 'ros_node'}, name


def test_track_lead_cap_setting_is_the_nodes():
    assert ST.track_max_lead_mm == pytest.approx(_shipped()['tracking_max_lead_m'] * 1000)


def test_the_panel_reloads_the_impedance_controller_like_the_launch_file():
    """After a driver restart the panel re-runs fr3_cell.launch.py's spawner;
    both must load the same (source) parameter file."""
    spec = importlib.util.spec_from_file_location('fr3_cell_launch',
                                                  core.FR3 / 'fr3_cell.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    assert core.IMPEDANCE_PARAMS.is_file()
    assert os.path.normpath(str(core.IMPEDANCE_PARAMS)) == launch.IMPEDANCE_PARAMS


def test_the_nodes_workspace_box_defaults_are_the_panels():
    shipped = _shipped()
    assert shipped['tracking_box_x_m'] == pytest.approx(list(ST.box_x))
    assert shipped['tracking_box_y_m'] == pytest.approx(list(ST.box_y))
    assert shipped['tracking_box_z_max_m'] == pytest.approx(ST.box_z_max)


def test_the_joint_guard_uses_franka_descriptions_limits():
    import ros_node
    limits = ros_node.limits_from_franka_description()
    if limits is None:
        pytest.skip('franka_description not found')
    shipped = _shipped()
    assert shipped['tracking_joint_lower'] == pytest.approx([lo for lo, _ in limits])
    assert shipped['tracking_joint_upper'] == pytest.approx([hi for _, hi in limits])
    assert 0.05 < shipped['tracking_joint_margin_rad'] <= 0.25


def test_grip_node_defaults_are_the_shipped_params_and_the_panels():
    import grip_node
    shipped = _shipped()
    for k, v in grip_node.DEFAULTS.items():
        if k.startswith('grip_'):
            assert shipped[k] == pytest.approx(v) if not isinstance(v, str) else shipped[k] == v, k
    assert shipped['grip_cube_m'] * 1000 == pytest.approx(ST.grip_cube_mm)
    assert shipped['grip_force_n'] == pytest.approx(ST.grip_force_n)


def test_target_b_age_limit_is_one_number():
    assert _shipped()['grip_target_max_age_s'] == pytest.approx(ST.grip_target_max_age_s)
    assert core.GRIP_TARGET_MAX_AGE_S == pytest.approx(ST.grip_target_max_age_s)


def test_the_panel_offers_cubes_the_grip_node_accepts():
    import grip_logic
    import view
    src = pathlib.Path(view.__file__).read_text()
    assert 'self._spin(st.grip_cube_mm, 10, 72, 0, None)' in src
    margin = _shipped()['grip_open_margin_m']
    assert grip_logic.max_cube_m(margin) * 1000 == pytest.approx(72.0)
    assert grip_logic.cube_problem(0.072, margin) is None          # the drawer's top accepted
    assert grip_logic.cube_problem(0.073, margin) is not None      # and nothing past it
    assert grip_logic.cube_problem(0.010, margin) is None          # the drawer's bottom too
    assert grip_logic.max_cube_m(0.006) == 0.0                     # a margin that allows none


def test_the_pose_topics_are_the_object_contracts():
    """tracking_node, grip_node and the panel read the object pose contract
    the vision node publishes (roscam/object_contract.py); ALIGN's raw
    freshness is TRACK's."""
    from roscam import object_contract as oc
    prm = yaml.safe_load((core.FR3 / 'fr3_params.yaml').read_text())['/**']['ros__parameters']
    assert prm['pose_topic'] == core.POSE_TOPIC == oc.POSE_TOPIC
    assert prm['raw_pose_topic'] == prm['tracking_raw_pose_topic'] == core.RAW_POSE_TOPIC \
        == oc.RAW_POSE_TOPIC
    assert core.POSE_QUALITY_TOPIC == oc.QUALITY_TOPIC
    assert tuple(core.POSE_SOURCES) == tuple(oc.SOURCES)
    assert core.RAW_MAX_AGE_S == prm['tracking_raw_timeout_s']
