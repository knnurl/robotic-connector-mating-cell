"""The FR3 cell panel's core: constants, geometry and the exit handoff.

No ROS, Qt or Tk imports (rclpy only inside shutdown_ros), so the actions,
the decisions and their tests load anywhere. Carried over from the Tk panel
(tools/fr3/cell_panel.py, retired 2026-09-24) unchanged: these are the
values and the handoff that ran on the arm.
"""

import datetime
import os
import pathlib
import sys
import time

import numpy as np

FR3 = pathlib.Path(__file__).resolve().parents[1]          # tools/fr3
sys.path.insert(0, str(FR3.parents[1] / 'roscam'))
from roscam.plane_normal import (inplane_angle, inplane_correction,  # noqa: E402,F401
                                 wrap_deg)


# ---- selectable step sizes ----------------------------------------------
STEP_MM_CHOICES = ['0.5', '1', '2', '5', '10', '20', '30', '50']
ROT_DEG_CHOICES = ['0.25', '0.5', '1', '2', '3', '5']
STEP_MM_DEFAULT, ROT_DEG_DEFAULT = '30', '3'

# ---- speed -----------------------------------------------------------------
# The ceiling of the position-mode speed slider, as a percentage of the
# trajectory speed MoveIt planned. Applied by stretching time_from_start.
CEIL_SPEED_PCT = 20.0     # never faster than this, whatever is selected

# ---- target standoff + convergence --------------------------------------
TARGET_MM_CHOICES = ['80', '100', '150', '200', '250', '300']
TARGET_MM_DEFAULT = '100'
POS_TOL_MM_CHOICES = ['1', '2', '3', '5']
POS_TOL_MM_DEFAULT = '2'
ROT_TOL_DEG = 1.0         # 'parallel' tolerance for auto-converge

# In-plane rotation about the optical axis - the 6th DOF. Nulling position
# and tilt leaves it wherever the arm happened to end up (measured 6.33 deg
# off vertical after a converged run). Optical convention: 0 = marker X
# (the red axis) points RIGHT, +90 straight DOWN, -90 straight UP.
# It is also the best-conditioned signal the marker gives (0.01 deg std) and
# is actuated almost purely by J7, since the optical axis sits 0.37 deg off
# TCP Z. For real mating this target becomes a task parameter: set it to the
# connector's keyway orientation rather than an arbitrary vertical.
INPLANE_TARGET_CHOICES = ['off', '0', '90', '-90', '180']
INPLANE_TARGET_DEFAULT = '90'
INPLANE_TOL_DEG = 0.5
MAX_ITERS_ABS = 400       # absolute backstop; the real cap is step-scaled
NO_PROGRESS_LIMIT = 4     # abort if the error stops improving

# ---- hard safety ceilings (a selection can never exceed these) ----------
CEIL_STEP_M = 0.050
CEIL_ROT_DEG = 5.0

# ---- robot-state gate ---------------------------------------------------------
# Motion only while the robot reports MOVE. Anything else (user stop,
# hand-guiding, IDLE) pauses the run, which resumes from rest once the robot is
# back in MOVE; a REFLEX ends it. Fail-closed: with the gate on, no fresh robot
# state means no motion.
# This gate cannot see the enabling device: measured 2026-09-15, holding and
# releasing this cell's enabling device changed no robot-state field at all,
# so software cannot see it over FCI.
ROBOT_STATE_TOPIC = '/franka_robot_state_broadcaster/robot_state'
# One relay for the whole panel; the two originals each started their own.
ROBOT_STATE_RELAY = '/cell_panel/robot_state'
# Decoding the 1 kHz state in Python costs 86% of a core, raw callbacks 31%
# (measured) - enough to starve this GUI. A C++ topic_tools throttle child
# relays it at 50 Hz for 5%, and 20 ms is ample: the robot stops itself.
ROBOT_STATE_RELAY_HZ = 50
ROBOT_STATE_STALE_S = 0.3
GATE_DEFAULT = True
GATE_RESUME_HOLD_S = 0.5     # held this long, unbroken, before motion resumes
MODE_IDLE, MODE_MOVE, MODE_REFLEX, MODE_USER_STOPPED = 1, 2, 4, 5
ROBOT_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX',
               5: 'USER_STOPPED', 6: 'ERROR_RECOVERY'}
GATE_REASONS = {0: 'robot mode OTHER', 1: 'robot IDLE - no control loop',
                3: 'hand-guiding', 5: 'robot USER_STOPPED (user stop)',
                6: 'automatic error recovery'}

# Whole-arm Z floor. fr3_link0 is the bolted base at z=0 and would trip any
# sensible floor, so it is excluded; everything else that can swing down is
# monitored, not just the TCP - an elbow can dip while the tool is high.
FLOOR_LINKS = ['fr3_link1', 'fr3_link2', 'fr3_link3', 'fr3_link4',
               'fr3_link5', 'fr3_link6', 'fr3_link7', 'fr3_link8',
               'fr3_hand', 'fr3_hand_tcp', 'fr3_leftfinger',
               'fr3_rightfinger']

MIN_FRACTION = 0.95       # reject incomplete Cartesian paths
POSE_STALE_S = 0.5        # marker measurement must be fresher than this
FLOOR_MARGIN_M = 0.020    # extra clearance under the predicted end pose
# The cell's ONE Z floor, absolute in the base frame (fr3_link0): ALIGN's
# whole-arm check (its default below, editable), the ladder's setpoints and
# the tracking node's equilibrium (fr3_params.yaml tracking_z_floor_m,
# pinned equal by a test).
FLOOR_Z_MM = 100.0
FLOOR_MM_DEFAULT = f'{FLOOR_Z_MM:g}'

# ---- dashboard -----------------------------------------------------------
IMAGE_MAX_W = 640         # native D405 width, so no downscale at 640x480

# The cell's one hand-eye calibration, which fr3_cell.launch.py also
# publishes as TF. ALIGN uses only its ROTATION, a rigid mounting property:
# unlike R_cam_base it does not change when the robot moves, so it stays
# valid across sessions and arm poses.
CALIB_PATH = FR3 / 'calib' / 'handeye.yaml'

# Per-iteration JSONL trace of auto-converge: what was measured, what was
# commanded, what the robot actually did. One file per run. With FR3_LOG_DIR
# set (fr3_env.sh) it goes to $FR3_LOG_DIR/YYYY-MM-DD/ beside the tracking
# node's logs instead - see trace_dir().
LOG_DIR = FR3 / 'logs'


IMPEDANCE_CONTROLLER = 'cartesian_impedance_stroke_controller'
# The SOURCE copy fr3_cell.launch.py spawns it with (pinned by a test).
IMPEDANCE_PARAMS = (FR3.parents[1] / 'fr3_mating_controllers' / 'config'
                    / 'cartesian_impedance_stroke.yaml')
ARM_CONTROLLER = 'fr3_arm_controller'

EQUILIBRIUM_TOPIC = f'/{IMPEDANCE_CONTROLLER}/equilibrium_pose'
# Continuous tracking (TRACKING_SPEC.md section 5). The node owns the 50 Hz
# equilibrium stream; the panel only starts and stops it, so that stream is
# still behind an operator button.
TRACKING_NODE = 'tracking_node'
TRACK_START_SRV = f'/{TRACKING_NODE}/start_tracking'
TRACK_STOP_SRV = f'/{TRACKING_NODE}/stop_tracking'
# The node's start makes FOUR service round-trips (ListControllers, read
# float_mode, read the gains it must restore, apply the profile), each bounded
# by wait_for_service(1 s) + tracking_profile_timeout_s (2 s), on top of ~1 s
# of tool-offset sampling and tracking_settle_s (1 s): ~14 s worst case, and
# 5 s in hand. At the old 6 s the panel reported failure while the arm was in
# fact tracking; a reply later still is caught by the status topic, which
# adopts a live tracker.
TRACK_CALL_TIMEOUT_S = 20.0
# The node's parameters (fr3_params.yaml). START writes ALIGN's goal with
# set_parameters_atomically - all of it or none - and the over-lead dropdown
# writes its one value live with plain set_parameters.
TRACK_PARAMS_SRV = f'/{TRACKING_NODE}/set_parameters_atomically'
TRACK_PARAM_SRV = f'/{TRACKING_NODE}/set_parameters'
OVER_LEAD_CHOICES = ['hold', 'stop', 'clamp']
OVER_LEAD_DEFAULT = 'hold'
# The node's own word on tracking: latched (transient_local), ~5 Hz and on
# every state change. Older than this, the node is silent - maybe gone.
TRACK_STATUS_TOPIC = f'/{TRACKING_NODE}/status'
# grip_node (cell/grip_node.py): GRIP and PLACE block until the sequence ends
# (open, approach, descend at 15 mm/s, grasp, lift: well under a minute);
# STOP answers at once. Status latched like the tracking node's.
GRIP_NODE = 'grip_node'
GRIP_SRV = f'/{GRIP_NODE}/grip'
PLACE_SRV = f'/{GRIP_NODE}/place'
PLACE_AT_SRV = f'/{GRIP_NODE}/place_at_target'
GRIP_TARGET_MAX_AGE_S = 600.0      # = fr3_params grip_target_max_age_s (pinned by a test)
GRIP_STOP_SRV = f'/{GRIP_NODE}/stop'
GRIP_PARAMS_SRV = f'/{GRIP_NODE}/set_parameters_atomically'
GRIP_STATUS_TOPIC = f'/{GRIP_NODE}/status'
GRIP_CALL_TIMEOUT_S = 180.0
# The vision node (cam_pub or vision_standalone, both ArucoPosePublisher).
# vision_standalone records its own frames while record_dir is set; the
# panel's REC button sets it beside the bag (PERCEPTION_PLAN Phase 0).
VISION_PARAMS_SRV = '/aruco_pose_publisher/set_parameters'
# The object pose contract (roscam/object_contract.py, PERCEPTION_PLAN 2):
# what the panel, TRACK and GRIP read. The vision node's object_source picks
# where it comes from; the panel changes it only while the cell is idle.
POSE_TOPIC = '/object/pose'
RAW_POSE_TOPIC = '/object/pose_raw'
POSE_QUALITY_TOPIC = '/object/pose_quality'
POSE_SOURCES = ('marker',)
# Predictions never drive committed motion (TRACKING_SPEC Decision 5): ALIGN,
# like TRACK (tracking_raw_timeout_s), moves only on a raw detection at most
# this old.
RAW_MAX_AGE_S = 0.25
TRACK_STATUS_STALE_S = 1.0
# Every state but idle: the node drives the arm, is about to, or is still
# putting back its gain snapshot.
TRACK_LIVE_STATES = ('starting', 'tracking', 'holding', 'stopping')
STATE_STALE_S = 0.3
DRIVER_DOWN_S = 2.0       # robot state silent this long: the driver is gone

# ---- pre-flight -------------------------------------------------------------
# Collision reflex thresholds. Contact thresholds only raise flags in the robot
# state; collision thresholds stop the robot - and on this cell a reflex also
# kills ros2_control_node. The upper Cartesian values sit about 10 N above the
# controller's force ceiling (max_force_n 30 N; max_torque_nm 10 Nm), while a
# shove above 40 N still reflexes. The reflex watches libfranka's ESTIMATED
# external wrench, so the margin shrinks by the estimate's bias - PRE-FLIGHT
# reports |F ext| at rest and warns above REST_FORCE_WARN_N. libfranka's own
# cartesian_impedance_control example sets 100 everywhere; these stay well
# below that.
CONTACT_TORQUE_NM = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
COLLISION_TORQUE_NM = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
CONTACT_WRENCH = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]      # N x3, Nm x3
COLLISION_WRENCH = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
CONTROLLER_MAX_FORCE_N = 30.0   # max_force_n in the controller yaml
PUSH_LIMIT_N = 10.0             # what the ladder asks the operator to stay under
PREFLIGHT_MODE_TIMEOUT_S = 5.0
REST_FORCE_WARN_N = 5.0         # |F ext| bias at rest worth fixing before FLOAT
EXIT_WAIT_S = 20.0              # at exit, how long to let a running action finish
SPIN_JOIN_S = 2.0               # at exit, how long to wait for the spin to stop

# ---- setpoints ---------------------------------------------------------------
# The equilibrium is a spring anchor. In free space the controller slews it at
# 5 cm/s and the arm keeps up, so MAX_LEAD_MM mostly caps how far one run of
# commands can carry the arm; if the arm is blocked it also caps the spring
# force on the soft axes (150 N/m x 60 mm = 9 N). Along stiff tool Z the
# controller's max_force_n (30 N) is the real force bound.
SETPOINT_MM_CHOICES = ['5', '10', '20', '50']
SETPOINT_MM_DEFAULT = '10'
AXIS_CHOICES = ['base Z (up)', 'base X', 'base Y', 'tool Z (stroke)']
MAX_LEAD_MM = 60.0

# ---- gains -------------------------------------------------------------------
# Live-tunable within the SAME limits the controller enforces (GainLimits in
# fr3_mating_controllers/include/fr3_mating_controllers/impedance_detail.hpp;
# test_cell_pins checks they agree). The controller rejects anything
# outside them; the panel refuses first so the operator sees why. zeta 0 is an
# undamped spring and a negative value injects energy - the force ceiling
# bounds how hard the arm pushes, not whether it oscillates.
TUNE_FIELDS = [('k lateral', 'k_xy', '150'), ('k tool Z', 'k_z', '800'),
               ('k roll/pitch', 'k_rp', '10'), ('k yaw', 'k_yaw', '20'),
               ('damping zeta', 'zeta', '1.0')]
GAIN_LIMITS = {'k_xy': (0.0, 3000.0), 'k_z': (0.0, 3000.0),
               'k_rp': (0.0, 300.0), 'k_yaw': (0.0, 300.0),
               'zeta': (0.1, 2.0)}

# ---- camera --------------------------------------------------------------
# OFF by default, and the subscription is CREATED on demand rather than
# filtered in the callback. /aruco/debug_image is subscribe-gated at the
# publisher, so not subscribing means the frames are never encoded, never
# serialised and never put on DDS. That matters here: this panel runs while a
# 1 kHz torque loop is live, and three stack deaths on 2026-09-16/22 were
# missed FCI deadlines. The feed is a convenience; the deadline is not.
IMAGE_TOPIC = '/aruco/debug_image'


def trace_dir():
    """Where a new trace goes, created on demand by open_trace."""
    root = os.environ.get('FR3_LOG_DIR')
    return (pathlib.Path(root) / datetime.date.today().isoformat() if root
            else LOG_DIR)


def q2R(x, y, z, w):
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = np.array([x, y, z, w]) / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
            y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
            y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def axis_angle_R(axis, ang):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def run_resumable(attempt, edge_count, resume):
    """attempt() -> (ok, msg), retried after a pause.

    If attempt fails AND it was interrupted while it ran (edge_count
    changed: the robot left MOVE, or PAUSE was pressed), resume() blocks
    until motion may continue and returns True to retry or False to give up.
    resume=None disables retrying. A failure with no edge is a real failure
    and is returned as is - never retried.
    """
    while True:
        if resume is not None and not resume():
            return False, 'interrupted, run ended'
        edges = edge_count()
        ok, msg = attempt()
        if ok or resume is None or edge_count() == edges:
            return ok, msg


def release_if_active(node, say=print):
    """Hand the arm back if the CONTROLLER MANAGER says impedance is active.

    Used on window close and at process exit, so the panel's own memory never
    decides whether the arm is left compliant. Returns True when nothing is
    left active, False when a release was needed and failed, None when the
    controller manager cannot be asked.
    """
    states = node.controllers()
    if states is None:
        return None
    if states.get(IMPEDANCE_CONTROLLER) != 'active':
        return True
    # Stop the 50 Hz stream BEFORE handing the arm back. The tracking node
    # does not care which controller is active, so an orphaned tracker's next
    # effect is autonomous motion the moment anyone activates the impedance
    # controller again. Only on this path: with the controller already
    # inactive or the driver gone there is nothing to stop, and the node
    # self-halts on the controller leaving ACTIVE anyway.
    node.call_trigger(node.track_stop_cli)
    ok, msg = node.switch([ARM_CONTROLLER], [IMPEDANCE_CONTROLLER])
    say(f'released {IMPEDANCE_CONTROLLER} on exit' if ok else
        f'*** could NOT release {IMPEDANCE_CONTROLLER} on exit ({msg}) - '
        'use RELEASE again or the robot E-stop ***')
    return ok


def exit_handoff(node, panel, say=print, wait_s=None):
    """The last word before the process exits, whatever ended the panel.

    Lets a running action finish (bounded), so its own restore can run; hands
    the arm back if the impedance controller is still active; and restores
    fr3_arm_controller if PRE-FLIGHT released it and it is still down. Ctrl+C
    and SIGTERM reach Panel.on_close first; an exception out of mainloop comes
    straight here. kill -9 cannot be caught at all.
    """
    wait_s = EXIT_WAIT_S if wait_s is None else wait_s
    if panel is not None:
        end = time.monotonic() + wait_s
        while panel.busy and time.monotonic() < end:
            time.sleep(0.05)
        if panel.busy:
            say('*** exiting while a panel action is still running - check '
                'the controllers: ros2 control list_controllers ***')
    result = release_if_active(node, say)
    if result is None:
        say('*** could not ask the controller manager on exit - if the '
            'impedance controller is active the arm is still compliant: '
            'RELEASE from a new panel, or use the E-stop ***')
    if panel is not None and getattr(panel, 'arm_released', False):
        states = node.controllers()
        if (states is not None and states.get(ARM_CONTROLLER) != 'active'
                and states.get(IMPEDANCE_CONTROLLER) != 'active'):
            ok, msg = node.switch([ARM_CONTROLLER], [])
            say(f'restored {ARM_CONTROLLER}, which PRE-FLIGHT had released'
                if ok else f'*** {ARM_CONTROLLER} is NOT active ({msg}) - '
                'activate it before moving the robot ***')
    return result


def shutdown_ros(node, panel, executor, spin, say=print):
    """Teardown, in an order that is the whole point.

    The spin MUST stop before the rclpy context goes away. A thread still
    inside spin() when rclpy.shutdown() runs takes a DDS thread down with the
    context - "terminate called without an active exception", i.e. the process
    ABORTS, in the very path whose job is handing the arm back. Observed
    2026-09-16 on a normal window close: the abort landed after the handoff by
    luck of the race, not by construction. So: hand back, stop the spin, join
    it, and only then drop the node and the context.
    """
    import rclpy
    try:
        # Window closed (on_close has already released), or an exception out
        # of mainloop: ask the controller manager, not the panel, what is live.
        exit_handoff(node, panel, say)
    finally:
        if panel is not None:
            panel.close_trace()
        node.close()
        executor.shutdown()
        spin.join(timeout=SPIN_JOIN_S)
        node.destroy_node()
        rclpy.shutdown()
