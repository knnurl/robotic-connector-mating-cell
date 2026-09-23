#!/usr/bin/env python3
"""FR3 cell control panel - alignment, impedance commissioning, tracking.

One window, three tabs, one robot. Merged 2026-09-22 from align_gui.py and
impedance_panel.py, which had drifted into two dashboards with thirteen
duplicated helpers between them and no shared idea of who held the arm.

  ALIGN       drive the wrist camera to a target pose above the marker
              (stepped MoveIt cartesian moves, proven on hardware)
  IMPEDANCE   the commissioning ladder: pre-flight, float, hold, setpoint
  TRACK       continuous marker following on the impedance controller

Shared across the tabs: one robot-state relay, one camera subscription
(created on demand - see IMAGE_TOPIC), one trace file per session, one
status strip, and one guarded close that always hands the arm back.

Every motion still needs a button press. Nothing here moves on its own, and
none of it replaces the hardware E-stop or the enabling device.

    python3 tools/fr3/cell_panel.py
"""

import collections
import datetime
import json
import os
import pathlib
import signal
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import numpy as np
import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticStatus
from franka_msgs.msg import FrankaRobotState
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetLoad
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from state_relay import start_throttle, stop_throttle   # noqa: E402
from theme import THEME as T, mix, rounded_rect, text_on   # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'roscam'))
from roscam.plane_normal import (inplane_angle, inplane_correction,  # noqa: E402
                                 wrap_deg)


# ---- selectable step sizes ----------------------------------------------
STEP_MM_CHOICES = ['0.5', '1', '2', '5', '10', '20', '30', '50']
ROT_DEG_CHOICES = ['0.25', '0.5', '1', '2', '3', '5']
STEP_MM_DEFAULT, ROT_DEG_DEFAULT = '30', '3'

# ---- selectable speed ----------------------------------------------------
# Percentage of the trajectory speed MoveIt planned. Applied by stretching
# time_from_start (this GetCartesianPath has no velocity-scaling field, so
# setting one in the request would silently do nothing).
SPEED_PCT_CHOICES = ['1', '2', '5', '10', '20']
SPEED_PCT_DEFAULT = '5'
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
PLOT_WINDOW_S = 30.0      # seconds of convergence history shown

# The cell's one hand-eye calibration, which fr3_cell.launch.py also
# publishes as TF. ALIGN uses only its ROTATION, a rigid mounting property:
# unlike R_cam_base it does not change when the robot moves, so it stays
# valid across sessions and arm poses.
CALIB_PATH = pathlib.Path(__file__).resolve().parent / 'calib' / 'handeye.yaml'

# Per-iteration JSONL trace of auto-converge: what was measured, what was
# commanded, what the robot actually did. One file per run. With FR3_LOG_DIR
# set (fr3_env.sh) it goes to $FR3_LOG_DIR/YYYY-MM-DD/ beside the tracking
# node's logs instead - see trace_dir().
LOG_DIR = pathlib.Path(__file__).with_name('logs')


IMPEDANCE_CONTROLLER = 'cartesian_impedance_stroke_controller'
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
# test_cell_panel checks they agree). The controller rejects anything
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


def _param_msg(name, v):
    """One rcl_interfaces Parameter, typed from the Python value. The type
    must be the one the receiving node declared, or the set is refused."""
    p = Parameter()
    p.name = name
    pv = ParameterValue()
    if isinstance(v, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = v
    elif isinstance(v, str):
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = v
    elif isinstance(v, (list, tuple)):
        pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        pv.double_array_value = [float(x) for x in v]
    else:
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(v)
    p.value = pv
    return p


class CellNode(Node):
    """One node for the whole panel.

    The two originals each started their own state relay and their own
    subscriptions to the same 1 kHz topic; merged, there is one relay, one
    camera subscription (created on demand) and one place that knows which
    controller holds the arm.
    """

    def __init__(self):
        super().__init__('cell_panel')
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('tcp_link', 'fr3_hand_tcp')
        self.declare_parameter('group', 'fr3_arm')
        # The frame every ALIGN error is measured in (see marker()).
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.base = self.get_parameter('base_frame').value
        self.tcp = self.get_parameter('tcp_link').value
        self.group = self.get_parameter('group').value
        self.cam = self.get_parameter('camera_frame').value

        self._lock = threading.Lock()

        # --- vision + joints (ALIGN) ---
        self._pose = None          # (pos(3), R, stamp_s, frame_id) as received
        self.marker_why = None     # why marker() cannot use it, or None
        self._joints = None        # (names, positions)
        self.create_subscription(PoseStamped, '/aruco/pose', self._cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_cb, 10)

        # --- camera overlay, created only while the pane asks for it ---
        self._image = None         # (seq, HxWx3 RGB uint8)
        self._image_seq = 0
        self._image_sub = None

        # --- MoveIt cartesian moves (ALIGN) ---
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.cart = self.create_client(GetCartesianPath,
                                       '/compute_cartesian_path')
        self.exec_ac = ActionClient(self, ExecuteTrajectory,
                                    '/execute_trajectory')
        # MoveIt's TrajectoryExecutionManager subscribes here and calls
        # stopExecution() on "stop" - this halts a trajectory MID-motion,
        # unlike cancelling the action goal which can let it run out.
        self.exec_event = self.create_publisher(
            String, '/trajectory_execution_event', 10)
        self._goal_handle = None
        self.last_cmd = {}

        # --- impedance + tracking ---
        self._state = None       # (pos, quat, force, mode, rate, stamp)
        self.on_sample = None
        self.eq_pub = self.create_publisher(PoseStamped, EQUILIBRIUM_TOPIC, 1)
        self.switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.list_cli = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        # ATOMIC: plain set_parameters applies each parameter on its own, so a
        # rejected value could leave the rest of a gain set live while the
        # panel reported it refused.
        self.param_cli = self.create_client(
            SetParametersAtomically,
            f'/{IMPEDANCE_CONTROLLER}/set_parameters_atomically')
        self.load_cli = self.create_client(SetLoad, '/service_server/set_load')
        self.collision_cli = self.create_client(
            SetForceTorqueCollisionBehavior,
            '/service_server/set_force_torque_collision_behavior')
        self.track_start_cli = self.create_client(Trigger, TRACK_START_SRV)
        self.track_stop_cli = self.create_client(Trigger, TRACK_STOP_SRV)
        self.track_params_cli = self.create_client(SetParametersAtomically,
                                                   TRACK_PARAMS_SRV)
        self.track_param_cli = self.create_client(SetParameters,
                                                  TRACK_PARAM_SRV)
        # Latched: a panel started mid-run still gets the node's last word.
        self._track_status = None  # (fields, monotonic stamp)
        self.create_subscription(
            DiagnosticStatus, TRACK_STATUS_TOPIC, self._track_status_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # --- one robot-state relay, two readers ---
        self._mode = None          # (robot_mode, monotonic stamp)
        self.gate_edges = 0
        self.on_mode_change = None
        self.relay_error = None
        try:
            self._relay = start_throttle(ROBOT_STATE_TOPIC, ROBOT_STATE_RELAY,
                                         hz=ROBOT_STATE_RELAY_HZ,
                                         node_name='cell_panel_relay')
        except Exception as e:                              # noqa: BLE001
            self._relay, self.relay_error = None, str(e)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(FrankaRobotState, ROBOT_STATE_RELAY,
                                 self._mode_cb, qos)
        self.create_subscription(FrankaRobotState, ROBOT_STATE_RELAY,
                                 self._state_cb, qos)

    def close(self):
        stop_throttle(self._relay)

    def _mode_cb(self, m):
        mode = int(m.robot_mode)
        with self._lock:
            prev = None if self._mode is None else self._mode[0]
            self._mode = (mode, time.monotonic())
            if prev == MODE_MOVE and mode != MODE_MOVE:
                self.gate_edges += 1
        hook = self.on_mode_change
        if mode != prev and hook is not None:
            hook(prev, mode)

    def robot_mode(self):
        """(robot_mode, age_s) of the latest state, or None before any."""
        with self._lock:
            r = self._mode
        return None if r is None else (r[0], time.monotonic() - r[1])

    def link_z(self, links):
        """Base-frame Z of each link, mm. Missing links are skipped."""
        out = {}
        for ln in links:
            try:
                t = self.tf_buf.lookup_transform(self.base, ln,
                                                 rclpy.time.Time())
                out[ln] = t.transform.translation.z
            except Exception:                               # noqa: BLE001
                pass
        return out

    def lowest_link(self, links):
        """(name, z) of the lowest monitored link, or None."""
        z = self.link_z(links)
        if not z:
            return None
        k = min(z, key=z.get)
        return k, z[k]

    def _cb(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        o = m.pose.orientation
        with self._lock:
            self._pose = (p, q2R(o.x, o.y, o.z, o.w),
                          self.get_clock().now().nanoseconds * 1e-9,
                          m.header.frame_id)

    def _joint_cb(self, m):
        with self._lock:
            self._joints = (list(m.name), list(m.position))

    def joints(self):
        """Arm joint positions as {name: rad}, or {} if not yet seen."""
        with self._lock:
            j = self._joints
        return {} if j is None else dict(zip(j[0], j[1]))

    def marker(self):
        """Fresh marker measurement in the camera optical frame, or None if
        missing, stale or not transformable.

        cam_pub publishes /aruco/pose in its filter_frame - fr3_link0 on this
        cell (fr3_cell.launch.py) - while every ALIGN error is a camera
        frame error. So anything else is re-expressed through the latest TF:
        the filter frame is fixed, the camera is what moved.
        """
        with self._lock:
            if self._pose is None:
                return None
            p, R, t, frame = self._pose
        if self.get_clock().now().nanoseconds * 1e-9 - t > POSE_STALE_S:
            return None
        if frame == self.cam:
            return p, R
        try:
            tf = self.tf_buf.lookup_transform(self.cam, frame,
                                              rclpy.time.Time())
        except Exception as e:                              # noqa: BLE001
            if self.marker_why is None:     # once per outage, not per tick
                self.get_logger().warning(
                    f'marker in {frame!r}, no TF to {self.cam}: {e}')
            self.marker_why = f'marker in {frame}: no TF to {self.cam}'
            return None
        self.marker_why = None
        tr, ro = tf.transform.translation, tf.transform.rotation
        R_cf = q2R(ro.x, ro.y, ro.z, ro.w)
        return R_cf @ p + np.array([tr.x, tr.y, tr.z]), R_cf @ R

    def tcp_pose(self):
        tf = self.tf_buf.lookup_transform(self.base, self.tcp,
                                          rclpy.time.Time())
        tr, ro = tf.transform.translation, tf.transform.rotation
        return (np.array([tr.x, tr.y, tr.z]),
                q2R(ro.x, ro.y, ro.z, ro.w))

    def move(self, d_base, R_delta=None, z_floor=None, slowdown=20.0,
             resume=None):
        """Cartesian move of the TCP. Returns (ok, message).

        z_floor is an absolute base-frame minimum for the TCP, checked
        against the goal BEFORE planning, so a bad vision scale or a sign
        error cannot drive the arm down past it.

        resume: if the move is interrupted (robot left MOVE, or PAUSE) it is
        called to wait; on True the move re-plans from where the arm
        stopped to the SAME absolute goal, so a paused step finishes the
        move that was asked for rather than starting a new one.
        """
        pos, Rt = self.tcp_pose()
        target = Pose()
        tp = pos + np.asarray(d_base)
        self.last_cmd = {
            'tcp_pos_before': pos.tolist(),
            'tcp_quat_before': R2q(Rt).tolist(),
            'joints_before': self.joints(),
            'goal_pos': tp.tolist(),
        }
        if z_floor is not None and tp[2] < z_floor:
            return False, (f'BLOCKED by Z floor: goal z={tp[2]*1000:.1f} mm '
                           f'< floor {z_floor*1000:.1f} mm')
        target.position.x, target.position.y, target.position.z = tp
        Rgoal = (R_delta @ Rt) if R_delta is not None else Rt
        q = R2q(Rgoal)
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = q
        self.last_cmd['goal_quat'] = q.tolist()
        # how much reorientation was actually requested, in degrees
        self.last_cmd['goal_rot_deg'] = float(np.degrees(np.arccos(
            np.clip((np.trace(Rgoal @ Rt.T) - 1.0) / 2.0, -1, 1))))

        ok, msg = run_resumable(lambda: self._plan_execute(target, slowdown),
                                lambda: self.gate_edges, resume)
        try:
            apos, aRt = self.tcp_pose()
            self.last_cmd['tcp_pos_after'] = apos.tolist()
            self.last_cmd['tcp_quat_after'] = R2q(aRt).tolist()
            self.last_cmd['joints_after'] = self.joints()
            # achieved vs requested reorientation - the key diagnostic for
            # "the level step ran but tilt did not change"
            self.last_cmd['achieved_rot_deg'] = float(np.degrees(np.arccos(
                np.clip((np.trace(aRt @ Rt.T) - 1.0) / 2.0, -1, 1))))
            self.last_cmd['achieved_trans_mm'] = float(
                np.linalg.norm(apos - pos) * 1000)
        except Exception:                                   # noqa: BLE001
            pass
        return ok, msg

    def _plan_execute(self, target, slowdown):
        """Plan from the CURRENT state to target, retime, execute."""
        if not self.cart.wait_for_service(timeout_sec=3.0):
            return False, 'compute_cartesian_path unavailable'
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base
        req.group_name = self.group
        req.link_name = self.tcp
        req.waypoints = [target]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        fut = self.cart.call_async(req)
        if not self._wait(fut, 10.0):
            return False, 'Cartesian planning timed out'
        res = fut.result()
        if res is None:
            return False, 'Cartesian planning call failed'
        if res.fraction < MIN_FRACTION:
            return False, f'path only {res.fraction * 100:.0f}% complete'

        traj = res.solution
        for pt in traj.joint_trajectory.points:
            tot = (pt.time_from_start.sec
                   + pt.time_from_start.nanosec * 1e-9) * slowdown
            pt.time_from_start.sec = int(tot)
            pt.time_from_start.nanosec = int((tot - int(tot)) * 1e9)
            pt.velocities = [v / slowdown for v in pt.velocities]
            pt.accelerations = [a / slowdown ** 2 for a in pt.accelerations]

        if not self.exec_ac.wait_for_server(timeout_sec=3.0):
            return False, 'execute_trajectory unavailable'
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        gfut = self.exec_ac.send_goal_async(goal)
        if not self._wait(gfut, 10.0):
            return False, 'goal send timed out'
        gh = gfut.result()
        if gh is None or not gh.accepted:
            return False, 'trajectory goal rejected'
        self._goal_handle = gh
        rfut = gh.get_result_async()
        ok = self._wait(rfut, 300.0)
        self._goal_handle = None
        if not ok or rfut.result() is None:
            return False, 'no result (timeout)'
        code = rfut.result().result.error_code.val
        return (code == 1), ('executed' if code == 1
                             else f'execution error code {code}')

    def stop(self):
        """Graceful: cancel the goal (may still finish the current segment)."""
        gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
            return True
        return False

    def halt(self):
        """Stop whatever is moving, now, without ending the run: MoveIt
        execution halts mid-trajectory. Never blocks, so it is safe from the
        spin thread.
        """
        msg = String()
        msg.data = 'stop'
        for _ in range(3):          # cheap, and the topic is best-effort
            self.exec_event.publish(msg)

    def interrupt(self):
        """Operator PAUSE: count it like the robot leaving MOVE, THEN halt -
        in that order, so the halted step reads as interrupted and resumes
        rather than as a failure that ends the run."""
        with self._lock:
            self.gate_edges += 1
        self.halt()

    def stop_now(self):
        """Instant: halt MoveIt execution mid-trajectory, then cancel.

        Publishing "stop" makes TrajectoryExecutionManager call
        stopExecution(), which stops the controller where it is instead of
        letting the queued trajectory play out. NOT a substitute for the
        hardware E-stop.
        """
        self.halt()
        gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        return True

    @staticmethod
    def _wait(fut, timeout_s):
        """Wait on a future completed by the background spinner."""
        end = time.time() + timeout_s
        while time.time() < end:
            if fut.done():
                return True
            time.sleep(0.02)
        return False

    def camera(self, on):
        """Create or destroy the debug-image subscription.

        Destroying it is what takes the frames off the wire: the publisher
        only encodes while something is subscribed.
        """
        if on and self._image_sub is None:
            self._image_sub = self.create_subscription(
                Image, IMAGE_TOPIC, self._image_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        elif not on and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            with self._lock:
                self._image = None
        return self._image_sub is not None

    def _image_cb(self, m):
        enc = m.encoding.lower()
        if enc not in ('bgr8', 'rgb8'):
            return
        arr = np.frombuffer(m.data, dtype=np.uint8)
        arr = arr.reshape(m.height, m.step)[:, :m.width * 3]
        arr = arr.reshape(m.height, m.width, 3)
        if enc == 'bgr8':
            arr = arr[:, :, ::-1]
        stride = max(1, int(np.ceil(m.width / IMAGE_MAX_W)))
        with self._lock:
            self._image_seq += 1
            self._image = (self._image_seq,
                           np.ascontiguousarray(arr[::stride, ::stride]))

    def image(self):
        """(sequence, RGB array) of the latest overlay, or None."""
        with self._lock:
            return self._image

    def _state_cb(self, m):
        p = m.o_t_ee.pose.position
        o = m.o_t_ee.pose.orientation
        f = m.o_f_ext_hat_k.wrench.force
        mode = int(m.robot_mode)
        rate = float(m.control_command_success_rate)
        with self._lock:
            self._state = (np.array([p.x, p.y, p.z]),
                           np.array([o.x, o.y, o.z, o.w]),
                           np.array([f.x, f.y, f.z]), mode, rate,
                           time.monotonic())
        hook = self.on_sample
        if hook is not None:
            hook({'pos': [p.x, p.y, p.z], 'quat': [o.x, o.y, o.z, o.w],
                  'force': [f.x, f.y, f.z],
                  'q': list(m.measured_joint_state.position),
                  'mode': mode, 'success_rate': rate})

    def state(self):
        """(pos, quat, force, mode, rate, age_s) or None before any message."""
        with self._lock:
            s = self._state
        return None if s is None else (*s[:5], time.monotonic() - s[5])

    def cm_reachable(self):
        return self.list_cli.service_is_ready()

    def publish_equilibrium(self, pos, quat):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fr3_link0'
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) = \
            [float(v) for v in pos]
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = [float(v) for v in quat]
        self.eq_pub.publish(m)

    def switch(self, activate, deactivate, timeout_s=10.0):
        """One STRICT switch. Between the impedance and arm controllers both
        sides are effort, so franka_hardware never stops the robot; with an
        empty activate list it releases a controller and the robot idles."""
        if not self.switch_cli.wait_for_service(timeout_sec=3.0):
            return False, 'controller_manager not available'
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        fut = self.switch_cli.call_async(req)
        if not self._wait(fut, timeout_s):
            return False, 'switch_controller timed out'
        res = fut.result()
        ok = bool(res is not None and res.ok)
        return ok, ('switched' if ok else 'controller_manager refused')

    def controllers(self, timeout_s=3.0):
        """{name: state}, or None if the controller manager did not answer -
        which must never be read as 'nothing is active'."""
        if not self.list_cli.service_is_ready():
            return None
        fut = self.list_cli.call_async(ListControllers.Request())
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return None
        return {c.name: c.state for c in fut.result().controller}

    def set_params(self, values, timeout_s=5.0):
        """values: {name: bool | float | [float, float, float]}, applied
        all-or-nothing by the controller."""
        if not self.param_cli.wait_for_service(timeout_sec=3.0):
            return False, f'{IMPEDANCE_CONTROLLER} parameters unavailable'
        req = SetParametersAtomically.Request()
        for name, v in values.items():
            req.parameters.append(_param_msg(name, v))
        fut = self.param_cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, 'set_parameters_atomically timed out'
        res = fut.result().result
        return bool(res.successful), ('applied' if res.successful
                                      else (res.reason or 'refused'))

    def set_tracking_params(self, values, atomic=True, timeout_s=5.0):
        """Write tracking_node parameters: atomic for START's goal, plain for
        the live policy. Answered, not waited on, like call_trigger."""
        cli = self.track_params_cli if atomic else self.track_param_cli
        if not cli.service_is_ready():
            return False, 'the tracking node is not running'
        req = (SetParametersAtomically if atomic else SetParameters).Request()
        for name, v in values.items():
            req.parameters.append(_param_msg(name, v))
        fut = cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, f'no answer from {cli.srv_name} in {timeout_s:.0f} s'
        res = fut.result()
        bad = [r for r in ([res.result] if atomic else res.results)
               if not r.successful]
        return not bad, ('applied' if not bad
                         else (bad[0].reason or 'refused'))

    def _track_status_cb(self, m):
        """Spin thread: keep it for the Tk tick, which is the only reader."""
        lvl = m.level          # a msg `byte`: rclpy delivers bytes of length 1
        lvl = lvl[0] if isinstance(lvl, (bytes, bytearray)) else int(lvl)
        f = {'level': lvl, 'message': m.message}
        f.update((kv.key, kv.value) for kv in m.values)
        with self._lock:
            self._track_status = (f, time.monotonic())

    def track_status(self):
        """(fields, age_s) of the tracking node's latest status, or None."""
        with self._lock:
            s = self._track_status
        return None if s is None else (s[0], time.monotonic() - s[1])

    def call_trigger(self, cli, timeout_s=TRACK_CALL_TIMEOUT_S):
        """One Trigger call, answered rather than waited on: the tracking
        node is optional, so a missing one is an answer, not a stall."""
        if not cli.service_is_ready():
            return False, 'the tracking node is not running'
        fut = cli.call_async(Trigger.Request())
        if not self._wait(fut, timeout_s) or fut.result() is None:
            # A timeout says nothing about what the node did - it may be
            # tracking. Never report this as "not started".
            return False, (f'no answer from {cli.srv_name} in {timeout_s:.0f} s '
                           '- the node may be tracking; press STOP TRACKING')
        res = fut.result()
        return bool(res.success), (res.message or '')

    def _call_franka(self, cli, req, timeout_s=5.0):
        """franka's parameter services reply success=False (never crash) when
        the robot rejects a command, e.g. because a controller is active."""
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, f'{cli.srv_name} not available'
        fut = cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, f'{cli.srv_name} timed out'
        res = fut.result()
        return bool(res.success), (res.error or 'ok')

    def set_load(self, mass, com_m, inertia_diag):
        req = SetLoad.Request()
        req.mass = float(mass)
        req.center_of_mass = [float(v) for v in com_m]    # flange frame, m
        inertia = [0.0] * 9                               # column-major
        inertia[0], inertia[4], inertia[8] = [float(v) for v in inertia_diag]
        req.load_inertia = inertia
        return self._call_franka(self.load_cli, req)

    def set_collision_behavior(self):
        req = SetForceTorqueCollisionBehavior.Request()
        req.lower_torque_thresholds_nominal = list(CONTACT_TORQUE_NM)
        req.upper_torque_thresholds_nominal = list(COLLISION_TORQUE_NM)
        req.lower_force_thresholds_nominal = list(CONTACT_WRENCH)
        req.upper_force_thresholds_nominal = list(COLLISION_WRENCH)
        return self._call_franka(self.collision_cli, req)


class Pane:
    """One task tab.

    Unknown attributes resolve on the shell, so every action method carried
    over from the two original panels keeps working unchanged - self.say,
    self.root, self.trace, self._card all still mean what they meant. The
    names in SHARED are deliberately forwarded on WRITE too: a pane setting
    self.busy must set the shell's flag, not shadow it with a pane-local one
    that on_close would never see.
    """

    SHARED = {'busy', 'abort', 'stopped', 'tracef', 'tracepath', 'photo',
              '_image_shown', 'last_outcome'}

    def __init__(self, shell, node, parent):
        object.__setattr__(self, 'shell', shell)
        self.n = node
        self.parent = parent

    def __getattr__(self, name):
        # Only reached when the pane itself has no such attribute.
        return getattr(self.shell, name)

    def __setattr__(self, name, value):
        if name in Pane.SHARED and 'shell' in self.__dict__:
            setattr(self.shell, name, value)
        else:
            object.__setattr__(self, name, value)

    # Every pane answers these three; the shell dispatches to the active one.
    def buttons(self):
        return []

    def pill_states(self):
        return []

    def banner_state(self, snap=None):
        return None


class AlignPane(Pane):
    """Camera <-> marker alignment, from align_gui.py; only the chrome moved
    to the shell."""

    NAME = 'ALIGN'

    def __init__(self, shell, node, parent):
        super().__init__(shell, node, parent)
        self.R = None                 # R_cam_base once calibration is loaded
        self.calib_meta = {}
        self.calib_state = 'waiting'
        self.hist = collections.deque(maxlen=600)
        self.gate_on = GATE_DEFAULT
        self.paused = False
        self.pause_reason = ''
        self.gate_fault = None
        self.user_paused = False
        self._mode_log = collections.deque(maxlen=64)

    def build(self, parent):
        self._build_readout(parent)
        self._build_settings(parent)
        self._build_calib(parent)
        self._build_actions(parent)
        self._read_calib_meta()

    def refresh(self):
        """Per-frame update of this tab. The shell reschedules, draws the
        pills and the banner, and owns the camera; snap is handed back so
        banner_state can describe the same instant this frame read."""
        snap = None
        m = self.n.marker()
        if m is None:
            for t_ in self.tiles.values():
                t_['val'].config(text='--', fg=T['critical'])
                t_['dot'].itemconfig(t_['id'], fill=T['critical'])
            self.sub.config(text=self.n.marker_why
                            or 'marker NOT VISIBLE / STALE')
        else:
            p, Rm = m
            mz = Rm[:, 2]
            tilt = float(np.degrees(np.arccos(
                np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
            err = p - np.array([0, 0, self.target_m()])
            en = float(np.linalg.norm(err))
            tol = self.pos_tol_m()
            lat = float(np.hypot(p[0], p[1]))
            tgt_ip = self.inplane_target()
            ip = inplane_angle(Rm)
            ip_err = (0.0 if (tgt_ip is None or ip is None)
                      else abs(wrap_deg(ip - tgt_ip)))
            ok = en <= tol and tilt <= ROT_TOL_DEG and ip_err <= INPLANE_TOL_DEG
            snap = {'err': en * 1000, 'tilt': tilt, 'ip_err': ip_err, 'ok': ok}
            self.hist.append((time.monotonic(), en * 1000, tilt, ip_err))
            for k, v, good, fmt in (
                    ('dist', p[2] * 1000, abs(err[2]) <= tol, '{:.1f}'),
                    ('lateral', lat * 1000, lat <= tol, '{:.2f}'),
                    ('tilt', tilt, tilt <= ROT_TOL_DEG, '{:.2f}'),
                    ('in-plane', ip, ip_err <= INPLANE_TOL_DEG, '{:+.1f}')):
                t_ = self.tiles[k]
                if v is None:
                    t_['val'].config(text='--', fg=T['ink2'])
                    t_['dot'].itemconfig(t_['id'], fill=T['muted'])
                    continue
                t_['val'].config(text=fmt.format(v), fg=T['ink'])
                if k == 'in-plane' and tgt_ip is None:
                    col = T['muted']
                else:
                    col = T['good'] if good else T['warning']
                t_['dot'].itemconfig(t_['id'], fill=col)
            fl = self.z_floor()
            self.sub.config(text=(
                f'err  x {err[0]*1000:+7.2f}   y {err[1]*1000:+7.2f}   '
                f'z {err[2]*1000:+7.2f} mm     |e| {en*1000:.2f} mm\n'
                f'arm Z floor '
                f'{"NOT SET" if fl is None else f"{fl*1000:.0f} mm"}   '
                f'target {self.target_m()*1000:g} mm   tol {tol*1000:g} mm / '
                f'{ROT_TOL_DEG:g} deg / {INPLANE_TOL_DEG:g} deg'))
        self.draw_plot('err', 0, self.pos_tol_m() * 1000)
        self.draw_plot('tilt', 1, ROT_TOL_DEG)
        self.draw_plot('ip', 2, INPLANE_TOL_DEG)
        self._update_calib_card()
        self._drain_mode_log()
        self._snap = snap

    def build_plots(self, parent):
        self._build_plots(parent)

    def buttons(self):
        return [self.tb, self.lb, self.ipb, self.ab]

    def _build_plots(self, parent):
        card = self._card(parent,
                          f'CONVERGENCE  |  last {PLOT_WINDOW_S:g} s, '
                          'dashed line = tolerance')
        card.pack(fill='x', pady=(12, 0))
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 10))
        self.plots = {}
        for key, title in (('err', 'POSITION |e|  mm'), ('tilt', 'TILT  deg'),
                           ('ip', 'IN-PLANE ERROR  deg')):
            col = tk.Frame(row, bg=T['surface'])
            col.pack(side='left', expand=True, fill='x',
                     padx=(0, 8) if key != 'ip' else 0)
            tk.Label(col, text=title, font=self.f_caption, fg=T['muted'],
                     bg=T['surface'], anchor='w').pack(fill='x')
            c = tk.Canvas(col, width=200, height=104, bg=T['surface'],
                          highlightthickness=0)
            c.pack(fill='x')
            self.plots[key] = c

    def _build_readout(self, parent):
        card = self._card(parent, 'MARKER IN CAMERA FRAME')
        card.pack(fill='x')
        body = tk.Frame(card, bg=T['surface'])
        body.pack(fill='x', padx=10, pady=(0, 4))
        self.tiles = {}
        for i, (key, unit) in enumerate((('dist', 'mm'), ('lateral', 'mm'),
                                         ('tilt', 'deg'),
                                         ('in-plane', 'deg'))):
            col = tk.Frame(body, bg=T['surface'])
            col.grid(row=0, column=i, sticky='w', padx=(0, 18))
            head = tk.Frame(col, bg=T['surface'])
            head.pack(anchor='w')
            dot = tk.Canvas(head, width=10, height=10, bg=T['surface'],
                            highlightthickness=0)
            dot.pack(side='left')
            did = dot.create_oval(2, 2, 9, 9, fill=T['muted'], outline='')
            tk.Label(head, text=key.upper(), font=self.f_caption,
                     fg=T['muted'], bg=T['surface']).pack(side='left',
                                                          padx=(5, 0))
            vrow = tk.Frame(col, bg=T['surface'])
            vrow.pack(anchor='w')
            val = tk.Label(vrow, text='--', font=self.f_value, fg=T['ink'],
                           bg=T['surface'])
            val.pack(side='left')
            tk.Label(vrow, text=unit, font=self.f_caption, fg=T['muted'],
                     bg=T['surface']).pack(side='left', padx=(3, 0),
                                           pady=(0, 3))
            self.tiles[key] = {'val': val, 'dot': dot, 'id': did}
        self.sub = tk.Label(card, text='', font=self.f_small, fg=T['muted'],
                            bg=T['surface'], anchor='w', justify='left')
        self.sub.pack(fill='x', padx=10, pady=(2, 10))

    def _build_settings(self, parent):
        row = tk.Frame(parent, bg=T['page'])
        row.pack(fill='x', pady=(12, 0))

        tgt = self._card(row, 'TARGET + SAFETY')
        tgt.pack(side='left', fill='both', expand=True)
        g = tk.Frame(tgt, bg=T['surface'])
        g.pack(fill='x', pady=(0, 6))
        self.v_target = tk.StringVar(value=TARGET_MM_DEFAULT)
        self.v_tol = tk.StringVar(value=POS_TOL_MM_DEFAULT)
        self.v_inplane = tk.StringVar(value=INPLANE_TARGET_DEFAULT)
        self.v_floor = tk.StringVar(value=FLOOR_MM_DEFAULT)
        self._row(g, 'standoff (mm)', self.v_target, TARGET_MM_CHOICES, 0, 0)
        self._row(g, 'pos tol (mm)', self.v_tol, POS_TOL_MM_CHOICES, 1, 0)
        self._row(g, 'in-plane (deg)', self.v_inplane,
                  INPLANE_TARGET_CHOICES, 2, 0)
        tk.Label(g, text='arm Z floor (mm)', font=self.f_caption,
                 fg=T['ink2'], bg=T['surface'], anchor='e').grid(
            row=3, column=0, padx=(10, 4), pady=3, sticky='e')
        tk.Entry(g, textvariable=self.v_floor, width=8, font=self.f_small,
                 bg=T['page'], fg=T['ink'], insertbackground=T['ink'],
                 relief='flat', highlightthickness=1,
                 highlightbackground=T['grid']).grid(row=3, column=1,
                                                     sticky='w', pady=3)
        self.v_gate = tk.BooleanVar(value=GATE_DEFAULT)
        self.gatecb = tk.Checkbutton(
            g, text='robot-state gate', variable=self.v_gate,
            command=self._toggle_gate, font=self.f_caption, fg=T['ink'],
            bg=T['surface'], activebackground=T['surface'],
            activeforeground=T['ink'], selectcolor=T['page'],
            disabledforeground=T['muted'], highlightthickness=0, bd=0)
        self.gatecb.grid(row=4, column=0, columnspan=2, padx=10, pady=(4, 0),
                         sticky='w')
        self._button(tgt, 'Auto floor from here', T['grid'],
                     self.auto_floor).pack(fill='x', padx=10, pady=(0, 10))

        mot = self._card(row, 'MOTION')
        mot.pack(side='left', fill='both', expand=True, padx=(12, 0))
        m = tk.Frame(mot, bg=T['surface'])
        m.pack(fill='x', pady=(0, 10))
        self.v_step = tk.StringVar(value=STEP_MM_DEFAULT)
        self.v_rot = tk.StringVar(value=ROT_DEG_DEFAULT)
        self.v_speed = tk.StringVar(value=SPEED_PCT_DEFAULT)
        self._row(m, 'step (mm)', self.v_step, STEP_MM_CHOICES, 0, 0)
        self._row(m, 'level (deg)', self.v_rot, ROT_DEG_CHOICES, 1, 0)
        self._row(m, 'speed (%)', self.v_speed, SPEED_PCT_CHOICES, 2, 0)

    def _build_calib(self, parent):
        card = self._card(parent, 'CALIBRATION  |  camera -> TCP rotation')
        card.pack(fill='x', pady=(12, 0))
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', padx=10, pady=(0, 4))
        self.calib_labels = {}
        for r, key in enumerate(('source', 'frames', 'residual', 'validated',
                                 'status')):
            tk.Label(g, text=key, font=self.f_caption, fg=T['muted'],
                     bg=T['surface'], anchor='w', width=9).grid(
                row=r, column=0, sticky='w')
            v = tk.Label(g, text='--', font=self.f_small, fg=T['ink2'],
                         bg=T['surface'], anchor='w')
            v.grid(row=r, column=1, sticky='w')
            self.calib_labels[key] = v
        self._button(card, 'Reload calibration', T['grid'],
                     self.load_calib).pack(fill='x', padx=10, pady=(6, 10))

    def _build_actions(self, parent):
        card = self._card(parent, 'ALIGN')
        card.pack(fill='x', pady=(12, 0))
        tk.Label(card, text='manual step - cartesian, one clamped move',
                 font=self.f_caption, fg=T['muted'], bg=T['surface'],
                 anchor='w').pack(fill='x', padx=10)
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(2, 8))
        self.tb = self._button(row, '', T['grid'],
                               lambda: self.go(self.translate))
        self.tb.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.lb = self._button(row, '', T['grid'],
                               lambda: self.go(self.level))
        self.lb.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.ipb = self._button(row, '', T['grid'],
                                lambda: self.go(self.inplane))
        self.ipb.pack(side='left', expand=True, fill='x')
        self.ab = self._button(card, '', T['good'],
                               lambda: self.go(self.auto_converge),
                               font=(self.f_button[0], 12, 'bold'), pady=12)
        self.ab.pack(fill='x', padx=10, pady=(0, 8))
        stop = tk.Frame(card, bg=T['surface'])
        stop.pack(fill='x', padx=10, pady=(0, 10))
        self.estopb = self._button(stop, 'STOP NOW', T['critical'],
                                   self.stop_now,
                                   font=(self.f_button[0], 14, 'bold'),
                                   pady=12)
        self.estopb.pack(side='left', expand=True, fill='both', padx=(0, 8))
        self.pauseb = self._button(stop, 'PAUSE', T['grid'], self.toggle_pause,
                                   font=(self.f_button[0], 12, 'bold'),
                                   pady=12)
        self.pauseb.config(width=7, state='disabled')
        self.pauseb.pack(side='left', fill='both', padx=(0, 8))
        self.stopb = self._button(stop, 'stop after\ncurrent move',
                                  T['serious'], self.stop,
                                  font=(self.f_button[0], 9, 'bold'), pady=6)
        self.stopb.pack(side='left', fill='both')

    # ------------------------------------------------------------ helpers

    def step_m(self):
        return min(float(self.v_step.get()) / 1000.0, CEIL_STEP_M)

    def rot_deg(self):
        return min(float(self.v_rot.get()), CEIL_ROT_DEG)

    def target_m(self):
        return float(self.v_target.get()) / 1000.0

    def pos_tol_m(self):
        return float(self.v_tol.get()) / 1000.0

    def slowdown(self):
        """Trajectory time-stretch factor from the selected speed percent."""
        pct = min(float(self.v_speed.get()), CEIL_SPEED_PCT)
        return 100.0 / pct

    def inplane_target(self):
        """Desired in-plane angle in degrees, or None when disabled."""
        v = self.v_inplane.get()
        return None if v == 'off' else float(v)

    def z_floor(self):
        s = self.v_floor.get().strip()
        if not s:
            return None
        try:
            return float(s) / 1000.0
        except ValueError:
            return None

    def relabel(self):
        self.tb.config(text=f'Translate ({self.step_m()*1000:g} mm)')
        self.lb.config(text=f'Level ({self.rot_deg():g} deg)')
        t = self.v_inplane.get()
        self.ipb.config(text='In-plane (off)' if t == 'off'
                        else f'In-plane -> {t} deg')
        self.ab.config(text=f'AUTO-CONVERGE   |   -> {self.v_target.get()} mm')

    def stop(self):
        """Graceful: end the loop, let the in-flight segment finish."""
        self.abort = True
        self.stopped = True
        self.say('STOP (graceful) - cancelling goal; current move may finish.'
                 if self.n.stop()
                 else 'STOP (graceful) - loop will not continue.')
        self.set_status('stopping after current move', T['serious'])
        self.trace({'rec': 'stop_graceful'})

    def stop_now(self):
        """Instant: halt the trajectory where it is - and a live tracker,
        which that halt does not reach."""
        self.abort = True
        self.stopped = True
        self.n.stop_now()
        ladder = self.ladder
        if ladder.tracking or ladder._track_state() in TRACK_LIVE_STATES:
            ladder.stop_tracking_now()      # off the Tk thread, never refused
            self.say('*** STOP NOW - stopping the tracking node; the arm '
                     'follows the marker until it answers (logged as STOP '
                     'TRACKING) ***')
        else:
            self.say('*** STOP NOW - halting motion mid-move ***')
        self.set_status('STOPPED (instant)', T['critical'])
        self.trace({'rec': 'stop_now'})

    # ------------------------------------------------------------ gate

    def robot_block(self):
        """None if the robot may move now, else (reason, fatal).

        fatal ends the run; otherwise it pauses. REFLEX is fatal whatever the
        toggle says: resuming by itself right after an error recovery would
        be a surprise.
        """
        rm = self.n.robot_mode()
        fresh = rm is not None and rm[1] <= ROBOT_STATE_STALE_S
        if fresh and rm[0] == MODE_REFLEX:
            return 'robot in REFLEX - run error recovery', True
        if self.user_paused:
            return 'paused by operator', False
        if not self.gate_on:
            return None
        if not fresh:
            return 'no robot state - is franka_ros2 up?', False
        if rm[0] == MODE_MOVE:
            return None
        return GATE_REASONS.get(rm[0], f'robot mode {rm[0]}'), False

    def _resume_hook(self):
        # always: PAUSE has to resume a Cartesian step with the gate off too
        return self.wait_gate

    def _resume_hint(self):
        return ('press RESUME to continue' if self.user_paused
                else 'resumes once the robot is back in MOVE')

    def toggle_pause(self):
        """PAUSE halts motion but keeps the run; RESUME continues from rest."""
        if not self.busy:
            return
        self.user_paused = not self.user_paused
        if self.user_paused:
            self.n.interrupt()
            self.say('PAUSE - motion halted, run kept. RESUME continues, '
                     'STOP NOW ends it.')
        else:
            self.say('RESUME pressed')
        self.trace({'rec': 'pause' if self.user_paused else 'resume'})
        self._paint_pause()

    def _paint_pause(self):
        b = self.pauseb
        color = (T['grid'] if not self.busy
                 else T['good'] if self.user_paused else T['warning'])
        fg = text_on(color)
        b.config(text='RESUME' if self.user_paused else 'PAUSE', bg=color,
                 fg=fg, activebackground=mix(color, '#ffffff', .15),
                 activeforeground=fg)
        b.bind('<Enter>', lambda e, c=color: b['state'] == 'normal'
               and b.config(bg=mix(c, '#ffffff', .12)))
        b.bind('<Leave>', lambda e, c=color: b.config(bg=c))

    def _run_finished(self, btns):
        for b in btns + [self.gatecb]:
            b.config(state='normal')
        self.pauseb.config(state='disabled')
        self._paint_pause()

    def wait_gate(self):
        """Block while the robot may not move. True = carry on; False = the
        run ends (STOP, or a robot fault - then self.gate_fault says which).

        Resumes only once motion has been allowed for GATE_RESUME_HOLD_S
        unbroken, then re-bases R from TF: the arm can move a little before
        the robot stops it, and a stale R would skew every later command.
        """
        blk = self.robot_block()
        if blk is None:
            return not self.abort
        if blk[1]:
            return self._gate_fault(blk[0])
        t0, held = time.time(), None
        self.paused, self.pause_reason = True, blk[0]
        self.say(f'PAUSED - {blk[0]}. {self._resume_hint()}; STOP NOW ends '
                 'the run.')
        self.set_status(f'paused: {blk[0]}', T['warning'])
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
                elif time.monotonic() - held >= GATE_RESUME_HOLD_S:
                    break
                time.sleep(0.02)
        finally:
            self.paused = False
        if self.abort:
            return False
        self.load_calib(quiet=True)
        dt = time.time() - t0
        self.say(f'RESUMED after {dt:.1f} s')
        self.set_status('moving...', T['warning'])
        self.trace({'rec': 'gate_resume', 'paused_s': dt})
        return True

    def _gate_fault(self, reason):
        self.gate_fault = 'robot_reflex'
        self.say(f'ABORT: {reason}')
        self.set_status('aborted: robot reflex', T['critical'])
        self.trace({'rec': 'gate_fault', 'reason': reason})
        return False

    def _on_mode_change(self, prev, mode):
        """Spin thread. Halts motion the moment the robot leaves MOVE."""
        if (self.busy and prev == MODE_MOVE
                and (self.gate_on or mode == MODE_REFLEX)):
            self.n.halt()
        self._mode_log.append((time.time(), prev, mode))

    def _drain_mode_log(self):
        while self._mode_log:
            t, prev, mode = self._mode_log.popleft()
            stamp = datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')
            self.say(f'robot mode {ROBOT_MODES.get(prev, prev)} -> '
                     f'{ROBOT_MODES.get(mode, mode)}  ({stamp[:-3]})')
            self.trace({'rec': 'robot_mode', 'prev': prev, 'mode': mode})

    def _toggle_gate(self):
        if self.busy:                    # also disabled while busy
            self.v_gate.set(self.gate_on)
            return
        self.gate_on = bool(self.v_gate.get())
        self.say('robot-state gate ON - motion only while the robot reports '
                 'MOVE' if self.gate_on else
                 'robot-state gate OFF - motion no longer waits for robot MOVE'
                 ' (a REFLEX still ends a run)')

    # ------------------------------------------------------------ calib io

    def _read_calib_meta(self):
        """Pull what the calibration card shows from CALIB_PATH."""
        if not CALIB_PATH.exists():
            self.calib_meta, self.calib_state = {}, 'missing'
            return
        try:
            self.calib_meta = yaml.safe_load(CALIB_PATH.read_text()) or {}
        except Exception:                                   # noqa: BLE001
            self.calib_meta, self.calib_state = {}, 'error'

    def _autoload(self):
        """Load the calibration once TF is up, and keep trying until it is.

        A fresh process under unicast DDS discovery can take a while to see
        TF, and MoveIt may simply not be running yet. Neither is an error, so
        this retries quietly rather than giving up; the calibration card
        shows the waiting state instead of a log line.
        """
        if self.R is not None:
            return
        if not CALIB_PATH.exists():
            self.calib_state = 'missing'
            return
        try:
            self.n.tcp_pose()
        except Exception:                                   # noqa: BLE001
            self.calib_state = 'waiting'
            self.root.after(2000, self._autoload)
            return
        self.load_calib(quiet=True)

    def load_calib(self, quiet=False):
        """Rebuild R_cam_base for the CURRENT arm pose from the saved file.

        Reloading after the arm has moved simply re-bases again. There is no
        save: level and in-plane steps update R analytically, and writing
        that back could silently replace the validated handeye_calib result.
        """
        self._read_calib_meta()
        if not CALIB_PATH.exists():
            if not quiet:
                self.say(f'no saved calibration at {CALIB_PATH.name}')
            return
        try:
            # quat_xyzw is TCP -> optical, i.e. R_tcp_cam
            R_cam_tcp = q2R(*self.calib_meta['quat_xyzw']).T
            _, R_base_tcp = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.calib_state = ('waiting' if 'does not exist' in str(e)
                                else 'error')
            if not quiet:
                self.say(f'load failed: {e}')
            return
        # R_cam_base = R_cam_tcp @ R_tcp_base, rebased to where the arm is
        self.R = R_cam_tcp @ R_base_tcp.T
        self.calib_state = 'loaded'
        self.say(f'calibration loaded ({CALIB_PATH.name}), re-based to the '
                 'current pose')

    def auto_floor(self):
        """Predict the end pose from the CURRENT measurement, floor below it.

        The optical axis is near-vertical here, so the TCP descends about
        (measured - target). Needs no hand-eye translation, and it bounds a
        runaway from a bad marker scale or a sign error.
        """
        m = self.n.marker()
        if m is None:
            self.say('auto floor: marker not visible')
            return
        try:
            pos, _ = self.n.tcp_pose()
        except Exception as e:                              # noqa: BLE001
            self.say(f'auto floor failed: {e}')
            return
        descent = float(m[0][2]) - self.target_m()
        floor = pos[2] - max(descent, 0.0) - FLOOR_MARGIN_M
        self.v_floor.set(f'{floor * 1000:.1f}')
        self.say(f'auto floor: TCP z now {pos[2]*1000:.1f} mm, expected '
                 f'descent {descent*1000:.1f} mm -> floor '
                 f'{floor*1000:.1f} mm ({FLOOR_MARGIN_M*1000:g} mm margin)')

    # ------------------------------------------------------------ live view

    def banner_state(self, snap=None):
        snap = getattr(self, "_snap", None) if snap is None else snap
        """(STATE, colour, subtitle) - the one-glance answer to 'what now?'."""
        if self.paused:
            return ('PAUSED', T['warning'],
                    f'{self.pause_reason}  -  {self._resume_hint()},  '
                    'STOP NOW ends the run')
        if self.busy:
            return ('ALIGNING', T['series'],
                    'cartesian steps running  -  STOP NOW halts them')
        if self.stopped:
            return ('STOPPED', T['critical'],
                    'halted by operator  -  starting a new run clears this')
        if snap is None:
            return ('NO MARKER', T['critical'],
                    'marker not visible, or vision is stale')
        if self.R is None:
            return ('NO CALIBRATION', T['warning'],
                    'waiting for TF  -  is MoveIt running?  '
                    '(Reload in CALIBRATION)')
        if snap['ok']:
            return ('ALIGNED', T['good'],
                    f'|e| {snap["err"]:.2f} mm   tilt {snap["tilt"]:.2f} deg'
                    f'   in-plane error {snap["ip_err"]:.2f} deg')
        if self.last_outcome and self.last_outcome != 'converged':
            return ('ABORTED', T['serious'],
                    'last run ended: '
                    f'{self.last_outcome.replace("_", " ")}')
        return ('READY', T['grid'],
                f'|e| {snap["err"]:.1f} mm   tilt {snap["tilt"]:.1f} deg'
                '  -  press AUTO-CONVERGE')

    def pill_states(self):
        """[(label, value, colour)] for the connection strip."""
        rm = self.n.robot_mode()
        if self.n.relay_error:
            franka = ('no relay', T['critical'])
        elif rm is None:
            franka = ('no state', T['critical'])
        elif rm[1] > ROBOT_STATE_STALE_S:
            franka = ('stale', T['critical'])
        else:
            franka = (ROBOT_MODES.get(rm[0], str(rm[0])),
                      T['good'] if rm[0] == MODE_MOVE
                      else T['critical'] if rm[0] in (0, MODE_REFLEX)
                      else T['warning'])
        moveit = (self.n.cart.service_is_ready()
                  and self.n.exec_ac.server_is_ready())
        blk = self.robot_block()
        if self.user_paused:
            gate = ('paused', T['warning'])
        elif not self.gate_on:
            gate = ('OFF', T['serious'])
        elif blk is None:
            gate = ('open', T['good'])
        else:
            gate = ('blocked', T['critical'] if blk[1] else T['warning'])
        return [('FRANKA',) + franka,
                ('MOVEIT', 'ready' if moveit else 'down',
                 T['good'] if moveit else T['critical']),
                ('GATE',) + gate]

    def draw_plot(self, key, idx, tol):
        """Rolling history with a dashed tolerance line (mating_panel style)."""
        c = self.plots[key]
        c.delete('all')
        w = max(c.winfo_width(), 160)
        hgt = int(c['height'])
        ml, mr, mt, mb = 30, 6, 6, 14
        pw, ph = w - ml - mr, hgt - mt - mb
        now, win = time.monotonic(), PLOT_WINDOW_S
        data = [(e[0], e[idx + 1]) for e in self.hist if now - e[0] < win]
        top = max([v for _, v in data] + [tol * 2.0]) * 1.12

        def px(ts):
            return ml + pw - (now - ts) / win * pw

        def py(v):
            return mt + ph - min(v / top, 1.0) * ph

        for frac in (0.25, 0.5, 0.75):
            gy = mt + ph * frac
            c.create_line(ml, gy, ml + pw, gy, fill=T['grid'])
        c.create_line(ml, mt + ph, ml + pw, mt + ph, fill=T['baseline'])
        ty = py(tol)
        c.create_line(ml, ty, ml + pw, ty, fill=T['muted'], dash=(2, 4))
        for vy, txt in ((mt + ph, '0'), (mt + 4, f'{top:.3g}')):
            c.create_text(ml - 4, vy, text=txt, anchor='e', fill=T['muted'],
                          font=('DejaVu Sans Mono', 7))
        # Tolerance is labelled on its own line, at the right. On the left
        # axis it printed over the '0' whenever the error dwarfed the
        # tolerance (372 mm vs 2 mm puts the line on the baseline). top is
        # at least 2.24 x tol, so the line stays in the lower ~45% and the
        # label above it never clips the top edge.
        c.create_text(ml + pw - 2, ty - 2, text=f'tol {tol:g}', anchor='se',
                      fill=T['muted'], font=('DejaVu Sans Mono', 7))
        if len(data) >= 2:
            pts = [q for ts, v in data for q in (px(ts), py(v))]
            c.create_line(*pts, fill=T['series'], width=2,
                          joinstyle='round', capstyle='round')
            lx, ly = px(data[-1][0]), py(data[-1][1])
            c.create_oval(lx - 3, ly - 3, lx + 3, ly + 3, fill=T['series'],
                          outline=T['surface'], width=2)

    def _update_calib_card(self):
        md = self.calib_meta
        self.calib_labels['source'].config(
            text=(f'{CALIB_PATH.name}   {md.get("method", "?")}, '
                  f'{md.get("poses", "?")} poses   '
                  f'{md.get("calibrated", "?")}'))
        self.calib_labels['frames'].config(
            text=(f'{md.get("parent_frame", "?")} -> '
                  f'{md.get("child_frame", "?")}'))
        if 'residual_mm' in md:
            self.calib_labels['residual'].config(
                text=(f'{md["residual_mm"]:.2f} mm / '
                      f'{md.get("residual_deg", float("nan")):.2f} deg'))
        if 'validated_scatter_mm' in md:
            self.calib_labels['validated'].config(
                text=(f'{md["validated_scatter_mm"]:.2f} mm static-marker '
                      'scatter'))
        text, col = {
            'loaded': ('loaded, re-based to the current pose', T['good']),
            'waiting': ('waiting for TF  -  is MoveIt running?', T['warning']),
            'missing': (f'{CALIB_PATH.name} not found', T['critical']),
            'error': ('could not read the calibration file', T['critical']),
        }[self.calib_state]
        self.calib_labels['status'].config(text=text, fg=col)

    # ------------------------------------------------------------ actions

    def _torque_block(self, ask=False):
        """Why ALIGN must not move the arm now, or None.

        ALIGN plans on fr3_arm_controller; while the impedance controller
        holds the arm - above all while the tracking node streams its
        equilibrium - a planned move would fight it. ask
        also asks the controller manager (it blocks, so worker thread only):
        a restarted panel's memory says nothing, and silence is no answer.
        """
        if self.ladder.tracking:
            return 'the tracking node is driving the arm - STOP TRACKING first'
        if self.ladder.active:
            return (f'{IMPEDANCE_CONTROLLER} holds the arm - RELEASE it on '
                    'IMPEDANCE & TRACK first')
        if not ask:
            return None
        states = self.n.controllers()
        if states is None:
            return ('the controller manager did not answer - cannot rule '
                    f'out {IMPEDANCE_CONTROLLER}')
        if states.get(IMPEDANCE_CONTROLLER) == 'active':
            return (f'{IMPEDANCE_CONTROLLER} is active - RELEASE it on '
                    'IMPEDANCE & TRACK first')
        return None

    def go_impl(self, fn, *a):
        """Run an action in a worker thread with the motion buttons disabled."""
        if self.busy:
            return
        blk = self.robot_block()
        if blk is not None:
            self.say(f'REFUSING: {blk[0]}'
                     + ('' if blk[1] else
                        ' (robot-state gate is on - TARGET + SAFETY)'))
            self.set_status(f'refused: {blk[0]}', T['serious'])
            return
        why = self._torque_block()
        if why is not None:
            self.say(f'REFUSING: {why}')
            self.set_status(f'refused: {why}', T['serious'])
            return
        self.busy = True
        self.stopped = False
        self.abort = False        # manual steps wait on the gate, which reads it
        self.gate_fault = None
        self.gatecb.config(state='disabled')
        self.user_paused = False
        self.pauseb.config(state='normal')
        self._paint_pause()
        if fn == self.auto_converge:
            self.last_outcome = None
        btns = [self.tb, self.lb, self.ipb, self.ab]
        for b in btns:
            b.config(state='disabled')
        self.set_status('moving...', T['warning'])

        def run():
            try:
                why = self._torque_block(ask=True)
                if why is not None:
                    self.say(f'REFUSING: {why}')
                    self.set_status(f'refused: {why}', T['serious'])
                    return
                fn(*a)
            except Exception as e:                      # noqa: BLE001
                self.say(f'ERROR: {e}')
                self.set_status(f'error: {e}', T['critical'])
            finally:
                self.busy = False
                self.user_paused = False
                self.root.after(0, self._run_finished, btns)
        threading.Thread(target=run, daemon=True).start()

    def translate(self, quiet=False):
        if self.R is None:
            self.say('no calibration loaded - use Reload in CALIBRATION')
            return False
        m = self.n.marker()
        if m is None:
            self.say('marker not visible')
            return False
        err = m[0] - np.array([0, 0, self.target_m()])
        d = self.R.T @ err
        nrm = np.linalg.norm(d)
        lim = self.step_m()
        if nrm < 1e-4:
            self.say('already within 0.1 mm - nothing to do')
            return True
        if nrm > lim:
            d *= lim / nrm
        if not quiet:
            self.say(f'translate: base delta [{d[0]*1000:+.2f} '
                     f'{d[1]*1000:+.2f} {d[2]*1000:+.2f}] mm '
                     f'(|e|={nrm*1000:.2f} mm)')
        ok, msg = self.n.move(d, z_floor=self.z_floor(),
                              slowdown=self.slowdown(),
                              resume=self._resume_hook())
        self.say(f'  -> {msg}')
        self.trace({'rec': 'translate', 'ok': ok, 'msg': msg,
                    'err_cam_mm': (err * 1000).tolist(),
                    'err_norm_mm': float(nrm * 1000),
                    'cmd_d_base_mm': (d * 1000).tolist(),
                    'marker_pos_cam': m[0].tolist(),
                    **self.n.last_cmd})
        return ok

    def level(self, quiet=False):
        if self.R is None:
            self.say('no calibration loaded - use Reload in CALIBRATION')
            return False
        m = self.n.marker()
        if m is None:
            self.say('marker not visible')
            return False
        mz = m[1][:, 2]
        tgt = np.array([0, 0, -1.0]) if mz[2] < 0 else np.array([0, 0, 1.0])
        ang = np.arccos(np.clip(mz @ tgt, -1, 1))
        if np.degrees(ang) < 0.15:
            self.say('already square within 0.15 deg')
            return True
        axis = np.cross(mz, tgt)
        if np.linalg.norm(axis) < 1e-8:
            self.say('degenerate rotation axis - skipping')
            return False
        tilt_before = float(np.degrees(np.arccos(
            np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
        full_ang = float(np.degrees(ang))
        ang = min(ang, np.radians(self.rot_deg()))
        Rc = axis_angle_R(axis, ang).T          # camera-frame correction
        D = self.R.T @ Rc @ self.R              # same rotation, base frame
        if not quiet:
            self.say(f'level: rotating {np.degrees(ang):.2f} deg')
        ok, msg = self.n.move(np.zeros(3), R_delta=D, z_floor=self.z_floor(),
                              slowdown=self.slowdown(),
                              resume=self._resume_hook())
        self.say(f'  -> {msg}')
        rec = {'rec': 'level', 'ok': ok, 'msg': msg,
               'tilt_before_deg': tilt_before,
               'full_correction_deg': full_ang,
               'clamped_cmd_deg': float(np.degrees(ang)),
               'marker_normal_cam': mz.tolist(),
               'axis_cam': (axis / np.linalg.norm(axis)).tolist(),
               'R_delta_base_quat': R2q(D).tolist(),
               **self.n.last_cmd}
        time.sleep(0.6)                 # let the filter settle before reading
        m2 = self.n.marker()
        if m2 is not None:
            mz2 = m2[1][:, 2]
            rec['tilt_after_deg'] = float(np.degrees(np.arccos(
                np.clip(abs(mz2 @ np.array([0, 0, 1.0])), -1, 1))))
            rec['tilt_change_deg'] = rec['tilt_after_deg'] - tilt_before
        self.trace(rec)
        if ok:
            self.R = Rc.T @ self.R
        return ok

    def inplane(self, quiet=False):
        """Rotate about the OPTICAL AXIS to bring the marker X axis to the
        selected in-plane target.

        This is the 6th DOF. Nulling position and tilt leaves it wherever
        the arm ended up. Actuated almost purely by J7 (the optical axis is
        0.37 deg off TCP Z), and the signal is the cleanest we have
        (0.01 deg std), so it converges fast.
        """
        tgt = self.inplane_target()
        if tgt is None:
            self.say('in-plane target is "off"')
            return True
        if self.R is None:
            self.say('no calibration loaded - use Reload in CALIBRATION')
            return False
        m = self.n.marker()
        if m is None:
            self.say('marker not visible')
            return False
        cur = inplane_angle(m[1])
        if cur is None:
            self.say('in-plane angle unavailable')
            return False
        ang = inplane_correction(cur, tgt, self.rot_deg())
        if abs(ang) < 1e-3:
            return True
        # Rotating the camera frame by Rc transforms measurements by Rc.T,
        # so a marker at angle `cur` reads `cur - ang` afterwards.
        Rc = axis_angle_R(np.array([0.0, 0.0, 1.0]), np.radians(ang))
        D = self.R.T @ Rc @ self.R          # same rotation, base frame
        if not quiet:
            self.say(f'in-plane: {cur:+.2f} -> {tgt:+.0f} deg, '
                     f'rotating {ang:+.2f} deg')
        ok, msg = self.n.move(np.zeros(3), R_delta=D, z_floor=self.z_floor(),
                              slowdown=self.slowdown(),
                              resume=self._resume_hook())
        self.say(f'  -> {msg}')
        rec = {'rec': 'inplane', 'ok': ok, 'msg': msg,
               'inplane_before_deg': cur, 'target_deg': tgt,
               'cmd_deg': float(ang), **self.n.last_cmd}
        time.sleep(0.4)
        m2 = self.n.marker()
        if m2 is not None:
            a2 = inplane_angle(m2[1])
            if a2 is not None:
                rec['inplane_after_deg'] = a2
                rec['inplane_change_deg'] = wrap_deg(a2 - cur)
        self.trace(rec)
        if ok:
            self.R = Rc.T @ self.R
        return ok

    def auto_converge(self):
        """Translate/level until the camera is at the target standoff and
        parallel. Aborts on STOP, marker loss, Z-floor block, plan failure,
        or lack of progress."""
        if self.R is None:
            self.say('no calibration loaded - use Reload in CALIBRATION')
            return
        if self.z_floor() is None:
            self.say('REFUSING: set a TCP Z floor first '
                     '("Auto floor from here")')
            self.set_status('refused: no Z floor set', T['critical'])
            return
        self.abort = False
        tol = self.pos_tol_m()
        m0 = self.n.marker()
        if m0 is None:
            self.say('marker not visible')
            return
        # Iteration cap scaled to the distance/step actually asked for: a
        # 245 mm descent in 2 mm steps legitimately needs >100 iterations.
        err0 = float(np.linalg.norm(m0[0] - np.array([0, 0, self.target_m()])))
        cap = int(min(MAX_ITERS_ABS, max(60, 3 * err0 / self.step_m() + 30)))
        self.say(f'=== AUTO-CONVERGE to {self.target_m()*1000:g} mm, '
                 f'tol {tol*1000:g} mm / {ROT_TOL_DEG:g} deg, '
                 f'cap {cap} iters, speed {self.v_speed.get()}% ===')
        self.open_trace({
            'target_mm': self.target_m() * 1000,
            'pos_tol_mm': tol * 1000, 'rot_tol_deg': ROT_TOL_DEG,
            'step_mm': self.step_m() * 1000, 'rot_step_deg': self.rot_deg(),
            'speed_pct': float(self.v_speed.get()),
            'z_floor_mm': self.z_floor() * 1000,
            'R_cam_base': self.R.tolist(),
            'cap': cap,
            'inplane_target': self.v_inplane.get(),
            'gate': self.gate_on,
            'controller': ARM_CONTROLLER,
        })
        outcome = 'exception'
        try:
            outcome = self._converge_loop(tol, cap)
        finally:
            self.close_trace(outcome)
            self.say(f'trace written: {self.tracepath}')

    def _converge_loop(self, tol, cap):
        """The loop body. Returns an outcome string; the caller closes the
        trace so every exit path (including exceptions) is recorded."""
        phase, prev, stale = None, np.inf, 0
        seen_edges = self.n.gate_edges
        for it in range(1, cap + 1):
            if self.abort:
                self.say('ABORTED by STOP')
                return 'stopped'
            if not self.wait_gate():
                return self.gate_fault or 'stopped'
            if self.n.gate_edges != seen_edges:
                # paused since the last look: restart the progress watchdog
                seen_edges, phase = self.n.gate_edges, None
            m = self.n.marker()
            if m is None:
                self.say('ABORT: marker lost / stale')
                self.set_status('aborted: marker lost', T['critical'])
                return 'marker_lost'
            # whole-arm floor - an elbow can dip below the floor while the
            # TCP goal pre-check still passes
            floor = self.z_floor()
            low = self.n.lowest_link(FLOOR_LINKS)
            if floor is not None and low is not None and low[1] < floor:
                self.say(f'ABORT: Z FLOOR - {low[0]} at '
                         f'{low[1]*1000:.1f} mm < {floor*1000:.1f} mm')
                self.set_status('aborted: Z floor', T['critical'])
                return 'z_floor'
            err = float(np.linalg.norm(m[0] - np.array([0, 0,
                                                        self.target_m()])))
            mz = m[1][:, 2]
            tilt = float(np.degrees(np.arccos(
                np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
            tgt_ip = self.inplane_target()
            ip = inplane_angle(m[1])
            ip_err = (0.0 if tgt_ip is None or ip is None
                      else abs(wrap_deg(ip - tgt_ip)))
            ipmsg = '' if tgt_ip is None else f'  in-plane={ip:+6.2f} deg'
            self.say(f'[{it:02d}] |e|={err*1000:7.2f} mm  '
                     f'tilt={tilt:5.2f} deg{ipmsg}')
            self.trace({'rec': 'iter', 'it': it, 'err_mm': err * 1000,
                        'tilt_deg': tilt,
                        'marker_pos_cam': m[0].tolist(),
                        'marker_normal_cam': mz.tolist(),
                        'marker_quat': R2q(m[1]).tolist(),
                        'inplane_deg': ip, 'inplane_err_deg': ip_err,
                        'joints': self.n.joints()})

            if err <= tol and tilt <= ROT_TOL_DEG and ip_err <= INPLANE_TOL_DEG:
                self.say(f'=== CONVERGED: {err*1000:.2f} mm, {tilt:.2f} deg'
                         f'{"" if tgt_ip is None else f", in-plane {ip:+.2f} deg"}'
                         f' in {it} iterations ===')
                self.set_status(f'converged: {err*1000:.2f} mm, '
                                f'{tilt:.2f} deg', T['good'])
                return 'converged'
            # Progress watchdog. Compared against the PREVIOUS iteration, not
            # the best ever, and reset when the phase switches: a level step
            # legitimately increases translation error (it rotates about the
            # TCP, and the camera swings on the unknown lever arm), so a
            # best-ever test would false-abort right after every level.
            if err > tol:
                cur = 't'
            elif tilt > ROT_TOL_DEG:
                cur = 'r'
            else:
                cur = 'p'                      # in-plane, about the optical axis
            if cur != phase:
                phase, prev, stale = cur, np.inf, 0
            metric = {'t': err, 'r': tilt, 'p': ip_err}[cur]
            if metric < prev - (1e-5 if cur == 't' else 1e-3):
                stale = 0
            else:
                stale += 1
                if stale >= NO_PROGRESS_LIMIT:
                    self.say(f'ABORT: no progress in {stale} iterations of '
                             f'phase "{cur}" (reduce step size; or the '
                             'orientation estimate is too noisy to level)')
                    self.set_status('aborted: no progress', T['serious'])
                    return f'no_progress_{cur}'
            prev = metric
            ok = {'t': self.translate, 'r': self.level,
                  'p': self.inplane}[cur](quiet=True)
            if not ok:
                if self.gate_fault:
                    return self.gate_fault
                if self.abort:
                    return 'stopped'
                self.say('ABORT: step failed (see message above)')
                self.set_status('aborted: step failed', T['critical'])
                return 'step_failed'
            time.sleep(0.5)          # let the filter settle after motion
        self.say(f'ABORT: hit iteration cap ({cap})')
        self.set_status('aborted: iteration cap', T['serious'])
        return 'iter_cap'


class LadderPane(Pane):
    """Impedance commissioning ladder and continuous tracking, from
    impedance_panel.py; only the chrome moved."""

    NAME = 'IMPEDANCE'

    def __init__(self, shell, node, parent):
        super().__init__(shell, node, parent)
        self.active = False          # impedance controller active
        self.floating = False
        self.setpoint = None
        self.z_floor = None
        self.preflight_ok = False
        self.arm_released = False
        self.driver_down_logged = False
        self.tracking = False        # the node is streaming; STOP is offered
        self._tracking_at = float('-inf')   # monotonic, last set by the panel
        self._track_seen = None      # last traced (state, reason, policy)
        self._painted = None         # what paint_tracking last showed

    def build(self, parent):
        self._build_ladder_readout(parent)
        self._build_ladder(parent)
        self._build_tuning(parent)
        self.paint_tracking()

    def buttons(self):
        return [self.b_pre, self.b_float, self.b_hold, self.b_minus,
                self.b_plus, self.b_here, self.b_track, self.b_release,
                self.b_gains]

    def _build_ladder_readout(self, parent):
        card = self._card(parent, 'ARM')
        card.pack(fill='x')
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.tiles = {}
        for key, unit in (('TCP Z', 'mm'), ('|F| ext', 'N'), ('Fz ext', 'N'),
                          ('spring lead', 'mm')):
            col = tk.Frame(row, bg=T['surface'])
            col.pack(side='left', padx=(0, 18))
            tk.Label(col, text=key.upper(), font=self.f_caption,
                     fg=T['muted'], bg=T['surface'], anchor='w').pack(
                fill='x')
            v = tk.Label(col, text='--', font=self.f_value, fg=T['ink'],
                         bg=T['surface'], anchor='w')
            v.pack(fill='x')
            tk.Label(col, text=unit, font=self.f_caption, fg=T['muted'],
                     bg=T['surface'], anchor='w').pack(fill='x')
            self.tiles[key] = v
        self.sub = tk.Label(card, text='', font=self.f_small, fg=T['muted'],
                            bg=T['surface'], anchor='w', justify='left')
        self.sub.pack(fill='x', padx=10, pady=(0, 10))

    def _build_ladder(self, parent):
        card = self._card(parent, 'LADDER  |  fr3_mating_controllers README')
        card.pack(fill='x')

        tk.Label(card, text='payload lives in Desk (end-effector profile) - '
                            'PRE-FLIGHT zeroes the FCI load so it is never '
                            'counted twice',
                 font=self.f_caption, fg=T['muted'], bg=T['surface'],
                 anchor='w', justify='left', wraplength=380).pack(
                     fill='x', padx=10, pady=(0, 6))
        self.b_pre = self._button(card, '0.  PRE-FLIGHT  (reflex thresholds)',
                                  T['warning'],
                                  lambda: self.go(self.preflight))
        self.b_pre.pack(fill='x', padx=10, pady=(2, 6))
        self.b_float = self._button(card, '1.  FLOAT  (RT proof)',
                                    T['series'], lambda: self.go(self.float_on))
        self.b_float.pack(fill='x', padx=10, pady=(0, 6))
        self.b_hold = self._button(card, '2.  HOLD  (float off, holds here)',
                                   T['good'], lambda: self.go(self.hold_on))
        self.b_hold.pack(fill='x', padx=10, pady=(0, 6))

        step = tk.Frame(card, bg=T['surface'])
        step.pack(fill='x', padx=10, pady=(0, 4))
        self.v_step = tk.StringVar(value=SETPOINT_MM_DEFAULT)
        self.v_axis = tk.StringVar(value=AXIS_CHOICES[0])
        ttk.Combobox(step, textvariable=self.v_step, values=SETPOINT_MM_CHOICES,
                     width=4, state='readonly', style='Dark.TCombobox',
                     font=self.f_small).pack(side='left')
        tk.Label(step, text='mm along', font=self.f_caption, fg=T['ink2'],
                 bg=T['surface']).pack(side='left', padx=4)
        ttk.Combobox(step, textvariable=self.v_axis, values=AXIS_CHOICES,
                     width=15, state='readonly', style='Dark.TCombobox',
                     font=self.f_small).pack(side='left')
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.b_minus = self._button(row, '3.  SETPOINT  -', T['grid'],
                                    lambda: self.go(self.setpoint_step, -1.0))
        self.b_minus.pack(side='left', expand=True, fill='x', padx=(0, 6))
        self.b_plus = self._button(row, 'SETPOINT  +', T['grid'],
                                   lambda: self.go(self.setpoint_step, 1.0))
        self.b_plus.pack(side='left', expand=True, fill='x')
        self.b_here = self._button(card, 'hold HERE (equilibrium = arm)',
                                   T['grid'], lambda: self.go(self.hold_here),
                                   pady=6)
        self.b_here.pack(fill='x', padx=10, pady=(0, 8))
        self.b_track = self._button(card, '3b.  TRACK  (continuous, follows '
                                          'the marker)', T['series'],
                                    lambda: self.go(self.start_tracking),
                                    pady=6)
        self.b_track.pack(fill='x', padx=10, pady=(0, 6))
        pol = tk.Frame(card, bg=T['surface'])
        pol.pack(fill='x', padx=10, pady=(0, 8))
        tk.Label(pol, text='over the lead cap', font=self.f_caption,
                 fg=T['ink2'], bg=T['surface']).pack(side='left')
        self.v_policy = tk.StringVar(value=OVER_LEAD_DEFAULT)
        cb = self._combo(pol, self.v_policy, OVER_LEAD_CHOICES)
        cb.pack(side='left', padx=4)
        # live, the call off the Tk thread: the node reads it on every tick
        cb.bind('<<ComboboxSelected>>', self._on_policy_selected)
        tk.Label(pol, text='hold = wait, stop = end, clamp = follow capped',
                 font=self.f_caption, fg=T['muted'],
                 bg=T['surface']).pack(side='left', padx=4)
        # STOP TRACKING is in the window header (CellPanel): the node drives
        # the arm whichever tab is on top.
        self.b_release = self._button(card, '4.  RELEASE  ->  arm controller',
                                      T['critical'],
                                      lambda: self.go(self.release),
                                      font=(self.f_button[0], 13, 'bold'),
                                      pady=13)
        self.b_release.pack(fill='x', padx=10, pady=(0, 10))

    def _build_tuning(self, parent):
        card = self._card(parent, 'GAINS  |  live, within the controller '
                                  'limits')
        card.pack(fill='x', pady=(12, 0))
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', padx=10, pady=(0, 6))
        self.tune = {}
        for r, (label, key, default) in enumerate(TUNE_FIELDS):
            lo, hi = GAIN_LIMITS[key]
            tk.Label(g, text=label, font=self.f_caption, fg=T['ink2'],
                     bg=T['surface'], anchor='e', width=12).grid(
                row=r, column=0, sticky='e', pady=2)
            var = tk.StringVar(value=default)
            self._entry(g, var, 8).grid(row=r, column=1, sticky='w', padx=6,
                                        pady=2)
            tk.Label(g, text=f'{lo:g} - {hi:g}', font=self.f_caption,
                     fg=T['muted'], bg=T['surface']).grid(row=r, column=2,
                                                          sticky='w')
            self.tune[key] = var
        self.b_gains = self._button(card, 'apply gains', T['grid'],
                                    lambda: self.go(self.apply_gains), pady=6)
        self.b_gains.pack(fill='x', padx=10, pady=(0, 10))

    # ------------------------------------------------------------ helpers

    def _sample(self, s):
        """Spin thread, every relayed state message (50 Hz): enough to see
        the hold-test overshoot and short RT dips, with joints for drift."""
        if self.tracef is None or not self.active:
            return
        anchor = self.setpoint
        self.trace({'rec': 'sample', **s,
                    'anchor': None if anchor is None else list(anchor[0]),
                    'floating': self.floating})

    def gains(self):
        """The gain entries as numbers, or None if one is not a number."""
        try:
            return {k: float(v.get()) for k, v in self.tune.items()}
        except ValueError:
            return None

    @staticmethod
    def gain_error(g):
        """Why a gain set is unacceptable, or None."""
        for key, (lo, hi) in GAIN_LIMITS.items():
            v = g[key]
            if not np.isfinite(v) or not lo <= v <= hi:
                return f'{key} must be within [{lo:g}, {hi:g}]'
        return None

    def _state_age(self):
        st = self.n.state()
        return float('inf') if st is None else st[5]

    def blocked(self):
        """Why the robot may not be driven right now, or None."""
        st = self.n.state()
        if st is None or st[5] > STATE_STALE_S:
            return 'no robot state (is franka_ros2 running?)'
        if st[3] == MODE_REFLEX:
            return 'robot in REFLEX - run error recovery'
        if st[3] != MODE_MOVE:
            return f'robot is {ROBOT_MODES.get(st[3], st[3])}, not MOVE'
        return None

    def _wait_mode(self, modes, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            st = self.n.state()
            if st is not None and st[5] <= STATE_STALE_S and st[3] in modes:
                return True
            time.sleep(0.05)
        return False

    def go_impl(self, fn, *a):
        """Run one action in a worker thread; buttons disabled meanwhile."""
        if self.busy:
            return
        self.busy = True
        for b in self.buttons():
            b.config(state='disabled')

        def run():
            try:
                fn(*a)
            except Exception as e:                          # noqa: BLE001
                self.say(f'ERROR: {e}')
                self.set_status(f'error: {e}', T['critical'])
            finally:
                self.busy = False
                self.root.after(0, self._run_finished)
        threading.Thread(target=run, daemon=True).start()

    def _run_finished(self):
        """Tk thread, after every action. Waking every button here is what
        un-greyed SETPOINT and TRACK under a running stream, so paint_tracking
        has the last word."""
        if self.busy:                # a newer action owns the buttons now
            return
        for b in self.buttons():
            b.config(state='normal')
        self.paint_tracking()

    def paint_tracking(self):
        """STOP TRACKING only exists while there is something to stop.

        A permanently visible stop button for an idle feature is noise, and
        worse, it trains the operator to read a red button as decoration.
        It and the TRACKING indicator sit in the window header, seen from
        every tab, from START (the node arms for seconds) until the end.
        While tracking IS live the ladder buttons go quiet instead: the node
        owns the equilibrium, so stepping it by hand would fight the stream,
        FLOAT would free the arm under it, and it puts back its own gain
        snapshot on stop.
        """
        if self.tracking or self._track_state() == 'starting':
            tb = self.track_banner()
            text, color = tb[:2] if tb else ('TRACKING', T['good'])
            self.track_ind.config(text=text, bg=color, fg=text_on(color))
            self.track_ind.pack(side='left', padx=(12, 0))
            self.b_track_stop.pack(side='left', padx=(8, 0))
        else:
            self.track_ind.pack_forget()
            self.b_track_stop.pack_forget()
        self.b_track.config(text='3b.  TRACKING...  (the node is driving)'
                            if self.tracking else
                            '3b.  TRACK  (continuous, follows the marker)')
        quiet = 'disabled' if self.tracking or self.busy else 'normal'
        for b in (self.b_float, self.b_track, self.b_minus, self.b_plus,
                  self.b_here, self.b_gains):
            b.config(state=quiet)

    def _set_tracking(self, on):
        """The panel's own start/stop, stamped: a status the node sent
        before it cannot flip it back (see _follow_track_status). Called
        from workers, so the repaint goes to the Tk thread."""
        self.tracking = on
        self.setpoint = None     # the node owns, then re-seeds, the anchor
        self._tracking_at = time.monotonic()
        self.root.after(0, self.paint_tracking)

    def _track_state(self):
        """The node's state if its status is fresh, else None."""
        got = self.n.track_status()
        if got is None or got[1] > TRACK_STATUS_STALE_S:
            return None
        return got[0].get('state')

    def track_banner(self):
        """(STATE, colour, subtitle) from the node's own status, or None
        when it has nothing to add to the ladder's banner."""
        got = self.n.track_status()
        if got is None or got[1] > TRACK_STATUS_STALE_S:
            if not self.tracking:
                return None
            return ('TRACKING?', T['critical'],
                    f'no status from {TRACKING_NODE}'
                    + ('' if got is None else f' for {got[1]:.0f} s')
                    + '  -  is it still running? Press STOP TRACKING')
        f = got[0]
        state, reason = f.get('state'), f.get('reason', '')
        if state == 'tracking':
            return ('TRACKING', T['good'],
                    f'error {f.get("pos_err_mm")} mm / '
                    f'{f.get("rot_err_deg")} deg   lead '
                    f'{f.get("lead_mm")} mm / {f.get("lead_deg")} deg   '
                    f'over lead: {f.get("policy")}  -  STOP TRACKING ends it')
        if state == 'holding':
            return (f'HOLDING - {reason}', T['warning'],
                    'armed, the arm holds still - it follows again by itself '
                    'once clear  -  STOP TRACKING ends it')
        if state == 'starting':
            return ('TRACK STARTING', T['series'],
                    'tool offset, gain profile, settle  -  STOP TRACKING '
                    'aborts it')
        if state == 'idle' and f.get('level', 0) >= 1 and reason:
            return (f'STOPPED - {reason}',
                    T['critical'] if f['level'] >= 2 else T['serious'],
                    'tracking is off; the arm holds where it is on the '
                    'impedance controller  -  TRACK starts it again')
        return None

    def _follow_track_status(self):
        """Tk thread, every tick. The node's latched status is the truth
        about tracking: a tracker found live is adopted (a panel restarted
        mid-run), one the node ended itself - over-lead stop, controller
        transition - is dropped. Never mid-action, and only by a message
        that arrived after the panel's own last start/stop, so a status
        already in flight cannot undo a press."""
        got = self.n.track_status()
        fresh = got is not None and got[1] <= TRACK_STATUS_STALE_S
        key = None
        if fresh:
            f, age = got
            key = (f.get('state'), f.get('reason'), f.get('policy'))
            if key != self._track_seen:
                self._track_seen = key
                self.trace({'rec': 'track_status', **f})
            live = f.get('state') in TRACK_LIVE_STATES
            if (live != self.tracking and not self.busy
                    and time.monotonic() - age > self._tracking_at):
                if live:
                    # the node tracks only on an ACTIVE, holding controller
                    self.active, self.floating = True, False
                    if f.get('policy') in OVER_LEAD_CHOICES:
                        self.v_policy.set(f['policy'])
                    self.say(f'{TRACKING_NODE} is already tracking - adopted;'
                             ' STOP TRACKING is at the top of the window')
                else:
                    self.say(f'{TRACKING_NODE} stopped tracking: '
                             f'{f.get("reason") or f.get("message")}')
                self._set_tracking(live)
        look = (self.tracking, self.busy, key)
        if look != self._painted:
            self._painted = look
            self.paint_tracking()

    def _refuse_while_tracking(self, what):
        """True, and says why, while the node streams: it owns the
        equilibrium, and on stop it restores the gains it snapshotted at
        START - a hand step fights the stream, new gains would be undone."""
        if not self.tracking:
            return False
        self.say(f'REFUSING: {what} while tracking - press STOP TRACKING '
                 'first')
        self.set_status('refused: tracking', T['serious'])
        return True

    def _clear_run_state(self):
        """Forget what this panel was doing. Tracking is NOT cleared here:
        the 50 Hz stream belongs to the tracking node, not to the panel's
        memory, and only STOP TRACKING ends it."""
        self.active, self.floating = False, False
        self.setpoint, self.z_floor = None, None

    def _driver_down(self):
        self.say('DRIVER DOWN - the controller manager is unreachable and '
                 'robot state has stopped. The robot is not being commanded: '
                 'a reflex or crash STOPS it, it does not free it. Relaunch '
                 'the stack, then run PRE-FLIGHT again.')
        self.set_status('DRIVER DOWN - relaunch the stack', T['critical'])
        self._clear_run_state()
        self.preflight_ok = False
        self.trace({'rec': 'driver_down'})

    # ------------------------------------------------------------ ladder

    def preflight(self):
        """Zero the FCI payload and set the collision thresholds, with the
        robot idle as franka requires, and the arm controller restored
        whatever happens.

        The payload itself lives in Desk's end-effector profile, which is
        persistent and is what the robot's own model uses. The FCI load is
        set to zero here so a value left by an earlier session can never be
        added on top of it - the double-count this panel used to invite.
        """
        self.preflight_ok = False
        states = self.n.controllers()
        if states is None:
            self.say('REFUSING: the controller manager is not answering - is '
                     'the stack up?')
            return
        if states.get(IMPEDANCE_CONTROLLER) == 'active':
            self.say('REFUSING: RELEASE the impedance controller before '
                     'PRE-FLIGHT')
            return
        if states.get(ARM_CONTROLLER) != 'active':
            self.say(f'REFUSING: {ARM_CONTROLLER} is not active '
                     f'({states.get(ARM_CONTROLLER)}) - the panel will only '
                     'release what it can restore')
            return
        why = self.blocked()
        if why is not None:
            self.say(f'REFUSING: {why}')
            return
        self.open_trace()
        self.say(f'PRE-FLIGHT: releasing {ARM_CONTROLLER} so the robot goes '
                 'idle - franka accepts these settings only then.')
        load_ok = collision_ok = restored = False
        # Flag BEFORE the call: a release that times out on our side can still
        # complete in the controller manager afterwards.
        self.arm_released = True
        try:
            ok, msg = self.n.switch([], [ARM_CONTROLLER])
            if not ok:
                self.say(f'PRE-FLIGHT aborted: could not release '
                         f'{ARM_CONTROLLER} ({msg})')
                return
            if not self._wait_mode({MODE_IDLE}, PREFLIGHT_MODE_TIMEOUT_S):
                self.say('PRE-FLIGHT aborted: the robot never reported IDLE')
                return
            load_ok, load_msg = self.n.set_load(0.0, [0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0])
            self.say('  FCI payload zeroed (Desk owns the end-effector '
                     'mass): ' + ('set' if load_ok else
                                  f'FAILED ({load_msg})'))
            collision_ok, collision_msg = self.n.set_collision_behavior()
            self.say(f'  collision reflex at {COLLISION_WRENCH[0]:.0f} N / '
                     f'{COLLISION_WRENCH[3]:.0f} Nm Cartesian, contact flag at '
                     f'{CONTACT_WRENCH[0]:.0f} N: ' +
                     ('set' if collision_ok else f'FAILED ({collision_msg})'))
        finally:
            restored = self._restore_arm_controller()
            self.preflight_ok = bool(load_ok and collision_ok and restored)
            self.trace({'rec': 'preflight', 'ok': self.preflight_ok,
                        'mass_kg': 0.0, 'com_m': [0.0, 0.0, 0.0],
                        'load_ok': load_ok, 'collision_ok': collision_ok,
                        'restored': restored,
                        'contact_torque_nm': CONTACT_TORQUE_NM,
                        'collision_torque_nm': COLLISION_TORQUE_NM,
                        'contact_wrench': CONTACT_WRENCH,
                        'collision_wrench': COLLISION_WRENCH})
        if self.preflight_ok:
            st = self.n.state()
            rest = (float('nan') if st is None
                    else float(np.linalg.norm(st[2])))
            self.say(f'PRE-FLIGHT done - the ladder is unlocked. |F ext| at '
                     f'rest {rest:.1f} N' +
                     (' - that bias eats into the reflex margin: check the '
                      'payload before FLOAT' if not rest <= REST_FORCE_WARN_N
                      else '') +
                     f'. Keep hand pushes under {PUSH_LIMIT_N:.0f} N.')
            self.set_status('pre-flight done', T['good'])
            self.trace({'rec': 'preflight_rest_force', 'force_n': rest})
        else:
            self.say('PRE-FLIGHT failed - the ladder stays locked')

    def _restore_arm_controller(self):
        """Bring fr3_arm_controller back, judged by the controller manager's
        answer rather than by how the release call ended. True once it is
        active and the robot is back in MOVE."""
        states = self.n.controllers()
        if states is not None and states.get(ARM_CONTROLLER) == 'active':
            back, back_msg = True, 'already active'
        else:
            back, back_msg = self.n.switch([ARM_CONTROLLER], [])
        restored = back and self._wait_mode({MODE_MOVE}, PREFLIGHT_MODE_TIMEOUT_S)
        if restored:
            self.arm_released = False
            self.say(f'  {ARM_CONTROLLER} active again')
        else:
            self.say(f'*** {ARM_CONTROLLER} did NOT come back '
                     f'({back_msg if not back else "robot not in MOVE"}) '
                     '- relaunch the stack before anything else ***')
            self.set_status('arm controller NOT restored', T['critical'])
        return restored

    def _activate(self, float_mode):
        """Put the impedance controller in charge, in the wanted mode."""
        if not self.preflight_ok:
            self.say('REFUSING: run 0. PRE-FLIGHT first - payload and '
                     'collision thresholds must be set for this driver '
                     'session')
            self.set_status('refused: pre-flight not done', T['serious'])
            return False
        why = self.blocked()
        if why is not None:
            self.say(f'REFUSING: {why}')
            self.set_status(f'refused: {why}', T['serious'])
            return False
        states = self.n.controllers()
        if states is None:
            self.say('REFUSING: the controller manager is not answering')
            return False
        if states.get(IMPEDANCE_CONTROLLER) is None:
            self.say(f'REFUSING: {IMPEDANCE_CONTROLLER} is not loaded - '
                     'fr3_cell.launch.py spawns it inactive; is it running?')
            self.set_status('refused: controller not loaded', T['critical'])
            return False
        # float_mode BEFORE activation: on_activate reads it, and activating
        # in hold mode when you meant float is the wrong surprise.
        ok, msg = self.n.set_params({'float_mode': bool(float_mode)})
        if not ok:
            self.say(f'could not set float_mode: {msg}')
            return False
        if states.get(IMPEDANCE_CONTROLLER) == 'active':
            # already live (e.g. a restarted panel): adopt it, so RELEASE works
            self.active = True
            self.floating = bool(float_mode)
            return True
        ok, msg = self.n.switch([IMPEDANCE_CONTROLLER], [ARM_CONTROLLER])
        if not ok:
            self.say(f'could not activate {IMPEDANCE_CONTROLLER}: {msg}')
            self.set_status('activation failed', T['critical'])
            return False
        self.active = True
        self.floating = bool(float_mode)
        self.setpoint = None
        self.say(f'{IMPEDANCE_CONTROLLER} ACTIVE - the arm is compliant now')
        self.trace({'rec': 'activate', 'float_mode': self.floating})
        return True

    def float_on(self):
        if self._refuse_while_tracking('FLOAT'):     # frees the arm under it
            return
        if not self._activate(True):
            return
        self.say('FLOAT: move the arm gently by hand - smooth, no buzz, no '
                 f'kicks. Keep |F| ext under {PUSH_LIMIT_N:.0f} N. RT pill '
                 'below ~99% is a network/RT problem, not a gain problem; a '
                 'slow sag means the payload is wrong.')
        self.set_status('floating - RT proof', T['series'])
        self.trace({'rec': 'float_on'})

    def hold_on(self):
        if self.active and not self.floating:
            # Already holding. The controller re-seeds only on float -> hold, so
            # nothing would change in the arm - and resetting the floor or the
            # anchor here would let repeated presses walk the floor down and
            # blank the spring-lead readout while the arm is still gliding.
            st = self.n.state()
            if self.z_floor is None and st is not None:
                # holding without a floor: a tracker this panel adopted
                self.z_floor = FLOOR_Z_MM / 1000.0
                self.say(f'already holding - Z floor set '
                         f'{self.z_floor*1000:.0f} mm above the base')
                self.trace({'rec': 'hold_on', 'z_floor': self.z_floor})
                return
            self.say('already holding - nothing to change. To re-seed the '
                     'equilibrium where the arm is, use "hold HERE".')
            return
        if not self.active:
            if not self._activate(False):
                return
        else:
            ok, msg = self.n.set_params({'float_mode': False})
            if not ok:
                self.say(f'float_mode not cleared: {msg}')
                return
            self.floating = False
        self.setpoint = None
        if self.n.state() is not None:
            # Absolute, so no FLOAT -> HOLD cycle can walk it down.
            self.z_floor = FLOOR_Z_MM / 1000.0
        floor = ('--' if self.z_floor is None else
                 f'{self.z_floor*1000:.0f} mm')
        self.say('HOLD: the controller re-seeded its equilibrium where the '
                 'arm is now - it must not move. Push the TCP gently: soft '
                 'sideways, stiffer along tool Z. One small overshoot on '
                 'release is normal; ringing, buzz or drift is not. Keep '
                 f'|F| ext under {PUSH_LIMIT_N:.0f} N. Setpoint floor: {floor}.')
        self.set_status('holding', T['good'])
        self.trace({'rec': 'hold_on', 'gains': self.gains(),
                    'z_floor': self.z_floor})

    def setpoint_step(self, sign):
        if self._refuse_while_tracking('SETPOINT'):
            return
        if not self.active or self.floating:
            self.say('SETPOINT needs the controller holding (step 2 first)')
            return
        if self.z_floor is None:
            self.say('REFUSING: no Z floor - press 2. HOLD first')
            return
        st = self.n.state()
        why = self.blocked()
        if why is not None or st is None:
            self.say(f'REFUSING: {why}')
            return
        pos, quat = st[0], st[1]
        try:
            step = min(float(self.v_step.get()), 50.0) / 1000.0 * sign
        except ValueError:
            self.say('step size is not a number')
            return
        axis = self.v_axis.get()
        if axis.startswith('tool Z'):
            d = q2R(*quat)[:, 2] * step
        else:
            idx = {'base X': 0, 'base Y': 1}.get(axis, 2)
            d = np.zeros(3)
            d[idx] = step
        anchor = (self.setpoint[0] if self.setpoint is not None else pos) + d
        # Only a step that LOWERS the anchor below the floor is refused, so an
        # arm floated down past it can still be stepped back up.
        if anchor[2] < self.z_floor and d[2] < 0.0:
            self.say(f'REFUSING: that puts the equilibrium at z '
                     f'{anchor[2]*1000:.0f} mm, below the floor '
                     f'{self.z_floor*1000:.0f} mm above the base - the camera '
                     'bracket hangs below the flange')
            self.set_status('refused: below Z floor', T['serious'])
            return
        lead = float(np.linalg.norm(anchor - pos))
        if lead * 1000.0 > MAX_LEAD_MM:
            self.say(f'REFUSING: that would put the equilibrium '
                     f'{lead*1000:.0f} mm from the arm (cap {MAX_LEAD_MM:.0f} '
                     'mm) - wait for the arm to catch up')
            self.set_status('refused: equilibrium too far', T['serious'])
            return
        self.setpoint = (anchor, quat)
        self.n.publish_equilibrium(anchor, quat)
        self.say(f'setpoint {step*1000:+.0f} mm along {axis} - lead now '
                 f'{lead*1000:.1f} mm; the arm glides at the slew limit')
        self.trace({'rec': 'setpoint', 'axis': axis, 'step_mm': step * 1000,
                    'anchor': anchor.tolist(), 'lead_mm': lead * 1000})

    def hold_here(self):
        if self._refuse_while_tracking('hold HERE'):
            return
        if not self.active or self.floating:
            self.say('nothing to re-seed: the controller is not holding')
            return
        st = self.n.state()
        if st is None:
            self.say('no robot state')
            return
        self.setpoint = (st[0], st[1])
        self.n.publish_equilibrium(st[0], st[1])
        self.say('equilibrium re-seeded at the current pose (zero lead)')
        self.trace({'rec': 'hold_here', 'anchor': st[0].tolist()})

    def start_tracking(self):
        """Hand the equilibrium to the tracking node, which then streams it
        at 50 Hz. The node checks its own preconditions again and may still
        refuse; these refusals are the ones the operator can act on here."""
        why = self.blocked()
        if why is not None:
            self.say(f'REFUSING: {why}')
            self.set_status(f'refused: {why}', T['serious'])
            return
        if not self.active or self.floating:
            self.say('REFUSING: TRACK needs the controller holding - press '
                     '2. HOLD first. A free-floating arm must not be '
                     'gain-stepped.')
            self.set_status('refused: not holding', T['serious'])
            return
        if self.z_floor is None:
            self.say('REFUSING: no Z floor - press 2. HOLD first')
            self.set_status('refused: no Z floor', T['serious'])
            return
        # The node holds the CAMERA where ALIGN would leave it, so it gets
        # ALIGN's goal first, all or nothing: half a goal is a wrong goal.
        # In-plane 'off' holds whatever angle the node sees at START.
        tgt_ip = self.align.inplane_target()
        goal = {'tracking_standoff_m': self.align.target_m(),
                'tracking_inplane_hold': tgt_ip is None,
                'tracking_over_lead_policy': self.v_policy.get()}
        if tgt_ip is not None:
            goal['tracking_inplane_deg'] = float(tgt_ip)
        ok, msg = self.n.set_tracking_params(goal)
        self.trace({'rec': 'track_goal', 'ok': ok, 'msg': msg, **goal})
        if not ok:
            self.say(f'REFUSING: could not hand the goal to {TRACKING_NODE} '
                     f'({msg})')
            self.set_status(f'refused: {msg}', T['serious'])
            return
        ok, msg = self.n.call_trigger(self.n.track_start_cli)
        if ok:
            self._set_tracking(True)
            self.say(f'TRACKING: {msg} - the arm follows the marker from '
                     'now on. STOP TRACKING is live at the top.')
            self.set_status('tracking', T['good'])
        else:
            self.say(f'tracking NOT started: {msg}')
            self.set_status(f'refused: {msg}', T['serious'])
        self.trace({'rec': 'track_start', 'ok': ok, 'msg': msg})

    def stop_tracking_now(self):
        """What STOP TRACKING is wired to: off the Tk thread so the live view
        keeps running, and outside go() so no other action can delay it or
        grey it out."""
        threading.Thread(target=self.stop_tracking, daemon=True).start()

    def stop_tracking(self):
        """Never refuses, whatever the panel thinks is going on. The node
        stops publishing, re-seeds the equilibrium where the arm is and puts
        back the gains it changed; the arm stays compliant and still held."""
        ok, msg = self.n.call_trigger(self.n.track_stop_cli)
        self._set_tracking(False)
        self.say(f'STOP TRACKING: {msg}')
        if ok:
            self.set_status('tracking stopped')
        else:
            self.set_status(f'stop tracking: {msg}', T['serious'])
        self.trace({'rec': 'track_stop', 'ok': ok, 'msg': msg})

    def _on_policy_selected(self, _evt=None):
        """Tk thread, the dropdown's handler: read the choice here, where Tk
        variables may be read, and write it from a worker."""
        threading.Thread(target=self.write_policy,
                         args=(self.v_policy.get(),), daemon=True).start()

    def write_policy(self, v=None):
        """Off the Tk thread. Live at any time - START sends it again with
        the goal anyway. Refused by a node that is there, the dropdown goes
        back to the policy the node reports, not the one it does not run."""
        v = self.v_policy.get() if v is None else v
        ok, msg = self.n.set_tracking_params(
            {'tracking_over_lead_policy': v}, atomic=False)
        got = None if ok else self.n.track_status()
        has = None
        if (got is not None and got[1] <= TRACK_STATUS_STALE_S
                and got[0].get('policy') in OVER_LEAD_CHOICES):
            has = got[0]['policy']
            self.root.after(0, self.v_policy.set, has)
        self.say(f'over-lead policy -> {v}'
                 + ('' if ok else f': NOT written ({msg}) - '
                    + (f'the node keeps {has}' if has
                       else 'TRACK sends it at start')))
        self.trace({'rec': 'track_policy', 'policy': v, 'ok': ok,
                    'msg': msg})

    def release(self):
        """Hand the arm back, deciding from the controller manager's state.

        Tracking is stopped first on the path that actually releases: RELEASE
        means the arm is no longer being driven, and that must include the
        50 Hz stream.
        """
        if not self.n.cm_reachable():
            if self._state_age() > DRIVER_DOWN_S:
                self._driver_down()
                self.close_trace()
            else:
                self.say('the controller manager is not answering - press '
                         'RELEASE again in a moment')
            return
        states = self.n.controllers()
        if states is None:
            self.say('the controller manager did not answer - press RELEASE '
                     'again')
            return
        if states.get(IMPEDANCE_CONTROLLER) != 'active':
            self.say('impedance controller is not active'
                     + (' - panel state corrected' if self.active else ''))
            self._clear_run_state()
            return
        t_ok, t_msg = self.n.call_trigger(self.n.track_stop_cli)
        if t_ok:
            self.say(f'tracking stopped first: {t_msg}')
        self._set_tracking(False)
        ok, msg = self.n.switch([ARM_CONTROLLER], [IMPEDANCE_CONTROLLER])
        if not ok:
            self.say(f'*** RELEASE FAILED ({msg}) - the arm is still on the '
                     'impedance controller; use the robot E-stop if it is '
                     'not behaving ***')
            self.set_status('RELEASE FAILED', T['critical'])
            self.trace({'rec': 'release', 'ok': False, 'msg': msg})
            return
        self._clear_run_state()
        self.say(f'released - {ARM_CONTROLLER} holds the arm again')
        self.set_status('released', T['muted'])
        self.trace({'rec': 'release', 'ok': True})
        self.close_trace()

    def apply_gains(self):
        if self._refuse_while_tracking('apply gains'):
            return
        g = self.gains()
        if g is None:
            self.say('REFUSING: gains must be numbers')
            return
        err = self.gain_error(g)
        if err is not None:
            self.say(f'REFUSING: {err} - zeta 0 is an undamped spring, and '
                     'the force ceiling does not stop oscillation')
            self.set_status('refused: gain out of range', T['serious'])
            return
        ok, msg = self.n.set_params({
            'k_pos_tool': [g['k_xy'], g['k_xy'], g['k_z']],
            'k_rot_tool': [g['k_rp'], g['k_rp'], g['k_yaw']],
            'damping_ratio': g['zeta']})
        self.say(f'gains {"applied" if ok else "REFUSED"}: k_pos '
                 f'[{g["k_xy"]:.0f} {g["k_xy"]:.0f} {g["k_z"]:.0f}] N/m, '
                 f'k_rot [{g["k_rp"]:.0f} {g["k_rp"]:.0f} {g["k_yaw"]:.0f}] '
                 f'Nm/rad, zeta {g["zeta"]:.2f}'
                 + ('' if ok else f' ({msg})'))
        self.trace({'rec': 'gains', 'ok': ok, **g})

    # ------------------------------------------------------------ live view

    def refresh(self):
        self._update_image()
        self._follow_track_status()
        st = self.n.state()
        if st is None or st[5] > DRIVER_DOWN_S:
            if not self.driver_down_logged:
                if not self.n.cm_reachable():
                    if self.preflight_ok or self.active:
                        self.say('robot state stopped and the controller '
                                 'manager does not answer - the driver is '
                                 'down. Relaunch the stack, then run '
                                 'PRE-FLIGHT again.')
                    self.preflight_ok = False
                    self.driver_down_logged = True
                elif self.active:
                    self.say('robot state stopped but the controller manager '
                             'still answers - the state relay may be stuck. '
                             'Impedance may be live: press RELEASE. Do not '
                             'relaunch while the controller manager is alive.')
                    self.driver_down_logged = True
        else:
            self.driver_down_logged = False
        if st is None or st[5] > STATE_STALE_S:
            for v in self.tiles.values():
                v.config(text='--', fg=T['critical'])
            self.sub.config(text='no robot state')
        else:
            pos, quat, force, mode, rate, _ = st
            fn = float(np.linalg.norm(force))
            lead = (0.0 if self.setpoint is None
                    else float(np.linalg.norm(self.setpoint[0] - pos)) * 1000)
            for key, val, col in (
                    ('TCP Z', f'{pos[2]*1000:.1f}', T['ink']),
                    ('|F| ext', f'{fn:.1f}',
                     T['warning'] if fn > PUSH_LIMIT_N else T['ink']),
                    ('Fz ext', f'{force[2]:+.1f}', T['ink']),
                    ('spring lead', f'{lead:.1f}',
                     T['warning'] if lead > 20 else T['ink'])):
                self.tiles[key].config(text=val, fg=col)
            floor = ('--' if self.z_floor is None else
                     f'{self.z_floor*1000:.0f} mm')
            self.sub.config(text=(
                f'TCP  x {pos[0]*1000:8.1f}   y {pos[1]*1000:8.1f}   '
                f'z {pos[2]*1000:8.1f} mm   Z floor {floor}\n'
                f'robot {ROBOT_MODES.get(mode, mode)}   '
                f'control success {rate*100:.1f}%   '
                f'external force [{force[0]:+.1f} {force[1]:+.1f} '
                f'{force[2]:+.1f}] N'))
        self.draw_pills()
        self.draw_banner()

    def pill_states(self):
        st = self.n.state()
        if st is None or st[5] > STATE_STALE_S:
            robot, rt = ('no state', T['critical']), ('--', T['muted'])
        else:
            robot = (ROBOT_MODES.get(st[3], str(st[3])),
                     T['good'] if st[3] == MODE_MOVE
                     else T['critical'] if st[3] == MODE_REFLEX
                     else T['warning'])
            rate = st[4] * 100.0
            rt = (f'{rate:.0f}%', T['good'] if rate >= 99.0
                  else T['warning'] if rate >= 95.0 else T['critical'])
        pre = ('done', T['good']) if self.preflight_ok else \
            ('needed', T['warning'])
        if self.active:
            ctrl = ('FLOAT', T['series']) if self.floating else \
                ('impedance', T['good'])
        else:
            ctrl = ('arm controller', T['muted'])
        return [('FRANKA',) + robot, ('RT',) + rt, ('PRE-FLIGHT',) + pre,
                ('CONTROL',) + ctrl]

    def banner_state(self):
        if self.active and self._state_age() > DRIVER_DOWN_S:
            if not self.n.cm_reachable():
                return ('DRIVER DOWN', T['critical'],
                        'robot state stopped and the controller manager is '
                        'gone - the robot is not being commanded; relaunch '
                        'the stack, then PRE-FLIGHT')
            return ('NO ROBOT STATE', T['critical'],
                    'state relay silent but the controller manager answers - '
                    'impedance may still be live: press RELEASE')
        why = self.blocked()
        # A live tracker outranks the ladder's view, a robot fault outranks
        # both; the node's STOPPED only replaces the plain HOLDING below.
        track = self.track_banner()
        if track is not None and why is None and self._track_state() != 'idle':
            return track
        if not self.active:
            if why:
                return ('NOT READY', T['critical'], why)
            if not self.preflight_ok:
                return ('PRE-FLIGHT NEEDED', T['warning'],
                        'press 0. PRE-FLIGHT - it sets the collision '
                        'reflex thresholds and zeroes the FCI payload')
            return ('INACTIVE', T['grid'],
                    'arm on fr3_arm_controller  -  press 1. FLOAT to start '
                    'the ladder')
        if why:
            return ('CHECK THE ROBOT', T['critical'],
                    f'{why}  -  press RELEASE')
        if self.floating:
            return ('FLOATING', T['series'],
                    'arm is free under gravity compensation  -  move it by '
                    'hand, then press 2. HOLD')
        lead = 0.0
        st = self.n.state()
        if self.setpoint is not None and st is not None:
            lead = float(np.linalg.norm(self.setpoint[0] - st[0])) * 1000
        if lead > 1.0:
            return ('MOVING', T['warning'],
                    f'equilibrium {lead:.1f} mm from the arm  -  gliding at '
                    'the slew limit')
        return track or ('HOLDING', T['good'],
                         'compliant hold  -  push the TCP gently to feel the '
                         'spring')


class CellPanel:
    """The window. Owns the chrome; the panes own the tasks.

    Left column, always visible: the camera, the convergence history and the
    log - both tasks want to see the marker and what just happened. Right
    column: one tab per task. The status strip, the pills and the banner
    always describe the ACTIVE tab, so there is never a question of which
    task the warning belongs to.
    """

    def __init__(self, node):
        self.n = node
        self.busy = False
        self.abort = False
        self.stopped = False
        self.last_outcome = None
        self.tracef = None
        self.tracepath = None
        self.photo = None
        self._image_shown = None
        self._closing = False
        self._built = False          # panes exist; active() is safe to call
        self._trace_lock = threading.Lock()
        self._tab_name = '?'         # set by the Tk thread; read by trace()

        self.root = tk.Tk()
        self.root.title('FR3 Cell Control  -  align, impedance, track')
        self.root.configure(bg=T['page'])
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        base = tkfont.nametofont('TkDefaultFont').actual()['family']
        self.f_caption = (base, 9)
        self.f_label = (base, 10)
        self.f_value = ('DejaVu Sans Mono', 15, 'bold')
        self.f_small = ('DejaVu Sans Mono', 9)
        self.f_button = (base, 10, 'bold')
        self.f_banner = (base, 18, 'bold')
        self.f_pill = tkfont.Font(family=base, size=9)
        self.f_pill_b = tkfont.Font(family=base, size=9, weight='bold')
        self._init_combo_style()

        outer = tk.Frame(self.root, bg=T['page'])
        outer.pack(fill='both', expand=True, padx=16, pady=12)
        head = tk.Frame(outer, bg=T['page'])
        head.pack(fill='x')
        tk.Label(head, text='FR3 CELL CONTROL', font=(base, 10, 'bold'),
                 fg=T['muted'], bg=T['page'], anchor='w').pack(side='left')
        # Tracking is shown and stopped here, above the tabs: the node drives
        # the arm whichever tab is on top. Both are packed by
        # LadderPane.paint_tracking only while there is something to stop.
        # STOP is outside go(): a stop that greys out while another action
        # runs is not a stop control, so it is never disabled or queued
        # behind self.busy.
        self.track_ind = tk.Label(head, text='', font=self.f_button, padx=10,
                                  pady=3)
        self.b_track_stop = self._button(
            head, 'STOP TRACKING', T['critical'], self.header_stop_tracking,
            pady=3)
        self.pills = tk.Canvas(head, height=26, width=10, bg=T['page'],
                               highlightthickness=0)
        self.pills.pack(side='right')
        self.banner = tk.Canvas(outer, height=64, bg=T['page'],
                                highlightthickness=0)
        self.banner.pack(fill='x', pady=(6, 10))

        body = tk.Frame(outer, bg=T['page'])
        body.pack(fill='both', expand=True)
        left = tk.Frame(body, bg=T['page'])
        left.pack(side='left', fill='both', expand=True)
        right = tk.Frame(body, bg=T['page'])
        right.pack(side='left', fill='y', padx=(12, 0))

        self.tabs = ttk.Notebook(right)
        self.tabs.pack(fill='both', expand=True)
        align_tab = tk.Frame(self.tabs, bg=T['page'])
        ladder_tab = tk.Frame(self.tabs, bg=T['page'])
        self.tabs.add(align_tab, text='  ALIGN  ')
        self.tabs.add(ladder_tab, text='  IMPEDANCE & TRACK  ')
        self.tabs.bind('<<NotebookTabChanged>>', self._on_tab_change)

        self.align = AlignPane(self, node, align_tab)
        self.ladder = LadderPane(self, node, ladder_tab)
        self.panes = (self.align, self.ladder)

        self._build_camera(left)
        self.align.build_plots(left)
        self._build_log(left)
        self.align.build(align_tab)
        self.ladder.build(ladder_tab)

        self.status = tk.Label(outer, text='ready', anchor='w', fg=T['muted'],
                               bg=T['page'], font=self.f_caption)
        self.status.pack(fill='x', pady=(6, 0))
        self._built = True
        self._tab_name = self.active().NAME

        node.on_mode_change = self.align._on_mode_change
        node.on_sample = self.ladder._sample
        self.say('Ready. ALIGN drives the camera; IMPEDANCE & TRACK commands '
                 'torque - run 0. PRE-FLIGHT there first.')
        if node.relay_error:
            self.say(f'WARNING: robot state relay failed ({node.relay_error})')
        self.root.after(1500, self.align._autoload)
        self.align.relabel()
        self.tick()

    # ------------------------------------------------------------ tabs

    def active(self):
        """The pane the operator is looking at."""
        return self.panes[self.tabs.index(self.tabs.select())]

    def _on_tab_change(self, _evt=None):
        """Tk thread: cache the tab name that trace() stamps on records."""
        self._tab_name = self.active().NAME
        self.set_status(f'{self._tab_name} tab')

    def go(self, fn, *a):
        """Dispatch to the active pane. Each pane keeps its own preconditions
        - ALIGN checks the robot-state gate, the ladder checks pre-flight -
        so neither inherits rules written for the other."""
        self.active().go_impl(fn, *a)

    def header_stop_tracking(self):
        """The header's STOP TRACKING. Never through go(): that drops a press
        while busy and hands it to the tab on top, whose own guards - ALIGN's
        torque interlock - would refuse the stop itself. The Trigger runs
        off the Tk thread, so the window stays live while the node answers."""
        self.ladder.stop_tracking_now()

    # ------------------------------------------------------------ chrome

    def _init_combo_style(self):
        st = ttk.Style()
        try:
            st.theme_use('clam')
        except tk.TclError:
            pass
        st.configure('Dark.TCombobox', fieldbackground=T['surface'],
                     background=T['grid'], foreground=T['ink'],
                     arrowcolor=T['ink2'], bordercolor=T['grid'],
                     lightcolor=T['grid'], darkcolor=T['grid'])
        st.map('Dark.TCombobox',
               fieldbackground=[('readonly', T['surface'])],
               foreground=[('readonly', T['ink'])])
        st.configure('TNotebook', background=T['page'], borderwidth=0)
        st.configure('TNotebook.Tab', background=T['page'], foreground=T['muted'],
                     padding=(14, 7), borderwidth=0)
        st.map('TNotebook.Tab', background=[('selected', T['surface'])],
               foreground=[('selected', T['ink'])])
        for k, v in (('background', T['surface']), ('foreground', T['ink']),
                     ('selectBackground', T['series']),
                     ('selectForeground', T['ink'])):
            self.root.option_add(f'*TCombobox*Listbox.{k}', v)

    def _card(self, parent, title):
        frame = tk.Frame(parent, bg=T['surface'],
                         highlightbackground=T['grid'], highlightthickness=1)
        tk.Label(frame, text=title, font=self.f_caption, fg=T['muted'],
                 bg=T['surface'], anchor='w').pack(fill='x', padx=10,
                                                   pady=(8, 6))
        return frame

    def _button(self, parent, text, color, cmd, font=None, pady=9):
        fg = text_on(color)
        b = tk.Button(parent, text=text, font=font or self.f_button, bg=color,
                      fg=fg, activebackground=mix(color, '#ffffff', .15),
                      activeforeground=fg, relief='flat', bd=0,
                      highlightthickness=0, cursor='hand2', padx=12,
                      pady=pady, command=cmd, disabledforeground=T['muted'])
        return b

    def _entry(self, parent, var, width):
        return tk.Entry(parent, textvariable=var, width=width,
                        font=self.f_small, bg=T['page'], fg=T['ink'],
                        insertbackground=T['ink'], relief='flat',
                        highlightthickness=1, highlightbackground=T['grid'],
                        highlightcolor=T['series'])

    def _combo(self, parent, var, vals, width=6):
        return ttk.Combobox(parent, textvariable=var, values=vals,
                            width=width, state='readonly',
                            style='Dark.TCombobox', font=self.f_small)

    def _row(self, parent, label, var, vals, r, c):
        lbl = tk.Label(parent, text=label, font=self.f_caption, fg=T['ink2'],
                       bg=T['surface'], anchor='e')
        lbl.grid(row=r, column=c * 2, padx=(10, 4), pady=3, sticky='e')
        # size to the longest choice so it fits
        width = max(6, max(len(str(v)) for v in vals) + 1)
        cb = self._combo(parent, var, vals, width=width)
        cb.grid(row=r, column=c * 2 + 1, padx=(0, 10), pady=3, sticky='w')
        return lbl, cb

    def _build_log(self, parent):
        card = self._card(parent, 'LOG')
        card.pack(fill='both', expand=True, pady=(12, 0))
        self.log = scrolledtext.ScrolledText(
            card, height=14, width=64, font=self.f_small, bg=T['page'],
            fg=T['ink2'], insertbackground=T['ink'], relief='flat', bd=0,
            highlightthickness=0)
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))

    def _build_camera(self, parent):
        """Shared by both tasks, and ON by default - ALIGN is useless without
        it, and this is what the proven alignment runs of 2026-09-11 and
        09-15 used.

        The toggle stays because the subscription is what costs anything:
        /aruco/debug_image is subscribe-gated at the publisher, so while
        nothing subscribes the frames are never encoded and never reach DDS.
        Turn it off if you ever want the quietest possible graph during
        torque work - but note the overlay is capped at debug_max_hz (5 Hz)
        on loopback, and none of the three stack deaths on 2026-09-16/22 was
        traced to it. Two were the laptop on battery; one correlated with the
        1 kHz state relay, which is a different subscription entirely.
        """
        card = self._card(parent, f'CAMERA  |  {IMAGE_TOPIC}')
        card.pack(fill='x')
        row = tk.Frame(card, bg=T['surface'])
        row.pack(fill='x', padx=10, pady=(0, 6))
        self.v_cam = tk.BooleanVar(value=True)
        tk.Checkbutton(
            row, text='show the marker view', variable=self.v_cam,
            command=self.toggle_camera, font=self.f_caption, bg=T['surface'],
            fg=T['ink2'], selectcolor=T['page'], activebackground=T['surface'],
            activeforeground=T['ink'], highlightthickness=0, bd=0).pack(
                side='left')
        tk.Label(row, text='off = no frames on DDS', font=self.f_small,
                 bg=T['surface'], fg=T['muted']).pack(side='right')
        self.image_label = tk.Label(
            card, bg=T['page'], fg=T['muted'], font=self.f_small, height=3,
            text='no frames yet - is the vision stack running?\n'
                 'terminal 2: fr3_cell  (tools/fr3/fr3_cell.launch.py)')
        self.image_label.pack(fill='x', padx=10, pady=(0, 10))
        self.n.camera(True)          # match the checkbox we just drew

    def toggle_camera(self):
        on = self.n.camera(bool(self.v_cam.get()))
        self._image_shown = None
        self.photo = None
        self.image_label.config(
            image='', height=3,
            text=('no frames yet - is the vision stack running?' if on else
                  'camera off - nothing is encoded or put on DDS'))
        self.trace({'rec': 'camera', 'on': on})

    def _update_image(self):
        if not self.v_cam.get():
            return
        got = self.n.image()
        if got is None or got[0] == self._image_shown:
            return
        seq, img = got
        hgt, wid = img.shape[:2]
        header = f'P6 {wid} {hgt} 255 '.encode()
        self.photo = tk.PhotoImage(data=header + img.tobytes())
        self.image_label.config(image=self.photo, text='', height=hgt)
        self._image_shown = seq

    # ------------------------------------------------------------ output

    def say(self, m):
        stamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log.insert('end', f'{stamp}  {m}\n')
        self.log.see('end')

    def set_status(self, m, color=None):
        self.status.config(text=m, fg=color or T['muted'])

    def trace(self, rec):
        """One trace per session, whichever tab wrote it. Called from worker,
        Tk and spin threads."""
        with self._trace_lock:
            if self.tracef is None:
                return
            rec['t'] = time.time()
            # Which tab wrote it: the name the Tk thread cached. Asking the
            # notebook here would be a Tk call under _trace_lock from the
            # spin thread, which deadlocks against a Tk-thread trace().
            rec.setdefault('tab', getattr(self, '_tab_name', '?'))
            try:
                self.tracef.write(json.dumps(rec, default=float) + '\n')
                self.tracef.flush()
            except Exception as e:                          # noqa: BLE001
                print(f'cell_panel: trace write failed: {e}', file=sys.stderr)

    def open_trace(self, header=None):
        with self._trace_lock:
            if self.tracef is not None:
                return
            folder = trace_dir()
            folder.mkdir(parents=True, exist_ok=True)
            name = ('cell_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    + '.jsonl')
            self.tracepath = folder / name
            self.tracef = self.tracepath.open('w')
        self.trace({'rec': 'session_start', **(header or {})})
        self.say(f'trace -> {self.tracepath}')

    def close_trace(self, outcome=None):
        self.trace({'rec': 'session_end', 'outcome': outcome})
        with self._trace_lock:
            if self.tracef is not None:
                self.tracef.close()
                self.tracef = None

    # ------------------------------------------------------------ live view

    def tick(self):
        # Reschedule FIRST: tkinter swallows exceptions raised in callbacks,
        # so a tick that re-armed itself at the end would stop the live view
        # for good on one error, leaving a stale banner that looks live.
        self.root.after(150, self.tick)
        try:
            self._update_image()
            for p in self.panes:
                p.refresh()
            self.draw_pills()
            self.draw_banner()
        except Exception as e:                              # noqa: BLE001
            print(f'cell_panel: live view update failed: {e}', file=sys.stderr)

    def draw_pills(self):
        """The ACTIVE tab's pills - (label, value, colour) triples, the shape
        both panels already used."""
        c = self.pills
        c.delete('all')
        items = []
        for label, value, col in self.active().pill_states():
            wl, wv = self.f_pill.measure(label), self.f_pill_b.measure(value)
            items.append((label, value, col, wl, 26 + wl + 6 + wv + 12))
        total = sum(it[-1] for it in items) + 8 * (len(items) - 1) + 2
        if int(c['width']) != total:
            c.config(width=total)
        x = 1
        for label, value, col, wl, w in items:
            rounded_rect(c, x, 2, x + w, 24, 11, fill=T['surface'],
                         outline=mix(col, T['surface'], 0.55))
            c.create_oval(x + 11, 9, x + 19, 17, fill=col, outline='')
            c.create_text(x + 26, 13, text=label, anchor='w',
                          font=self.f_pill, fill=T['muted'])
            c.create_text(x + 26 + wl + 6, 13, text=value, anchor='w',
                          font=self.f_pill_b,
                          fill=T['ink2'] if col == T['muted'] else col)
            x += w + 8

    def draw_banner(self):
        """The ACTIVE tab's one-glance answer to 'what now?'."""
        c = self.banner
        c.delete('all')
        w = max(c.winfo_width(), 400)
        state, color, sub = self.active().banner_state()
        rounded_rect(c, 0, 0, w, 62, 12, fill=color, outline='')
        ink = text_on(color)
        c.create_text(20, 24, text=state, anchor='w', font=self.f_banner,
                      fill=ink)
        c.create_text(20, 49, text=sub, anchor='w', font=self.f_caption,
                      fill=mix(ink, color, 0.3))

    # ------------------------------------------------------------ exit

    def install_signal_handlers(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._on_signal)

    def _on_signal(self, _signum, _frame):
        # Launch follows the terminal's Ctrl-C with SIGTERM; once on_close
        # has destroyed the root there is nothing left to close.
        if self._closing:
            return
        self.root.after(0, self.on_close)

    def on_close(self):
        """Never leave the arm on the impedance controller, and never leave a
        run mid-motion."""
        if self.busy:
            self.say('an action is still running - close again once it has '
                     'finished, so the arm is never left mid-switch')
            return
        self.busy = True
        result = release_if_active(self.n, self.say)
        if result is False or (result is None
                               and self.ladder._state_age() <= DRIVER_DOWN_S):
            self.say('*** cannot confirm the impedance controller is released '
                     '- NOT closing. Press RELEASE, or use the E-stop ***')
            self.busy = False
            return
        self.busy = False
        self.close_trace()
        self._closing = True
        self.root.destroy()


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


def main():
    # rclpy's own signal handling is off: Ctrl-C must reach CellPanel.on_close,
    # which needs ROS alive to hand the arm back.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CellNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    panel = None
    try:
        panel = CellPanel(node)
        panel.install_signal_handlers()
        panel.root.mainloop()
    finally:
        shutdown_ros(node, panel, executor, spin)


if __name__ == '__main__':
    main()
