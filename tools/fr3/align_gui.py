#!/usr/bin/env python3
"""FR3 camera <-> marker alignment dashboard.

Drives the wrist camera to a target pose above an ArUco marker: a set
standoff along the optical axis, the camera square to the marker, and the
marker's X axis at a chosen in-plane angle. Every motion needs a button
press; nothing moves on its own.

Calibration: the camera->TCP rotation comes from roscam.handeye_calib
(tools/fr3/handeye_rotation.json) and is re-based to the current arm pose on
load. A position servo needs only that rotation - the camera->TCP
translation cancels out of the control law:

    camera moves by d (base)  =>  marker shifts by -R @ d (camera frame),
    R = R_cam_base.  To drive the marker to target t:  d = R.T @ (p - t)

Backends:
  cartesian  plan + execute one clamped step at a time through move_group
  servo      stream velocity = gain * error to moveit_servo, so the motion
             is one continuous coarse-to-fine move. The arm is handed to a
             joint-position controller for the run and handed back after
             (needs tools/fr3/fr3_servo.launch.py)

Safety: whole-arm Z floor on both backends (every link but the bolted
base), marker-loss deadman, clamped steps and capped velocities, and STOP NOW
halts either backend mid-motion. PAUSE holds a run where it is and RESUME
continues it from rest. The robot-state gate (on by default) also pauses
whenever the robot leaves MOVE, and a REFLEX ends the run. None of it replaces
the hardware E-stop or the enabling device.

Run (vision publishing /aruco/pose, MoveIt up, servo launch for 'servo'):

    python3 tools/fr3/align_gui.py
"""

import collections
import datetime
import json
import pathlib
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

# Shared dark-dashboard palette - single source of truth is the operator
# panel, so the two GUIs cannot drift apart.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'gui'))
from mating_panel import (THEME as T, mix, rounded_rect,  # noqa: E402
                          text_on)

from state_relay import start_throttle, stop_throttle   # noqa: E402

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

# ---- motion backend ------------------------------------------------------
# 'cartesian' plans and executes each clamped step through move_group: safe
# and pre-checked, but every step accelerates from rest and stops again.
# 'servo' streams a velocity proportional to the error, so the motion is
# continuous and naturally coarse-then-fine - one move, no stop-start.
BACKEND_CHOICES = ['cartesian', 'servo']
BACKEND_DEFAULT = 'cartesian'

# Servo speed profiles. Velocity = gain * error, clamped to the caps. The
# FR3 itself permits 3000 mm/s, so every profile here is far below the
# hardware limit; these are chosen for watchability and reaction time.
SERVO_PROFILES = {
    'conservative': {'lin': 0.030, 'ang': 15.0, 'gain': 0.8},
    'moderate':     {'lin': 0.060, 'ang': 25.0, 'gain': 1.2},
    'brisk':        {'lin': 0.120, 'ang': 40.0, 'gain': 1.5},
}
SERVO_PROFILE_DEFAULT = 'moderate'
SERVO_RATE_HZ = 30.0      # must be well inside incoming_command_timeout

# Controllers. franka_hardware allows exactly ONE command mode at a time:
# planned moves run on the effort trajectory controller, the servo stream on
# a joint-position controller. Streaming into the trajectory controller is
# what stalled the arm and made it buzz (see fr3_servo_controllers.yaml), so
# a servo run swaps controllers and always swaps back.
ARM_CONTROLLER = 'fr3_arm_controller'
SERVO_CONTROLLER = 'fr3_servo_position_controller'
CTRL_SWITCH_TIMEOUT_S = 5.0
# Gap between releasing one controller and claiming the next, so the old
# command mode has actually stopped (see switch_controllers).
CTRL_SWITCH_SETTLE_S = 0.3
SERVO_SETTLE_S = 0.6      # zero-twist hold before handing the arm back

# Command shaping for the servo stream (see CommandShaper). These are the
# values validated against a sim using the noise measured on this cell and
# 80 ms latency. Sending the raw command instead is what shook the arm into
# cartesian_reflex on the brisk profile.
SHAPER_ALPHA = 0.35          # command low-pass, ~2.3 Hz at 30 Hz
SHAPER_LIN_ACC = 0.25        # m/s^2   max change of the linear command
SHAPER_ANG_ACC_DEG = 40.0    # deg/s^2 max change of the angular command
DEAD_TILT_DEG = 0.4          # orientation deadband near the target, so the
DEAD_IP_DEG = 0.3            # measurement noise cannot dither the wrist
OSC_WINDOW = 30              # frames (1 s) the oscillation watchdog watches
OSC_FLIP_LIMIT = 0.25        # abort if >25% of those frames reverse
OSC_EPS_LIN = 0.001          # m/s   reversals below this are noise, ignored
OSC_EPS_ANG = float(np.radians(0.3))

# ---- robot-state gate ---------------------------------------------------------
# Motion only while the robot reports MOVE. Anything else (user stop,
# hand-guiding, IDLE) pauses the run, which resumes from rest once the robot is
# back in MOVE; a REFLEX ends it. Fail-closed: with the gate on, no fresh robot
# state means no motion.
# This gate cannot see the enabling device: measured 2026-09-15, holding and
# releasing this cell's enabling device changed no robot-state field at all,
# so software cannot see it over FCI.
ROBOT_STATE_TOPIC = '/franka_robot_state_broadcaster/robot_state'
ROBOT_STATE_RELAY = '/align_gui/robot_state'
# Decoding the 1 kHz state in Python costs 86% of a core, raw callbacks 31%
# (measured) - enough to starve this GUI. A C++ topic_tools throttle child
# relays it at 50 Hz for 5%, and 20 ms is ample: the robot stops itself.
ROBOT_STATE_RELAY_HZ = 50
ROBOT_STATE_STALE_S = 0.3
GATE_DEFAULT = True
GATE_RESUME_HOLD_S = 0.5     # held this long, unbroken, before motion resumes
MODE_MOVE, MODE_REFLEX, MODE_USER_STOPPED = 2, 4, 5
ROBOT_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX',
               5: 'USER_STOPPED', 6: 'ERROR_RECOVERY'}
GATE_REASONS = {0: 'robot mode OTHER', 1: 'robot IDLE - no control loop',
                3: 'hand-guiding', 5: 'robot USER_STOPPED (user stop)',
                6: 'automatic error recovery'}

# ---- servo stall watchdog --------------------------------------------------------
# Measured 2026-09-15: conservative servo commanded ~10 mm/s and 7 deg/s for
# 90 s while the arm did not move at all (error flat at 12.7 mm / 8.8 deg).
# Abort when commanded motion produces no progress for this long.
SERVO_STALL_S = 4.0
SERVO_STALL_MM = 0.5         # progress that counts: position ...
SERVO_STALL_DEG = 0.3        # ... or tilt / in-plane

# What to do once converged. 'hold' stops the stream and leaves the arm
# idle; 'station-keep' keeps correcting, which also tracks a marker that
# moves - but leaves the robot live until stopped.
CONVERGE_CHOICES = ['hold', 'station-keep']
CONVERGE_DEFAULT = 'hold'

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
FLOOR_MM_DEFAULT = '50'   # whole-arm Z floor, base frame

# ---- dashboard -----------------------------------------------------------
IMAGE_MAX_W = 640         # native D405 width, so no downscale at 640x480
PLOT_WINDOW_S = 30.0      # seconds of convergence history shown

# Saved calibration. Holds the camera->TCP ROTATION, which is a rigid
# mounting property: unlike R_cam_base it does not change when the robot
# moves, so it stays valid across sessions and arm poses.
CALIB_PATH = pathlib.Path(__file__).with_name('handeye_rotation.json')

# Per-iteration JSONL trace of auto-converge: what was measured, what was
# commanded, what the robot actually did. One file per run.
LOG_DIR = pathlib.Path(__file__).with_name('logs')


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


def clamp_norm(v, lim):
    """Scale v down to norm lim, preserving its direction."""
    n = float(np.linalg.norm(v))
    return v * (lim / n) if n > lim and n > 1e-12 else v


class CommandShaper:
    """Makes the servo velocity stream smooth enough for the FR3.

    The raw gain*error command is rough. Measured on this cell, the angular
    command reversed sign on 13% of frames with single-frame jumps of
    11.7 deg/s - orientation noise, amplified because the tilt direction is
    ill-conditioned at small tilt - and brisk added 27.8 mm/s linear jumps
    that shook the arm into cartesian_reflex. A sim with the measured noise
    and 80 ms latency reproduces the 11.5 deg/s jumps, and this shaper cuts
    them 6-16x without slowing convergence.

      filter()       low-pass the command, and zero orientation inside a
                     deadband near target so noise cannot dither the wrist
      limit()        acceleration limit: every change is a ramp, including
                     the first one out of standstill and any resume
      oscillating()  watchdog: stop before libfranka does if the shaped
                     command keeps reversing
    """

    def __init__(self, alpha=SHAPER_ALPHA, lin_acc=SHAPER_LIN_ACC,
                 ang_acc_deg=SHAPER_ANG_ACC_DEG, dead_tilt_deg=DEAD_TILT_DEG,
                 dead_ip_deg=DEAD_IP_DEG, window=OSC_WINDOW,
                 flip_limit=OSC_FLIP_LIMIT):
        self.alpha = float(alpha)
        self.lin_acc = float(lin_acc)
        self.ang_acc = float(np.radians(ang_acc_deg))
        self.dead_tilt = float(dead_tilt_deg)
        self.dead_ip = float(dead_ip_deg)
        self.flip_limit = float(flip_limit)
        self._flips = collections.deque(maxlen=int(window))
        self.reset()

    def reset(self):
        """Back to rest. Call when motion stops, so a restart ramps up."""
        self._f_lin = None
        self._f_ang = None
        self.lin = np.zeros(3)
        self.ang = np.zeros(3)
        self._flips.clear()

    def filter(self, lin, ang, tilt_deg, ip_err_deg):
        a = self.alpha
        lin = np.asarray(lin, dtype=float)
        ang = np.asarray(ang, dtype=float)
        self._f_lin = lin if self._f_lin is None else a * lin + (1 - a) * self._f_lin
        self._f_ang = ang if self._f_ang is None else a * ang + (1 - a) * self._f_ang
        out_ang = self._f_ang
        if tilt_deg < self.dead_tilt and ip_err_deg < self.dead_ip:
            out_ang = np.zeros(3)
        return self._f_lin.copy(), np.array(out_ang, dtype=float)

    def limit(self, lin, ang, dt):
        lin = self.lin + clamp_norm(np.asarray(lin, dtype=float) - self.lin,
                                    self.lin_acc * dt)
        ang = self.ang + clamp_norm(np.asarray(ang, dtype=float) - self.ang,
                                    self.ang_acc * dt)
        # a reversal only counts when both sides are clearly non-zero, so
        # a deliberate single crossing or near-zero dithering does not
        rev = (np.any(lin * self.lin < -OSC_EPS_LIN ** 2)
               or np.any(ang * self.ang < -OSC_EPS_ANG ** 2))
        self._flips.append(bool(rev))
        self.lin, self.ang = lin, ang
        return lin.copy(), ang.copy()

    def flip_rate(self):
        return float(np.mean(self._flips)) if self._flips else 0.0

    def oscillating(self):
        return (len(self._flips) == self._flips.maxlen
                and self.flip_rate() > self.flip_limit)


class StallWatchdog:
    """Trips when servo keeps commanding motion but the error stops shrinking.

    Progress is any of position / tilt / in-plane error dropping by its
    threshold. Errors are low-passed first, so per-frame measurement noise
    (tilt 0.31 deg) cannot pass for progress. Diverging also counts as no
    progress, which is what it is.
    """

    def __init__(self, window_s=None, mm=None, deg=None, alpha=0.1):
        self.window_s = float(SERVO_STALL_S if window_s is None else window_s)
        mm = SERVO_STALL_MM if mm is None else mm
        deg = SERVO_STALL_DEG if deg is None else deg
        self.thr = np.array([mm, deg, deg], dtype=float)
        self.alpha = float(alpha)
        self.reset(0.0)

    def reset(self, now):
        self._f = None
        self._ref = None
        self._t = now

    def update(self, now, err_m, tilt_deg, ip_err_deg):
        """Feed one measurement. True once there was no progress for window_s."""
        x = np.array([err_m * 1000.0, tilt_deg, ip_err_deg], dtype=float)
        a = self.alpha
        self._f = x if self._f is None else a * x + (1 - a) * self._f
        if self._ref is None or np.any(self._f < self._ref - self.thr):
            self._ref, self._t = self._f.copy(), now
        return now - self._t > self.window_s


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


class AlignNode(Node):
    def __init__(self):
        super().__init__('camera_align_gui')
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('tcp_link', 'fr3_hand_tcp')
        self.declare_parameter('group', 'fr3_arm')
        self.base = self.get_parameter('base_frame').value
        self.tcp = self.get_parameter('tcp_link').value
        self.group = self.get_parameter('group').value

        self._lock = threading.Lock()
        self._pose = None          # (pos(3), R_cam_marker, stamp_s)
        self._joints = None        # (names, positions)
        self.create_subscription(PoseStamped, '/aruco/pose', self._cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_cb, 10)
        # Marker overlay for the dashboard. cam_pub only encodes it while
        # something subscribes and caps it at debug_max_hz (5 Hz); DDS is
        # pinned to loopback, so it never reaches the robot NIC.
        self._image = None         # (seq, HxWx3 RGB uint8)
        self._image_seq = 0
        self.create_subscription(Image, '/aruco/debug_image',
                                 self._image_cb, 2)

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
        self.last_cmd = {}         # filled by move(), read by the trace log

        # Servo backend. delta_twist_cmds is the velocity stream; servo
        # halts by itself if the stream stops (incoming_command_timeout),
        # which makes silence a deadman rather than a runaway.
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.servo_start = self.create_client(Trigger,
                                              '/servo_node/start_servo')
        self.servo_stop = self.create_client(Trigger, '/servo_node/stop_servo')
        self.switch_ctrl = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        # Which controller holds the arm, as far as we know: 'arm', 'servo',
        # or None when a switch failed and it is genuinely unknown.
        self.ctrl = 'arm'

        # Robot mode for the robot-state gate and the FRANKA pill, through
        # the throttle child (see ROBOT_STATE_RELAY_HZ). Best effort, depth 1:
        # a slow reader must never back-pressure the robot side.
        self._mode = None          # (robot_mode, monotonic stamp)
        self.gate_edges = 0        # interruptions: robot left MOVE, or PAUSE
        self.on_mode_change = None  # Gui hook, called in the spin thread
        self.relay_error = None
        self._relay = self._start_state_relay()
        self.create_subscription(
            FrankaRobotState, ROBOT_STATE_RELAY, self._mode_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _start_state_relay(self):
        """Spawn the C++ throttle (see tools/fr3/state_relay.py for why)."""
        try:
            return start_throttle(ROBOT_STATE_TOPIC, ROBOT_STATE_RELAY,
                                  hz=ROBOT_STATE_RELAY_HZ,
                                  node_name='align_gui_state_relay')
        except Exception as e:                              # noqa: BLE001
            self.relay_error = str(e)
            self.get_logger().error(f'robot state relay not started: {e}')
            return None

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

    def publish_twist(self, lin, ang, frame):
        """One velocity command. lin m/s, ang rad/s, both in `frame`."""
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = [float(v) for v in lin]
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = [float(v) for v in ang]
        self.twist_pub.publish(m)

    def zero_twist(self, n=3):
        """Explicit stop. Sent repeatedly because it is the deadman."""
        for _ in range(n):
            self.publish_twist((0, 0, 0), (0, 0, 0), self.base)
            time.sleep(0.02)

    def call_servo(self, cli, timeout_s=5.0):
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, 'servo service unavailable (is fr3_servo.launch.py up?)'
        fut = cli.call_async(Trigger.Request())
        if not self._wait(fut, timeout_s):
            return False, 'servo service timed out'
        r = fut.result()
        return (bool(r.success), r.message) if r is not None else (False, 'no response')

    def switch_controllers(self, activate, deactivate, mode):
        """Swap ros2_control controllers. Returns (ok, message).

        TWO calls, releasing the old controller before claiming the new one -
        never both in one request. franka_hardware 2.0.2's
        perform_command_mode_switch handles the modes in a fixed order within
        a single pass: asked for effort in and position out, it starts torque
        control and THEN calls stopRobot() for the outgoing position mode, so
        the next write() has no active control and takes ros2_control_node
        down with a std::runtime_error. Measured here 2026-09-15, with the
        arm stationary. Split in two, each pass changes one mode only.

        Between the calls no controller holds the arm, which is the same
        state as a freshly started stack: the robot holds position.
        """
        if not self.switch_ctrl.wait_for_service(timeout_sec=3.0):
            return False, 'controller_manager not available'
        if deactivate:
            ok, msg = self._switch_once([], list(deactivate))
            if not ok:
                self.ctrl = None
                return False, f'could not release {deactivate[0]}: {msg}'
            time.sleep(CTRL_SWITCH_SETTLE_S)   # let the mode actually stop
        ok, msg = self._switch_once(list(activate), [])
        self.ctrl = mode if ok else None
        return ok, msg

    def _switch_once(self, activate, deactivate):
        """One switch_controller call. STRICT: a partial switch would leave
        the arm held by the wrong controller, or by none."""
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=int(CTRL_SWITCH_TIMEOUT_S))
        fut = self.switch_ctrl.call_async(req)
        if not self._wait(fut, CTRL_SWITCH_TIMEOUT_S + 5.0):
            return False, 'switch_controller timed out'
        res = fut.result()
        ok = bool(res is not None and res.ok)
        return ok, ('switched' if ok
                    else 'controller_manager refused the switch')

    def _cb(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        o = m.pose.orientation
        with self._lock:
            self._pose = (p, q2R(o.x, o.y, o.z, o.w),
                          self.get_clock().now().nanoseconds * 1e-9)

    def _joint_cb(self, m):
        with self._lock:
            self._joints = (list(m.name), list(m.position))

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

    def joints(self):
        """Arm joint positions as {name: rad}, or {} if not yet seen."""
        with self._lock:
            j = self._joints
        return {} if j is None else dict(zip(j[0], j[1]))

    def marker(self):
        """Fresh marker measurement, or None if missing/stale."""
        with self._lock:
            if self._pose is None:
                return None
            p, R, t = self._pose
        if self.get_clock().now().nanoseconds * 1e-9 - t > POSE_STALE_S:
            return None
        return p, R

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
        execution halts mid-trajectory and the servo stream goes to zero.
        Never blocks, so it is safe from the spin thread.
        """
        msg = String()
        msg.data = 'stop'
        for _ in range(3):          # cheap, and the topic is best-effort
            self.exec_event.publish(msg)
        z = TwistStamped()
        z.header.frame_id = self.base
        for _ in range(5):          # no sleep: this is the instant path
            z.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(z)

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
        # stop_servo is fired without waiting - an instant stop must never
        # block on a service round-trip.
        if self.servo_stop.service_is_ready():
            self.servo_stop.call_async(Trigger.Request())
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


class Gui:
    """Two-column operator dashboard.

    Left: live camera with the marker axes, convergence history, log.
    Right: readouts, target + safety, motion backend, calibration, actions.

    Presentation only - every motion still goes through translate / level /
    inplane / servo_converge / auto_converge further down, unchanged.
    """

    def __init__(self, node):
        self.n = node
        self.R = None                 # R_cam_base once calibration is loaded
        self.busy = False
        self.abort = False            # set by STOP, polled by the run loops
        self.stopped = False          # latched by STOP until the next run
        self.last_outcome = None      # outcome of the last traced run
        self.tracef = None            # open trace file during auto-converge
        self.tracepath = None
        self.calib_meta = {}          # parsed from CALIB_PATH for the card
        self.calib_state = 'waiting'  # waiting | loaded | missing | error
        self.photo = None             # keep a reference or tk drops the image
        self._image_shown = None      # sequence number of the image on screen
        self.hist = collections.deque(maxlen=600)   # (t, err_mm, tilt, ip_err)
        self.gate_on = GATE_DEFAULT   # plain bool: read by worker + spin threads
        self.paused = False           # a run is waiting on the robot
        self.pause_reason = ''
        self.gate_fault = None        # outcome when the robot ended a run
        self.user_paused = False      # PAUSE pressed; RESUME clears it
        self._mode_log = collections.deque(maxlen=64)  # drained by tick

        self.root = tk.Tk()
        self.root.title('FR3 Camera Alignment')
        self.root.configure(bg=T['page'])

        base = tkfont.nametofont('TkDefaultFont').actual()['family']
        self.f_caption = (base, 9)
        self.f_label = (base, 10)
        self.f_value = ('DejaVu Sans Mono', 16, 'bold')
        self.f_small = ('DejaVu Sans Mono', 9)
        self.f_button = (base, 10, 'bold')
        self.f_banner = (base, 20, 'bold')
        self._init_combo_style()

        outer = tk.Frame(self.root, bg=T['page'])
        outer.pack(fill='both', expand=True, padx=16, pady=12)
        head = tk.Frame(outer, bg=T['page'])
        head.pack(fill='x')
        tk.Label(head, text='FR3 CAMERA  <->  MARKER ALIGNMENT',
                 font=(base, 10, 'bold'), fg=T['muted'], bg=T['page'],
                 anchor='w').pack(side='left')
        self.f_pill = tkfont.Font(family=base, size=9)
        self.f_pill_b = tkfont.Font(family=base, size=9, weight='bold')
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

        self._build_camera(left)
        self._build_plots(left)
        self._build_log(left)
        self._build_readout(right)
        self._build_settings(right)
        self._build_calib(right)
        self._build_actions(right)

        self.status = tk.Label(outer, text='ready', anchor='w',
                               fg=T['muted'], bg=T['page'],
                               font=self.f_caption)
        self.status.pack(fill='x', pady=(6, 0))

        self._read_calib_meta()
        self.say('Ready. Calibration loads automatically once TF is up.')
        if node.relay_error:
            self.say(f'WARNING: robot state relay failed ({node.relay_error})'
                     ' - with the gate on, nothing can move')
        node.on_mode_change = self._on_mode_change
        self.root.after(1500, self._autoload)
        self.relabel()
        self.tick()

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
        for k, v in (('background', T['surface']), ('foreground', T['ink']),
                     ('selectBackground', T['series']),
                     ('selectForeground', T['ink'])):
            self.root.option_add(f'*TCombobox*Listbox.{k}', v)

    def _card(self, parent, title):
        frame = tk.Frame(parent, bg=T['surface'],
                         highlightbackground=T['grid'], highlightthickness=1)
        tk.Label(frame, text=title, font=self.f_caption, fg=T['muted'],
                 bg=T['surface'], anchor='w').pack(fill='x', padx=10,
                                                   pady=(8, 2))
        return frame

    def _button(self, parent, text, color, cmd, font=None, pady=9):
        fg = text_on(color)
        b = tk.Button(parent, text=text, font=font or self.f_button,
                      bg=color, fg=fg,
                      activebackground=mix(color, '#ffffff', .15),
                      activeforeground=fg, relief='flat', bd=0,
                      highlightthickness=0, cursor='hand2', padx=14,
                      pady=pady, command=cmd,
                      disabledforeground=T['muted'])
        b.bind('<Enter>', lambda e, w=b, c=color:
               w['state'] == 'normal' and w.config(bg=mix(c, '#ffffff', .12)))
        b.bind('<Leave>', lambda e, w=b, c=color: w.config(bg=c))
        return b

    def _combo(self, parent, var, vals, width=6):
        cb = ttk.Combobox(parent, textvariable=var, values=vals, width=width,
                          state='readonly', style='Dark.TCombobox',
                          font=self.f_small)
        cb.bind('<<ComboboxSelected>>', lambda _e: self.relabel())
        return cb

    def _row(self, parent, label, var, vals, r, c):
        lbl = tk.Label(parent, text=label, font=self.f_caption, fg=T['ink2'],
                       bg=T['surface'], anchor='e')
        lbl.grid(row=r, column=c * 2, padx=(10, 4), pady=3, sticky='e')
        # size to the longest choice so 'cartesian' / 'conservative' fit
        width = max(6, max(len(str(v)) for v in vals) + 1)
        cb = self._combo(parent, var, vals, width=width)
        cb.grid(row=r, column=c * 2 + 1, padx=(0, 10), pady=3, sticky='w')
        return lbl, cb

    # ------------------------------------------------------------ layout

    def _build_camera(self, parent):
        card = self._card(parent, 'LIVE CAMERA  |  /aruco/debug_image, '
                                  '5 Hz, loopback only')
        card.pack(fill='x')
        self.image_label = tk.Label(card, bg=T['page'], fg=T['muted'],
                                    font=self.f_label, width=78, height=22,
                                    text='waiting for camera image ...')
        self.image_label.pack(padx=10, pady=(0, 10))

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

    def _build_log(self, parent):
        card = self._card(parent, 'LOG')
        card.pack(fill='both', expand=True, pady=(12, 0))
        self.log = scrolledtext.ScrolledText(
            card, height=7, font=self.f_small, bg=T['page'], fg=T['ink2'],
            insertbackground=T['ink'], relief='flat', bd=0,
            highlightthickness=0)
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))

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
        m.pack(fill='x', pady=(0, 4))
        self.v_backend = tk.StringVar(value=BACKEND_DEFAULT)
        self.v_servo = tk.StringVar(value=SERVO_PROFILE_DEFAULT)
        self.v_converge = tk.StringVar(value=CONVERGE_DEFAULT)
        self.v_step = tk.StringVar(value=STEP_MM_DEFAULT)
        self.v_rot = tk.StringVar(value=ROT_DEG_DEFAULT)
        self.v_speed = tk.StringVar(value=SPEED_PCT_DEFAULT)
        self._row(m, 'backend', self.v_backend, BACKEND_CHOICES, 0, 0)
        self.servo_rows = [
            self._row(m, 'servo speed', self.v_servo,
                      list(SERVO_PROFILES), 1, 0),
            self._row(m, 'on converge', self.v_converge,
                      CONVERGE_CHOICES, 2, 0)]
        self.cart_rows = [
            self._row(m, 'step (mm)', self.v_step, STEP_MM_CHOICES, 3, 0),
            self._row(m, 'level (deg)', self.v_rot, ROT_DEG_CHOICES, 4, 0),
            self._row(m, 'speed (%)', self.v_speed, SPEED_PCT_CHOICES, 5, 0)]
        self.servo_state = tk.Label(mot, text='', font=self.f_caption,
                                    fg=T['muted'], bg=T['surface'],
                                    anchor='w', justify='left')
        self.servo_state.pack(fill='x', padx=10, pady=(0, 10))

    def _build_calib(self, parent):
        card = self._card(parent, 'CALIBRATION  |  camera -> TCP rotation')
        card.pack(fill='x', pady=(12, 0))
        g = tk.Frame(card, bg=T['surface'])
        g.pack(fill='x', padx=10, pady=(0, 4))
        self.calib_labels = {}
        for r, key in enumerate(('source', 'residual', 'validated',
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

    def say(self, m):
        self.log.insert('end', m + '\n')
        self.log.see('end')

    def trace(self, rec):
        """Append one JSON record to the current run's trace file."""
        if self.tracef is None:
            return
        rec['t'] = time.time()
        try:
            self.tracef.write(json.dumps(rec, default=float) + '\n')
            self.tracef.flush()
        except Exception as e:                              # noqa: BLE001
            self.say(f'trace write failed: {e}')

    def open_trace(self, header):
        LOG_DIR.mkdir(exist_ok=True)
        name = ('autoconverge_'
                + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.jsonl')
        self.tracepath = LOG_DIR / name
        self.tracef = self.tracepath.open('w')
        self.trace({'rec': 'run_start', **header})
        self.say(f'trace -> tools/fr3/logs/{name}')

    def close_trace(self, outcome):
        # the banner reads this: auto_converge itself returns nothing
        self.last_outcome = outcome
        if self.tracef is None:
            return
        self.trace({'rec': 'run_end', 'outcome': outcome})
        self.tracef.close()
        self.tracef = None

    def set_status(self, m, color=None):
        self.status.config(text=m, fg=color or T['muted'])

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
        be = self.v_backend.get()
        self.ab.config(text=(f'AUTO-CONVERGE   |   {be}   |   '
                             f'-> {self.v_target.get()} mm'))
        # dim the settings that do not drive AUTO-CONVERGE on this backend;
        # they stay editable because the manual step buttons use them
        servo = be == 'servo'
        for rows, on in ((self.servo_rows, servo),
                         (self.cart_rows, not servo)):
            for lbl, _cb in rows:
                lbl.config(fg=T['ink'] if on else T['muted'])

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
        """Instant: halt the trajectory (or servo stream) where it is."""
        self.abort = True
        self.stopped = True
        self.n.stop_now()
        self.say('*** STOP NOW - halting motion mid-move ***')
        self.set_status('STOPPED (instant)', T['critical'])
        self.trace({'rec': 'stop_now'})

    # ------------------------------------------------------------ gate

    def robot_block(self):
        """None if the robot may move now, else (reason, fatal).

        fatal ends the run; otherwise it pauses. REFLEX is fatal whatever the
        toggle says: resuming by itself right after an error recovery would
        be a surprise, and servo would otherwise stream into a faulted arm.
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
            self.calib_meta = json.loads(CALIB_PATH.read_text())
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
            R_cam_tcp = np.array(self.calib_meta['R_cam_tcp'], dtype=float)
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
        src = self.calib_meta.get('source', CALIB_PATH.name)
        self.say(f'calibration loaded ({src}), re-based to the current pose')

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

    def tick(self):
        snap = None
        m = self.n.marker()
        if m is None:
            for t_ in self.tiles.values():
                t_['val'].config(text='--', fg=T['critical'])
                t_['dot'].itemconfig(t_['id'], fill=T['critical'])
            self.sub.config(text='marker NOT VISIBLE / STALE')
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
        self.draw_banner(snap)
        self._update_image()
        self._update_calib_card()
        self._update_servo_state()
        self._drain_mode_log()
        self.draw_pills()
        self.root.after(150, self.tick)

    def _banner_state(self, snap):
        """(STATE, colour, subtitle) - the one-glance answer to 'what now?'."""
        if self.paused:
            return ('PAUSED', T['warning'],
                    f'{self.pause_reason}  -  {self._resume_hint()},  '
                    'STOP NOW ends the run')
        if self.busy:
            return ('ALIGNING', T['series'],
                    f'{self.v_backend.get()} backend running  -  '
                    'STOP NOW halts it')
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

    def draw_banner(self, snap):
        c = self.banner
        c.delete('all')
        w = max(c.winfo_width(), 400)
        state, color, sub = self._banner_state(snap)
        rounded_rect(c, 0, 0, w, 62, 12, fill=color, outline='')
        ink = text_on(color)
        c.create_text(20, 24, text=state, anchor='w', font=self.f_banner,
                      fill=ink)
        c.create_text(20, 49, text=sub, anchor='w', font=self.f_caption,
                      fill=mix(ink, color, 0.3))

    def _pill_states(self):
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
        ready = self.n.servo_start.service_is_ready()
        if self.n.ctrl is None:
            servo = ('CTRL STUCK', T['critical'])
        elif self.n.ctrl == 'servo':
            servo = ('streaming', T['good'])
        else:
            servo = ('ready' if ready else 'off',
                     T['good'] if ready else T['muted'])
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
                ('SERVO',) + servo,
                ('GATE',) + gate]

    def draw_pills(self):
        c = self.pills
        c.delete('all')
        items = []
        for label, value, col in self._pill_states():
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

    def _update_image(self):
        got = self.n.image()
        if got is None or got[0] == self._image_shown:
            return
        seq, img = got
        hgt, wid = img.shape[:2]
        header = f'P6 {wid} {hgt} 255 '.encode()
        self.photo = tk.PhotoImage(data=header + img.tobytes())
        self.image_label.config(image=self.photo, text='', width=wid,
                                height=hgt)
        self._image_shown = seq

    def _update_calib_card(self):
        md = self.calib_meta
        when = (md.get('calibrated_utc') or md.get('saved_utc') or '?')[:10]
        self.calib_labels['source'].config(
            text=(f'{md.get("source", "unknown")}   {md.get("method", "?")}, '
                  f'{md.get("poses", "?")} poses   {when}'))
        if 'residual_mm' in md:
            self.calib_labels['residual'].config(
                text=(f'{md["residual_mm"]:.2f} mm / '
                      f'{md.get("residual_deg", float("nan")):.2f} deg'))
        if 'validated_scatter_mm' in md:
            guess = md.get('guess_scatter_mm')
            self.calib_labels['validated'].config(
                text=(f'{md["validated_scatter_mm"]:.2f} mm static-marker '
                      'scatter' + (f'  (guess was {guess:.0f} mm)'
                                   if guess else '')))
        text, col = {
            'loaded': ('loaded, re-based to the current pose', T['good']),
            'waiting': ('waiting for TF  -  is MoveIt running?', T['warning']),
            'missing': (f'{CALIB_PATH.name} not found', T['critical']),
            'error': ('could not read the calibration file', T['critical']),
        }[self.calib_state]
        self.calib_labels['status'].config(text=text, fg=col)

    def _update_servo_state(self):
        ready = self.n.servo_start.service_is_ready()
        if self.v_backend.get() == 'servo':
            self.servo_state.config(
                text=('servo node: ready - the arm swaps to '
                      f'{SERVO_CONTROLLER} for a run' if ready else
                      'servo node NOT running\n'
                      'start tools/fr3/fr3_servo.launch.py'),
                fg=T['good'] if ready else T['critical'])
        else:
            self.servo_state.config(
                text=f'servo node: {"ready" if ready else "not running"} '
                     '(unused on cartesian)',
                fg=T['muted'])

    # ------------------------------------------------------------ actions

    def go(self, fn, *a):
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

    def servo_converge(self):
        """Continuous 6-DOF alignment by streaming velocity to moveit_servo.

        One motion, coarse to fine: velocity = gain * error, clamped. All
        six DOF move together, so levelling no longer perturbs translation
        as a separate phase - they converge simultaneously.

        Safety here is live rather than pre-planned, because nothing is
        planned ahead:
          * whole-arm Z floor, every cycle, not just the TCP
          * marker loss or staleness -> immediate zero twist
          * velocity caps on both linear and angular
          * silence is the deadman: servo halts itself if the stream stops,
            and the position controller then simply holds where it is
          * the arm is handed to SERVO_CONTROLLER for the run, and handed
            back on every exit path
          * command shaping + oscillation watchdog (CommandShaper)
          * PAUSE / robot-state gate: halt, then resume from rest; a REFLEX
            ends the run
          * stall watchdog: abort if the arm stops following (StallWatchdog)
        """
        prof = SERVO_PROFILES[self.v_servo.get()]
        tol, tgt_ip = self.pos_tol_m(), self.inplane_target()
        floor = self.z_floor()

        # Refuse rather than trip instantly: if a link is ALREADY under the
        # floor the run could never start, and a silent abort would look
        # like a servo fault.
        low = self.n.lowest_link(FLOOR_LINKS)
        if low is not None and floor is not None and low[1] < floor:
            self.say(f'REFUSING: {low[0]} is already {low[1]*1000:.1f} mm, '
                     f'below the {floor*1000:.1f} mm floor')
            self.set_status('refused: link below floor', T['critical'])
            return 'refused_below_floor'

        ok, msg = self.n.switch_controllers([SERVO_CONTROLLER],
                                            [ARM_CONTROLLER], 'servo')
        if not ok:
            self.say(f'REFUSING: could not hand the arm to {SERVO_CONTROLLER}'
                     f' ({msg}) - is fr3_servo.launch.py running?')
            self.set_status('servo unavailable: controller switch',
                            T['critical'])
            return 'controller_switch_failed'
        self.say(f'arm -> {SERVO_CONTROLLER} (joint position interface)')

        ok, msg = self.n.call_servo(self.n.servo_start)
        if not ok:
            self.say(f'start_servo failed: {msg}')
            self.n.switch_controllers([ARM_CONTROLLER], [SERVO_CONTROLLER],
                                      'arm')
            self.set_status('servo unavailable', T['critical'])
            return 'servo_start_failed'
        self.say(f'servo START ({msg}) - profile {self.v_servo.get()}: '
                 f'{prof["lin"]*1000:.0f} mm/s, {prof["ang"]:.0f} deg/s, '
                 f'gain {prof["gain"]}')

        period, settled, t_start = 1.0 / SERVO_RATE_HZ, 0, time.time()
        shaper, announced = CommandShaper(), False
        stall = StallWatchdog()
        stall.reset(time.monotonic())
        outcome = 'servo_timeout'
        try:
            while time.time() - t_start < 300.0:
                if self.abort:
                    outcome = 'stopped'
                    break
                if self.robot_block() is not None:
                    self.n.zero_twist()
                    shaper.reset()          # resume ramps up from rest
                    settled, t_pause = 0, time.time()
                    if not self.wait_gate():
                        outcome = self.gate_fault or 'stopped'
                        break
                    t_start += time.time() - t_pause    # pause is not run time
                    stall.reset(time.monotonic())
                    continue
                m = self.n.marker()
                if m is None:                       # deadman
                    self.n.zero_twist()
                    self.say('ABORT: marker lost / stale')
                    outcome = 'marker_lost'
                    break
                low = self.n.lowest_link(FLOOR_LINKS)
                if floor is not None and low is not None and low[1] < floor:
                    self.n.zero_twist()
                    self.say(f'ABORT: Z FLOOR - {low[0]} at '
                             f'{low[1]*1000:.1f} mm < {floor*1000:.1f} mm')
                    outcome = 'z_floor'
                    break

                lin_c, ang_c, err, tilt, ip = self._servo_error(m, tgt_ip)
                converged = (err <= tol and tilt <= ROT_TOL_DEG
                             and (tgt_ip is None
                                  or abs(wrap_deg(ip - tgt_ip))
                                  <= INPLANE_TOL_DEG))
                if converged:
                    settled += 1
                    if settled >= int(SERVO_RATE_HZ * 0.3):
                        self.n.zero_twist()
                        shaper.reset()      # a later correction ramps from rest
                        stall.reset(time.monotonic())
                        if not announced:
                            self.say(f'=== CONVERGED: {err*1000:.2f} mm, '
                                     f'{tilt:.2f} deg'
                                     f'{"" if tgt_ip is None else f", in-plane {ip:+.2f} deg"}'
                                     f' in {time.time()-t_start:.1f} s ===')
                            if self.v_converge.get() == 'station-keep':
                                self.say('station-keeping: still LIVE, press '
                                         'STOP to release')
                            announced = True
                        if self.v_converge.get() == 'station-keep':
                            settled = 0
                            time.sleep(period)
                            continue
                        outcome = 'converged'
                        break
                else:
                    settled = 0
                    # hysteresis: re-announce only after a real departure
                    if err > 2 * tol or tilt > 2 * ROT_TOL_DEG:
                        announced = False

                ipe = 0.0 if tgt_ip is None else abs(wrap_deg(ip - tgt_ip))
                # near tolerance the error legitimately creeps, so only watch
                # for a stall while clearly far from the target
                if not (err > 2 * tol or tilt > 2 * ROT_TOL_DEG
                        or ipe > 2 * INPLANE_TOL_DEG):
                    stall.reset(time.monotonic())
                elif stall.update(time.monotonic(), err, tilt, ipe):
                    self.n.zero_twist()
                    self.say(f'ABORT: arm not following servo - no progress '
                             f'in {SERVO_STALL_S:g} s at |e| {err*1000:.1f} mm'
                             f', tilt {tilt:.1f} deg')
                    self.set_status('aborted: servo stalled', T['serious'])
                    outcome = 'servo_stalled'
                    break
                lin_c, ang_c = shaper.filter(lin_c, ang_c, tilt, ipe)
                # camera-frame command -> base frame for servo
                lin = self._clamp(self.R.T @ lin_c, prof['lin'])
                ang = self._clamp(self.R.T @ ang_c, np.radians(prof['ang']))
                lin, ang = shaper.limit(lin, ang, period)
                if shaper.oscillating():
                    self.n.zero_twist()
                    self.say(f'ABORT: command oscillating '
                             f'({shaper.flip_rate():.0%} of frames reversing) '
                             '- stopped before libfranka reflexes; try a '
                             'slower servo speed')
                    self.set_status('aborted: oscillation', T['serious'])
                    outcome = 'oscillation'
                    break
                if self.robot_block() is not None:
                    continue                # top of the loop pauses
                self.n.publish_twist(lin, ang, self.n.base)
                self.trace({'rec': 'servo', 'err_mm': err * 1000,
                            'tilt_deg': tilt, 'inplane_deg': ip,
                            'flip_rate': shaper.flip_rate(),
                            'lin_mms': (lin * 1000).tolist(),
                            'ang_dps': np.degrees(ang).tolist(),
                            'lowest_link': low[0] if low else None,
                            'lowest_z_mm': low[1] * 1000 if low else None})
                time.sleep(period)
        finally:
            self.n.zero_twist(5)
            time.sleep(SERVO_SETTLE_S)
            sok, smsg = self.n.call_servo(self.n.servo_stop)
            self.say(f'servo STOP ({smsg if sok else "failed: " + smsg})')
            bok, bmsg = self.n.switch_controllers([ARM_CONTROLLER],
                                                  [SERVO_CONTROLLER], 'arm')
            if bok:
                self.say(f'arm -> {ARM_CONTROLLER} (planned moves again)')
            else:
                self.say(f'*** {ARM_CONTROLLER} NOT RESTORED ({bmsg}) - '
                         'cartesian moves will fail until it is back ***')
                self.set_status('controller NOT restored', T['critical'])
        return outcome

    _clamp = staticmethod(clamp_norm)

    def _servo_error(self, m, tgt_ip):
        """6-DOF error as (linear, angular) velocity commands in CAMERA frame.

        Signs are taken from the three PROVEN discrete steps, which do not
        share a convention - getting this wrong drives the arm away from
        the target, so each is derived explicitly:

          translate: d = R.T @ (p - target)   -> camera moves +err
          level:     Rc = axis_angle(axis, a).T  -> camera rotates -a about
                     axis, where axis = cross(marker_normal, target_normal)
          inplane:   Rc = axis_angle(+Z, d)   -> camera rotates +d about Z,
                     where d = wrap(current - target)

        So tilt is negated and in-plane is not. test_servo_signs_match_
        discrete pins this against the discrete implementations.
        """
        gain = SERVO_PROFILES[self.v_servo.get()]['gain']
        err_vec = m[0] - np.array([0, 0, self.target_m()])
        err = float(np.linalg.norm(err_vec))

        mz = m[1][:, 2]
        tilt = float(np.degrees(np.arccos(
            np.clip(abs(mz @ np.array([0, 0, 1.0])), -1, 1))))
        tgt_n = np.array([0, 0, -1.0]) if mz[2] < 0 else np.array([0, 0, 1.0])
        axis = np.cross(mz, tgt_n)
        na = float(np.linalg.norm(axis))
        # minus: level() applies the TRANSPOSE of the aligning rotation
        ang_vec = -(axis / na) * np.radians(tilt) if na > 1e-8 else np.zeros(3)

        ip = inplane_angle(m[1])
        if tgt_ip is not None and ip is not None:
            # plus: inplane() applies the rotation directly, not transposed
            ang_vec = ang_vec + np.array([0.0, 0.0, 1.0]) * np.radians(
                wrap_deg(ip - tgt_ip))
        return (gain * err_vec, gain * ang_vec, err, tilt,
                (ip if ip is not None else 0.0))

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
            'backend': self.v_backend.get(),
            'servo_profile': self.v_servo.get(),
            'on_converge': self.v_converge.get(),
            'inplane_target': self.v_inplane.get(),
            'gate': self.gate_on,
            'controller': (SERVO_CONTROLLER if self.v_backend.get() == 'servo'
                           else ARM_CONTROLLER),
        })
        outcome = 'exception'
        try:
            outcome = (self.servo_converge()
                       if self.v_backend.get() == 'servo'
                       else self._converge_loop(tol, cap))
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
            # whole-arm floor, same as the servo backend - an elbow can dip
            # below the floor while the TCP goal pre-check still passes
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


def main():
    rclpy.init()
    node = AlignNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = Gui(node)
    try:
        gui.root.mainloop()
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
