"""The cell panel's ROS side: one node for the whole window.

CellNodeBase is the Tk panel's CellNode, carried over unchanged (its MoveIt
moves, controller switches, parameter writes, marker TF handling and the
50 Hz state relay ran on the arm). CellNode adds the controller poll,
parameter readback, error recovery, heartbeat and the vision ages. The node
is named 'cell_panel', so only one panel can drive the arm at a time
(other_panels()).

Every callback here only stores data under the node's lock. Nothing in this
file touches a widget: the GUI thread reads these getters at 10 Hz
(actions.Cell.snapshot) and marshals everything else through Qt signals.
"""

import collections
import pathlib
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticStatus
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetLoad
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.msg import TransitionEvent
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

import core
from core import (COLLISION_TORQUE_NM, COLLISION_WRENCH, CONTACT_TORQUE_NM,
                  CONTACT_WRENCH, EQUILIBRIUM_TOPIC, IMAGE_MAX_W, IMAGE_TOPIC,
                  IMPEDANCE_CONTROLLER, MIN_FRACTION, MODE_MOVE, POSE_STALE_S, R2q,
                  ROBOT_STATE_RELAY, ROBOT_STATE_RELAY_HZ, ROBOT_STATE_TOPIC,
                  TRACK_CALL_TIMEOUT_S, TRACK_PARAMS_SRV, TRACK_PARAM_SRV,
                  TRACK_START_SRV, TRACK_STATUS_TOPIC, TRACK_STOP_SRV, q2R,
                  GRIP_SRV, PLACE_SRV, GRIP_STOP_SRV, GRIP_PARAMS_SRV, GRIP_STATUS_TOPIC,
                  run_resumable)

sys.path.insert(0, str(core.FR3))
from state_relay import start_throttle, stop_throttle   # noqa: E402

# GUI liveness, for a controller-side watchdog that does not exist yet
# (TODO C1). Published from the GUI thread, so a frozen window stops it.
HEARTBEAT_TOPIC = '~/heartbeat'          # -> /cell_panel/heartbeat
RECOVERY_ACTION = '/action_server/error_recovery'
DESCRIPTION_TOPIC = '/robot_description'
RAW_POSE_TOPIC = '/aruco/pose_raw'
# The driver's executor and DDS threads run at FIFO 99, above its 1 kHz
# control loop (FIFO 50), so every service call into it competes with the
# loop (2026-09-24: RT success dips 4x more often while polling at 2 Hz).
# Poll rarely; commands and transition events wake the poll at once.
CM_POLL_S = 5.0            # list_controllers backstop
PARAM_POLL_S = 10.0        # get_parameters backstop on the impedance controller
JUMP_WINDOW_S = 1.0
READ_PARAMS = ('float_mode', 'k_pos_tool', 'k_rot_tool', 'damping_ratio',
               'setpoint_slew_mps', 'setpoint_slew_rps')
FR3_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]


def _value(pv):
    """A rcl_interfaces ParameterValue as a Python value."""
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if t == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    return None


def _set_flags(errs):
    """Names of the bool fields set in a franka_msgs/Errors."""
    return [k for k in errs.get_fields_and_field_types()
            if getattr(errs, k, False) is True]


def limits_from_urdf(text):
    """[(lower, upper)] for fr3_joint1..7 from a URDF string, or None."""
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return None
    found = {}
    for j in root.iter('joint'):
        lim = j.find('limit')
        if j.get('name') in FR3_JOINTS and lim is not None:
            found[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return [found[n] for n in FR3_JOINTS] if len(found) == 7 else None


def limits_from_franka_description():
    """Fallback: franka_description's joint_limits.yaml, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = (pathlib.Path(get_package_share_directory('franka_description'))
                / 'robots' / 'fr3' / 'joint_limits.yaml')
        doc = yaml.safe_load(path.read_text())
        return [(doc[f'joint{i}']['limit']['lower'], doc[f'joint{i}']['limit']['upper'])
                for i in range(1, 8)]
    except Exception:                                       # noqa: BLE001
        return None


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


class CellNodeBase(Node):
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
        self.grip_cli = self.create_client(Trigger, GRIP_SRV)
        self.place_cli = self.create_client(Trigger, PLACE_SRV)
        self.grip_stop_cli = self.create_client(Trigger, GRIP_STOP_SRV)
        self.grip_params_cli = self.create_client(SetParametersAtomically, GRIP_PARAMS_SRV)
        self._grip_status = None   # (fields, monotonic stamp)
        self.create_subscription(
            DiagnosticStatus, GRIP_STATUS_TOPIC, self._grip_status_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
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

    def _grip_status_cb(self, m):
        f = {'message': m.message}
        f.update((kv.key, kv.value) for kv in m.values)
        with self._lock:
            self._grip_status = (f, time.monotonic())

    def grip_status(self):
        """The grip node's latest status fields, or None."""
        with self._lock:
            s = self._grip_status
        return None if s is None else dict(s[0])

    def grip_stop(self):
        """Fire and forget: grip_node holds at once; nothing to wait for."""
        if self.grip_stop_cli.service_is_ready():
            self.grip_stop_cli.call_async(Trigger.Request())

    def set_grip_params(self, values, timeout_s=5.0):
        """The drawer's cube size, force and box, all or none, before GRIP."""
        if not self.grip_params_cli.service_is_ready():
            return False, 'grip_node is not running'
        req = SetParametersAtomically.Request()
        req.parameters = [_param_msg(k, v) for k, v in values.items()]
        fut = self.grip_params_cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return False, f'no answer from {self.grip_params_cli.srv_name}'
        r = fut.result().result
        return bool(r.successful), ('applied' if r.successful else r.reason or 'refused')

    def track_status(self):
        """(fields, age_s) of the tracking node's latest status, or None."""
        with self._lock:
            s = self._track_status
        return None if s is None else (s[0], time.monotonic() - s[1])

    def call_trigger(self, cli, timeout_s=TRACK_CALL_TIMEOUT_S, who='the tracking node'):
        """One Trigger call, answered rather than waited on: the tracking
        node is optional, so a missing one is an answer, not a stall. The
        wait also ends if the server leaves the graph mid-call - a future
        never completes once its server is gone."""
        if not cli.service_is_ready():
            return False, f'{who} is not running'
        fut = cli.call_async(Trigger.Request())
        end = time.time() + timeout_s
        gone_since = None
        while not fut.done() and time.time() < end:
            if cli.service_is_ready():
                gone_since = None
            elif gone_since is None:
                gone_since = time.time()
            elif time.time() - gone_since > 1.0:
                return False, (f'{who} went away mid-call - whatever it last commanded is '
                               'still held by the controller; press STOP NOW')
            time.sleep(0.02)
        if not fut.done() and who != 'the tracking node':
            return False, (f'no answer from {cli.srv_name} in {timeout_s:.0f} s - {who} may '
                           'still be moving the arm; press STOP NOW')
        if not fut.done() or fut.result() is None:
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


class CellNode(CellNodeBase):

    def __init__(self):
        super().__init__()
        self._extra = None          # (q, dq, errors, last_errors)
        self._marker_at = None      # monotonic arrival of /aruco/pose
        self._raw_at = None
        self._image_at = None
        self._last_marker = None    # (pos, frame) for the jump check
        self._jumps = collections.deque(maxlen=64)   # (monotonic, mm)
        self._ctrl = None           # ({name: state}, monotonic)
        self._params = None         # ({name: value}, monotonic)
        self._limits = limits_from_franka_description()
        self._limits_source = 'franka_description' if self._limits else None

        self.create_subscription(PoseStamped, RAW_POSE_TOPIC, self._raw_cb, 10)
        self.create_subscription(
            TransitionEvent, f'/{IMPEDANCE_CONTROLLER}/transition_event',
            self._transition_cb, 10)
        self.create_subscription(
            String, DESCRIPTION_TOPIC, self._description_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_params_cli = self.create_client(
            GetParameters, f'/{IMPEDANCE_CONTROLLER}/get_parameters')
        self.recover_ac = ActionClient(self, ErrorRecovery, RECOVERY_ACTION)
        self.hb_pub = self.create_publisher(Header, HEARTBEAT_TOPIC, 10)

        self._stop = threading.Event()
        self._wake = threading.Event()
        self._params_due = False
        self._poller = threading.Thread(target=self._poll, daemon=True)
        self._poller.start()

    def close(self):
        self._stop.set()
        self._wake.set()
        self._poller.join(timeout=2.0)
        super().close()

    # ------------------------------------------------------------ callbacks

    def _state_cb(self, m):
        super()._state_cb(m)
        js = m.measured_joint_state
        with self._lock:
            self._extra = (list(js.position), list(js.velocity),
                           _set_flags(m.current_errors),
                           _set_flags(m.last_motion_errors))

    def _cb(self, m):
        super()._cb(m)
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        now = time.monotonic()
        with self._lock:
            last = self._last_marker
            if last is not None and last[1] == m.header.frame_id:
                self._jumps.append((now, float(np.linalg.norm(p - last[0])) * 1000))
            self._last_marker = (p, m.header.frame_id)
            self._marker_at = now

    def _raw_cb(self, _m):
        with self._lock:
            self._raw_at = time.monotonic()

    def _image_cb(self, m):
        super()._image_cb(m)
        with self._lock:
            self._image_at = time.monotonic()

    def _transition_cb(self, _m):
        self._wake.set()            # re-read the controller manager now

    def _description_cb(self, m):
        lim = limits_from_urdf(m.data)
        if lim is not None:
            with self._lock:
                self._limits, self._limits_source = lim, DESCRIPTION_TOPIC

    # ------------------------------------------------------------ poller

    def _poll(self):
        """Off the executor: the controller manager's truth, and what the
        impedance controller really runs. Answers arrive via the spin thread."""
        next_params = 0.0
        while not self._stop.is_set():
            states = self.controllers(timeout_s=1.0)
            now = time.monotonic()
            if states is not None:
                with self._lock:
                    self._ctrl = (states, now)
            elif not self.list_cli.service_is_ready():
                with self._lock:
                    self._ctrl = None         # gone from the graph: the driver is down
            if now >= next_params or self._params_due:
                self._params_due = False
                vals = self.read_params()
                if vals is not None:
                    with self._lock:
                        self._params = (vals, time.monotonic())
                next_params = now + PARAM_POLL_S
            self._wake.wait(CM_POLL_S)
            self._wake.clear()

    def read_params(self, timeout_s=1.0):
        """{name: value} of READ_PARAMS from the impedance controller."""
        if not self.get_params_cli.service_is_ready():
            return None
        req = GetParameters.Request()
        req.names = list(READ_PARAMS)
        fut = self.get_params_cli.call_async(req)
        if not self._wait(fut, timeout_s) or fut.result() is None:
            return None
        vals = fut.result().values
        if len(vals) != len(READ_PARAMS):
            return None
        return {k: _value(v) for k, v in zip(READ_PARAMS, vals)}

    def refresh_now(self):
        """After a command: re-read the controller manager and parameters now."""
        self._params_due = True
        self._wake.set()

    # ------------------------------------------------------------ getters

    def controller_states(self, stale_s=12.0):
        """{name: state} if the controller manager answered lately, else None."""
        with self._lock:
            c = self._ctrl
        if c is None or time.monotonic() - c[1] > stale_s:
            return None
        return dict(c[0])

    def applied_params(self, stale_s=25.0):
        with self._lock:
            p = self._params
        if p is None or time.monotonic() - p[1] > stale_s:
            return None
        return dict(p[0])

    def extra(self):
        with self._lock:
            return self._extra

    def _age(self, t):
        return None if t is None else time.monotonic() - t

    def marker_age(self):
        with self._lock:
            return self._age(self._marker_at)

    def raw_age(self):
        with self._lock:
            return self._age(self._raw_at)

    def image_age(self):
        with self._lock:
            return self._age(self._image_at)

    def jump_mm(self):
        """Largest frame-to-frame marker jump within the last second."""
        now = time.monotonic()
        with self._lock:
            recent = [mm for t, mm in self._jumps if now - t <= JUMP_WINDOW_S]
        return max(recent, default=0.0)

    def joint_limits(self):
        with self._lock:
            return self._limits, self._limits_source

    # ------------------------------------------------------------ commands

    def heartbeat(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'cell_panel'
        self.hb_pub.publish(h)

    def recover(self, timeout_s=30.0):
        """franka_hardware's automatic error recovery. Blocks: worker only."""
        if not self.recover_ac.wait_for_server(timeout_sec=2.0):
            return False, f'{RECOVERY_ACTION} not available - the driver is down?'
        gfut = self.recover_ac.send_goal_async(ErrorRecovery.Goal())
        if not self._wait(gfut, 5.0) or gfut.result() is None:
            return False, 'error recovery goal not answered'
        gh = gfut.result()
        if not gh.accepted:
            return False, 'error recovery goal rejected'
        rfut = gh.get_result_async()
        if not self._wait(rfut, timeout_s) or rfut.result() is None:
            return False, 'error recovery timed out'
        ok = rfut.result().status == 4          # GoalStatus.STATUS_SUCCEEDED
        return ok, 'recovered' if ok else f'recovery failed (status {rfut.result().status})'

    def other_panels(self):
        """How many OTHER 'cell_panel' nodes are on the graph (a second panel)."""
        names = [n for n, ns in self.get_node_names_and_namespaces()
                 if n == 'cell_panel' and ns == '/']
        return max(0, len(names) - 1)


def spin_up():
    """rclpy up, with its own signal handling OFF: Ctrl-C must reach
    the window, which needs ROS alive to hand the arm back."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CellNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    return node, executor, spin
