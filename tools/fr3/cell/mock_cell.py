#!/usr/bin/env python3
"""A fake FR3 cell for the cell panel: everything it talks to, no robot.

    python3 tools/fr3/cell/mock_cell.py          # terminal A
    python3 tools/fr3/cell/cell.py --mock     # terminal B
    # or both at once: fr3_cell mock:=true

It fakes the robot state (with errors and joint velocities), /joint_states,
TF (arm links + the hand-eye from calib/handeye.yaml), /robot_description,
cam_pub's /aruco/pose, /aruco/pose_raw and a drawn /aruco/debug_image, the
controller manager (list/switch, STRICT), the impedance controller's
parameters, equilibrium and transition events, franka's set_load /
collision / error-recovery, MoveIt's compute_cartesian_path and
execute_trajectory (with the trajectory_execution_event stop), and a
tracking_node that follows the marker. The arm is a first-order toy: it
follows trajectories and the slew-limited equilibrium, nothing more.

ISOLATED (isolate.py): always ROS_DOMAIN_ID 88 over loopback, re-execing
itself if started anywhere else, so it cannot meet the real cell.

Faults - a std_msgs/String on /mock_cell/fault (the GUI's MOCK panel sends
these): reflex, driver_down, vision_stale, marker_lost, pose_jump, rt_dip,
user_stop, push, moveit_down, clear.
"""

import isolate
isolate.isolate()

import math  # noqa: E402
import pathlib  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import ListControllers, SwitchController  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue  # noqa: E402
from franka_msgs.action import ErrorRecovery  # noqa: E402
from franka_msgs.msg import FrankaRobotState  # noqa: E402
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetLoad  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from lifecycle_msgs.msg import TransitionEvent  # noqa: E402
from moveit_msgs.action import ExecuteTrajectory  # noqa: E402
from moveit_msgs.srv import GetCartesianPath  # noqa: E402
from rcl_interfaces.msg import SetParametersResult  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor  # noqa: E402,E501
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
ARM, IMP = 'fr3_arm_controller', 'cartesian_impedance_stroke_controller'
CALIB = yaml.safe_load((HERE.parent / 'calib' / 'handeye.yaml').read_text())
JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
LIMITS = [(-2.7437, 2.7437), (-1.7837, 1.7837), (-2.9007, 2.9007), (-3.0421, -0.1518),
          (-2.8065, 2.8065), (0.5445, 4.5169), (-3.0159, 3.0159)]
Q0 = np.array([0.0, -0.40, 0.0, -2.10, 0.0, 1.75, 0.785])
# Planned speed at 100 %: the panel stretches it by 100 / its speed setting.
PLAN_MPS, PLAN_RPS = 0.25, 1.0
FX = FY = 430.0
CX, CY, W, H = 320.0, 240.0, 640, 480
MARKER_M = 0.04
TRACK_PROFILE = {'k_pos_tool': [1500.0] * 3, 'k_rot_tool': [90.0] * 3,
                 'damping_ratio': 0.5, 'setpoint_slew_mps': 0.10,
                 'setpoint_slew_rps': 0.5}


def homog(pos, quat_xyzw):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    m[:3, 3] = pos
    return m


T_TCP_CAM = homog(CALIB['xyz'], CALIB['quat_xyzw'])


def pose_msg(m, frame, stamp):
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, m[:3, 3])
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = map(float, q)
    return msg


def tf_msg(m, parent, child, stamp):
    p = pose_msg(m, parent, stamp).pose
    t = TransformStamped()
    t.header.frame_id, t.header.stamp, t.child_frame_id = parent, stamp, child
    t.transform.translation.x = p.position.x
    t.transform.translation.y = p.position.y
    t.transform.translation.z = p.position.z
    t.transform.rotation = p.orientation
    return t


def step_toward(cur, goal, max_m, max_rad):
    """Move pose cur toward goal by at most max_m / max_rad."""
    out = cur.copy()
    d = goal[:3, 3] - cur[:3, 3]
    n = np.linalg.norm(d)
    out[:3, 3] += d if n <= max_m else d * (max_m / n)
    r_rel = Rotation.from_matrix(goal[:3, :3] @ cur[:3, :3].T)
    ang = np.linalg.norm(r_rel.as_rotvec())
    if ang > 1e-9:
        f = min(1.0, max_rad / ang)
        out[:3, :3] = (Rotation.from_rotvec(r_rel.as_rotvec() * f).as_matrix()
                       @ cur[:3, :3])
    return out


class World:
    """The one shared state. Every node below reads and writes it under lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.tcp = homog([0.45, 0.02, 0.42],
                         Rotation.from_euler('xyz', [math.pi, 0, 0]).as_quat())
        self.tcp0 = self.tcp.copy()
        # The marker on the table, seen off-centre, tilted 3 deg and turned
        # to 70 deg in the image: ALIGN has all four errors to null.
        ip = math.radians(70.0)
        x, z = np.array([math.cos(ip), math.sin(ip), 0.0]), np.array([0.0, 0.0, -1.0])
        t_cam_marker = np.eye(4)
        t_cam_marker[:3, :3] = (Rotation.from_euler('x', 3, degrees=True).as_matrix()
                                @ np.column_stack([x, np.cross(z, x), z]))
        t_cam_marker[:3, 3] = [0.028, -0.019, 0.27]
        self.marker = self.tcp @ T_TCP_CAM @ t_cam_marker
        self.controllers = {ARM: 'active', IMP: 'inactive',
                            'joint_state_broadcaster': 'active',
                            'franka_robot_state_broadcaster': 'active'}
        self.eq = None
        self.float_mode = False
        self.slew = [0.05, 0.5]
        self.plan = None          # goal pose of the last compute_cartesian_path
        self.traj = None          # (start, goal, t0, duration)
        self.q = Q0.copy()
        self.dq = np.zeros(7)
        self.reflex = self.user_stop = False
        self.errors = set()
        self.last_errors = set()
        self.faults = set()
        self.push = np.zeros(3)
        self.thresholds = None
        self.load_mass = None

    def robot_mode(self):
        if self.reflex:
            return 4
        if self.user_stop:
            return 5
        active = any(self.controllers.get(c) == 'active' for c in (ARM, IMP))
        return 2 if active else 1

    def step(self, dt):
        mode = self.robot_mode()
        if mode == 2 and self.traj is not None and self.controllers[ARM] == 'active':
            start, goal, t0, dur = self.traj
            f = min(1.0, (time.monotonic() - t0) / max(dur, 1e-3))
            slerp = Slerp([0, 1], Rotation.from_matrix([start[:3, :3], goal[:3, :3]]))
            self.tcp = np.eye(4)
            self.tcp[:3, :3] = slerp(f).as_matrix()
            self.tcp[:3, 3] = start[:3, 3] + f * (goal[:3, 3] - start[:3, 3])
            if f >= 1.0:
                self.traj = None
        elif (mode == 2 and self.controllers[IMP] == 'active' and not self.float_mode
              and self.eq is not None):
            self.tcp = step_toward(self.tcp, self.eq, self.slew[0] * dt, self.slew[1] * dt)
        # Joints: a fixed linear map from the TCP displacement, plus the
        # in-plane turn on J7. Not kinematics - enough for limits and speeds.
        d = self.tcp[:3, 3] - self.tcp0[:3, 3]
        yaw = Rotation.from_matrix(self.tcp[:3, :3] @ self.tcp0[:3, :3].T).as_rotvec()[2]
        q = Q0 + np.array([d[1] * 2.0, d[0] * 1.5 - d[2] * 0.8, 0.0, d[2] * 2.2 + d[0],
                           0.0, -d[2] * 1.4, -yaw])
        self.dq = (q - self.q) / dt
        self.q = q


class Arm(Node):
    """Robot state, joint states, TF, vision and the fault switch."""

    def __init__(self, w, cell):
        super().__init__('fake_cell')
        self.w, self.cell = w, cell
        self.tfb = TransformBroadcaster(self)
        StaticTransformBroadcaster(self).sendTransform(tf_msg(
            T_TCP_CAM, CALIB['parent_frame'], CALIB['child_frame'],
            self.get_clock().now().to_msg()))
        self.state_pub = self.create_publisher(
            FrankaRobotState, '/franka_robot_state_broadcaster/robot_state', 1)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 1)
        self.raw_pub = self.create_publisher(PoseStamped, '/aruco/pose_raw', 1)
        self.img_pub = self.create_publisher(Image, '/aruco/debug_image', 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_pub = self.create_publisher(String, '/robot_description', latched)
        self.desc_pub.publish(String(data=self._urdf()))
        self.create_subscription(String, '/mock_cell/fault', self._fault, 10)
        self.create_timer(0.01, self._tick)
        self.create_timer(1.0 / 15, self._vision)
        self.create_timer(0.2, self._image)

    @staticmethod
    def _urdf():
        joints = ''.join(
            f'<joint name="{n}" type="revolute"><parent link="l{i}"/><child link="l{i+1}"/>'
            f'<limit lower="{lo}" upper="{hi}" effort="87" velocity="2.62"/></joint>'
            for i, (n, (lo, hi)) in enumerate(zip(JOINTS, LIMITS)))
        links = ''.join(f'<link name="l{i}"/>' for i in range(8))
        return f'<robot name="fr3_mock">{links}{joints}</robot>'

    def _fault(self, msg):
        f = msg.data.strip()
        w = self.w
        with w.lock:
            if f == 'clear':
                w.faults.clear()
                w.reflex = w.user_stop = False
                w.errors.clear()
                w.push[:] = 0.0
            elif f == 'reflex':
                w.reflex = True
                w.errors = {'cartesian_reflex'}
                w.last_errors = {'cartesian_reflex'}
                w.traj = None
            elif f == 'user_stop':
                w.user_stop = True
                w.traj = None
            elif f == 'push':
                w.push[:] = [0.0, 3.0, -11.5]
            elif f in ('vision_stale', 'marker_lost', 'pose_jump', 'rt_dip'):
                w.faults.add(f)
        if f == 'driver_down':
            self.cell.driver(False)
        elif f == 'moveit_down':
            self.cell.moveit(False)
        elif f == 'clear':
            self.cell.driver(True)
            self.cell.moveit(True)
        self.get_logger().info(f'fault: {f}')

    def _tick(self):
        w = self.w
        with w.lock:
            w.step(0.01)
            tcp, q, dq = w.tcp.copy(), w.q.copy(), w.dq.copy()
            mode, driver = w.robot_mode(), self.cell.driver_up
            errors, last = set(w.errors), set(w.last_errors)
            force = w.push + np.random.normal(0.0, 0.15, 3)
            rate = 0.93 if 'rt_dip' in w.faults else 0.999
        now = self.get_clock().now().to_msg()
        links = [tf_msg(tcp, 'fr3_link0', 'fr3_hand_tcp', now)]
        base = np.array([0.0, 0.0, 0.333])
        for i, name in enumerate(['fr3_link1', 'fr3_link2', 'fr3_link3', 'fr3_link4',
                                  'fr3_link5', 'fr3_link6', 'fr3_link7', 'fr3_link8',
                                  'fr3_hand', 'fr3_leftfinger', 'fr3_rightfinger']):
            f = min(1.0, (i + 1) / 9.0)
            m = np.eye(4)
            m[:3, 3] = base + f * (tcp[:3, 3] - base) + [0, 0, 0.12 * math.sin(math.pi * f)]
            links.append(tf_msg(m, 'fr3_link0', name, now))
        self.tfb.sendTransform(links)
        if not driver:
            return
        s = FrankaRobotState()
        s.header.stamp = now
        s.robot_mode = mode
        s.o_t_ee = pose_msg(tcp, 'fr3_link0', now)
        s.o_f_ext_hat_k.wrench.force.x, s.o_f_ext_hat_k.wrench.force.y, \
            s.o_f_ext_hat_k.wrench.force.z = map(float, force)
        s.control_command_success_rate = rate
        s.measured_joint_state.name = JOINTS
        s.measured_joint_state.position = [float(v) for v in q]
        s.measured_joint_state.velocity = [float(v) for v in dq]
        s.measured_joint_state.effort = [0.2, -21.0, 0.4, 14.0, 0.6, 2.1, 0.1]
        for e in errors:
            setattr(s.current_errors, e, True)
        for e in last:
            setattr(s.last_motion_errors, e, True)
        self.state_pub.publish(s)
        js = JointState()
        js.header.stamp = now
        js.name, js.position = JOINTS, [float(v) for v in q]
        self.js_pub.publish(js)

    def cam(self):
        with self.w.lock:
            return self.w.tcp @ T_TCP_CAM, self.w.marker.copy(), set(self.w.faults)

    def visible(self, t_cam_marker):
        p = t_cam_marker[:3, 3]
        return p[2] > 0.03 and math.degrees(math.atan2(np.hypot(p[0], p[1]), p[2])) < 35

    def _vision(self):
        cam, marker, faults = self.cam()
        if 'vision_stale' in faults or 'marker_lost' in faults:
            return
        t_cm = np.linalg.inv(cam) @ marker
        if not self.visible(t_cm):
            return
        m = marker.copy()
        m[:3, 3] += np.random.normal(0.0, 0.0002, 3)
        if 'pose_jump' in faults:
            m[:3, 3] += [0.03, 0.0, 0.0]
            with self.w.lock:
                self.w.faults.discard('pose_jump')
        now = self.get_clock().now().to_msg()
        self.pose_pub.publish(pose_msg(m, 'fr3_link0', now))
        self.raw_pub.publish(pose_msg(t_cm, CALIB['child_frame'], now))

    def _image(self):
        """Subscribe-gated like cam_pub: nothing is drawn with no reader."""
        if self.img_pub.get_subscription_count() == 0:
            return
        cam, marker, faults = self.cam()
        if 'vision_stale' in faults:
            return
        import cv2
        img = np.full((H, W, 3), 38, np.uint8)
        img[:, :, 1] += (np.linspace(0, 18, W)[None, :]).astype(np.uint8)
        t_cm = np.linalg.inv(cam) @ marker
        if 'marker_lost' not in faults and self.visible(t_cm):
            h = MARKER_M / 2
            corners = np.array([[-h, -h, 0, 1], [h, -h, 0, 1], [h, h, 0, 1], [-h, h, 0, 1]])
            pc = (t_cm @ corners.T).T[:, :3]
            px = np.column_stack([FX * pc[:, 0] / pc[:, 2] + CX,
                                  FY * pc[:, 1] / pc[:, 2] + CY]).astype(np.int32)
            cv2.fillPoly(img, [px], (235, 235, 235))
            inner = (px - px.mean(axis=0)) * 0.72 + px.mean(axis=0)
            cv2.fillPoly(img, [inner.astype(np.int32)], (15, 15, 15))
            ctr = px.mean(axis=0).astype(int)
            for axis, col in (([0.03, 0, 0, 1], (0, 0, 230)), ([0, 0.03, 0, 1], (0, 200, 0))):
                a = t_cm @ np.array(axis)
                tip = (int(FX * a[0] / a[2] + CX), int(FY * a[1] / a[2] + CY))
                cv2.line(img, tuple(ctr), tip, col, 2)
        cv2.putText(img, 'MOCK CAMERA', (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (150, 150, 150), 1)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width, msg.encoding, msg.step = H, W, 'bgr8', W * 3
        msg.data = img.tobytes()
        self.img_pub.publish(msg)


class Impedance(Node):
    """The impedance controller's parameter surface, equilibrium and
    lifecycle transitions - the same limits as impedance_detail.hpp."""

    LIVE = {'k_pos_tool': (0.0, 3000.0), 'k_rot_tool': (0.0, 300.0),
            'damping_ratio': (0.1, 2.0), 'nullspace_stiffness': (0.0, 50.0),
            'setpoint_slew_mps': (0.001, 0.25), 'setpoint_slew_rps': (0.001, 1.0)}
    CONFIGURE_ONLY = ('arm_id', 'max_force_n', 'max_torque_nm', 'tau_max_nm',
                      'tau_rate_limit')

    def __init__(self, w):
        super().__init__(IMP)
        self.w = w
        self.declare_parameter('float_mode', False)
        self.declare_parameter('k_pos_tool', [150.0, 150.0, 800.0])
        self.declare_parameter('k_rot_tool', [10.0, 10.0, 20.0])
        self.declare_parameter('damping_ratio', 1.0)
        self.declare_parameter('nullspace_stiffness', 5.0)
        self.declare_parameter('setpoint_slew_mps', 0.05)
        self.declare_parameter('setpoint_slew_rps', 0.5)
        self.declare_parameter('max_force_n', 30.0)
        self.add_on_set_parameters_callback(self._check)
        self.trans_pub = self.create_publisher(TransitionEvent, '~/transition_event', 10)
        self.create_subscription(PoseStamped, '~/equilibrium_pose', self._eq, 1)

    def _check(self, params):
        slew = list(self.w.slew)
        for p in params:
            if p.name in self.CONFIGURE_ONLY:
                return SetParametersResult(successful=False,
                                           reason=f'{p.name} is read at configure time only')
            if p.name in self.LIVE:
                lo, hi = self.LIVE[p.name]
                vals = ([p.value] if isinstance(p.value, (int, float, bool))
                        else list(p.value))
                if not all(lo <= float(v) <= hi for v in vals):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be within [{lo:g}, {hi:g}]')
        for p in params:
            if p.name == 'setpoint_slew_mps':
                slew[0] = float(p.value)
            elif p.name == 'setpoint_slew_rps':
                slew[1] = float(p.value)
            elif p.name == 'float_mode':
                with self.w.lock:
                    if self.w.float_mode and not p.value:
                        self.w.eq = self.w.tcp.copy()      # float OFF re-seeds
                    self.w.float_mode = bool(p.value)
        with self.w.lock:
            self.w.slew = slew
        return SetParametersResult(successful=True)

    def _eq(self, msg):
        o = msg.pose.orientation
        q = np.array([o.x, o.y, o.z, o.w])
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        p = msg.pose.position
        with self.w.lock:
            if self.w.controllers[IMP] == 'active':
                self.w.eq = homog([p.x, p.y, p.z], q)

    def announce(self, active):
        ev = TransitionEvent()
        ev.start_state.id, ev.start_state.label = (2, 'inactive') if active else (3, 'active')
        ev.goal_state.id, ev.goal_state.label = (3, 'active') if active else (2, 'inactive')
        self.trans_pub.publish(ev)


class ControllerManager(Node):
    def __init__(self, w, imp):
        super().__init__('controller_manager')
        self.w, self.imp = w, imp
        self.create_service(ListControllers, '~/list_controllers', self._list)
        self.create_service(SwitchController, '~/switch_controller', self._switch)

    def _list(self, _req, res):
        with self.w.lock:
            res.controller = [ControllerState(name=n, state=s)
                              for n, s in self.w.controllers.items()]
        return res

    def _switch(self, req, res):
        w = self.w
        with w.lock:
            c = w.controllers
            ok = (not w.reflex
                  and all(c.get(n) == 'active' for n in req.deactivate_controllers)
                  and all(c.get(n) == 'inactive' for n in req.activate_controllers))
            if ok:
                for n in req.deactivate_controllers:
                    c[n] = 'inactive'
                for n in req.activate_controllers:
                    c[n] = 'active'
                if IMP in req.activate_controllers:
                    w.eq = w.tcp.copy()                     # seeded where the arm is
                if IMP in req.deactivate_controllers:
                    w.eq = None
                if ARM in req.deactivate_controllers:
                    w.traj = None
        if ok and IMP in req.activate_controllers:
            self.imp.announce(True)
        if ok and IMP in req.deactivate_controllers:
            self.imp.announce(False)
        res.ok = ok
        return res


class Franka(Node):
    """service_server (payload, reflex thresholds) - both refused unless the
    robot is idle, as franka does."""

    def __init__(self, w):
        super().__init__('service_server')
        self.w = w
        self.create_service(SetLoad, '~/set_load', self._load)
        self.create_service(SetForceTorqueCollisionBehavior,
                            '~/set_force_torque_collision_behavior', self._collision)

    def _idle(self):
        with self.w.lock:
            return self.w.robot_mode() == 1

    def _load(self, req, res):
        res.success = self._idle()
        res.error = '' if res.success else 'mock: rejected, the robot is not idle'
        if res.success:
            with self.w.lock:
                self.w.load_mass = req.mass
        return res

    def _collision(self, req, res):
        res.success = self._idle()
        res.error = '' if res.success else 'mock: rejected, the robot is not idle'
        if res.success:
            with self.w.lock:
                self.w.thresholds = list(req.upper_force_thresholds_nominal)
        return res


class Recovery(Node):
    def __init__(self, w):
        super().__init__('action_server')
        self.w = w
        self.srv = ActionServer(self, ErrorRecovery, '~/error_recovery', self._run)

    def _run(self, gh):
        time.sleep(0.5)
        with self.w.lock:
            self.w.reflex = False
            self.w.errors.clear()
        gh.succeed()
        return ErrorRecovery.Result()


class MoveGroup(Node):
    def __init__(self, w):
        super().__init__('move_group')
        self.w = w
        self.stop_flag = threading.Event()
        grp = ReentrantCallbackGroup()
        self.create_service(GetCartesianPath, '/compute_cartesian_path', self._plan,
                            callback_group=grp)
        self.srv = ActionServer(self, ExecuteTrajectory, '/execute_trajectory', self._exec,
                                cancel_callback=lambda _g: CancelResponse.ACCEPT,
                                callback_group=grp)
        self.create_subscription(String, '/trajectory_execution_event', self._event, 10,
                                 callback_group=grp)

    def _event(self, msg):
        if msg.data == 'stop':
            self.stop_flag.set()

    def _plan(self, req, res):
        p = req.waypoints[-1]
        goal = homog([p.position.x, p.position.y, p.position.z],
                     [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        with self.w.lock:
            start = self.w.tcp.copy()
            self.w.plan = goal
            q = [float(v) for v in self.w.q]
        dist = np.linalg.norm(goal[:3, 3] - start[:3, 3])
        ang = np.linalg.norm(Rotation.from_matrix(goal[:3, :3] @ start[:3, :3].T).as_rotvec())
        dur = max(0.3, dist / PLAN_MPS, ang / PLAN_RPS)
        jt = res.solution.joint_trajectory
        jt.joint_names = JOINTS
        from trajectory_msgs.msg import JointTrajectoryPoint
        a, b = JointTrajectoryPoint(), JointTrajectoryPoint()
        a.positions, b.positions = q, q
        a.velocities = b.velocities = [0.0] * 7
        a.accelerations = b.accelerations = [0.0] * 7
        b.time_from_start.sec = int(dur)
        b.time_from_start.nanosec = int((dur - int(dur)) * 1e9)
        jt.points = [a, b]
        res.fraction = 1.0 if goal[2, 3] > 0.02 else 0.4
        res.error_code.val = 1
        return res

    def _exec(self, gh):
        pts = gh.request.trajectory.joint_trajectory.points
        t = pts[-1].time_from_start
        dur = t.sec + t.nanosec * 1e-9
        self.stop_flag.clear()
        with self.w.lock:
            goal = self.w.plan
            self.w.traj = (self.w.tcp.copy(), goal, time.monotonic(), dur)
        res = ExecuteTrajectory.Result()
        while True:
            time.sleep(0.02)
            with self.w.lock:
                done = self.w.traj is None
                mode = self.w.robot_mode()
                if self.stop_flag.is_set() or gh.is_cancel_requested or mode != 2:
                    self.w.traj = None
            if gh.is_cancel_requested:
                gh.canceled()
                return res
            if self.stop_flag.is_set():
                res.error_code.val = -7                     # PREEMPTED
                gh.abort()
                return res
            if mode != 2:
                res.error_code.val = -4                     # CONTROL_FAILED
                gh.abort()
                return res
            if done:
                res.error_code.val = 1
                gh.succeed()
                return res


class Tracker(Node):
    """tracking_node's surface: start/stop, the latched ~/status, and a
    follow law that holds the camera over the marker at the standoff."""

    def __init__(self, w, cell):
        super().__init__('tracking_node')
        self.w, self.cell = w, cell
        self.declare_parameter('tracking_standoff_m', 0.10)
        self.declare_parameter('tracking_inplane_deg', 90.0)
        self.declare_parameter('tracking_inplane_hold', False)
        self.declare_parameter('tracking_over_lead_policy', 'hold')
        self.declare_parameter('tracking_box_x_m', [0.20, 0.80])
        self.declare_parameter('tracking_box_y_m', [-0.45, 0.45])
        self.declare_parameter('tracking_box_z_max_m', 0.80)
        self.add_on_set_parameters_callback(self._check)
        self.state, self.reason, self.level = 'idle', '', 0
        self.snapshot = None
        self.started_at = 0.0
        self.err = (float('nan'), float('nan'))
        self.lead = float('nan')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(DiagnosticStatus, '~/status', latched)
        self.create_service(Trigger, '~/start_tracking', self._start)
        self.create_service(Trigger, '~/stop_tracking', self._stop)
        self.eq_pub = self.create_publisher(PoseStamped, f'/{IMP}/equilibrium_pose', 1)
        self.create_timer(0.02, self._tick)
        self.create_timer(0.2, self._publish)

    def _check(self, params):
        for p in params:
            if p.name == 'tracking_over_lead_policy' and p.value not in ('hold', 'stop', 'clamp'):
                return SetParametersResult(successful=False,
                                           reason='tracking_over_lead_policy must be hold, '
                                                  'stop or clamp')
        return SetParametersResult(successful=True)

    def _set(self, state, reason='', level=0):
        changed = (state, reason) != (self.state, self.reason)
        self.state, self.reason, self.level = state, reason, level
        if changed:
            self._publish()

    def _imp(self):
        return self.cell.imp

    def _start(self, _req, res):
        with self.w.lock:
            active = self.w.controllers[IMP] == 'active'
            floating = self.w.float_mode
        imp = self._imp()
        if not active or imp is None:
            res.success, res.message = False, f'{IMP} is not active'
            return res
        if floating:
            res.success, res.message = False, 'float_mode is on - HOLD first'
            return res
        self.snapshot = [imp.get_parameter(k).value for k in TRACK_PROFILE]
        imp.set_parameters([Parameter(k, value=v) for k, v in TRACK_PROFILE.items()])
        self.started_at = time.monotonic()
        self._set('starting')
        res.success, res.message = True, 'tracking (mock profile track)'
        return res

    def _stop(self, _req, res):
        was = self.state
        self._halt('stopped by operator', 0)
        res.success, res.message = True, ('stopped' if was != 'idle' else 'was not tracking')
        return res

    def _halt(self, reason, level):
        imp = self._imp()
        if self.snapshot is not None and imp is not None:
            imp.set_parameters([Parameter(k, value=v)
                                for k, v in zip(TRACK_PROFILE, self.snapshot)])
        self.snapshot = None
        with self.w.lock:
            if self.w.controllers.get(IMP) == 'active':
                self.w.eq = self.w.tcp.copy()
        self._set('idle', reason, level)

    def _tick(self):
        if self.state == 'idle':
            return
        with self.w.lock:
            active = self.w.controllers.get(IMP) == 'active'
            tcp, marker, faults = self.w.tcp.copy(), self.w.marker.copy(), set(self.w.faults)
        if not active or self._imp() is None:
            self.snapshot = None
            self._set('idle', f'{IMP} left ACTIVE', 1)
            return
        if self.state == 'starting' and time.monotonic() - self.started_at < 1.0:
            return
        if 'vision_stale' in faults or 'marker_lost' in faults:
            if self.state != 'holding':
                self._publish_eq(tcp)
            self._set('holding', 'marker stale', 1)
            return
        standoff = self.get_parameter('tracking_standoff_m').value
        n = marker[:3, 2]                                   # marker normal, toward the camera
        cam = tcp @ T_TCP_CAM
        cam_goal = cam.copy()
        cam_goal[:3, 3] = marker[:3, 3] + standoff * n
        goal = cam_goal @ np.linalg.inv(T_TCP_CAM)
        lead = np.linalg.norm(goal[:3, 3] - tcp[:3, 3])
        self.err = (float(np.linalg.norm(cam_goal[:3, 3] - cam[:3, 3]) * 1000), 0.2)
        self.lead = float(min(lead, 0.01) * 1000)
        if lead > 0.06 and self.get_parameter('tracking_over_lead_policy').value == 'hold':
            self._set('holding', f'goal {lead*1000:.0f} mm beyond the 60 mm lead cap', 1)
            return
        if goal[2, 3] < 0.10:
            self._set('holding', 'goal below the floor 100 mm', 1)
            return
        self._set('tracking')
        self._publish_eq(step_toward(tcp, goal, 0.01, 0.017))

    def _publish_eq(self, m):
        self.eq_pub.publish(pose_msg(m, 'fr3_link0', self.get_clock().now().to_msg()))

    def _publish(self):
        msg = DiagnosticStatus()
        msg.level = bytes([self.level])
        msg.name = 'tracking_node'
        msg.message = self.state + (f' - {self.reason}' if self.reason else '')
        vals = {'state': self.state, 'reason': self.reason,
                'policy': self.get_parameter('tracking_over_lead_policy').value,
                'pos_err_mm': f'{self.err[0]:.1f}', 'rot_err_deg': f'{self.err[1]:.2f}',
                'lead_mm': f'{self.lead:.1f}', 'lead_deg': '0.10',
                'marker_age_s': '0.050', 'raw_age_s': '0.050', 'buzz_nm': '0.02',
                'standoff_m': f'{self.get_parameter("tracking_standoff_m").value:.3f}',
                'inplane_deg': f'{self.get_parameter("tracking_inplane_deg").value:.1f}'}
        msg.values = [KeyValue(key=k, value=v) for k, v in vals.items()]
        self.status_pub.publish(msg)


class MockCell:
    """Owns the executor, so the driver and MoveIt can die and come back."""

    def __init__(self):
        self.w = World()
        self.ex = MultiThreadedExecutor(num_threads=8)
        self.driver_nodes, self.moveit_nodes = [], []
        self.imp = None
        self.driver_up = False
        self.driver(True)
        self.moveit(True)
        self.arm = Arm(self.w, self)
        self.tracker = Tracker(self.w, self)
        for n in (self.arm, self.tracker):
            self.ex.add_node(n)

    def driver(self, up):
        """ros2_control_node and everything in it: controller manager, the
        impedance controller, franka's service and action servers."""
        if up == self.driver_up:
            return
        if up:
            with self.w.lock:
                self.w.controllers.update({ARM: 'active', IMP: 'inactive'})
                self.w.eq, self.w.traj = None, None
                self.w.reflex = False
                self.w.errors.clear()
            self.imp = Impedance(self.w)
            self.driver_nodes = [self.imp, ControllerManager(self.w, self.imp),
                                 Franka(self.w), Recovery(self.w)]
            for n in self.driver_nodes:
                self.ex.add_node(n)
        else:
            nodes, self.driver_nodes, self.imp = self.driver_nodes, [], None
            for n in nodes:
                self.ex.remove_node(n)
                n.destroy_node()
        self.driver_up = up

    def moveit(self, up):
        if up and not self.moveit_nodes:
            self.moveit_nodes = [MoveGroup(self.w)]
            self.ex.add_node(self.moveit_nodes[0])
        elif not up:
            for n in self.moveit_nodes:
                self.ex.remove_node(n)
                n.destroy_node()
            self.moveit_nodes = []


def main():
    assert isolate.isolated()
    rclpy.init()
    cell = MockCell()
    print(f'mock cell up on ROS_DOMAIN_ID {isolate.DOMAIN} (loopback only)', flush=True)
    try:
        cell.ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                       # noqa: BLE001
        if rclpy.ok():                  # a real failure, not SIGTERM's shutdown
            raise
    finally:
        cell.ex.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
