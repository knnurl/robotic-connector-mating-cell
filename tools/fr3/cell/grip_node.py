#!/usr/bin/env python3
"""grip_node: grip a marked cube with the Franka Hand on the impedance
controller, one GRIP press at a time.

    ~/grip   Trigger, blocks until done: read the cube from the marker, open
             the Hand, glide the equilibrium above the cube, descend, grasp,
             lift. The arm stays on the impedance controller throughout, with
             the operator's gains - compliant if the height is off.
    ~/place  Trigger, blocks: the reverse, back down to where it was gripped,
             open, up again.
    ~/stop   Trigger, instant: the running sequence holds where the arm is.
             The Hand is left as it is (never drops the cube).
    ~/status DiagnosticStatus, latched: state, step, holding, reason.

Rules it keeps: the cube pose is read ONCE, from fresh raw detections, while
the camera can still see the marker - on the way down it cannot. Every
target is checked against the Z floor and the workspace box before anything
moves, and the friction lead is clamped inside them. Contact is a change of
more than grip_contact_n in the external force since the move began: on the
way down it holds (GRIP), or it is the set-down (PLACE, only within
grip_arrive_window_m of the target - earlier it holds with the Hand closed).
PLACE goes over the set-down point before lowering. A joint within
grip_joint_margin_rad of its stop holds. STOP cancels a gripper goal in
flight. cell_panel keeps GRIP and TRACK apart: both command the equilibrium.

Run by fr3_cell.launch.py with fr3_params.yaml; parameters grip_*.
"""

import datetime
import json
import math
import pathlib
import sys
import threading
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import rclpy                                                    # noqa: E402
from rclpy.action import ActionClient                           # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup        # noqa: E402
from rclpy.duration import Duration                             # noqa: E402
from rclpy.executors import MultiThreadedExecutor               # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time                                     # noqa: E402
import tf2_ros                                                  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue      # noqa: E402
from franka_msgs.action import Grasp, Move                      # noqa: E402
from franka_msgs.msg import FrankaRobotState                    # noqa: E402
from geometry_msgs.msg import PoseStamped                       # noqa: E402
from rcl_interfaces.srv import GetParameters                    # noqa: E402
from std_srvs.srv import Trigger                                # noqa: E402

import core                                                     # noqa: E402
import grip_logic as G                                          # noqa: E402
from state_relay import start_throttle, stop_throttle           # noqa: E402

RATE_HZ = 50.0
DEFAULTS = {
    'equilibrium_topic': '/cartesian_impedance_stroke_controller/equilibrium_pose',
    'impedance_controller': core.IMPEDANCE_CONTROLLER,
    'tracking_raw_pose_topic': '/aruco/pose_raw',
    'tracking_z_floor_m': 0.10,
    'tracking_box_x_m': [0.20, 0.80],
    'tracking_box_y_m': [-0.45, 0.45],
    'tracking_box_z_max_m': 0.80,
    'tracking_joint_lower': [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159],
    'tracking_joint_upper': [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
    'grip_base_frame': 'fr3_link0',
    'grip_tcp_frame': 'fr3_hand_tcp',
    'grip_cube_m': 0.055,
    'grip_open_margin_m': 0.020,
    'grip_depth_m': 0.025,
    'grip_approach_m': 0.10,
    'grip_lift_m': 0.05,
    'grip_force_n': 20.0,
    'grip_epsilon_m': 0.008,
    'grip_travel_mps': 0.04,
    'grip_descend_mps': 0.015,
    'grip_turn_radps': 0.4,
    'grip_contact_n': 10.0,
    'grip_tol_m': 0.003,
    'grip_tol_deg': 3.0,
    'grip_settle_s': 8.0,
    'grip_ki': 0.5,
    'grip_raw_timeout_s': 0.25,
    'grip_joint_margin_rad': 0.05,
    'grip_arrive_window_m': 0.015,
}
FRICTION_N = 6.5                 # tracking_law kFrictionBreakawayN
FRICTION_NM = 0.6                # tracking_law kFrictionBreakawayNm
LEAD_FORCE_MAX_N = 15.0          # tracking_law kLeadForceMaxN, per tool axis here
LEAD_TORQUE_MAX_NM = 1.5         # the track profile's 0.017 rad * 90 Nm/rad
STOP_GRACE_S = 1.0               # a STOP this recent refuses a GRIP/PLACE that follows it


class Stopped(Exception):
    pass


class Failed(Exception):
    pass


def pose_of(msg_pose):
    p = msg_pose.position
    q = msg_pose.orientation
    return np.array([p.x, p.y, p.z]), core.q2R(q.x, q.y, q.z, q.w)


class GripNode(Node):
    def __init__(self):
        super().__init__('grip_node', automatically_declare_parameters_from_overrides=True)
        for name, value in DEFAULTS.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
        cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._state = None               # (ee (p, R), F_ext (3,) N, q, monotonic)
        self._raw = []                   # [(PoseStamped, monotonic)]
        self._busy = threading.Lock()
        self._stop = threading.Event()
        self._stop_t = -math.inf         # monotonic time of the last ~/stop
        self.holding = None              # (grasp tcp pose, marker R, T_tcp_ee) while gripped
        self.last_eq = None              # the equilibrium last published, this sequence
        self.logf = None

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.eq_pub = self.create_publisher(PoseStamped, self.p('equilibrium_topic'), 10)
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '~/status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.relay = start_throttle(core.ROBOT_STATE_TOPIC, '/grip_node/robot_state',
                                    hz=RATE_HZ, node_name='grip_node_relay')
        self.create_subscription(FrankaRobotState, '/grip_node/robot_state', self._state_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=cb)
        self.create_subscription(PoseStamped, self.p('tracking_raw_pose_topic'), self._raw_cb,
                                 10, callback_group=cb)
        imp = self.p('impedance_controller')
        self.get_params_cli = self.create_client(GetParameters, f'/{imp}/get_parameters',
                                                 callback_group=cb)
        self.move_ac = ActionClient(self, Move, '/franka_gripper/move', callback_group=cb)
        self.grasp_ac = ActionClient(self, Grasp, '/franka_gripper/grasp', callback_group=cb)
        self.create_service(Trigger, '~/grip', self._grip_srv, callback_group=cb)
        self.create_service(Trigger, '~/place', self._place_srv, callback_group=cb)
        self.create_service(Trigger, '~/stop', self._stop_srv, callback_group=cb)
        self.report('idle', '', 'ready')
        self.get_logger().info('grip_node ready: ~/grip, ~/place, ~/stop')

    # ------------------------------------------------------------ inputs

    def p(self, name):
        return self.get_parameter(name).value

    def _state_cb(self, m):
        ee = pose_of(m.o_t_ee.pose)
        f = m.o_f_ext_hat_k.wrench.force
        with self._lock:
            self._state = (ee, np.array([f.x, f.y, f.z]),
                           list(m.measured_joint_state.position), time.monotonic())

    def _raw_cb(self, m):
        now = time.monotonic()
        with self._lock:
            self._raw = [(r, t) for r, t in self._raw if now - t < 1.0] + [(m, now)]

    def state(self):
        with self._lock:
            s = self._state
        if s is None or time.monotonic() - s[3] > 0.1:
            raise Failed('no fresh robot state - is the driver (T1) up?')
        return s

    # ------------------------------------------------------------ outputs

    def report(self, state, step, reason, level=DiagnosticStatus.OK):
        msg = DiagnosticStatus(name='grip_node', level=level, message=reason)
        msg.values = [KeyValue(key='state', value=state), KeyValue(key='step', value=step),
                      KeyValue(key='holding', value='true' if self.holding else 'false'),
                      KeyValue(key='reason', value=reason)]
        self.status_pub.publish(msg)

    def publish(self, pose):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.p('grip_base_frame')
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) = (float(v) for v in pose[0])
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = G.R2q(pose[1])
        self.eq_pub.publish(m)
        self.last_eq = pose

    def trace(self, rec):
        if self.logf is not None:
            rec['t'] = time.time()
            self.logf.write(json.dumps(rec, default=lambda v: np.asarray(v).tolist()) + '\n')
            self.logf.flush()

    def hold(self):
        """Publish where the arm is: the controller's slew converges on it."""
        try:
            self.publish(self.state()[0])
        except Failed:
            pass

    # ------------------------------------------------------------ services

    def _stop_srv(self, _req, res):
        self._stop_t = time.monotonic()
        self._stop.set()
        res.success, res.message = True, 'grip stopping - holding where the arm is'
        return res

    def _grip_srv(self, _req, res):
        return self._run(res, 'grip', self.grip)

    def _place_srv(self, _req, res):
        return self._run(res, 'place', self.place)

    def _run(self, res, name, fn):
        if not self._busy.acquire(blocking=False):
            res.success, res.message = False, 'a grip or place is already running'
            return res
        try:
            # A STOP NOW that overtook this request (a separate service, no
            # ordering) must not be cleared and forgotten.
            if time.monotonic() - self._stop_t < STOP_GRACE_S:
                res.success, res.message = False, f'{name} not started - STOP NOW came just before it'
                self.report('stopped', '', res.message, DiagnosticStatus.WARN)
                return res
            self._stop.clear()
            self.last_eq = None
            try:
                folder = core.trace_dir()
                folder.mkdir(parents=True, exist_ok=True)
                self.logf = (folder / f'grip_{datetime.datetime.now():%Y%m%d_%H%M%S_%f}.jsonl'
                             ).open('x')
            except OSError as e:
                res.success, res.message = False, f'{name} not started: cannot write its log ({e})'
                self.report('failed', '', res.message, DiagnosticStatus.WARN)
                return res
            try:
                self.trace({'rec': 'start', 'what': name,
                            'params': {k: self.p(k) for k in DEFAULTS}})
                msg = fn()
                res.success, res.message = True, msg
                self.report('holding' if self.holding else 'idle', '', msg)
            except Stopped:
                self.hold()
                res.success, res.message = False, f'{name} stopped - holding where the arm is'
                self.report('stopped', '', res.message, DiagnosticStatus.WARN)
            except Failed as e:
                self.hold()
                res.success, res.message = False, f'{name} refused/failed: {e}'
                self.report('failed', '', res.message, DiagnosticStatus.WARN)
            except Exception as e:                              # noqa: BLE001
                self.hold()
                res.success, res.message = False, f'{name} crashed: {e!r} - holding'
                self.report('failed', '', res.message, DiagnosticStatus.ERROR)
            finally:
                self.trace({'rec': 'end', 'ok': res.success, 'msg': res.message})
                self.logf.close()
                self.logf = None
        finally:
            self._busy.release()
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------ the sequences

    def grip(self):
        if self.holding:
            raise Failed('already holding a cube - PLACE it first')
        cube, margin = self.p('grip_cube_m'), self.p('grip_open_margin_m')
        why = G.cube_problem(cube, margin)
        if why:
            raise Failed(why)
        self.check_controller()
        marker = self.marker_in_base()
        t_tcp_ee = self.tool_offset()
        ee_now = self.state()[0]
        tcp_now = G.compose(ee_now, G.inverse(t_tcp_ee))
        grasp = G.grasp_tcp(marker[0], marker[1], tcp_now[1],
                            G.grasp_depth(cube, self.p('grip_depth_m')))
        approach = G.above(grasp, marker[1], self.p('grip_approach_m'))
        lift = G.above(grasp, marker[1], self.p('grip_lift_m'))
        for name, pose in (('approach', approach), ('grasp', grasp), ('lift', lift)):
            self.check_target(name, G.compose(pose, t_tcp_ee))
        self.trace({'rec': 'plan', 'marker': marker, 'grasp': grasp, 'approach': approach,
                    'lift': lift, 't_tcp_ee': t_tcp_ee})
        self.step('open', lambda: self.hand_move(G.open_width(cube, margin)))
        self.step('approach', lambda: self.move_to(approach, t_tcp_ee, self.p('grip_travel_mps')))
        self.step('descend', lambda: self.move_to(grasp, t_tcp_ee, self.p('grip_descend_mps'),
                                                  contact='hold'))
        # From here the Hand may close on the cube whatever else happens (a
        # STOP leaves a grasp where it got to), so assume it holds: PLACE is
        # then the way out, and it is harmless if the fingers are open.
        self.holding = (grasp, marker[1], t_tcp_ee)
        try:
            self.step('grasp', lambda: self.hand_grasp(cube))
        except Failed as e:
            self.holding = None
            self.hand_move(G.open_width(cube, margin))
            self.move_to(approach, t_tcp_ee, self.p('grip_travel_mps'))
            raise Failed(f'{e} - opened and backed off above the cube') from None
        self.step('lift', lambda: self.move_to(lift, t_tcp_ee, self.p('grip_travel_mps')))
        return f'gripped the {cube*1000:.0f} mm cube and lifted it {self.p("grip_lift_m")*1000:.0f} mm'

    def place(self):
        if not self.holding:
            raise Failed('not holding a cube')
        grasp, marker_R, t_tcp_ee = self.holding
        self.check_controller()
        approach = G.above(grasp, marker_R, self.p('grip_approach_m'))
        for name, pose in (('approach', approach), ('set-down', grasp)):
            self.check_target(name, G.compose(pose, t_tcp_ee))
        # Over the set-down point first, whatever the arm did since GRIP:
        # never a long diagonal with the cube at table height.
        self.step('over', lambda: self.move_to(approach, t_tcp_ee, self.p('grip_travel_mps'),
                                               contact='hold'))
        self.step('lower', lambda: self.move_to(grasp, t_tcp_ee, self.p('grip_descend_mps'),
                                                contact='arrive'))
        self.step('open', lambda: self.hand_move(
            G.open_width(self.p('grip_cube_m'), self.p('grip_open_margin_m'))))
        self.holding = None
        self.step('retreat', lambda: self.move_to(approach, t_tcp_ee, self.p('grip_travel_mps')))
        return 'placed the cube and backed off above it'

    def step(self, name, fn):
        if self._stop.is_set():
            raise Stopped()
        self.report('busy', name, f'{name}...')
        self.trace({'rec': 'step', 'step': name})
        fn()

    # ------------------------------------------------------------ pieces

    def check_controller(self):
        """Holding on the impedance controller, not floating - and its
        stiffness, which the friction integrator is scaled by."""
        if not self.get_params_cli.wait_for_service(timeout_sec=1.0):
            raise Failed(f'{self.p("impedance_controller")} is not answering - HOLD first')
        req = GetParameters.Request(names=['float_mode', 'k_pos_tool', 'k_rot_tool'])
        fut = self.get_params_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.01)
        if not fut.done() or fut.result() is None or len(fut.result().values) != 3:
            raise Failed('could not read the impedance controller parameters')
        floating, kp, kr = fut.result().values
        if floating.bool_value:
            raise Failed('the controller is floating - press HOLD first')
        self.k_pos_axes = np.array(list(kp.double_array_value), dtype=float)
        self.k_rot_axes = np.array(list(kr.double_array_value), dtype=float)
        if (self.k_pos_axes.shape != (3,) or self.k_rot_axes.shape != (3,)
                or self.k_pos_axes.min() <= 0.0 or self.k_rot_axes.min() <= 0.0):
            raise Failed('a stiffness in force is zero or missing - apply gains first')

    def marker_in_base(self):
        """The mean of the fresh raw detections, each in the base frame at
        its own image stamp (eye-in-hand: the arm must not have moved)."""
        now = time.monotonic()
        with self._lock:
            raw = [r for r, t in self._raw if now - t < 0.5]
        if not raw or now - max(t for _, t in self._raw) > self.p('grip_raw_timeout_s'):
            raise Failed('the marker is not in view - GRIP reads the cube from it')
        if len(raw) < 3:
            raise Failed(f'only {len(raw)} fresh detection(s) - hold still and try again')
        base = self.p('grip_base_frame')
        poses = []
        for r in raw[-5:]:
            try:
                tf = self.tf.lookup_transform(base, r.header.frame_id, r.header.stamp,
                                              timeout=Duration(seconds=0.1))
            except tf2_ros.TransformException as e:
                raise Failed(f'no transform to {base} at the image stamp ({e})') from None
            t = tf.transform
            cam = (np.array([t.translation.x, t.translation.y, t.translation.z]),
                   core.q2R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
            poses.append(G.compose(cam, pose_of(r.pose)))
        p = np.mean([q[0] for q in poses], axis=0)
        spread = max(float(np.linalg.norm(q[0] - p)) for q in poses)
        if spread > 0.005:
            raise Failed(f'the marker pose scatters {spread*1000:.1f} mm - hold still')
        return p, poses[-1][1]

    def tool_offset(self):
        """T_tcp_ee: TF's hand TCP against the controller's o_t_ee, the arm still."""
        base, tcp = self.p('grip_base_frame'), self.p('grip_tcp_frame')
        try:
            tf = self.tf.lookup_transform(base, tcp, Time(), timeout=Duration(seconds=0.5))
        except tf2_ros.TransformException as e:
            raise Failed(f'no TF {base} -> {tcp} ({e})') from None
        t = tf.transform
        T_tcp = (np.array([t.translation.x, t.translation.y, t.translation.z]),
                 core.q2R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        return G.compose(G.inverse(T_tcp), self.state()[0])

    def check_target(self, name, ee):
        box = (*self.p('tracking_box_x_m'), *self.p('tracking_box_y_m'),
               self.p('tracking_box_z_max_m'))
        why = G.target_problem(ee[0], self.p('tracking_z_floor_m'), box)
        if why:
            raise Failed(f'the {name} pose {why}')

    def move_to(self, tcp_target, t_tcp_ee, speed, contact=None):
        """Glide the equilibrium to tcp_target, then settle on it with a
        friction integrator (like TRACK's lead, rotation included) until
        within grip_tol_m / grip_tol_deg.

        contact, judged as |F_ext - F_ext at the start| so a held cube's
        weight or a model bias is not contact: None; 'hold' (fail above
        grip_contact_n, the Hand stays closed); 'arrive' (PLACE: a contact
        within grip_arrive_window_m of the target is the set-down - earlier
        it fails like 'hold')."""
        target = G.compose(tcp_target, t_tcp_ee)
        # From the equilibrium last commanded, lead included: starting from
        # the measured pose would drop up to 15 N of pull in one step.
        start = self.last_eq if self.last_eq is not None else self.state()[0]
        dur = G.glide_time(start, target, speed, self.p('grip_turn_radps'))
        kp, kr = self.k_pos_axes, self.k_rot_axes
        deadband = FRICTION_N / float(kp.max())
        rot_deadband = FRICTION_NM / float(kr.max())
        lead_cap = LEAD_FORCE_MAX_N / kp           # per tool axis: 15 N whatever its k
        rot_cap = LEAD_TORQUE_MAX_NM / kr
        lead, rot_lead = np.zeros(3), np.zeros(3)
        tol, tol_rad = self.p('grip_tol_m'), math.radians(self.p('grip_tol_deg'))
        floor = self.p('tracking_z_floor_m')
        box = (*self.p('tracking_box_x_m'), *self.p('tracking_box_y_m'),
               self.p('tracking_box_z_max_m'))
        f0 = self.state()[1]
        self.trace({'rec': 'glide', 'k_pos': kp, 'k_rot': kr, 'deadband_mm': deadband * 1000,
                    'lead_cap_mm': lead_cap * 1000, 'glide_s': dur, 'contact': contact})
        t0 = time.monotonic()
        dt = 1.0 / RATE_HZ
        while True:
            if self._stop.is_set():
                raise Stopped()
            ee, force, q, _ = self.state()
            why = G.joint_problem(q, self.p('tracking_joint_lower'),
                                  self.p('tracking_joint_upper'),
                                  self.p('grip_joint_margin_rad'))
            if why:
                raise Failed(why)
            push = float(np.linalg.norm(force - f0))
            if contact and push > self.p('grip_contact_n'):
                left = float(np.linalg.norm(target[0] - ee[0]))
                self.trace({'rec': 'contact', 'force_n': push, 'left_mm': left * 1000, 'ee': ee})
                if contact == 'arrive' and left < self.p('grip_arrive_window_m'):
                    self.publish(ee)
                    return
                raise Failed(f'{push:.1f} N of contact {left*1000:.0f} mm before the target - '
                             + ('fingers on the cube? check the cube size and the marker'
                                if contact == 'hold' else
                                'something is in the way; the Hand stays closed'))
            elapsed = time.monotonic() - t0
            goal = G.glide(start, target, elapsed / dur)
            err = target[0] - ee[0]
            rerr = G.rotvec(target[1] @ ee[1].T)
            if elapsed >= dur:                                  # settling: beat friction
                # Integrate down to half the tolerance, not only half the
                # friction band: at these stiffnesses 0.5 * F/k is ~3 mm, the
                # tolerance itself, and the arm stalled either side of it
                # (2026-09-25: one GRIP in three stuck at 4.2 mm). Hunting is
                # no risk here - the step ends once it is inside the tolerance.
                R = ee[1]                                       # the tool axes
                if np.linalg.norm(err) > min(deadband, tol) * 0.5:
                    lead = R @ np.clip(R.T @ (lead + err * self.p('grip_ki') * dt),
                                       -lead_cap, lead_cap)
                if np.linalg.norm(rerr) > min(rot_deadband, tol_rad) * 0.5:
                    rot_lead = R @ np.clip(R.T @ (rot_lead + rerr * self.p('grip_ki') * dt),
                                           -rot_cap, rot_cap)
                turn = math.degrees(float(np.linalg.norm(rerr)))
                if np.linalg.norm(err) < tol and turn < self.p('grip_tol_deg'):
                    self.trace({'rec': 'arrived', 'err_mm': float(np.linalg.norm(err)) * 1000,
                                'turn_deg': turn, 'lead_mm': float(np.linalg.norm(lead)) * 1000,
                                'rot_lead_deg': math.degrees(float(np.linalg.norm(rot_lead)))})
                    return
                if elapsed > dur + self.p('grip_settle_s'):
                    raise Failed(f'did not settle: {np.linalg.norm(err)*1000:.1f} mm / '
                                 f'{turn:.1f} deg off after {self.p("grip_settle_s"):g} s - '
                                 'stiffer gains, or check for contact')
            # The lead may push past a limit its unleaded target respected.
            self.publish((G.clamp_to_limits(goal[0] + lead, floor, box),
                          G.from_rotvec(rot_lead) @ goal[1]))
            time.sleep(dt)

    def _action(self, client, goal, what, timeout_s):
        """One gripper goal. On STOP or a timeout the goal is CANCELLED, so
        the fingers stop where they are rather than finish on their own."""
        if not client.wait_for_server(timeout_sec=2.0):
            raise Failed(f'the gripper {what} action is not available - is the Hand up in T1?')
        fut = client.send_goal_async(goal)

        def cancel_once_accepted(f):
            h = f.result()
            if h is not None and h.accepted:
                h.cancel_goal_async()
        t0 = time.monotonic()
        while not fut.done():
            if self._stop.is_set() or time.monotonic() - t0 > 5.0:
                fut.add_done_callback(cancel_once_accepted)
                if self._stop.is_set():
                    raise Stopped()
                raise Failed(f'the gripper did not accept the {what}')
            time.sleep(0.01)
        handle = fut.result()
        if not handle.accepted:
            raise Failed(f'the gripper refused the {what}')
        res = handle.get_result_async()
        while not res.done():
            if self._stop.is_set():
                handle.cancel_goal_async()
                raise Stopped()
            if time.monotonic() - t0 > timeout_s:
                handle.cancel_goal_async()
                raise Failed(f'the gripper {what} did not finish in {timeout_s:g} s')
            time.sleep(0.01)
        r = res.result().result
        self.trace({'rec': what, 'success': r.success, 'error': r.error})
        if not r.success:
            raise Failed(f'gripper {what} failed: {r.error or "no reason given"}')

    def hand_move(self, width):
        self._action(self.move_ac, Move.Goal(width=float(width), speed=0.1), 'move', 10.0)

    def hand_grasp(self, width):
        goal = Grasp.Goal(width=float(width), speed=0.05, force=float(self.p('grip_force_n')))
        goal.epsilon.inner = goal.epsilon.outer = float(self.p('grip_epsilon_m'))
        self._action(self.grasp_ac, goal, 'grasp', 10.0)

    def close(self):
        stop_throttle(self.relay)


def main():
    rclpy.init()
    node = GripNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    except Exception:                                           # noqa: BLE001
        if rclpy.ok():                  # SIGTERM shuts the context under the spin
            raise
    finally:
        node._stop.set()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
