#!/usr/bin/env python3
"""No-robot smoke test of tracking_node: the real node in a fake FR3 cell.

    python3 tools/fr3/sim/tracking_smoke.py          # tools/run_tests.sh runs it
    python3 tools/fr3/sim/tracking_smoke.py --gdb    # backtrace if the node dies

It fakes everything tracking_node talks to - the controller manager's
list_controllers, the impedance controller's gain parameters and its
equilibrium (a first-order arm that follows it), the robot state and TF, the
hand-eye TF from calib/handeye.yaml, and cam_pub's /aruco/pose +
/aruco/pose_raw - then drives START -> follow -> over-cap jump -> raw
dropout -> STOP and checks each step. Its first version found the segfault
that killed the first hardware START on 2026-09-23.

ISOLATED: it always runs on ROS_DOMAIN_ID 87 with a loopback-only DDS config
of its own, re-executing itself if started anywhere else, so it cannot see
or be seen by a live cell on domain 0. It never loads the real controller.
Exit code: 0 only if every check passes.
"""

import math
import os
import subprocess
import sys
import tempfile
import threading
import time

DOMAIN = '87'
HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
NODE = os.path.join(WS, 'install/mating_controller/lib/mating_controller/tracking_node')
LO_ONLY = """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">
  <General><Interfaces><NetworkInterface name="lo"/></Interfaces>
    <AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
    <Peers><Peer address="localhost"/></Peers></Discovery>
</Domain></CycloneDDS>
"""

if os.environ.get('ROS_DOMAIN_ID') != DOMAIN or not os.environ.get('_FR3_SMOKE'):
    fd, xml = tempfile.mkstemp(suffix='.xml', prefix='fr3_smoke_dds_')
    with os.fdopen(fd, 'w') as f:
        f.write(LO_ONLY)
    os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='0', _FR3_SMOKE='1',
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file://' + xml)
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np  # noqa: E402  (after the re-exec, so rclpy sees the isolated env)
import rclpy  # noqa: E402
import yaml  # noqa: E402
from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from franka_msgs.msg import FrankaRobotState  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

IMP = 'cartesian_impedance_stroke_controller'
OPERATOR_GAINS = [150.0, 150.0, 800.0]     # what the panel's HOLD leaves
TRACK_GAINS = [1500.0, 1500.0, 1500.0]     # fr3_params.yaml track_k_pos_tool


def homog(pos, quat_xyzw):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    m[:3, 3] = pos
    return m


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


CALIB = yaml.safe_load(open(os.path.join(WS, 'tools/fr3/calib/handeye.yaml')))
T_TCP_CAM = homog(CALIB['xyz'], CALIB['quat_xyzw'])
T_TCP_EE = homog([0.0, 0.0, -0.005], [0, 0, 0, 1])   # non-identity on purpose


class Cell(Node):
    """Arm, robot state, TF and vision."""

    def __init__(self):
        super().__init__('fake_cell')
        self.lock = threading.Lock()
        # TCP pointing down, 0.45 m out, 0.35 m up.
        self.tcp = homog([0.45, 0.0, 0.35],
                         Rotation.from_euler('xyz', [math.pi, 0, 0]).as_quat())
        # The marker where ALIGN leaves it: 100 mm ahead of the camera, its
        # normal facing it, marker X at +90 deg in the image.
        x, z = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0])
        t_cam_marker = np.eye(4)
        t_cam_marker[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        t_cam_marker[:3, 3] = [0.0, 0.0, 0.10]
        self.marker = self.tcp @ T_TCP_CAM @ t_cam_marker
        self.eq = None
        self.eq_count = 0
        self.raw_on = True
        self.buzz_nm = 0.0                 # amplitude of an injected 40 Hz torque on J1
        self.tfb = TransformBroadcaster(self)
        StaticTransformBroadcaster(self).sendTransform(tf_msg(
            T_TCP_CAM, 'fr3_hand_tcp', 'camera_color_optical_frame',
            self.get_clock().now().to_msg()))
        self.state_pub = self.create_publisher(
            FrankaRobotState, '/franka_robot_state_broadcaster/robot_state', 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 1)
        self.raw_pub = self.create_publisher(PoseStamped, '/aruco/pose_raw', 1)
        self.create_subscription(PoseStamped, f'/{IMP}/equilibrium_pose', self._eq, 10)
        self.create_timer(0.01, self._arm)
        self.create_timer(1.0 / 15, self._vision)

    def _eq(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.eq = homog([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
            self.eq_count += 1

    def ee(self):
        return self.tcp @ T_TCP_EE

    def _arm(self):
        with self.lock:
            if self.eq is not None:        # the spring pulls the EE to the equilibrium
                ee = self.ee()
                ee[:3, 3] += 0.1 * (self.eq[:3, 3] - ee[:3, 3])
                ee[:3, :3] = self.eq[:3, :3]
                self.tcp = ee @ np.linalg.inv(T_TCP_EE)
            tcp = self.tcp.copy()
        now = self.get_clock().now().to_msg()
        self.tfb.sendTransform(tf_msg(tcp, 'fr3_link0', 'fr3_hand_tcp', now))
        state = FrankaRobotState()
        state.header.stamp = now
        state.o_t_ee = pose_msg(tcp @ T_TCP_EE, 'fr3_link0', now)
        # Gravity-like joint torques, plus the 2026-09-23 wrist buzz on demand.
        # Published at 100 Hz, not the robot's 1 kHz: this checks the wiring;
        # the thresholds were checked against recorded 1 kHz data.
        tau = [0.2, -21.0, 0.4, 14.0, 0.6, 2.1, 0.1]
        tau[0] += self.buzz_nm * math.sin(2 * math.pi * 40.0 * time.monotonic())
        state.measured_joint_state.effort = tau
        self.state_pub.publish(state)

    def _vision(self):
        with self.lock:
            marker, cam, raw = self.marker.copy(), self.tcp @ T_TCP_CAM, self.raw_on
        now = self.get_clock().now().to_msg()
        self.pose_pub.publish(pose_msg(marker, 'fr3_link0', now))
        if raw:
            self.raw_pub.publish(pose_msg(np.linalg.inv(cam) @ marker,
                                          'camera_color_optical_frame', now))


class Controller(Node):
    """The impedance controller's parameter surface, nothing more."""

    def __init__(self):
        super().__init__(IMP)
        self.declare_parameter('float_mode', False)
        self.declare_parameter('k_pos_tool', OPERATOR_GAINS)
        self.declare_parameter('k_rot_tool', [10.0, 10.0, 20.0])
        self.declare_parameter('damping_ratio', 1.0)
        self.declare_parameter('setpoint_slew_mps', 0.05)
        self.declare_parameter('setpoint_slew_rps', 0.5)

    def k_pos(self):
        return list(self.get_parameter('k_pos_tool').value)


class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager')
        self.create_service(ListControllers, '~/list_controllers', self._list)

    def _list(self, _req, res):
        res.controller = [ControllerState(name=IMP, state='active')]
        return res


class Operator(Node):
    """What cell_panel does: START / STOP and the latched status."""

    def __init__(self):
        super().__init__('smoke_operator')
        self.status = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticStatus, '/tracking_node/status',
                                 self._status, latched)
        self.start = self.create_client(Trigger, '/tracking_node/start_tracking')
        self.stop = self.create_client(Trigger, '/tracking_node/stop_tracking')

    def _status(self, msg):
        self.status = {kv.key: kv.value for kv in msg.values}

    def state(self):
        return self.status.get('state'), self.status.get('reason', '')


def main():
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN or 'fr3_smoke_dds_' not in uri:
        print('refusing: not on the isolated smoke-test domain')
        return 2
    print(f'isolated: ROS_DOMAIN_ID={DOMAIN}, loopback-only DDS ({uri[7:]})')
    if not os.path.exists(NODE):
        print(f'tracking_node not built: {NODE}')
        return 2
    log_dir = tempfile.mkdtemp(prefix='fr3_smoke_')
    cmd = [NODE, '--ros-args', '--params-file',
           os.path.join(WS, 'tools/fr3/fr3_params.yaml'),
           '-p', 'tracking_log_dir:=' + log_dir]
    if '--gdb' in sys.argv:
        cmd = ['gdb', '-q', '-batch', '-ex', 'run', '-ex', 'bt', '--args'] + cmd

    rclpy.init()
    cell, ctl, cm, op = Cell(), Controller(), ControllerManager(), Operator()
    executor = MultiThreadedExecutor(num_threads=6)
    for n in (cell, ctl, cm, op):
        executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()
    node_log = open(os.path.join(log_dir, 'tracking_node.out'), 'w')
    node = subprocess.Popen(cmd, stdout=node_log, stderr=subprocess.STDOUT)
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(('PASS ' if ok else 'FAIL ') + name + (f'  [{detail}]' if detail else ''),
              flush=True)

    def wait_for(pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and node.poll() is None and not pred():
            time.sleep(0.05)
        return pred()

    def call(client):
        fut = client.call_async(Trigger.Request())
        wait_for(fut.done, 25.0)
        return fut.result() if fut.done() else None

    def arm_moved_from(p0):
        with cell.lock:
            return cell.tcp[:3, 3] - p0

    try:
        up = (op.start.wait_for_service(timeout_sec=30)
              and wait_for(lambda: op.state()[0] == 'idle', 10))
        check('node up and idle', up, str(op.state()))
        time.sleep(1.5)                    # let TF and robot state settle

        r = call(op.start)
        check('START succeeds', bool(r and r.success), r.message if r else 'no reply')
        check('track profile applied', ctl.k_pos() == TRACK_GAINS, str(ctl.k_pos()))
        check('status tracking', wait_for(lambda: op.state()[0] == 'tracking', 3),
              str(op.state()))
        n0 = cell.eq_count
        time.sleep(2.0)
        check('equilibria stream at ~50 Hz', cell.eq_count - n0 > 60,
              f'{cell.eq_count - n0} in 2 s')
        with cell.lock:
            gap = np.linalg.norm(cell.eq[:3, 3] - cell.ee()[:3, 3])
        check('starts on the ALIGN pose', gap < 0.012, f'{gap * 1000:.1f} mm')

        with cell.lock:
            p0 = cell.tcp[:3, 3].copy()
            cell.marker[:3, 3] += [0.020, 0.0, 0.0]
        time.sleep(4.0)
        moved = arm_moved_from(p0)[0]
        check('follows a 20 mm marker move', 0.014 < moved < 0.026,
              f'{moved * 1000:.1f} mm in x')

        with cell.lock:
            p1 = cell.tcp[:3, 3].copy()
            cell.marker[:3, 3] += [0.100, 0.0, 0.0]
        held = wait_for(lambda: op.state()[0] == 'holding', 3)
        time.sleep(1.0)
        drift = np.linalg.norm(arm_moved_from(p1))
        check('a 100 mm jump holds (policy hold)', held and drift < 0.015,
              f'{op.state()}, drift {drift * 1000:.1f} mm')
        with cell.lock:
            cell.marker[:3, 3] -= [0.100, 0.0, 0.0]
        check('resumes once back inside the cap',
              wait_for(lambda: op.state()[0] == 'tracking', 4), str(op.state()))

        cell.raw_on = False
        check('a raw-detection dropout holds',
              wait_for(lambda: op.state() == ('holding', 'no fresh raw detection'), 3),
              str(op.state()))
        cell.raw_on = True
        check('resumes when detections return',
              wait_for(lambda: op.state()[0] == 'tracking', 3), str(op.state()))

        with cell.lock:                   # a goal under the absolute 100 mm floor
            cell.marker[:3, 3] -= [0.0, 0.0, 0.30]
        check('a goal below the 100 mm cell floor holds',
              wait_for(lambda: op.state()[0] == 'holding'
                       and 'below the floor 100 mm' in op.state()[1], 3), str(op.state()))
        with cell.lock:
            cell.marker[:3, 3] += [0.0, 0.0, 0.30]
        check('resumes above the floor', wait_for(lambda: op.state()[0] == 'tracking', 4),
              str(op.state()))

        r = call(op.stop)
        check('STOP succeeds', bool(r and r.success), r.message if r else 'no reply')
        check('operator gains restored', ctl.k_pos() == OPERATOR_GAINS, str(ctl.k_pos()))
        check('status idle', wait_for(lambda: op.state()[0] == 'idle', 3), str(op.state()))
        logs = [f for f in os.listdir(log_dir)
                if f.startswith('tracking_') and f.endswith('.jsonl')]
        check('tracking log named in local time',
              len(logs) == 1 and logs[0][18:20] == time.strftime('%H'), str(logs))

        # The buzz watchdog: a 40 Hz torque on J1 while tracking must end the
        # session by itself and put the operator's gains back.
        time.sleep(1.0)
        r = call(op.start)
        check('START again', bool(r and r.success) and ctl.k_pos() == TRACK_GAINS,
              r.message if r else 'no reply')
        cell.buzz_nm = 4.0
        stopped = wait_for(lambda: op.state()[0] in ('stopping', 'idle')
                           and 'buzz' in op.state()[1], 2)
        check('a 40 Hz buzz stops tracking by itself', stopped, str(op.state()))
        check('operator gains restored after the buzz',
              wait_for(lambda: ctl.k_pos() == OPERATOR_GAINS, 2), str(ctl.k_pos()))
        cell.buzz_nm = 0.0
        check('node alive throughout', node.poll() is None, f'exit code {node.poll()}')
    finally:
        if node.poll() is None:
            node.terminate()
        try:
            node.wait(timeout=20)
        except subprocess.TimeoutExpired:
            node.kill()
        executor.shutdown()
        rclpy.shutdown()
        os.remove(uri[len('file://'):])
    print(f'{sum(results)}/{len(results)} checks passed (node output: {log_dir})')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
