#!/usr/bin/env python3
"""Eye-in-hand calibration tool: solves the TCP -> camera transform.

Collect mode (default, needs the robot + vision running):

    ros2 run roscam handeye_calib --ros-args -p base_frame:=rv5as_base \
        -p tcp_frame:=rv5as_default_tcp

    Jog the robot to 10-15 diverse poses keeping the marker in view; press
    Enter at each to record a sample pair (TF base->tcp + /aruco/pose_raw).
    Press 's' to solve and print the static_transform_publisher command,
    'q' to quit. Samples are saved to a YAML file as you go.

Solve-only mode (no ROS graph needed):

    ros2 run roscam handeye_calib --solve handeye_samples.yaml

The result is T_tcp->camera_optical, i.e. exactly the transform to publish:

    ros2 run tf2_ros static_transform_publisher --x ... --frame-id <tcp> --child-frame-id <optical>
"""

import argparse
import math
import sys
import threading

import cv2
import numpy as np
import yaml

METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(m)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def to_homogeneous(rotation, translation):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation).reshape(3)
    return T


def consistency_residual(base_T_tcp_list, cam_T_marker_list, tcp_T_cam):
    """The marker is fixed in the base frame, so base_T_marker computed from
    every sample must agree. Returns (position spread [m], angle spread [rad])
    as the RMS deviation from the mean marker pose."""
    positions = []
    rotations = []
    for base_T_tcp, cam_T_marker in zip(base_T_tcp_list, cam_T_marker_list):
        base_T_marker = base_T_tcp @ tcp_T_cam @ cam_T_marker
        positions.append(base_T_marker[:3, 3])
        rotations.append(base_T_marker[:3, :3])
    positions = np.array(positions)
    pos_rms = float(np.sqrt(np.mean(np.sum(
        (positions - positions.mean(axis=0)) ** 2, axis=1))))

    mean_rot = rotations[0]
    angles = []
    for rot in rotations:
        delta = mean_rot.T @ rot
        angles.append(math.acos(max(-1.0, min(1.0, (np.trace(delta) - 1.0) / 2.0))))
    ang_rms = float(np.sqrt(np.mean(np.square(angles))))
    return pos_rms, ang_rms


def solve_handeye(base_T_tcp_list, cam_T_marker_list):
    """Solve eye-in-hand calibration. Returns (tcp_T_cam 4x4, method name,
    per-method results dict {name: (tcp_T_cam, pos_rms, ang_rms)})."""
    if len(base_T_tcp_list) < 3:
        raise ValueError('Need at least 3 samples (10+ recommended).')

    R_g2b = [T[:3, :3] for T in base_T_tcp_list]
    t_g2b = [T[:3, 3] for T in base_T_tcp_list]
    R_t2c = [T[:3, :3] for T in cam_T_marker_list]
    t_t2c = [T[:3, 3] for T in cam_T_marker_list]

    results = {}
    for name, method in METHODS.items():
        try:
            R_c2g, t_c2g = cv2.calibrateHandEye(
                R_g2b, t_g2b, R_t2c, t_t2c, method=method)
        except cv2.error:
            continue
        tcp_T_cam = to_homogeneous(R_c2g, t_c2g)
        pos_rms, ang_rms = consistency_residual(
            base_T_tcp_list, cam_T_marker_list, tcp_T_cam)
        results[name] = (tcp_T_cam, pos_rms, ang_rms)

    if not results:
        raise RuntimeError('All hand-eye solvers failed (degenerate motions?).')

    # Rank by combined residual (1 mm position ~ 0.5 deg rotation weighting)
    best = min(results, key=lambda k: results[k][1] + 0.1 * results[k][2])
    return results[best][0], best, results


def print_solution(tcp_T_cam, method, results, tcp_frame, optical_frame):
    q = matrix_to_quat(tcp_T_cam[:3, :3])
    t = tcp_T_cam[:3, 3]
    print('\n=== Hand-eye result (eye-in-hand): T_tcp->camera_optical ===')
    for name, (_, pos_rms, ang_rms) in sorted(results.items()):
        marker = ' <-- selected' if name == method else ''
        print(f'  {name:<11} residual: {pos_rms * 1000:6.2f} mm, '
              f'{math.degrees(ang_rms):5.2f} deg{marker}')
    print(f'\n  translation [m]: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]')
    print(f'  quaternion xyzw: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]')
    print('\nPublish it with:\n')
    print(f'ros2 run tf2_ros static_transform_publisher '
          f'--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} '
          f'--qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} '
          f'--frame-id {tcp_frame} --child-frame-id {optical_frame}\n')
    print('Sanity check: residual should be a few mm / well under 1 deg. If it')
    print('is large, the sample poses were not diverse enough or the marker moved.')


def samples_to_matrices(samples):
    base_T_tcp_list, cam_T_marker_list = [], []
    for s in samples:
        base_T_tcp_list.append(to_homogeneous(
            quat_to_matrix(*s['base_T_tcp']['quaternion_xyzw']),
            s['base_T_tcp']['translation']))
        cam_T_marker_list.append(to_homogeneous(
            quat_to_matrix(*s['cam_T_marker']['quaternion_xyzw']),
            s['cam_T_marker']['translation']))
    return base_T_tcp_list, cam_T_marker_list


def solve_from_file(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    base_T_tcp_list, cam_T_marker_list = samples_to_matrices(data['samples'])
    tcp_T_cam, method, results = solve_handeye(base_T_tcp_list, cam_T_marker_list)
    print(f'Loaded {len(base_T_tcp_list)} samples from {path}')
    print_solution(tcp_T_cam, method, results,
                   data.get('tcp_frame', '<tcp_frame>'),
                   data.get('optical_frame', '<camera_optical_frame>'))


def collect(argv):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    class Collector(Node):
        def __init__(self):
            super().__init__('handeye_collector')
            self.declare_parameter('base_frame', 'rv5as_base')
            self.declare_parameter('tcp_frame', 'rv5as_default_tcp')
            self.declare_parameter('pose_topic', '/aruco/pose_raw')
            self.declare_parameter('output', 'handeye_samples.yaml')
            self.base_frame = self.get_parameter('base_frame').value
            self.tcp_frame = self.get_parameter('tcp_frame').value
            self.output = self.get_parameter('output').value
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.latest_pose = None
            self.optical_frame = None
            self.create_subscription(
                PoseStamped, self.get_parameter('pose_topic').value,
                self.pose_cb, 10)
            self.samples = []

        def pose_cb(self, msg):
            self.latest_pose = msg
            self.optical_frame = msg.header.frame_id

        def record(self):
            if self.latest_pose is None:
                print('No marker pose received yet - is cam_pub running and the marker visible?')
                return
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, rclpy.time.Time())
            except Exception as e:
                print(f'TF {self.base_frame} -> {self.tcp_frame} unavailable: {e}')
                return
            p = self.latest_pose.pose
            self.samples.append({
                'base_T_tcp': {
                    'translation': [tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z],
                    'quaternion_xyzw': [tf.transform.rotation.x,
                                        tf.transform.rotation.y,
                                        tf.transform.rotation.z,
                                        tf.transform.rotation.w],
                },
                'cam_T_marker': {
                    'translation': [p.position.x, p.position.y, p.position.z],
                    'quaternion_xyzw': [p.orientation.x, p.orientation.y,
                                        p.orientation.z, p.orientation.w],
                },
            })
            self.save()
            print(f'Recorded sample {len(self.samples)} '
                  f'(saved to {self.output}). Move to a new, DIFFERENT pose.')

        def save(self):
            with open(self.output, 'w') as f:
                yaml.safe_dump({
                    'base_frame': self.base_frame,
                    'tcp_frame': self.tcp_frame,
                    'optical_frame': self.optical_frame,
                    'samples': self.samples,
                }, f)

        def solve(self):
            try:
                mats = samples_to_matrices(self.samples)
                tcp_T_cam, method, results = solve_handeye(*mats)
            except (ValueError, RuntimeError) as e:
                print(f'Cannot solve: {e}')
                return
            print_solution(tcp_T_cam, method, results,
                           self.tcp_frame, self.optical_frame or '<optical>')

    rclpy.init(args=argv)
    node = Collector()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Hand-eye collection. Keep the marker rigid and in view.\n'
          '  [Enter] record sample   [s] solve   [q] quit')
    try:
        while True:
            cmd = input('> ').strip().lower()
            if cmd == '':
                node.record()
            elif cmd == 's':
                node.solve()
            elif cmd == 'q':
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


def main(args=None):
    argv = args if args is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solve', metavar='SAMPLES_YAML',
                        help='solve from a previously recorded sample file')
    known, ros_args = parser.parse_known_args(argv)
    if known.solve:
        solve_from_file(known.solve)
    else:
        collect(ros_args)


if __name__ == '__main__':
    main()
