#!/usr/bin/env python3
"""Connector 6-DOF pose refinement: marker prior + depth ICP.

The ArUco marker gives a coarse prior of where the connector is; this node
crops the depth cloud around that prediction and registers a point sample of
the connector CAD model (STL) against it with ICP, publishing the refined
pose on /connector/pose (PoseStamped, camera optical frame) in the same
contract cam_pub uses — so the mating controller can consume it by just
setting `pose_topic: /connector/pose`.

Template convention: model the STL in the *connector frame the controller
expects*: origin at the mate point, Z out of the work surface. Then the
controller's connector_offset_* parameters stay zero.

If ICP fails its quality gates (occlusion, bad depth), the node publishes
the marker-derived prior instead (never worse than marker-only behaviour)
and logs the degradation.
"""

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from roscam.handeye_calib import matrix_to_quat, quat_to_matrix, to_homogeneous
from roscam.icp import (crop_points, depth_to_points, icp, load_stl,
                        sample_mesh, voxel_downsample)
from roscam.pose_kf import PoseKF


def rpy_deg_to_matrix(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class ConnectorPoseNode(Node):
    def __init__(self):
        super().__init__('connector_pose')

        self.declare_parameter('template_stl', '')
        self.declare_parameter('template_points', 1500)
        self.declare_parameter('depth_topic',
                               '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_pose_topic', '/aruco/pose')
        # Prior: connector pose in the MARKER frame (teach once, roughly).
        self.declare_parameter('marker_t_connector_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('marker_t_connector_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('marker_max_age_s', 0.5)
        self.declare_parameter('crop_radius_m', 0.05)
        self.declare_parameter('voxel_m', 0.002)
        self.declare_parameter('depth_stride', 2)
        self.declare_parameter('max_corr_dist_m', 0.008)
        self.declare_parameter('icp_max_iter', 30)
        # Quality gates: below/above these, fall back to the marker prior.
        self.declare_parameter('min_inlier_fraction', 0.6)
        self.declare_parameter('max_rms_m', 0.004)
        # Refinement sanity: ICP may not move the pose further than this from
        # the prior (a bigger jump means it latched onto the wrong geometry).
        self.declare_parameter('max_refine_translation_m', 0.02)
        self.declare_parameter('process_every_n', 5)
        self.declare_parameter('fallback_to_prior', True)

        template_path = str(self.get_parameter('template_stl').value)
        if not template_path:
            raise RuntimeError(
                'Parameter template_stl is required (export your connector '
                'CAD as STL, origin at the mate point, Z out of the surface).')
        V, F = load_stl(template_path)
        pts, normals = sample_mesh(
            V, F, int(self.get_parameter('template_points').value) * 4,
            return_normals=True)
        self.template, self.template_normals = voxel_downsample(
            pts, float(self.get_parameter('voxel_m').value), normals)
        self.get_logger().info(
            f'Template: {template_path} -> {len(self.template)} points '
            f'(extent {1000 * (V.max(0) - V.min(0))} mm)')

        xyz = list(self.get_parameter('marker_t_connector_xyz').value)
        rpy = list(self.get_parameter('marker_t_connector_rpy_deg').value)
        self.T_marker_connector = to_homogeneous(rpy_deg_to_matrix(*rpy), xyz)

        self.marker_max_age = float(self.get_parameter('marker_max_age_s').value)
        self.crop_radius = float(self.get_parameter('crop_radius_m').value)
        self.voxel = float(self.get_parameter('voxel_m').value)
        self.stride = int(self.get_parameter('depth_stride').value)
        self.max_corr = float(self.get_parameter('max_corr_dist_m').value)
        self.icp_max_iter = int(self.get_parameter('icp_max_iter').value)
        self.min_inlier_fraction = float(self.get_parameter('min_inlier_fraction').value)
        self.max_rms = float(self.get_parameter('max_rms_m').value)
        self.max_refine = float(self.get_parameter('max_refine_translation_m').value)
        self.every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.fallback = bool(self.get_parameter('fallback_to_prior').value)

        self.bridge = CvBridge()
        self.intrinsics = None       # (fx, fy, cx, cy)
        self.latest_marker = None    # (4x4 T_cam_marker, arrival monotonic time)
        self.frame_count = 0

        # Single-frame ICP tilt is depth-noise-limited (~1 deg on a ~20 mm
        # part); the connector is static relative to the camera between
        # control steps, so a low-process-noise filter averages it down.
        self.kf = PoseKF(sigma_accel=0.02, sigma_rot_rate_deg=3.0,
                         meas_std_pos=0.001, meas_std_rot_deg=1.5)
        self.kf_rejects = 0
        self.last_icp_stamp = None

        self.pose_pub = self.create_publisher(PoseStamped, '/connector/pose', 10)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.info_cb, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('marker_pose_topic').value),
            self.marker_cb, 10)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self.depth_cb, 5)

    def info_cb(self, msg):
        if self.intrinsics is None:
            k = np.array(msg.k).reshape(3, 3)
            self.intrinsics = (k[0, 0], k[1, 1], k[0, 2], k[1, 2])
            self.get_logger().info('Camera intrinsics received.')

    def marker_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        T = to_homogeneous(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
        self.latest_marker = (T, time.monotonic())

    def depth_cb(self, msg):
        self.frame_count += 1
        if (self.frame_count % self.every_n or self.intrinsics is None
                or self.latest_marker is None):
            return
        T_cam_marker, arrival = self.latest_marker
        if time.monotonic() - arrival > self.marker_max_age:
            return  # marker stale: publish nothing, controller holds

        prior = T_cam_marker @ self.T_marker_connector

        depth = self.bridge.imgmsg_to_cv2(msg)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 1e-3  # mm -> m
        points = depth_to_points(depth.astype(np.float32), *self.intrinsics,
                                 stride=self.stride)
        scene = crop_points(points, prior[:3, 3], self.crop_radius)

        refined, ok, why = self.refine(scene, prior)
        if ok:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = 0.0 if self.last_icp_stamp is None else stamp - self.last_icp_stamp
            self.last_icp_stamp = stamp
            pos = refined[:3, 3]
            quat = matrix_to_quat(refined[:3, :3])
            if self.kf.initialized:
                self.kf.predict(dt)
            if self.kf.update(pos, quat):
                self.kf_rejects = 0
            else:
                self.kf_rejects += 1
                if self.kf_rejects >= 5:  # connector genuinely moved
                    self.get_logger().info('Re-acquiring connector pose.')
                    self.kf.reset()
                    self.kf.update(pos, quat)
                    self.kf_rejects = 0
            self.publish(msg.header, self.kf.position, self.kf.quaternion)
        elif self.fallback:
            self.publish(msg.header, prior[:3, 3], matrix_to_quat(prior[:3, :3]))
            self.get_logger().warn(f'ICP fallback to marker prior: {why}',
                                   throttle_duration_sec=2.0)

    def refine(self, scene, prior):
        """Run gated ICP. Returns (T, ok, reason_if_not_ok)."""
        if len(scene) < 50:
            return prior, False, f'only {len(scene)} depth points in crop'
        scene = voxel_downsample(scene, self.voxel)
        T, rms, inlier_frac = icp(scene, self.template, prior,
                                  template_normals=self.template_normals,
                                  max_corr_dist=self.max_corr,
                                  max_iter=self.icp_max_iter)
        if inlier_frac < self.min_inlier_fraction:
            return prior, False, f'inlier fraction {inlier_frac:.2f}'
        if rms > self.max_rms:
            return prior, False, f'rms {rms * 1000:.1f} mm'
        shift = float(np.linalg.norm(T[:3, 3] - prior[:3, 3]))
        if shift > self.max_refine:
            return prior, False, f'refinement jumped {shift * 1000:.0f} mm from prior'
        self.get_logger().info(
            f'ICP ok: rms {rms * 1000:.1f} mm, inliers {inlier_frac:.2f}, '
            f'shift {shift * 1000:.1f} mm',
            throttle_duration_sec=2.0)
        return T, True, ''

    def publish(self, header, pos, q):
        out = PoseStamped()
        out.header = header  # optical frame, depth timestamp
        out.pose.position.x = float(pos[0])
        out.pose.position.y = float(pos[1])
        out.pose.position.z = float(pos[2])
        out.pose.orientation.x = float(q[0])
        out.pose.orientation.y = float(q[1])
        out.pose.orientation.z = float(q[2])
        out.pose.orientation.w = float(q[3])
        self.pose_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConnectorPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
