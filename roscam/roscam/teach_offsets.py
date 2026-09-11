#!/usr/bin/env python3
"""Auto-teach the connector offsets from ICP - replaces the caliper loop.

The controller aims at a point taught in the MARKER frame
(connector_offset_x/y/z + tool_yaw_offset_deg, setup guide section 2.4).
The manual procedure measures those with calipers and iterates. But the ICP
node already computes exactly the needed quantity: the connector's pose,
and the marker's pose, both in the camera frame. One inverse-compose gives
the offsets:

    T_marker_connector = inv(T_cam_marker) @ T_cam_connector

This tool pairs time-close samples from /aruco/pose_raw (marker, optical
frame) and /connector/pose (ICP-refined, optical frame), averages them, and
prints a paste-ready YAML block - offsets for the controller and the
matching marker_t_connector prior for connector_pose itself.

Procedure:
  1. Cell in teach mode (enable_insertion: false), robot hovering at
     standoff, marker AND connector in view.
  2. Run connector_pose with your template STL and, critically,
     `-p fallback_to_prior:=false` - fallback poses ARE the prior, so
     teaching from them would just echo your rough guess back at you.
  3. ros2 run roscam teach_offsets
  4. Paste the printed block into your params file; re-run to confirm the
     residual is small.

Both inputs must be in the SAME (optical) frame: this tool uses
/aruco/pose_raw, which stays optical even when cam_pub filters in a fixed
frame.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from roscam.handeye_calib import quat_to_matrix, to_homogeneous


def pose_to_matrix(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return to_homogeneous(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def offsets_from_pair(T_cam_marker, T_cam_connector):
    """(connector xyz in the marker frame, yaw about the marker normal, deg).

    Assumes the template convention from the setup guide: connector frame
    Z points out of the work surface, like the marker's."""
    T_mc = np.linalg.inv(T_cam_marker) @ T_cam_connector
    xyz = T_mc[:3, 3].copy()
    yaw_deg = math.degrees(math.atan2(T_mc[1, 0], T_mc[0, 0]))
    return xyz, yaw_deg


def average_offsets(samples):
    """Mean + spread over (xyz, yaw_deg) samples. Yaw uses the circular
    mean so teaching near +/-180 deg does not average to garbage."""
    xyz = np.array([s[0] for s in samples])
    yaws = np.radians([s[1] for s in samples])
    xyz_mean = xyz.mean(axis=0)
    xyz_std = xyz.std(axis=0)
    yaw_mean = math.degrees(math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean()))
    # Spread around the circular mean, shortest-way residuals.
    residuals = np.degrees(np.arctan2(np.sin(yaws - math.radians(yaw_mean)),
                                      np.cos(yaws - math.radians(yaw_mean))))
    return xyz_mean, xyz_std, yaw_mean, float(np.std(residuals))


class TeachOffsetsNode(Node):
    def __init__(self):
        super().__init__('teach_offsets')
        self.declare_parameter('marker_pose_topic', '/aruco/pose_raw')
        self.declare_parameter('connector_pose_topic', '/connector/pose')
        self.declare_parameter('samples', 100)
        # A pair is only valid when both poses come from (nearly) the same
        # instant - the robot hovers during teaching, so this is generous.
        self.declare_parameter('pair_max_dt_s', 0.15)
        self.declare_parameter('timeout_s', 60.0)

        self.wanted = int(self.get_parameter('samples').value)
        self.pair_max_dt = float(self.get_parameter('pair_max_dt_s').value)
        self.samples = []
        self.latest_marker = None  # (T, stamp_s)
        self.done = False

        self.create_subscription(
            PoseStamped, str(self.get_parameter('marker_pose_topic').value),
            self.marker_cb, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('connector_pose_topic').value),
            self.connector_cb, 10)
        self.get_logger().info(
            f'Collecting {self.wanted} marker/connector pose pairs... '
            '(connector_pose must run with fallback_to_prior:=false)')

    @staticmethod
    def _stamp_s(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def marker_cb(self, msg):
        self.latest_marker = (pose_to_matrix(msg), self._stamp_s(msg))

    def connector_cb(self, msg):
        if self.done or self.latest_marker is None:
            return
        T_marker, t_marker = self.latest_marker
        if abs(self._stamp_s(msg) - t_marker) > self.pair_max_dt:
            return
        self.samples.append(offsets_from_pair(T_marker, pose_to_matrix(msg)))
        if len(self.samples) % 20 == 0:
            self.get_logger().info(f'{len(self.samples)}/{self.wanted} pairs')
        if len(self.samples) >= self.wanted:
            self.done = True

    def report(self):
        if len(self.samples) < 10:
            self.get_logger().error(
                f'Only {len(self.samples)} pairs collected - need both topics '
                'alive and time-close. Is connector_pose running (with a '
                'template) and the marker visible?')
            return False
        xyz, xyz_std, yaw, yaw_std = average_offsets(self.samples)
        spread_mm = float(np.max(xyz_std)) * 1000.0
        print('\n# ---- taught from %d ICP samples ----' % len(self.samples))
        print('# spread: %.2f mm max-axis, %.2f deg yaw (1 sigma)'
              % (spread_mm, yaw_std))
        if spread_mm > 2.0 or yaw_std > 1.5:
            print('# WARNING: large spread - ICP is unstable on this part/'
                  'view. Improve depth data before trusting these numbers.')
        print('# controller (rv5as_params.yaml / fr3_params.yaml):')
        print(f'connector_offset_x: {xyz[0]:.4f}')
        print(f'connector_offset_y: {xyz[1]:.4f}')
        print(f'connector_offset_z: {xyz[2]:.4f}')
        print(f'tool_yaw_offset_deg: {yaw:.2f}')
        print('# connector_pose prior (same numbers, its own parameter):')
        print(f'marker_t_connector_xyz: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TeachOffsetsNode()
    deadline = time.monotonic() + float(node.get_parameter('timeout_s').value)
    ok = False
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        ok = node.report()
    except KeyboardInterrupt:
        ok = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
