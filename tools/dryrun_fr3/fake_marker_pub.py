#!/usr/bin/env python3
"""Synthetic ArUco vision for the FR3 dry run.

Simulates roscam/cam_pub: a marker fixed in the robot base frame is
re-expressed in the (moving) camera optical frame via TF and published as
PoseStamped on /aruco/pose with realistic noise, exactly the contract the
mating controller consumes. Only publishes when the marker is in front of
the camera, so occlusion/visibility behaviour is exercised too.
"""

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    p = (v[0], v[1], v[2], 0.0)
    r = quat_mul(quat_mul(q, p), quat_conj(q))
    return (r[0], r[1], r[2])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class FakeMarkerPublisher(Node):
    def __init__(self):
        super().__init__('fake_marker_publisher')

        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        # Marker fixed in the base frame: slightly tilted (roll/pitch!) and
        # yawed so the full 6-DOF alignment is exercised.
        self.declare_parameter('marker_xyz', [0.45, 0.10, 0.05])
        self.declare_parameter('marker_rpy_deg', [6.0, -4.0, 25.0])
        self.declare_parameter('pos_noise_m', 0.0005)
        self.declare_parameter('rot_noise_deg', 0.2)

        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.m_xyz = list(self.get_parameter('marker_xyz').value)
        rpy = [math.radians(v) for v in self.get_parameter('marker_rpy_deg').value]
        self.m_q = quat_from_rpy(*rpy)
        self.pos_noise = float(self.get_parameter('pos_noise_m').value)
        self.rot_noise = math.radians(float(self.get_parameter('rot_noise_deg').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info(
            f'Fake marker at {self.m_xyz} in {self.base_frame}, '
            f'rpy_deg={self.get_parameter("marker_rpy_deg").value}')

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.optical_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return

        t = tf.transform.translation
        q = (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w)

        # marker in optical frame: p_cam = R * p_base + t
        p = quat_rotate(q, self.m_xyz)
        p = (p[0] + t.x, p[1] + t.y, p[2] + t.z)
        mq = quat_mul(q, self.m_q)

        if p[2] < 0.03:  # behind / too close to the camera: not visible
            return

        noise_axis = [random.gauss(0, 1) for _ in range(3)]
        norm = math.sqrt(sum(v * v for v in noise_axis)) or 1.0
        ang = random.gauss(0, self.rot_noise)
        s = math.sin(ang / 2)
        nq = (noise_axis[0] / norm * s, noise_axis[1] / norm * s,
              noise_axis[2] / norm * s, math.cos(ang / 2))
        mq = quat_mul(nq, mq)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.optical_frame
        msg.pose.position.x = p[0] + random.gauss(0, self.pos_noise)
        msg.pose.position.y = p[1] + random.gauss(0, self.pos_noise)
        msg.pose.position.z = p[2] + random.gauss(0, self.pos_noise)
        msg.pose.orientation.x = mq[0]
        msg.pose.orientation.y = mq[1]
        msg.pose.orientation.z = mq[2]
        msg.pose.orientation.w = mq[3]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
