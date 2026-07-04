#!/usr/bin/env python3
"""ArUco marker pose publisher for the connector-mating cell.

Subscribes to the RealSense color image and camera_info, detects the target
ArUco marker, estimates its full 6-DOF pose (SOLVEPNP_IPPE_SQUARE) and
publishes it as a PoseStamped in the camera optical frame:

  /aruco/pose       geometry_msgs/PoseStamped  (validated + low-pass filtered)
  /aruco/pose_raw   geometry_msgs/PoseStamped  (validated, unfiltered)
  /aruco/debug_image  sensor_msgs/Image        (optional, marker overlay)

Intrinsics come from camera_info, so no calibration files are needed.
Measurements are gated on reprojection error and on translation jumps before
they reach the filter, so a single bad detection cannot yank the robot.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def rotation_matrix_to_quaternion(m):
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
    t = np.trace(m)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quaternion_slerp(q0, q1, alpha):
    """Spherical interpolation from q0 toward q1 by fraction alpha."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:  # take the short way around
        q1 = -q1
        dot = -dot
    if dot > 0.9995:  # nearly identical: lerp is fine and stable
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * alpha
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * math.cos(theta) + q2 * math.sin(theta)


def quaternion_angle(q0, q1):
    """Angle in radians between two quaternions."""
    dot = abs(float(np.dot(q0, q1)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_id', 11)
        self.declare_parameter('marker_size_m', 0.021)
        self.declare_parameter('aruco_dictionary', 'DICT_6X6_250')
        # EMA/slerp fraction applied per accepted measurement (1.0 = no filtering)
        self.declare_parameter('filter_alpha', 0.35)
        self.declare_parameter('max_reprojection_error_px', 2.0)
        # A translation jump larger than this against the last accepted pose is
        # rejected as an outlier, unless it persists (re-acquisition).
        self.declare_parameter('max_translation_jump_m', 0.05)
        self.declare_parameter('rejects_before_reacquire', 5)
        self.declare_parameter('publish_debug_image', True)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.max_reproj_err = float(self.get_parameter('max_reprojection_error_px').value)
        self.max_jump = float(self.get_parameter('max_translation_jump_m').value)
        self.rejects_before_reacquire = int(self.get_parameter('rejects_before_reacquire').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        half = self.marker_size / 2.0
        # Corner order required by SOLVEPNP_IPPE_SQUARE (matches detectMarkers output)
        self.obj_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        self.detect = self._make_detector(str(self.get_parameter('aruco_dictionary').value))

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        # Filter / outlier-gate state
        self.filt_t = None
        self.filt_q = None
        self.last_accepted_t = None
        self.consecutive_rejects = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.pose_raw_pub = self.create_publisher(PoseStamped, '/aruco/pose_raw', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 2)

        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback, 10)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.image_callback, 5)

        self.get_logger().info(
            f'Tracking ArUco id {self.marker_id} ({self.marker_size * 1000:.0f} mm), '
            f'waiting for camera_info...')

    def _make_detector(self, dictionary_name):
        try:
            dict_id = getattr(cv2.aruco, dictionary_name)
        except AttributeError:
            self.get_logger().error(
                f'Unknown ArUco dictionary "{dictionary_name}", using DICT_6X6_250')
            dict_id = cv2.aruco.DICT_6X6_250
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            return lambda gray: detector.detectMarkers(gray)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return lambda gray: cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
            self.get_logger().info('Camera intrinsics received.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)

        measurement = None
        if ids is not None and self.marker_id in ids.flatten():
            index = int(np.where(ids.flatten() == self.marker_id)[0][0])
            img_points = corners[index][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if success:
                measurement = self._validate(rvec, tvec, img_points)

        if measurement is not None:
            self._publish_pose(msg.header, *measurement)
            if self.publish_debug:
                cv2.aruco.drawDetectedMarkers(color_image, [corners[index]])
                cv2.drawFrameAxes(color_image, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_size)

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _validate(self, rvec, tvec, img_points):
        """Gate a raw solvePnP result. Returns (t, q) or None if rejected."""
        projected, _ = cv2.projectPoints(
            self.obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reproj_err = float(np.mean(np.linalg.norm(
            projected.reshape(-1, 2) - img_points, axis=1)))
        if reproj_err > self.max_reproj_err:
            self.get_logger().warn(
                f'Rejected detection: reprojection error {reproj_err:.2f} px',
                throttle_duration_sec=2.0)
            return None

        t = tvec.reshape(3).astype(np.float64)
        if self.last_accepted_t is not None:
            jump = float(np.linalg.norm(t - self.last_accepted_t))
            if jump > self.max_jump:
                self.consecutive_rejects += 1
                if self.consecutive_rejects < self.rejects_before_reacquire:
                    self.get_logger().warn(
                        f'Rejected detection: {jump * 1000:.0f} mm jump',
                        throttle_duration_sec=2.0)
                    return None
                # The "jump" persisted: the marker really moved. Re-acquire.
                self.get_logger().info('Re-acquiring marker after persistent jump.')
                self.filt_t = None
                self.filt_q = None
        self.consecutive_rejects = 0
        self.last_accepted_t = t

        rot_matrix, _ = cv2.Rodrigues(rvec)
        q = rotation_matrix_to_quaternion(rot_matrix)
        return t, q

    def _publish_pose(self, header, t, q):
        # Low-pass: EMA on translation, slerp on orientation
        if self.filt_t is None:
            self.filt_t = t
            self.filt_q = q
        else:
            a = self.filter_alpha
            self.filt_t = (1.0 - a) * self.filt_t + a * t
            self.filt_q = quaternion_slerp(self.filt_q, q, a)

        raw = PoseStamped()
        raw.header = header  # camera optical frame, image timestamp
        raw.pose.position.x = float(t[0])
        raw.pose.position.y = float(t[1])
        raw.pose.position.z = float(t[2])
        raw.pose.orientation.x = float(q[0])
        raw.pose.orientation.y = float(q[1])
        raw.pose.orientation.z = float(q[2])
        raw.pose.orientation.w = float(q[3])
        self.pose_raw_pub.publish(raw)

        filt = PoseStamped()
        filt.header = header
        filt.pose.position.x = float(self.filt_t[0])
        filt.pose.position.y = float(self.filt_t[1])
        filt.pose.position.z = float(self.filt_t[2])
        filt.pose.orientation.x = float(self.filt_q[0])
        filt.pose.orientation.y = float(self.filt_q[1])
        filt.pose.orientation.z = float(self.filt_q[2])
        filt.pose.orientation.w = float(self.filt_q[3])
        self.pose_pub.publish(filt)

        self.get_logger().info(
            f'Marker at [{self.filt_t[0]:.3f}, {self.filt_t[1]:.3f}, '
            f'{self.filt_t[2]:.3f}] m (optical frame)',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
