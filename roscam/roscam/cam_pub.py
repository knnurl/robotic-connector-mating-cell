#!/usr/bin/env python3
"""ArUco marker pose publisher for the connector-mating cell.

Subscribes to the RealSense color image and camera_info, detects the target
ArUco marker, estimates its full 6-DOF pose (SOLVEPNP_IPPE_SQUARE) and
publishes it as a PoseStamped in the camera optical frame:

  /aruco/pose       geometry_msgs/PoseStamped  (validated + low-pass filtered)
  /aruco/pose_raw   geometry_msgs/PoseStamped  (validated, unfiltered)
  /aruco/debug_image  sensor_msgs/Image        (optional, marker overlay)

Intrinsics come from camera_info, so no calibration files are needed.
Measurements are gated on reprojection error, then fused by a Kalman filter
(constant-velocity translation + small-angle orientation, see pose_kf.py)
whose innovation gate rejects outliers, so a single bad detection cannot
yank the robot. During brief detection dropouts (< max_prediction_s) the
filter's prediction is published so the controller rides through flicker;
longer occlusions go silent and the controller holds position.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from roscam.pose_kf import PoseKF


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


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_id', 11)
        self.declare_parameter('marker_size_m', 0.021)
        self.declare_parameter('aruco_dictionary', 'DICT_6X6_250')
        self.declare_parameter('max_reprojection_error_px', 2.0)
        # Kalman filter tuning (see pose_kf.py)
        self.declare_parameter('sigma_accel', 0.08)         # m/s^2 process noise
        self.declare_parameter('sigma_rot_rate_deg', 15.0)  # deg/s process noise
        self.declare_parameter('meas_std_pos', 0.002)       # m measurement noise
        self.declare_parameter('meas_std_rot_deg', 1.0)     # deg measurement noise
        self.declare_parameter('gate_sigma', 3.0)           # innovation gate
        # Publish predicted poses for at most this long after the last
        # accepted detection; beyond it, go silent (controller holds).
        self.declare_parameter('max_prediction_s', 0.3)
        # A measurement rejected by the gate this many times in a row is
        # treated as the marker genuinely having moved (re-acquisition).
        self.declare_parameter('rejects_before_reacquire', 5)
        self.declare_parameter('publish_debug_image', True)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.max_reproj_err = float(self.get_parameter('max_reprojection_error_px').value)
        self.max_prediction_s = float(self.get_parameter('max_prediction_s').value)
        self.rejects_before_reacquire = int(self.get_parameter('rejects_before_reacquire').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.kf = PoseKF(
            sigma_accel=float(self.get_parameter('sigma_accel').value),
            sigma_rot_rate_deg=float(self.get_parameter('sigma_rot_rate_deg').value),
            meas_std_pos=float(self.get_parameter('meas_std_pos').value),
            meas_std_rot_deg=float(self.get_parameter('meas_std_rot_deg').value),
            gate_sigma=float(self.get_parameter('gate_sigma').value))

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

        # Filter timing state
        self.last_frame_stamp = None   # stamp of the previous processed frame
        self.last_accept_stamp = None  # stamp of the last accepted measurement
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

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_frame_stamp is None else stamp - self.last_frame_stamp
        self.last_frame_stamp = stamp

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

        if self.kf.initialized:
            self.kf.predict(dt)

        published = False
        if measurement is not None:
            t, q = measurement
            accepted = self.kf.update(t, q)
            if not accepted:
                self.consecutive_rejects += 1
                if self.consecutive_rejects >= self.rejects_before_reacquire:
                    # Persistent disagreement: the marker really moved.
                    self.get_logger().info('Re-acquiring marker after persistent jump.')
                    self.kf.reset()
                    accepted = self.kf.update(t, q)
                else:
                    self.get_logger().warn(
                        'Rejected detection: inconsistent with filter prediction',
                        throttle_duration_sec=2.0)
            if accepted:
                self.consecutive_rejects = 0
                self.last_accept_stamp = stamp
                self._publish(self.pose_raw_pub, msg.header, t, q)
                self._publish(self.pose_pub, msg.header,
                              self.kf.position, self.kf.quaternion)
                published = True
                self.get_logger().info(
                    f'Marker at [{self.kf.position[0]:.3f}, {self.kf.position[1]:.3f}, '
                    f'{self.kf.position[2]:.3f}] m (optical frame)',
                    throttle_duration_sec=1.0)
            if self.publish_debug:
                cv2.aruco.drawDetectedMarkers(color_image, [corners[index]])
                cv2.drawFrameAxes(color_image, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_size)

        # Bridge brief dropouts with the filter prediction; go silent beyond
        # the horizon so the controller holds position.
        if (not published and self.kf.initialized and
                self.last_accept_stamp is not None and
                stamp - self.last_accept_stamp <= self.max_prediction_s):
            self._publish(self.pose_pub, msg.header,
                          self.kf.position, self.kf.quaternion)
            self.get_logger().warn('Publishing predicted pose (marker not detected).',
                                   throttle_duration_sec=1.0)

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _validate(self, rvec, tvec, img_points):
        """Gate a raw solvePnP result on reprojection error.
        Returns (t, q) or None if rejected."""
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
        rot_matrix, _ = cv2.Rodrigues(rvec)
        q = rotation_matrix_to_quaternion(rot_matrix)
        return t, q

    @staticmethod
    def _fill_pose(msg, t, q):
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

    def _publish(self, publisher, header, t, q):
        out = PoseStamped()
        out.header = header  # camera optical frame, image timestamp
        self._fill_pose(out, t, q)
        publisher.publish(out)


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
