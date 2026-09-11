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

With an eye-in-hand camera the optical frame MOVES with the robot, so robot
motion looks like marker motion to the filter: a step move can trip the
innovation gate on perfectly good detections, and predictions made during
motion are systematically wrong. Set `filter_frame` (e.g. the robot base)
to run the filter where the marker is genuinely static: each detection is
re-expressed via TF at the image timestamp before fusion, and /aruco/pose
is then published in that frame (the controller transforms it to the
planning frame either way). /aruco/pose_raw always stays in the optical
frame - the hand-eye calibration tool depends on that.

Frame sources (`source` parameter):
  topic      (default) images arrive as ROS topics from a camera driver -
             the classic wiring; unchanged behaviour.
  realsense  capture IN-PROCESS via pyrealsense2 (roscam.rs_capture): no
             image ever enters the DDS graph, only poses do (~2 KB/s vs
             ~13-26 MB/s). This is the FR3 bandwidth fix - image traffic
             cannot contend with the 1 kHz FCI loop if it never exists.
             Intrinsics come from the SDK; frames are stamped with node
             time minus `capture_latency_s`. The debug image is throttled
             to `debug_max_hz` and only encoded while subscribed.
  external   no subscriptions, no capture: an embedding process (see
             vision_standalone.py) injects intrinsics via set_intrinsics()
             and frames via process_frame().
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from roscam.plane_normal import (disambiguate_by_normal, fuse_orientation,
                                 marker_plane_normal)
from roscam.pose_kf import PoseKF, compose_pose
from roscam.rs_capture import RsCapture


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
    def __init__(self, **node_kwargs):
        super().__init__('aruco_pose_publisher', **node_kwargs)

        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_id', 11)
        self.declare_parameter('marker_size_m', 0.021)
        self.declare_parameter('aruco_dictionary', 'DICT_6X6_250')
        # Resolve the IPPE_SQUARE mirror ambiguity (single-marker only; a
        # board is already well-conditioned). 'depth' uses the depth-fitted
        # plane normal as ground truth and falls back to temporal lock-in
        # when depth is absent; 'temporal' only locks onto the first branch
        # seen; 'off' restores the old cv2.solvePnP behaviour.
        self.declare_parameter('tilt_disambiguation', 'depth')
        self.declare_parameter('tilt_depth_scale', 1.6)
        # Where the OUT-OF-PLANE part of the orientation comes from.
        # 'depth': rebuild it from the depth plane normal, keeping ArUco's
        # in-plane rotation. Measured on this cell, IPPE reads the tilt
        # ~2.6 deg high with a floor a control loop cannot drive through,
        # while the depth fit is unbiased at 0.11 mm rms. 'aruco' keeps the
        # IPPE orientation as-is (previous behaviour).
        self.declare_parameter('tilt_source', 'depth')
        # Multi-marker grid board (occlusion robustness + accuracy):
        # board_markers_x*y > 1 switches from the single marker to a grid of
        # ids marker_id..marker_id+N-1. Pose = board centre, so taught
        # offsets keep their meaning. Print with `cv2.aruco.GridBoard(...)
        # .generateImage()` using the same geometry.
        self.declare_parameter('board_markers_x', 1)
        self.declare_parameter('board_markers_y', 1)
        self.declare_parameter('board_marker_separation_m', 0.005)
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
        # Fixed frame to run the filter in ('' = filter in the optical
        # frame, the legacy behaviour). Requires TF <filter_frame> -> optical
        # (robot state publisher + hand-eye static TF).
        self.declare_parameter('filter_frame', '')
        # Frame source: 'topic' (ROS image topics, default), 'realsense'
        # (in-process pyrealsense2 - zero image traffic on the DDS graph),
        # 'external' (embedder feeds frames; see vision_standalone.py).
        self.declare_parameter('source', 'topic')
        self.declare_parameter('optical_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('capture_width', 640)
        self.declare_parameter('capture_height', 480)
        self.declare_parameter('capture_fps', 15)
        # Self-stamped frames: node time minus this capture latency estimate.
        self.declare_parameter('capture_latency_s', 0.02)
        # Debug-image rate cap outside topic mode (it exists for humans;
        # full rate is pure bandwidth waste).
        self.declare_parameter('debug_max_hz', 5.0)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.max_reproj_err = float(self.get_parameter('max_reprojection_error_px').value)
        self.max_prediction_s = float(self.get_parameter('max_prediction_s').value)
        self.rejects_before_reacquire = int(self.get_parameter('rejects_before_reacquire').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.filter_frame = str(self.get_parameter('filter_frame').value)
        self.source = str(self.get_parameter('source').value)
        if self.source not in ('topic', 'realsense', 'external'):
            raise RuntimeError(f"source must be topic|realsense|external, got '{self.source}'")
        self.optical_frame_id = str(self.get_parameter('optical_frame_id').value)
        self.capture_latency = float(self.get_parameter('capture_latency_s').value)
        debug_max_hz = float(self.get_parameter('debug_max_hz').value)
        # topic mode keeps the historical every-frame debug behaviour.
        self._debug_period = 0.0 if self.source == 'topic' else 1.0 / max(debug_max_hz, 0.1)
        self._last_debug_t = 0.0

        self.tf_buffer = None
        if self.filter_frame:
            self.tf_buffer = Buffer()
            # Own spin thread so a blocking lookup inside frame processing
            # cannot starve the /tf subscriptions.
            self.tf_listener = TransformListener(self.tf_buffer, self,
                                                 spin_thread=True)

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

        self.aruco_dict = self._resolve_dictionary(
            str(self.get_parameter('aruco_dictionary').value))
        self.detect = self._make_detector(self.aruco_dict)

        # Multi-marker board: ids marker_id..marker_id+N-1 in a grid. The
        # published pose is the BOARD CENTRE (same semantics as the single
        # marker's centre) and any visible subset of markers suffices -
        # partial occlusion by the gripper no longer kills the pose.
        bx = int(self.get_parameter('board_markers_x').value)
        by = int(self.get_parameter('board_markers_y').value)
        self.board = None
        self.board_ids = np.array([self.marker_id], dtype=np.int32)
        if bx * by > 1:
            sep = float(self.get_parameter('board_marker_separation_m').value)
            self.board_ids = np.arange(self.marker_id, self.marker_id + bx * by,
                                       dtype=np.int32)
            self.board = cv2.aruco.GridBoard((bx, by), self.marker_size, sep,
                                             self.aruco_dict, self.board_ids)
            pts = np.concatenate([np.asarray(p).reshape(-1, 3)
                                  for p in self.board.getObjPoints()])
            self.board_center = ((pts.min(axis=0) + pts.max(axis=0)) / 2.0
                                 ).astype(np.float32)
            self.get_logger().info(
                f'Board mode: {bx}x{by} markers (ids {self.board_ids[0]}..'
                f'{self.board_ids[-1]}), separation {sep * 1000:.1f} mm; '
                'pose = board centre.')

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        # Filter timing state
        self.last_frame_stamp = None   # stamp of the previous processed frame
        self.tilt_disambiguation = str(
            self.get_parameter('tilt_disambiguation').value).lower()
        self.tilt_depth_scale = float(
            self.get_parameter('tilt_depth_scale').value)
        self.tilt_source = str(self.get_parameter('tilt_source').value).lower()
        self._last_normal = None       # last accepted marker normal (cam)
        self._depth_fit = None         # last depth plane fit, for diagnostics
        # Depth-fused rotation, or None. Must exist before any frame: the
        # board path never calls _solve_single, and process_frame reads it.
        self._fused_R = None
        self.last_disambig = {}        # which IPPE branch was taken, and why
        self.last_accept_stamp = None  # stamp of the last accepted measurement
        self.consecutive_rejects = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.pose_raw_pub = self.create_publisher(PoseStamped, '/aruco/pose_raw', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 2)

        self._capture = None
        self._capture_thread = None
        self._stop_capture = threading.Event()
        if self.source == 'topic':
            self.create_subscription(
                CameraInfo, str(self.get_parameter('camera_info_topic').value),
                self.camera_info_callback, 10)
            self.create_subscription(
                Image, str(self.get_parameter('image_topic').value),
                self.image_callback, 5)
            waiting = 'waiting for camera_info...'
        elif self.source == 'realsense':
            self._capture = RsCapture(
                width=int(self.get_parameter('capture_width').value),
                height=int(self.get_parameter('capture_height').value),
                fps=int(self.get_parameter('capture_fps').value),
                # depth is needed to resolve the IPPE mirror ambiguity
                enable_depth=(self.tilt_disambiguation == 'depth'))
            self._capture.on_event = lambda m: self.get_logger().warn(m)
            intr = self._capture.start()
            self.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy, intr.coeffs)
            self._capture_thread = threading.Thread(target=self._capture_loop,
                                                    daemon=True)
            self._capture_thread.start()
            waiting = (f'in-process RealSense capture {intr.width}x{intr.height} - '
                       f'no image topics on the graph')
        else:  # external: an embedder feeds set_intrinsics() + process_frame()
            waiting = 'external source - waiting for embedder frames'

        self.get_logger().info(
            f'Tracking ArUco id {self.marker_id} ({self.marker_size * 1000:.0f} mm), '
            f'{waiting}')

    def _resolve_dictionary(self, dictionary_name):
        try:
            dict_id = getattr(cv2.aruco, dictionary_name)
        except AttributeError:
            self.get_logger().error(
                f'Unknown ArUco dictionary "{dictionary_name}", using DICT_6X6_250')
            dict_id = cv2.aruco.DICT_6X6_250
        return cv2.aruco.getPredefinedDictionary(dict_id)

    def _make_detector(self, aruco_dict):
        if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            return lambda gray: detector.detectMarkers(gray)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return lambda gray: cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    def _detect_pose(self, corners, ids, depth_m=None):
        """Pose of the single marker (IPPE_SQUARE) or of the board from
        whatever subset of its markers is visible (planar IPPE, >= 1 marker).
        Returns (rvec, tvec, obj_pts, img_pts, used_corners) or None."""
        if ids is None:
            return None
        flat = ids.flatten()
        if self.board is None:
            if self.marker_id not in flat:
                return None
            index = int(np.where(flat == self.marker_id)[0][0])
            img_pts = corners[index][0].astype(np.float32)
            rvec, tvec = self._solve_single(img_pts, depth_m)
            if rvec is None:
                return None
            return rvec, tvec, self.obj_points, img_pts, [corners[index]]

        keep = [i for i, mid in enumerate(flat) if mid in self.board_ids]
        if not keep:
            return None
        kept_corners = [corners[i] for i in keep]
        kept_ids = flat[keep].reshape(-1, 1).astype(np.int32)
        obj_pts, img_pts = self.board.matchImagePoints(kept_corners, kept_ids)
        if obj_pts is None or len(obj_pts) < 4:
            return None
        # Re-centre so the published pose is the board CENTRE, not a corner.
        obj_pts = obj_pts.reshape(-1, 3).astype(np.float32) - self.board_center
        img_pts = img_pts.reshape(-1, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        return rvec, tvec, obj_pts, img_pts, kept_corners

    def _solve_single(self, img_pts, depth_m):
        """Single-marker pose with the IPPE mirror ambiguity resolved.

        IPPE_SQUARE has TWO solutions for a planar square: equal tilt
        magnitude, mirrored tilt direction. cv2.solvePnP returns only the
        lower-reprojection-error one, and for a small near-fronto-parallel
        marker the two errors are nearly equal, so the choice flips frame to
        frame - measured here as a >120 deg flip of the normal's in-plane
        direction on 62% of frames, which makes orientation control diverge.

        solvePnPGeneric returns both; we pick by agreement with a reference
        normal: the depth-fitted plane (unambiguous) when available, else the
        last accepted normal (temporal lock-in).
        """
        n_sol, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            self.obj_points, img_pts, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not n_sol:
            self.last_disambig = {'mode': 'none', 'n_solutions': 0}
            return None, None
        if n_sol == 1 or self.tilt_disambiguation == 'off':
            self.last_disambig = {'mode': 'off', 'n_solutions': int(n_sol)}
            self._last_normal = cv2.Rodrigues(rvecs[0])[0][:, 2]
            return rvecs[0], tvecs[0]

        normals = [cv2.Rodrigues(r)[0][:, 2] for r in rvecs]

        ref, src = None, 'none'
        self._depth_fit = None
        if self.tilt_disambiguation == 'depth' and depth_m is not None:
            fit = marker_plane_normal(
                depth_m, img_pts, self.camera_matrix[0, 0],
                self.camera_matrix[1, 1], self.camera_matrix[0, 2],
                self.camera_matrix[1, 2], scale=self.tilt_depth_scale)
            if fit is not None:
                ref, src = fit['normal'], 'depth'
                self._depth_fit = fit
        if ref is None and self._last_normal is not None:
            ref, src = self._last_normal, 'temporal'

        if ref is None:
            # Nothing to compare against yet: keep OpenCV's pick, and say so.
            pick, agree = 0, float('nan')
            src = 'unverified'
            self.get_logger().warn(
                'IPPE ambiguity unresolved (no depth, no history) - first '
                'orientation may be the mirror solution',
                throttle_duration_sec=10.0)
        else:
            pick, agree = disambiguate_by_normal(normals, ref)

        self._last_normal = normals[pick]
        self.last_disambig = {
            'mode': src, 'n_solutions': int(n_sol), 'picked': int(pick),
            'agreement': float(agree),
            'flipped_opencv_choice': bool(pick != 0),
        }
        # Out-of-plane from depth, in-plane from ArUco (see tilt_source).
        # Computed here but NOT substituted into rvec: _validate gates on
        # reprojection error against the ArUco corners, and swapping the
        # orientation first shifts them ~1.8 px at 2.6 deg on a 21 mm marker
        # at 100 mm - right against the 2.0 px gate, so the good detection
        # would be thrown away. process_frame applies it after the gate.
        self._fused_R = None
        if self.tilt_source == 'depth' and self._depth_fit is not None:
            R = cv2.Rodrigues(rvecs[pick])[0]
            fused = fuse_orientation(R, self._depth_fit['normal'])
            if fused is not None:
                z = np.array([0.0, 0.0, 1.0])
                self._fused_R = fused
                self.last_disambig['tilt_aruco_deg'] = float(np.degrees(
                    np.arccos(np.clip(abs(R[:, 2] @ z), -1, 1))))
                self.last_disambig['tilt_depth_deg'] = float(np.degrees(
                    np.arccos(np.clip(abs(fused[:, 2] @ z), -1, 1))))
                self.last_disambig['tilt_source'] = 'depth'
        return rvecs[pick], tvecs[pick]

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
            self.get_logger().info('Camera intrinsics received.')

    def set_intrinsics(self, fx, fy, cx, cy, coeffs=None):
        """Intrinsics from an SDK/embedder instead of camera_info."""
        self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy],
                                       [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array(coeffs or [0.0] * 5,
                                    dtype=np.float64).reshape(1, -1)

    def self_stamped_header(self):
        """Header for frames captured in-process: node time minus the
        capture latency estimate, in the configured optical frame."""
        h = Header()
        h.stamp = (self.get_clock().now()
                   - Duration(seconds=self.capture_latency)).to_msg()
        h.frame_id = self.optical_frame_id
        return h

    def shutdown_capture(self):
        self._stop_capture.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)
        if self._capture is not None:
            self._capture.stop()

    def _capture_loop(self):
        while rclpy.ok() and not self._stop_capture.is_set():
            frame = self._capture.wait_frame(timeout_s=1.0)
            if frame is None:
                self.get_logger().warn('RealSense frame timeout',
                                       throttle_duration_sec=5.0)
                continue
            try:
                self.process_frame(frame.bgr, self.self_stamped_header(),
                                   depth_m=frame.depth_m)
            except Exception as e:  # keep capturing; a bad frame is not fatal
                self.get_logger().error(f'frame processing failed: {e}',
                                        throttle_duration_sec=5.0)

    def image_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(color_image, msg.header)

    def process_frame(self, color_image, header, depth_m=None):
        """Full detection/filter/publish pipeline for one BGR frame.

        depth_m (HxW metres, aligned to colour) is optional and used only to
        resolve the IPPE mirror ambiguity; everything else is unchanged when
        it is absent.
        """
        if self.camera_matrix is None:
            return

        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_frame_stamp is None else stamp - self.last_frame_stamp
        self.last_frame_stamp = stamp

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)

        measurement = None
        raw_optical = None
        rvec = tvec = used_corners = None
        det = self._detect_pose(corners, ids, depth_m)
        if det is not None:
            rvec, tvec, obj_pts, img_pts, used_corners = det
            raw_optical = self._validate(rvec, tvec, img_pts, obj_pts)
            if raw_optical is not None and self._fused_R is not None:
                # Gate passed on the ArUco fit; now take the unbiased
                # out-of-plane orientation from depth (see _solve_single).
                raw_optical = (raw_optical[0],
                               rotation_matrix_to_quaternion(self._fused_R))
                rvec = cv2.Rodrigues(self._fused_R)[0]   # debug axes too
            measurement = raw_optical
            if raw_optical is not None and self.filter_frame:
                # Filter where the marker is static: re-express the
                # detection in filter_frame via TF at the image stamp.
                measurement = self._to_filter_frame(raw_optical, header)

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
                # Raw stays in the optical frame (hand-eye calib needs it).
                self._publish(self.pose_raw_pub, header, *raw_optical)
                self._publish(self.pose_pub, self._pose_header(header),
                              self.kf.position, self.kf.quaternion)
                published = True
                self.get_logger().info(
                    f'Marker at [{self.kf.position[0]:.3f}, {self.kf.position[1]:.3f}, '
                    f'{self.kf.position[2]:.3f}] m '
                    f'({self.filter_frame or "optical"} frame)',
                    throttle_duration_sec=1.0)
            if self.publish_debug:
                cv2.aruco.drawDetectedMarkers(color_image, used_corners)
                cv2.drawFrameAxes(color_image, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_size)

        # Bridge brief dropouts with the filter prediction; go silent beyond
        # the horizon so the controller holds position.
        if (not published and self.kf.initialized and
                self.last_accept_stamp is not None and
                stamp - self.last_accept_stamp <= self.max_prediction_s):
            self._publish(self.pose_pub, self._pose_header(header),
                          self.kf.position, self.kf.quaternion)
            self.get_logger().warn('Publishing predicted pose (marker not detected).',
                                   throttle_duration_sec=1.0)

        # Debug image: only while subscribed, and rate-capped outside topic
        # mode - it exists for humans, full rate is bandwidth waste.
        now_mono = time.monotonic()
        if (self.publish_debug and self.debug_pub.get_subscription_count() > 0
                and now_mono - self._last_debug_t >= self._debug_period):
            self._last_debug_t = now_mono
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)

    def _validate(self, rvec, tvec, img_points, obj_points):
        """Gate a raw solvePnP result on reprojection error.
        Returns (t, q) or None if rejected."""
        projected, _ = cv2.projectPoints(
            obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
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

    def _to_filter_frame(self, measurement, header):
        """Re-express an optical-frame measurement in filter_frame using TF
        at the image timestamp. Returns (t, q) or None if TF is unavailable
        (the frame is then treated as having no detection: the filter
        predicts briefly, then goes silent and the controller holds)."""
        t, q = measurement
        try:
            tf = self.tf_buffer.lookup_transform(
                self.filter_frame, header.frame_id,
                Time.from_msg(header.stamp), timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(
                f'TF {header.frame_id} -> {self.filter_frame} unavailable '
                f'({e}); dropping detection',
                throttle_duration_sec=2.0)
            return None
        tr = tf.transform.translation
        ro = tf.transform.rotation
        p_ft = np.array([tr.x, tr.y, tr.z])
        q_ft = np.array([ro.x, ro.y, ro.z, ro.w])
        return compose_pose(p_ft, q_ft, t, q)

    def _pose_header(self, image_header):
        """Header for the filtered pose: image stamp, filter frame if set."""
        if not self.filter_frame:
            return image_header
        h = Header()
        h.stamp = image_header.stamp
        h.frame_id = self.filter_frame
        return h

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
        node.shutdown_capture()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
