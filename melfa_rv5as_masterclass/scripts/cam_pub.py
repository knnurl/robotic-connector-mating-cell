# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 17:19:53 2024

@author: matt
"""
#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
import yaml
import math  # Import math for mathematical functions
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_publisher')

        # Publishers for tvec, rvec, distance, and angle_deg
        self.tvec_pub = self.create_publisher(Float32MultiArray, '/aruco/tvec', 10)
        self.rvec_pub = self.create_publisher(Float32MultiArray, '/aruco/rvec', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco/distance', 10)
        self.angle_pub = self.create_publisher(Float32, '/aruco/angle', 10)

        # Last log time for rate limiting
        self.last_log_time = time.time()

        # Load camera calibration data
        self.camera_matrix, self.dist_coeffs = self.load_camera_calibration('test.yaml')

        # Set up RealSense camera and ArUco marker detection
        self.pipeline = self.setup_realsense_camera()
        self.aruco_dict, self.parameters = self.setup_aruco_marker_detection()

        # Marker settings
        self.target_id = 11
        self.marker_size = 0.021  # Marker size (21mm)
        half_size = self.marker_size / 2
        self.obj_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Timer for periodic execution
        self.timer = self.create_timer(0.01, self.timer_callback)

    def load_camera_calibration(self, file_path):
        with open(file_path, 'r') as f:
            calibration_data = yaml.load(f, Loader=yaml.FullLoader)
        camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
        dist_coeffs = np.array(calibration_data['dist_coeff']['data']).reshape((1, 5))
        return camera_matrix, dist_coeffs

    def setup_realsense_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 90)
        pipeline.start(config)
        return pipeline

    def setup_aruco_marker_detection(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        return aruco_dict, parameters

    def timer_callback(self):
        # Wait for a new frame
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # Convert the color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.target_id in ids:
            index = np.where(ids == self.target_id)[0][0]

            # SolvePnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corners[index][0], self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                current_time = time.time()
                if current_time - self.last_log_time >= 0.5:  # Publish every 0.5 seconds
                    self.last_log_time = current_time

                    # Publish tvec and rvec
                    tvec_msg = Float32MultiArray(data=tvec.flatten().tolist())
                    rvec_msg = Float32MultiArray(data=rvec.flatten().tolist())
                    self.tvec_pub.publish(tvec_msg)
                    self.rvec_pub.publish(rvec_msg)

                    # Calculate and publish distance
                    distance = np.linalg.norm(tvec)
                    self.get_logger().info(f'Distance to marker: {distance:.3f} meters')
                    self.distance_pub.publish(Float32(data=distance))

                    # Calculate and publish angle deviation
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    marker_Y_axis = np.array([0, 1, 0], dtype=np.float32)
                    marker_Y_axis_in_camera = rotation_matrix @ marker_Y_axis
                    marker_Y_axis_in_camera_XY = marker_Y_axis_in_camera[:2]
                    angle_rad = np.arctan2(marker_Y_axis_in_camera_XY[0], marker_Y_axis_in_camera_XY[1])
                    angle_deg = np.degrees(angle_rad)
                    self.get_logger().info(f'Angle deviation: {angle_deg:.2f} degrees')
                    self.angle_pub.publish(Float32(data=angle_deg))

                # Optional: Display the frame with the marker
                cv2.imshow("ArUco Marker Detection", color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.pipeline.stop()
                    cv2.destroyAllWindows()
                    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    aruco_marker_publisher = ArucoMarkerPublisher()
    rclpy.spin(aruco_marker_publisher)
    aruco_marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

