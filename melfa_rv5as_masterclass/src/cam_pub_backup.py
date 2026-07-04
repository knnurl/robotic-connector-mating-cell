#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pyrealsense2 as rs
import cv2
import numpy as np
import yaml
import time
import math

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create a publisher for the distance topic
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)

        # Initialize parameters
        self.last_log_time = time.time()
        self.target_id = 11  # Marker ID to track
        self.marker_size = 0.021  # Marker size (21mm)

        # Load camera calibration data
        self.camera_matrix, self.dist_coeffs = self.load_camera_calibration('test.yaml')

        # Set up RealSense camera
        self.pipeline = self.setup_realsense_camera()

        # Set up ArUco marker detection
        self.aruco_dict, self.parameters = self.setup_aruco_marker_detection()

        # Define the real-world coordinates of the marker corners in meters
        half_size = self.marker_size / 2
        self.obj_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Create a timer to periodically process frames
        self.timer = self.create_timer(0.1, self.process_frame)

    def load_camera_calibration(self, file_path):
        with open(file_path, 'r') as f:
            calibration_data = yaml.load(f, Loader=yaml.FullLoader)
        
        # Extract the camera matrix and distortion coefficients
        camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
        dist_coeffs = np.array(calibration_data['dist_coeff']['data']).reshape((1, 5))
        return camera_matrix, dist_coeffs

    def setup_realsense_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()

        # Configure the stream to RGB 848x480 and 30fps
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        pipeline.start(config)
        return pipeline

    def setup_aruco_marker_detection(self):
        # ArUco dictionary and detector parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        return aruco_dict, parameters

    def process_frame(self):
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
            index = np.where(ids == self.target_id)[0][0]  # Get the index of the target marker

            # SolvePnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points,           # 3D points in the marker coordinate system
                corners[index][0],    # Corresponding 2D image points
                self.camera_matrix,        # Camera intrinsic matrix
                self.dist_coeffs,          # Distortion coefficients
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                current_time = time.time()
                if current_time - self.last_log_time >= 0.5:  # Check if 0.5 seconds have passed
                    self.last_log_time = current_time

                    # Calculate the distance to the marker
                    distance = np.linalg.norm(tvec)

                    # Publish the distance to the ROS2 topic
                    self.publish_distance(distance)

    def publish_distance(self, distance):
        msg = Float32()
        msg.data = float(distance)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Distance: {distance:.3f} meters")

    def stop(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.stop()
        camera_publisher.get_logger().info('Shutting down Camera Publisher node.')
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

