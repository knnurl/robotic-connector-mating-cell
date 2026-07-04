# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 17:19:53 2024

@author: matt
"""

import os
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Custom constructor for !!opencv-matrix
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    return mapping

# Register the constructor for 'tag:yaml.org,2002:opencv-matrix'
yaml.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)

# Load RoboDK camera calibration settings
def load_camera_calibration(file_path):
    with open(file_path, 'r') as f:
        calibration_data = yaml.load(f, Loader=yaml.FullLoader)

    # Extract the camera matrix and distortion coefficients
    camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
    dist_coeffs = np.array(calibration_data['dist_coeff']['data']).reshape((1, 5))
    return camera_matrix, dist_coeffs

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')

        # Get the package share directory to locate the test.yaml file
        package_share_directory = get_package_share_directory('roscam')  # Replace 'roscam' with your package name
        calibration_file_path = os.path.join(package_share_directory, 'resource', 'test.yaml')

        # Load camera calibration data
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(calibration_file_path)

        # Set up ArUco marker detection
        self.aruco_dict, self.parameters = self.setup_aruco_marker_detection()

        # Define the target ArUco marker ID
        self.target_id = 11  # Marker ID to track

        # Marker size in meters
        self.marker_size = 0.021  # Marker size (21mm)

        # Publisher for distance
        self.distance_publisher = self.create_publisher(Point, '/cam', 10)

        # Create a CvBridge instance
        self.bridge = CvBridge()

        # Subscriber to the /cam/image topic
        self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)

    def setup_aruco_marker_detection(self):
        # ArUco dictionary and detector parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        return aruco_dict, parameters

    def image_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the frame
        self.process_frame(color_image)

    def process_frame(self, color_image):
        # Convert to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.target_id in ids:
            index = np.where(ids == self.target_id)[0][0]  # Get the index of the target marker

            # Define the real-world coordinates of the marker corners in meters
            half_size = self.marker_size / 2
            obj_points = np.array([
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            # SolvePnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                obj_points,           # 3D points in the marker coordinate system
                corners[index][0],    # Corresponding 2D image points
                self.camera_matrix,   # Camera intrinsic matrix
                self.dist_coeffs,     # Distortion coefficients
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # Create the Point message
                distance_msg = Point()
                distance_msg.y = float(tvec[0])  # x translation
                distance_msg.x = float(tvec[1])  # y translation
                distance_msg.z = float(tvec[2])  # z translation (distance)

                # Publish the distance to the topic
                self.distance_publisher.publish(distance_msg)

                # Log the translation data
                self.get_logger().info(
                    f"Published data - x: {distance_msg.x:.3f}, y: {distance_msg.y:.3f}, z: {distance_msg.z:.3f} meters"
                )

                # Draw the coordinate axes on the marker
                cv2.drawFrameAxes(color_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length=self.marker_size)

        # Display the frame with the ArUco marker overlay
        cv2.imshow("ArUco Marker Detection", color_image)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

