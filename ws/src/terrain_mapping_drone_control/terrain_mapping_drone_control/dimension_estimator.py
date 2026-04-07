#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class DimensionEstimator(Node):
    def __init__(self):
        super().__init__('dimension_estimator')

        # Camera parameters (assumed)
        self.marker_real_size = 0.2  # meters (real-world size of ArUco marker)
        self.fx = 554.0  # Focal length in pixels (example value, tune if needed)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        # ROS2 Subscription
        self.image_sub = self.create_subscription(
            Image,
            '/rgb_camera',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Dimension Estimator Node Started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                marker = corners[i][0]

                # Calculate width of the marker in pixels
                top_edge = np.linalg.norm(marker[0] - marker[1])
                bottom_edge = np.linalg.norm(marker[2] - marker[3])
                avg_pixel_width = (top_edge + bottom_edge) / 2.0

                # Estimate distance using pinhole camera model
                if avg_pixel_width > 0:
                    distance = (self.fx * self.marker_real_size) / avg_pixel_width

                    self.get_logger().info(
                        f"Marker ID {ids[i][0]} | Pixel Width: {avg_pixel_width:.2f} px | Estimated Distance: {distance:.2f} m"
                    )
                else:
                    self.get_logger().warn('Marker detected but pixel width too small')

def main(args=None):
    rclpy.init(args=args)
    node = DimensionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()