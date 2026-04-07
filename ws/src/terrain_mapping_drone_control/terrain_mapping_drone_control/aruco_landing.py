#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand
from rclpy.qos import qos_profile_sensor_data

class ArucoLanding(Node):
    def __init__(self):
        super().__init__('aruco_landing')

        self.altitude = 2.0  # Target takeoff height
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.marker_id = 0  # <<< Set your marker ID after finding it

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.image_sub = self.create_subscription(Image, '/rgb_camera', self.image_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0, self.takeoff)

        self.takeoff_sent = False
        self.landing_sent = False

        self.get_logger().info('Aruco Landing Node Started')

    def takeoff(self):
        if not self.takeoff_sent:
            self.publish_vehicle_command(400, 1)  # Arm
            self.publish_vehicle_command(22, self.altitude)  # Takeoff
            self.takeoff_sent = True
            self.get_logger().info('Sent Arm and Takeoff commands')

    def publish_vehicle_command(self, command, param7=0.0):
        msg = VehicleCommand()
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f'Detected ArUco IDs: {ids.flatten()}')  # <<< Print detected IDs

            if self.marker_id in ids:
                idx = np.where(ids == self.marker_id)[0][0]
                marker_corners = corners[idx][0]
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                img_center_x = frame.shape[1] // 2
                img_center_y = frame.shape[0] // 2

                error_x = center_x - img_center_x
                error_y = center_y - img_center_y

                cmd = Twist()
                kp = 0.12  # Aggressive speed

                # ✅ Corrected movement
                cmd.linear.x = kp * error_y   # Forward/backward (vertical error)
                cmd.linear.y = -kp * error_x  # Left/right (horizontal error)

                self.cmd_vel_pub.publish(cmd)

                self.get_logger().info(f'Aruco Detected: error_x={error_x}, error_y={error_y}, cmd_x={cmd.linear.x:.2f}, cmd_y={cmd.linear.y:.2f}')

                if abs(error_x) < 120 and abs(error_y) < 120:
                    if not self.landing_sent:
                        self.get_logger().info('Centered above Aruco, initiating Landing...')
                        self.publish_vehicle_command(21)  # LAND
                        self.landing_sent = True
            else:
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)

        else:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLanding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()