#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint

class TakeoffSmoothClimbRollRight(Node):
    def __init__(self):
        super().__init__('takeoff_smooth_climb_roll_right')

        self.final_altitude = 15.0  # meters
        self.final_y_position = 5.0  # meters to the right
        self.climb_rate = 1.0  # meters/sec
        self.roll_rate = 1.0  # meters/sec
        self.height_threshold = 0.2
        self.position_threshold = 0.2
        self.hover_time = 1.0  # seconds of hover

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Subscriber
        self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, 10)

        self.vehicle_odometry = VehicleOdometry()

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.offboard_setpoint_counter = 0
        self.in_offboard = False
        self.armed = False

        # Smooth control targets
        self.current_target_altitude = 0.0
        self.current_target_y = 0.0

        # State machine
        self.state = 'climb'  # climb -> hover1 -> roll_right -> hover2
        self.hover_start_time = None

        self.last_update_time = time.time()

    def odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, self.current_target_y, -self.current_target_altitude]
        msg.yaw = 0.0  # Maintain yaw facing north
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_setpoint()

        current_height = -self.vehicle_odometry.position[2]
        current_y = self.vehicle_odometry.position[1]

        # After enough initial setpoints, switch to OFFBOARD and ARM
        if self.offboard_setpoint_counter == 50 and not self.in_offboard:
            self.engage_offboard_mode()
            self.arm()
            self.in_offboard = True

        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Smooth climb
        if self.state == 'climb' and self.in_offboard:
            if self.current_target_altitude < self.final_altitude:
                self.current_target_altitude += self.climb_rate * dt
                if self.current_target_altitude >= self.final_altitude:
                    self.current_target_altitude = self.final_altitude
                    self.get_logger().info('Reached target altitude, starting hover1...')
                    self.hover_start_time = time.time()
                    self.state = 'hover1'

        # Hover after climb
        elif self.state == 'hover1':
            if time.time() - self.hover_start_time > self.hover_time:
                self.get_logger().info('Hover1 done, starting roll right...')
                self.state = 'roll_right'

        # Smooth roll right
        elif self.state == 'roll_right':
            if self.current_target_y < self.final_y_position:
                self.current_target_y += self.roll_rate * dt
                if self.current_target_y >= self.final_y_position:
                    self.current_target_y = self.final_y_position
                    self.get_logger().info('Reached right position, starting hover2...')
                    self.hover_start_time = time.time()
                    self.state = 'hover2'

        # Final hover
        elif self.state == 'hover2':
            self.get_logger().info('Hovering at final position.')

        self.get_logger().info(f"Target Alt: {self.current_target_altitude:.2f} m, Target Y: {self.current_target_y:.2f} m | Current Alt: {current_height:.2f} m, Current Y: {current_y:.2f} m")

        self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffSmoothClimbRollRight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()