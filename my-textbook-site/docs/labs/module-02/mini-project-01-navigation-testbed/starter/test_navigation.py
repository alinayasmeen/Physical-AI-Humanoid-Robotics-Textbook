#!/usr/bin/env python3
"""
Navigation Testbed Test Script - Starter Template
Module 2 Mini-Project 1

This script tests the navigation testbed by:
1. Verifying robot sensors are publishing
2. Sending the robot to waypoints
3. Reporting navigation progress

TODO: Complete the marked sections
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
import time


class NavigationTester(Node):
    """Test node for navigation testbed validation."""

    def __init__(self):
        super().__init__('navigation_tester')

        # Waypoint definitions (x, y) coordinates
        self.waypoints = [
            (-8.0, 0.0),   # Start position
            (-3.0, 3.0),   # Waypoint 1
            (3.0, -2.0),   # Waypoint 2
            (8.0, 0.0),    # Goal position
        ]
        self.current_waypoint_idx = 0

        # Sensor status tracking
        self.lidar_received = False
        self.camera_received = False
        self.imu_received = False
        self.odom_received = False

        # Current robot position (from odometry)
        self.current_x = 0.0
        self.current_y = 0.0

        # TODO: Create publisher for velocity commands
        # Topic: /cmd_vel
        # Message type: Twist
        self.cmd_vel_pub = None  # Replace with actual publisher

        # TODO: Create subscriber for LiDAR data
        # Topic: /scan (or your configured topic)
        # Message type: LaserScan
        # Callback: self.lidar_callback

        # TODO: Create subscriber for camera data
        # Topic: /camera/image_raw (or your configured topic)
        # Message type: Image
        # Callback: self.camera_callback

        # TODO: Create subscriber for IMU data
        # Topic: /imu/data (or your configured topic)
        # Message type: Imu
        # Callback: self.imu_callback

        # TODO: Create subscriber for odometry
        # Topic: /odom
        # Message type: Odometry
        # Callback: self.odom_callback

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Navigation Tester initialized')

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan data."""
        if not self.lidar_received:
            self.get_logger().info(f'LiDAR: Receiving {len(msg.ranges)} ranges')
            self.lidar_received = True

        # TODO: Store LiDAR data for obstacle detection
        # Check minimum range for collision avoidance
        pass

    def camera_callback(self, msg: Image):
        """Process camera image data."""
        if not self.camera_received:
            self.get_logger().info(f'Camera: {msg.width}x{msg.height} image')
            self.camera_received = True

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        if not self.imu_received:
            self.get_logger().info('IMU: Receiving orientation and acceleration')
            self.imu_received = True

    def odom_callback(self, msg: Odometry):
        """Process odometry data."""
        if not self.odom_received:
            self.get_logger().info('Odometry: Receiving position updates')
            self.odom_received = True

        # TODO: Update current robot position
        # self.current_x = msg.pose.pose.position.x
        # self.current_y = msg.pose.pose.position.y
        pass

    def check_sensors(self) -> bool:
        """Verify all sensors are publishing."""
        sensors_ok = self.lidar_received and self.imu_received

        if not sensors_ok:
            missing = []
            if not self.lidar_received:
                missing.append('LiDAR')
            if not self.camera_received:
                missing.append('Camera')
            if not self.imu_received:
                missing.append('IMU')
            if not self.odom_received:
                missing.append('Odometry')

            if missing:
                self.get_logger().warn(f'Waiting for sensors: {", ".join(missing)}')

        return sensors_ok

    def distance_to_waypoint(self, waypoint_idx: int) -> float:
        """Calculate distance to a waypoint."""
        if waypoint_idx >= len(self.waypoints):
            return float('inf')

        wx, wy = self.waypoints[waypoint_idx]
        dx = wx - self.current_x
        dy = wy - self.current_y

        return (dx**2 + dy**2)**0.5

    def navigate_to_waypoint(self, waypoint_idx: int):
        """
        Generate velocity commands to navigate to waypoint.

        TODO: Implement simple navigation logic
        - Calculate direction to waypoint
        - Set linear and angular velocity
        - Consider obstacle avoidance using LiDAR
        """
        if waypoint_idx >= len(self.waypoints):
            return

        wx, wy = self.waypoints[waypoint_idx]

        # TODO: Calculate direction vector
        dx = wx - self.current_x
        dy = wy - self.current_y

        # TODO: Create and publish Twist message
        # cmd = Twist()
        # cmd.linear.x = ...
        # cmd.angular.z = ...
        # self.cmd_vel_pub.publish(cmd)
        pass

    def control_loop(self):
        """Main control loop - runs at 10Hz."""

        # First, check if sensors are ready
        if not self.check_sensors():
            return

        # Check if we've reached current waypoint
        distance = self.distance_to_waypoint(self.current_waypoint_idx)

        if distance < 0.5:  # Within 0.5m of waypoint
            waypoint_name = ['Start', 'Waypoint 1', 'Waypoint 2', 'Goal'][
                min(self.current_waypoint_idx, 3)
            ]
            self.get_logger().info(f'Reached {waypoint_name}!')

            self.current_waypoint_idx += 1

            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('Navigation complete! All waypoints reached.')
                # Stop the robot
                if self.cmd_vel_pub is not None:
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                return

        # Navigate to current waypoint
        self.navigate_to_waypoint(self.current_waypoint_idx)

    def run_sensor_check(self):
        """Run a 10-second sensor check and report results."""
        self.get_logger().info('Starting sensor check (10 seconds)...')

        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Report results
        self.get_logger().info('=== Sensor Check Results ===')
        self.get_logger().info(f'LiDAR: {"OK" if self.lidar_received else "NOT RECEIVED"}')
        self.get_logger().info(f'Camera: {"OK" if self.camera_received else "NOT RECEIVED"}')
        self.get_logger().info(f'IMU: {"OK" if self.imu_received else "NOT RECEIVED"}')
        self.get_logger().info(f'Odometry: {"OK" if self.odom_received else "NOT RECEIVED"}')

        all_ok = all([self.lidar_received, self.imu_received])
        return all_ok


def main(args=None):
    rclpy.init(args=args)

    tester = NavigationTester()

    # Run sensor check first
    if not tester.run_sensor_check():
        tester.get_logger().error('Sensor check failed. Please verify robot configuration.')
        tester.destroy_node()
        rclpy.shutdown()
        return

    tester.get_logger().info('Sensor check passed. Starting navigation test...')

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
