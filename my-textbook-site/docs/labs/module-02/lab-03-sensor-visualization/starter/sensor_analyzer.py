#!/usr/bin/env python3
"""
Sensor Analyzer - Module 2 Lab 3 Starter Code

This script subscribes to simulated sensor topics and analyzes the data.
Complete the TODOs to implement the sensor analysis functionality.

Prerequisites:
- ROS 2 Humble
- Gazebo Fortress with sensors publishing
- ros_gz bridge running

Usage:
    python3 sensor_analyzer.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
import numpy as np


class SensorAnalyzer(Node):
    """Analyzes data from multiple simulated sensors."""

    def __init__(self):
        super().__init__('sensor_analyzer')

        # Statistics tracking
        self.camera_count = 0
        self.lidar_count = 0
        self.imu_count = 0

        # TODO 1: Create subscription for camera images
        # Topic: /camera/image_raw
        # Message type: sensor_msgs/msg/Image
        # Callback: self.camera_callback
        # self.camera_sub = self.create_subscription(...)

        # TODO 2: Create subscription for LiDAR scans
        # Topic: /scan
        # Message type: sensor_msgs/msg/LaserScan
        # Callback: self.lidar_callback
        # self.lidar_sub = self.create_subscription(...)

        # TODO 3: Create subscription for IMU data
        # Topic: /imu/data
        # Message type: sensor_msgs/msg/Imu
        # Callback: self.imu_callback
        # self.imu_sub = self.create_subscription(...)

        # Timer for periodic statistics report
        self.report_timer = self.create_timer(2.0, self.report_statistics)

        self.get_logger().info('Sensor Analyzer started')
        self.get_logger().info('Waiting for sensor data...')

    def camera_callback(self, msg):
        """Process camera image messages."""
        self.camera_count += 1

        # TODO 4: Extract and log image properties
        # - Image dimensions (width x height)
        # - Encoding format
        # - Data size
        # self.get_logger().info(f'Camera: {msg.width}x{msg.height}, {msg.encoding}')

    def lidar_callback(self, msg):
        """Process LiDAR scan messages."""
        self.lidar_count += 1

        # TODO 5: Analyze LiDAR data
        # - Find minimum range (closest obstacle)
        # - Find maximum range
        # - Count valid readings (not inf or nan)
        # ranges = np.array(msg.ranges)
        # valid_ranges = ranges[np.isfinite(ranges)]
        # if len(valid_ranges) > 0:
        #     min_range = np.min(valid_ranges)
        #     max_range = np.max(valid_ranges)
        #     self.get_logger().info(f'LiDAR: min={min_range:.2f}m, max={max_range:.2f}m')

    def imu_callback(self, msg):
        """Process IMU messages."""
        self.imu_count += 1

        # TODO 6: Extract IMU data
        # - Orientation (quaternion)
        # - Angular velocity (rad/s)
        # - Linear acceleration (m/s^2)
        # orientation = msg.orientation
        # angular_vel = msg.angular_velocity
        # linear_acc = msg.linear_acceleration
        # self.get_logger().info(
        #     f'IMU: ang_vel=({angular_vel.x:.2f}, {angular_vel.y:.2f}, {angular_vel.z:.2f})'
        # )

    def report_statistics(self):
        """Report sensor message statistics."""
        # TODO 7: Report message counts and rates
        # Calculate messages per second for each sensor
        self.get_logger().info('--- Sensor Statistics ---')
        self.get_logger().info(f'Camera messages: {self.camera_count}')
        self.get_logger().info(f'LiDAR messages: {self.lidar_count}')
        self.get_logger().info(f'IMU messages: {self.imu_count}')

        # TODO 8: Check for missing sensors
        # Warn if any sensor has 0 messages
        # if self.camera_count == 0:
        #     self.get_logger().warn('No camera data received!')


def main():
    """Main entry point."""
    rclpy.init()

    analyzer = SensorAnalyzer()

    try:
        print("=" * 50)
        print("Sensor Analyzer - Module 2 Lab 3")
        print("=" * 50)
        print("\nListening for sensor data...")
        print("Make sure Gazebo and ros_gz bridge are running!\n")

        rclpy.spin(analyzer)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
