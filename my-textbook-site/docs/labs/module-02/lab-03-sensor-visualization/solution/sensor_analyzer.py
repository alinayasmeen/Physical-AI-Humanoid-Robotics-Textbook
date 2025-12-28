#!/usr/bin/env python3
"""
Sensor Analyzer - Module 2 Lab 3 Solution

Complete implementation of multi-sensor data analysis.

Usage:
    python3 sensor_analyzer.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
import numpy as np
import time


class SensorAnalyzer(Node):
    """Analyzes data from multiple simulated sensors."""

    def __init__(self):
        super().__init__('sensor_analyzer')

        # Statistics tracking
        self.camera_count = 0
        self.lidar_count = 0
        self.imu_count = 0
        self.start_time = time.time()

        # Latest data storage
        self.latest_camera = None
        self.latest_lidar = None
        self.latest_imu = None

        # Create subscriptions
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for periodic statistics report
        self.report_timer = self.create_timer(2.0, self.report_statistics)

        self.get_logger().info('Sensor Analyzer started')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /camera/image_raw (sensor_msgs/Image)')
        self.get_logger().info('  - /scan (sensor_msgs/LaserScan)')
        self.get_logger().info('  - /imu/data (sensor_msgs/Imu)')

    def camera_callback(self, msg):
        """Process camera image messages."""
        self.camera_count += 1
        self.latest_camera = msg

        # Log every 30th message to avoid spam
        if self.camera_count % 30 == 0:
            data_size = len(msg.data) / 1024  # KB
            self.get_logger().info(
                f'Camera [{self.camera_count}]: '
                f'{msg.width}x{msg.height}, {msg.encoding}, '
                f'{data_size:.1f} KB'
            )

    def lidar_callback(self, msg):
        """Process LiDAR scan messages."""
        self.lidar_count += 1
        self.latest_lidar = msg

        # Analyze LiDAR data
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            max_range = np.max(valid_ranges)
            mean_range = np.mean(valid_ranges)
            valid_percent = (len(valid_ranges) / len(ranges)) * 100

            # Log every 10th message
            if self.lidar_count % 10 == 0:
                self.get_logger().info(
                    f'LiDAR [{self.lidar_count}]: '
                    f'min={min_range:.2f}m, max={max_range:.2f}m, '
                    f'mean={mean_range:.2f}m, valid={valid_percent:.1f}%'
                )
        else:
            if self.lidar_count % 10 == 0:
                self.get_logger().warn(
                    f'LiDAR [{self.lidar_count}]: No valid ranges!'
                )

    def imu_callback(self, msg):
        """Process IMU messages."""
        self.imu_count += 1
        self.latest_imu = msg

        # Extract IMU data
        orientation = msg.orientation
        angular_vel = msg.angular_velocity
        linear_acc = msg.linear_acceleration

        # Log every 100th message
        if self.imu_count % 100 == 0:
            # Calculate orientation as roll/pitch/yaw (simplified)
            # Note: For accurate conversion, use tf_transformations
            self.get_logger().info(
                f'IMU [{self.imu_count}]: '
                f'ang_vel=({angular_vel.x:.3f}, {angular_vel.y:.3f}, {angular_vel.z:.3f}) rad/s, '
                f'lin_acc=({linear_acc.x:.2f}, {linear_acc.y:.2f}, {linear_acc.z:.2f}) m/s²'
            )

    def report_statistics(self):
        """Report sensor message statistics."""
        elapsed = time.time() - self.start_time

        self.get_logger().info('=' * 40)
        self.get_logger().info('SENSOR STATISTICS')
        self.get_logger().info('=' * 40)

        # Camera stats
        camera_rate = self.camera_count / elapsed if elapsed > 0 else 0
        status = 'OK' if self.camera_count > 0 else 'NO DATA'
        self.get_logger().info(
            f'Camera:  {self.camera_count:5d} msgs  ({camera_rate:5.1f} Hz) [{status}]'
        )

        # LiDAR stats
        lidar_rate = self.lidar_count / elapsed if elapsed > 0 else 0
        status = 'OK' if self.lidar_count > 0 else 'NO DATA'
        self.get_logger().info(
            f'LiDAR:   {self.lidar_count:5d} msgs  ({lidar_rate:5.1f} Hz) [{status}]'
        )

        # IMU stats
        imu_rate = self.imu_count / elapsed if elapsed > 0 else 0
        status = 'OK' if self.imu_count > 0 else 'NO DATA'
        self.get_logger().info(
            f'IMU:     {self.imu_count:5d} msgs  ({imu_rate:5.1f} Hz) [{status}]'
        )

        # Warnings for missing data
        if self.camera_count == 0:
            self.get_logger().warn(
                'No camera data! Check topic /camera/image_raw'
            )
        if self.lidar_count == 0:
            self.get_logger().warn(
                'No LiDAR data! Check topic /scan'
            )
        if self.imu_count == 0:
            self.get_logger().warn(
                'No IMU data! Check topic /imu/data'
            )

        self.get_logger().info('=' * 40)

    def get_summary(self):
        """Return a summary of sensor status."""
        return {
            'camera': {
                'count': self.camera_count,
                'active': self.camera_count > 0,
                'latest': self.latest_camera is not None
            },
            'lidar': {
                'count': self.lidar_count,
                'active': self.lidar_count > 0,
                'latest': self.latest_lidar is not None
            },
            'imu': {
                'count': self.imu_count,
                'active': self.imu_count > 0,
                'latest': self.latest_imu is not None
            }
        }


def main():
    """Main entry point."""
    rclpy.init()

    analyzer = SensorAnalyzer()

    try:
        print("=" * 50)
        print("Sensor Analyzer - Module 2 Lab 3 (Solution)")
        print("=" * 50)
        print()
        print("Listening for sensor data from Gazebo...")
        print("Make sure:")
        print("  1. Gazebo is running with sensor-equipped robot")
        print("  2. ros_gz bridge is active")
        print()
        print("Press Ctrl+C to stop")
        print()

        rclpy.spin(analyzer)

    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("FINAL SUMMARY")
        print("=" * 50)

        summary = analyzer.get_summary()
        for sensor, data in summary.items():
            status = 'Active' if data['active'] else 'Inactive'
            print(f"  {sensor.upper()}: {data['count']} messages ({status})")

        print("=" * 50)
        print("Shutting down...")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
