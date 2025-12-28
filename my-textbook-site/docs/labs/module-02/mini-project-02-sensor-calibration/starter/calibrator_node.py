#!/usr/bin/env python3
"""
Sensor Calibration Node - Starter Template
Module 2 Mini-Project 2

This node:
1. Subscribes to sensor topics (camera, LiDAR, IMU)
2. Collects calibration data samples
3. Compares measurements to ground truth
4. Calculates error metrics
5. Generates a calibration report

TODO: Complete the marked sections
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo
from geometry_msgs.msg import Vector3
import numpy as np
import math
import time
from datetime import datetime


class SensorCalibrator(Node):
    """ROS 2 node for sensor calibration and validation."""

    def __init__(self):
        super().__init__('sensor_calibrator')

        # Ground truth values
        self.ground_truth = {
            'distance_markers': {
                'marker_1m': 1.0,
                'marker_2m': 2.0,
                'marker_3m': 3.0,
                'marker_5m': 5.0,
            },
            'gravity': 9.81,  # m/s^2
        }

        # Data storage for calibration samples
        self.camera_samples = []
        self.lidar_samples = []
        self.imu_samples = []

        # Calibration state
        self.calibration_running = False
        self.samples_to_collect = 100
        self.warmup_complete = False

        # Sensor status
        self.camera_received = False
        self.lidar_received = False
        self.imu_received = False

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # TODO: Create subscriber for camera image
        # Topic: /camera/image_raw
        # Message type: Image
        # Callback: self.camera_callback

        # TODO: Create subscriber for camera info
        # Topic: /camera/camera_info
        # Message type: CameraInfo
        # Callback: self.camera_info_callback

        # TODO: Create subscriber for LiDAR
        # Topic: /scan
        # Message type: LaserScan
        # Callback: self.lidar_callback

        # TODO: Create subscriber for IMU
        # Topic: /imu/data
        # Message type: Imu
        # Callback: self.imu_callback

        # Timer for calibration progress
        self.create_timer(1.0, self.calibration_progress)

        self.get_logger().info('Sensor Calibrator initialized')
        self.get_logger().info('Waiting for sensors...')

    def camera_callback(self, msg: Image):
        """Process camera image for calibration."""
        if not self.camera_received:
            self.get_logger().info(
                f'Camera: {msg.width}x{msg.height} {msg.encoding}'
            )
            self.camera_received = True

        if self.calibration_running and len(self.camera_samples) < self.samples_to_collect:
            # TODO: Store camera data for calibration
            # Consider storing: timestamp, image dimensions, encoding
            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'width': msg.width,
                'height': msg.height,
            }
            self.camera_samples.append(sample)

    def camera_info_callback(self, msg: CameraInfo):
        """Process camera intrinsic parameters."""
        # TODO: Store camera calibration matrix (K)
        # msg.k contains the 3x3 camera matrix
        pass

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan for calibration."""
        if not self.lidar_received:
            self.get_logger().info(
                f'LiDAR: {len(msg.ranges)} samples, '
                f'range: [{msg.range_min:.2f}, {msg.range_max:.2f}]m'
            )
            self.lidar_received = True

        if self.calibration_running and len(self.lidar_samples) < self.samples_to_collect:
            # TODO: Store LiDAR data for calibration
            # Store: ranges, angles, timestamp
            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
            }
            self.lidar_samples.append(sample)

    def imu_callback(self, msg: Imu):
        """Process IMU data for calibration."""
        if not self.imu_received:
            self.get_logger().info('IMU: Receiving data')
            self.imu_received = True

        if self.calibration_running and len(self.imu_samples) < self.samples_to_collect:
            # TODO: Store IMU data for calibration
            # Store: linear_acceleration, angular_velocity, orientation
            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'linear_acceleration': [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                'angular_velocity': [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                'orientation': [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ],
            }
            self.imu_samples.append(sample)

    def calibration_progress(self):
        """Monitor calibration progress."""
        if not self.calibration_running:
            # Check if we can start calibration
            if self.camera_received and self.lidar_received and self.imu_received:
                if not self.warmup_complete:
                    self.get_logger().info('All sensors detected. Warming up...')
                    self.warmup_complete = True
                    # Schedule calibration start
                    self.create_timer(2.0, self.start_calibration)
            return

        # Report progress
        camera_progress = len(self.camera_samples) / self.samples_to_collect * 100
        lidar_progress = len(self.lidar_samples) / self.samples_to_collect * 100
        imu_progress = len(self.imu_samples) / self.samples_to_collect * 100

        self.get_logger().info(
            f'Progress - Camera: {camera_progress:.0f}%, '
            f'LiDAR: {lidar_progress:.0f}%, '
            f'IMU: {imu_progress:.0f}%'
        )

        # Check if calibration is complete
        if (len(self.camera_samples) >= self.samples_to_collect and
            len(self.lidar_samples) >= self.samples_to_collect and
            len(self.imu_samples) >= self.samples_to_collect):

            self.calibration_running = False
            self.analyze_calibration_data()

    def start_calibration(self):
        """Start the calibration data collection."""
        if not self.calibration_running:
            self.get_logger().info(
                f'Starting calibration - collecting {self.samples_to_collect} samples...'
            )
            self.calibration_running = True

    def analyze_lidar_accuracy(self) -> dict:
        """
        Analyze LiDAR range accuracy.

        TODO: Implement LiDAR accuracy analysis
        - Calculate mean and std of range measurements
        - Compare to ground truth distances
        - Calculate RMSE for each target distance
        """
        if not self.lidar_samples:
            return {'error': 'No LiDAR samples'}

        # TODO: Analyze range accuracy
        # Hint: Extract front-facing range values (index ~180 for 360 samples)
        # Compare to known target distances

        result = {
            'samples': len(self.lidar_samples),
            'mean_range': 0.0,
            'std_range': 0.0,
            'rmse': {},
        }

        return result

    def analyze_imu_accuracy(self) -> dict:
        """
        Analyze IMU accuracy.

        TODO: Implement IMU accuracy analysis
        - Calculate gravity magnitude from accelerometer
        - Calculate orientation drift
        - Compute noise statistics
        """
        if not self.imu_samples:
            return {'error': 'No IMU samples'}

        # TODO: Calculate gravity magnitude
        # Hint: sqrt(ax^2 + ay^2 + az^2) when stationary

        # TODO: Calculate accelerometer noise (standard deviation)

        # TODO: Calculate gyroscope noise (standard deviation)

        result = {
            'samples': len(self.imu_samples),
            'gravity_magnitude': 0.0,
            'gravity_error': 0.0,
            'accel_noise': [0.0, 0.0, 0.0],
            'gyro_noise': [0.0, 0.0, 0.0],
        }

        return result

    def analyze_camera_accuracy(self) -> dict:
        """
        Analyze camera calibration data.

        TODO: Implement camera analysis
        - Verify resolution consistency
        - Check for dropped frames
        """
        if not self.camera_samples:
            return {'error': 'No camera samples'}

        result = {
            'samples': len(self.camera_samples),
            'resolution': f'{self.camera_samples[0]["width"]}x{self.camera_samples[0]["height"]}',
        }

        return result

    def analyze_calibration_data(self):
        """Analyze all collected calibration data and generate report."""
        self.get_logger().info('=== Analyzing Calibration Data ===')

        # Analyze each sensor
        camera_results = self.analyze_camera_accuracy()
        lidar_results = self.analyze_lidar_accuracy()
        imu_results = self.analyze_imu_accuracy()

        # Generate report
        self.generate_report(camera_results, lidar_results, imu_results)

    def generate_report(self, camera_results: dict, lidar_results: dict, imu_results: dict):
        """Generate calibration report."""
        report_lines = [
            '=== Sensor Calibration Report ===',
            f'Date: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            f'Samples collected: {self.samples_to_collect}',
            '',
            '--- Camera Calibration ---',
            f'Samples: {camera_results.get("samples", 0)}',
            f'Resolution: {camera_results.get("resolution", "N/A")}',
            '',
            '--- LiDAR Calibration ---',
            f'Samples: {lidar_results.get("samples", 0)}',
            f'Mean range: {lidar_results.get("mean_range", 0):.3f}m',
            f'Std range: {lidar_results.get("std_range", 0):.3f}m',
            '',
            '--- IMU Calibration ---',
            f'Samples: {imu_results.get("samples", 0)}',
            f'Gravity: {imu_results.get("gravity_magnitude", 0):.3f} m/s²',
            f'Gravity error: {imu_results.get("gravity_error", 0):.3f} m/s²',
            '',
            '--- Status ---',
            'Calibration complete. Review results above.',
        ]

        report = '\n'.join(report_lines)
        self.get_logger().info('\n' + report)

        # TODO: Save report to file
        # with open('calibration_report.txt', 'w') as f:
        #     f.write(report)


def main(args=None):
    rclpy.init(args=args)

    calibrator = SensorCalibrator()

    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        calibrator.get_logger().info('Calibration interrupted')
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
