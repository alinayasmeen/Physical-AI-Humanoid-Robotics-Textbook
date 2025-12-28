#!/usr/bin/env python3
"""
Sensor Calibration Node - Complete Solution
Module 2 Mini-Project 2

This node:
1. Subscribes to sensor topics (camera, LiDAR, IMU)
2. Collects calibration data samples
3. Compares measurements to ground truth
4. Calculates error metrics
5. Generates a calibration report
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo
import numpy as np
import math
from datetime import datetime
from typing import Dict, List, Optional


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

        # Thresholds for pass/fail
        self.thresholds = {
            'lidar_range_error': 0.05,  # meters
            'imu_gravity_error': 0.1,   # m/s^2
            'imu_accel_noise': 0.02,    # m/s^2
            'imu_gyro_noise': 0.002,    # rad/s
        }

        # Data storage for calibration samples
        self.camera_samples: List[Dict] = []
        self.lidar_samples: List[Dict] = []
        self.imu_samples: List[Dict] = []

        # Camera intrinsics
        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_fov: Optional[float] = None

        # Calibration state
        self.calibration_running = False
        self.samples_to_collect = 100
        self.warmup_complete = False
        self.start_time: Optional[float] = None

        # Sensor status
        self.camera_received = False
        self.lidar_received = False
        self.imu_received = False

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Create subscribers
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            sensor_qos
        )

        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            sensor_qos
        )

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Timer for calibration progress
        self.progress_timer = self.create_timer(1.0, self.calibration_progress)

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
            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'step': msg.step,
            }
            self.camera_samples.append(sample)

    def camera_info_callback(self, msg: CameraInfo):
        """Process camera intrinsic parameters."""
        if self.camera_matrix is None:
            # Extract camera matrix K (3x3)
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            fx = self.camera_matrix[0, 0]
            width = msg.width

            # Calculate horizontal FOV from focal length
            if fx > 0:
                self.camera_fov = 2 * math.atan(width / (2 * fx))
                self.get_logger().info(
                    f'Camera FOV: {math.degrees(self.camera_fov):.1f} degrees'
                )

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan for calibration."""
        if not self.lidar_received:
            self.get_logger().info(
                f'LiDAR: {len(msg.ranges)} samples, '
                f'range: [{msg.range_min:.2f}, {msg.range_max:.2f}]m'
            )
            self.lidar_received = True

        if self.calibration_running and len(self.lidar_samples) < self.samples_to_collect:
            # Convert to numpy for analysis
            ranges = np.array(msg.ranges)

            # Filter invalid readings
            valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
            valid_ranges = ranges[valid_mask]

            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'ranges': ranges,
                'valid_ranges': valid_ranges,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
            }
            self.lidar_samples.append(sample)

    def imu_callback(self, msg: Imu):
        """Process IMU data for calibration."""
        if not self.imu_received:
            self.get_logger().info('IMU: Receiving data')
            self.imu_received = True

        if self.calibration_running and len(self.imu_samples) < self.samples_to_collect:
            sample = {
                'timestamp': self.get_clock().now().nanoseconds,
                'linear_acceleration': np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]),
                'angular_velocity': np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ]),
                'orientation': np.array([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]),
            }
            self.imu_samples.append(sample)

    def calibration_progress(self):
        """Monitor calibration progress."""
        if not self.calibration_running:
            # Check if we can start calibration
            all_sensors = self.camera_received and self.lidar_received and self.imu_received
            if all_sensors and not self.warmup_complete:
                self.get_logger().info('All sensors detected. Warming up (2 seconds)...')
                self.warmup_complete = True
                self.create_timer(2.0, self.start_calibration)
            return

        # Report progress
        camera_count = len(self.camera_samples)
        lidar_count = len(self.lidar_samples)
        imu_count = len(self.imu_samples)

        self.get_logger().info(
            f'Collecting: Camera={camera_count}, '
            f'LiDAR={lidar_count}, IMU={imu_count} / {self.samples_to_collect}'
        )

        # Check if calibration is complete
        if (camera_count >= self.samples_to_collect and
            lidar_count >= self.samples_to_collect and
            imu_count >= self.samples_to_collect):

            self.calibration_running = False
            elapsed = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
            self.get_logger().info(f'Data collection complete in {elapsed:.1f} seconds')
            self.analyze_calibration_data()

    def start_calibration(self):
        """Start the calibration data collection."""
        if not self.calibration_running:
            self.get_logger().info(
                f'Starting calibration - collecting {self.samples_to_collect} samples...'
            )
            self.calibration_running = True
            self.start_time = self.get_clock().now().nanoseconds

    def analyze_lidar_accuracy(self) -> Dict:
        """Analyze LiDAR range accuracy."""
        if not self.lidar_samples:
            return {'status': 'FAIL', 'error': 'No LiDAR samples'}

        # Extract all range arrays
        all_ranges = [s['valid_ranges'] for s in self.lidar_samples]

        # Calculate statistics
        all_ranges_flat = np.concatenate([r for r in all_ranges if len(r) > 0])

        # Get front-facing range (index 180 for 360 samples = 0 degrees)
        front_ranges = []
        for sample in self.lidar_samples:
            ranges = sample['ranges']
            if len(ranges) >= 180:
                # Get range at 0 degrees (front)
                front_idx = len(ranges) // 2
                r = ranges[front_idx]
                if sample['range_min'] < r < sample['range_max']:
                    front_ranges.append(r)

        front_ranges = np.array(front_ranges)

        result = {
            'samples': len(self.lidar_samples),
            'total_readings': len(all_ranges_flat),
            'mean_range': float(np.mean(all_ranges_flat)) if len(all_ranges_flat) > 0 else 0,
            'std_range': float(np.std(all_ranges_flat)) if len(all_ranges_flat) > 0 else 0,
            'min_range': float(np.min(all_ranges_flat)) if len(all_ranges_flat) > 0 else 0,
            'max_range': float(np.max(all_ranges_flat)) if len(all_ranges_flat) > 0 else 0,
        }

        if len(front_ranges) > 0:
            result['front_mean'] = float(np.mean(front_ranges))
            result['front_std'] = float(np.std(front_ranges))

        # Determine pass/fail
        result['status'] = 'PASS' if result['std_range'] < self.thresholds['lidar_range_error'] else 'FAIL'

        return result

    def analyze_imu_accuracy(self) -> Dict:
        """Analyze IMU accuracy."""
        if not self.imu_samples:
            return {'status': 'FAIL', 'error': 'No IMU samples'}

        # Extract accelerometer data
        accel_data = np.array([s['linear_acceleration'] for s in self.imu_samples])
        gyro_data = np.array([s['angular_velocity'] for s in self.imu_samples])

        # Calculate gravity magnitude (should be ~9.81 when stationary)
        gravity_magnitudes = np.linalg.norm(accel_data, axis=1)
        mean_gravity = float(np.mean(gravity_magnitudes))
        gravity_error = abs(mean_gravity - self.ground_truth['gravity'])

        # Calculate noise (standard deviation)
        accel_noise = np.std(accel_data, axis=0)
        gyro_noise = np.std(gyro_data, axis=0)

        result = {
            'samples': len(self.imu_samples),
            'gravity_magnitude': mean_gravity,
            'gravity_expected': self.ground_truth['gravity'],
            'gravity_error': gravity_error,
            'accel_noise': accel_noise.tolist(),
            'accel_noise_mean': float(np.mean(accel_noise)),
            'gyro_noise': gyro_noise.tolist(),
            'gyro_noise_mean': float(np.mean(gyro_noise)),
        }

        # Calculate orientation stability
        orientations = np.array([s['orientation'] for s in self.imu_samples])
        orientation_std = np.std(orientations, axis=0)
        result['orientation_stability'] = float(np.mean(orientation_std))

        # Determine pass/fail
        gravity_ok = gravity_error < self.thresholds['imu_gravity_error']
        accel_ok = result['accel_noise_mean'] < self.thresholds['imu_accel_noise']
        gyro_ok = result['gyro_noise_mean'] < self.thresholds['imu_gyro_noise']

        result['status'] = 'PASS' if (gravity_ok and accel_ok and gyro_ok) else 'FAIL'
        result['gravity_status'] = 'PASS' if gravity_ok else 'FAIL'
        result['accel_status'] = 'PASS' if accel_ok else 'FAIL'
        result['gyro_status'] = 'PASS' if gyro_ok else 'FAIL'

        return result

    def analyze_camera_accuracy(self) -> Dict:
        """Analyze camera calibration data."""
        if not self.camera_samples:
            return {'status': 'FAIL', 'error': 'No camera samples'}

        # Check resolution consistency
        widths = [s['width'] for s in self.camera_samples]
        heights = [s['height'] for s in self.camera_samples]

        resolution_consistent = (len(set(widths)) == 1 and len(set(heights)) == 1)

        # Calculate frame timing
        timestamps = [s['timestamp'] for s in self.camera_samples]
        if len(timestamps) > 1:
            time_diffs = np.diff(timestamps) / 1e9  # Convert to seconds
            mean_interval = np.mean(time_diffs)
            actual_fps = 1.0 / mean_interval if mean_interval > 0 else 0
        else:
            actual_fps = 0

        result = {
            'samples': len(self.camera_samples),
            'resolution': f'{widths[0]}x{heights[0]}',
            'resolution_consistent': resolution_consistent,
            'actual_fps': actual_fps,
            'encoding': self.camera_samples[0]['encoding'],
        }

        if self.camera_fov is not None:
            result['horizontal_fov'] = math.degrees(self.camera_fov)

        result['status'] = 'PASS' if resolution_consistent else 'FAIL'

        return result

    def analyze_calibration_data(self):
        """Analyze all collected calibration data and generate report."""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('     ANALYZING CALIBRATION DATA')
        self.get_logger().info('=' * 50)

        # Analyze each sensor
        camera_results = self.analyze_camera_accuracy()
        lidar_results = self.analyze_lidar_accuracy()
        imu_results = self.analyze_imu_accuracy()

        # Generate and display report
        self.generate_report(camera_results, lidar_results, imu_results)

    def generate_report(self, camera_results: Dict, lidar_results: Dict, imu_results: Dict):
        """Generate calibration report."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        report_lines = [
            '',
            '=' * 50,
            '        SENSOR CALIBRATION REPORT',
            '=' * 50,
            f'Date: {timestamp}',
            f'Samples per sensor: {self.samples_to_collect}',
            '',
            '-' * 50,
            '  CAMERA CALIBRATION',
            '-' * 50,
            f'Status: {camera_results.get("status", "N/A")}',
            f'Resolution: {camera_results.get("resolution", "N/A")}',
            f'Encoding: {camera_results.get("encoding", "N/A")}',
            f'Actual FPS: {camera_results.get("actual_fps", 0):.1f}',
        ]

        if 'horizontal_fov' in camera_results:
            report_lines.append(f'Horizontal FOV: {camera_results["horizontal_fov"]:.1f} degrees')

        report_lines.extend([
            '',
            '-' * 50,
            '  LIDAR CALIBRATION',
            '-' * 50,
            f'Status: {lidar_results.get("status", "N/A")}',
            f'Samples: {lidar_results.get("samples", 0)}',
            f'Mean range: {lidar_results.get("mean_range", 0):.3f} m',
            f'Std range: {lidar_results.get("std_range", 0):.4f} m',
            f'Range coverage: [{lidar_results.get("min_range", 0):.2f}, '
            f'{lidar_results.get("max_range", 0):.2f}] m',
        ])

        if 'front_mean' in lidar_results:
            report_lines.extend([
                f'Front range mean: {lidar_results["front_mean"]:.3f} m',
                f'Front range std: {lidar_results["front_std"]:.4f} m',
            ])

        report_lines.extend([
            '',
            '-' * 50,
            '  IMU CALIBRATION',
            '-' * 50,
            f'Status: {imu_results.get("status", "N/A")}',
            f'Samples: {imu_results.get("samples", 0)}',
            f'Gravity magnitude: {imu_results.get("gravity_magnitude", 0):.3f} m/s^2',
            f'  Expected: {imu_results.get("gravity_expected", 9.81):.3f} m/s^2',
            f'  Error: {imu_results.get("gravity_error", 0):.4f} m/s^2 '
            f'({imu_results.get("gravity_status", "N/A")})',
            '',
            'Accelerometer noise (std):',
            f'  X: {imu_results.get("accel_noise", [0,0,0])[0]:.5f} m/s^2',
            f'  Y: {imu_results.get("accel_noise", [0,0,0])[1]:.5f} m/s^2',
            f'  Z: {imu_results.get("accel_noise", [0,0,0])[2]:.5f} m/s^2',
            f'  Mean: {imu_results.get("accel_noise_mean", 0):.5f} m/s^2 '
            f'({imu_results.get("accel_status", "N/A")})',
            '',
            'Gyroscope noise (std):',
            f'  X: {imu_results.get("gyro_noise", [0,0,0])[0]:.6f} rad/s',
            f'  Y: {imu_results.get("gyro_noise", [0,0,0])[1]:.6f} rad/s',
            f'  Z: {imu_results.get("gyro_noise", [0,0,0])[2]:.6f} rad/s',
            f'  Mean: {imu_results.get("gyro_noise_mean", 0):.6f} rad/s '
            f'({imu_results.get("gyro_status", "N/A")})',
            '',
            '-' * 50,
            '  OVERALL RESULTS',
            '-' * 50,
        ])

        # Overall status
        camera_pass = camera_results.get('status') == 'PASS'
        lidar_pass = lidar_results.get('status') == 'PASS'
        imu_pass = imu_results.get('status') == 'PASS'

        report_lines.extend([
            f'Camera:  {"PASS" if camera_pass else "FAIL"}',
            f'LiDAR:   {"PASS" if lidar_pass else "FAIL"}',
            f'IMU:     {"PASS" if imu_pass else "FAIL"}',
            '',
            f'Overall: {"ALL TESTS PASSED" if all([camera_pass, lidar_pass, imu_pass]) else "SOME TESTS FAILED"}',
            '=' * 50,
        ])

        report = '\n'.join(report_lines)
        self.get_logger().info(report)

        # Save report to file
        try:
            with open('calibration_report.txt', 'w') as f:
                f.write(report)
            self.get_logger().info('Report saved to: calibration_report.txt')
        except Exception as e:
            self.get_logger().warn(f'Could not save report: {e}')


def main(args=None):
    rclpy.init(args=args)

    calibrator = SensorCalibrator()

    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        calibrator.get_logger().info('Calibration interrupted by user')
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
