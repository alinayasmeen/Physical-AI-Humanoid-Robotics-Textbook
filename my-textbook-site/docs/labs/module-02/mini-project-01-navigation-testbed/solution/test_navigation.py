#!/usr/bin/env python3
"""
Navigation Testbed Test Script - Complete Solution
Module 2 Mini-Project 1

This script tests the navigation testbed by:
1. Verifying robot sensors are publishing
2. Sending the robot to waypoints
3. Reporting navigation progress
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
import math
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

        # Current robot state
        self.current_x = -8.0  # Start at spawn position
        self.current_y = 0.0
        self.current_yaw = 0.0

        # LiDAR data for obstacle detection
        self.min_range = float('inf')
        self.obstacle_detected = False

        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.waypoint_tolerance = 0.5  # meters
        self.obstacle_threshold = 0.5  # meters

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber for LiDAR data
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            sensor_qos
        )

        # Subscriber for camera data
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            sensor_qos
        )

        # Subscriber for IMU data
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Subscriber for odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer for control loop (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Navigation Tester initialized')
        self.get_logger().info(f'Waypoints: {self.waypoints}')

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan data."""
        if not self.lidar_received:
            self.get_logger().info(
                f'LiDAR: Receiving {len(msg.ranges)} ranges, '
                f'angle range: [{math.degrees(msg.angle_min):.1f}, '
                f'{math.degrees(msg.angle_max):.1f}] degrees'
            )
            self.lidar_received = True

        # Find minimum range (excluding invalid readings)
        valid_ranges = [r for r in msg.ranges
                        if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_range = min(valid_ranges)
            self.obstacle_detected = self.min_range < self.obstacle_threshold

    def camera_callback(self, msg: Image):
        """Process camera image data."""
        if not self.camera_received:
            self.get_logger().info(
                f'Camera: {msg.width}x{msg.height} {msg.encoding} image'
            )
            self.camera_received = True

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        if not self.imu_received:
            self.get_logger().info(
                f'IMU: Receiving orientation and acceleration data'
            )
            self.imu_received = True

        # Extract yaw from quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        """Process odometry data."""
        if not self.odom_received:
            self.get_logger().info('Odometry: Receiving position updates')
            self.odom_received = True

        # Update current robot position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from odometry quaternion as backup
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def check_sensors(self) -> bool:
        """Verify all required sensors are publishing."""
        # Minimum required: LiDAR and either IMU or Odometry
        sensors_ok = self.lidar_received and (self.imu_received or self.odom_received)

        if not sensors_ok:
            missing = []
            if not self.lidar_received:
                missing.append('LiDAR')
            if not self.camera_received:
                missing.append('Camera (optional)')
            if not self.imu_received and not self.odom_received:
                missing.append('IMU/Odometry')

            if missing:
                self.get_logger().warn(
                    f'Waiting for sensors: {", ".join(missing)}',
                    throttle_duration_sec=2.0
                )

        return sensors_ok

    def distance_to_waypoint(self, waypoint_idx: int) -> float:
        """Calculate Euclidean distance to a waypoint."""
        if waypoint_idx >= len(self.waypoints):
            return float('inf')

        wx, wy = self.waypoints[waypoint_idx]
        dx = wx - self.current_x
        dy = wy - self.current_y

        return math.sqrt(dx**2 + dy**2)

    def angle_to_waypoint(self, waypoint_idx: int) -> float:
        """Calculate angle to waypoint relative to robot heading."""
        if waypoint_idx >= len(self.waypoints):
            return 0.0

        wx, wy = self.waypoints[waypoint_idx]
        dx = wx - self.current_x
        dy = wy - self.current_y

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_yaw

        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff

    def navigate_to_waypoint(self, waypoint_idx: int):
        """
        Generate velocity commands to navigate to waypoint.

        Uses simple proportional control with obstacle avoidance.
        """
        if waypoint_idx >= len(self.waypoints):
            return

        cmd = Twist()

        # Check for obstacles
        if self.obstacle_detected:
            self.get_logger().warn(
                f'Obstacle detected at {self.min_range:.2f}m - stopping',
                throttle_duration_sec=1.0
            )
            # Simple avoidance: turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(cmd)
            return

        # Calculate angle to waypoint
        angle_to_goal = self.angle_to_waypoint(waypoint_idx)
        distance = self.distance_to_waypoint(waypoint_idx)

        # Proportional control
        if abs(angle_to_goal) > 0.1:  # Need to turn
            cmd.linear.x = 0.1  # Slow forward while turning
            cmd.angular.z = self.angular_speed * (angle_to_goal / abs(angle_to_goal))
            # Scale angular velocity by angle magnitude
            cmd.angular.z *= min(1.0, abs(angle_to_goal) / 0.5)
        else:
            # Heading is good, move forward
            cmd.linear.x = min(self.linear_speed, distance * 0.5)
            cmd.angular.z = angle_to_goal * 0.5  # Minor correction

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Send stop command to robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        """Main control loop - runs at 10Hz."""

        # First, check if sensors are ready
        if not self.check_sensors():
            return

        # Check if we've reached current waypoint
        distance = self.distance_to_waypoint(self.current_waypoint_idx)

        if distance < self.waypoint_tolerance:
            waypoint_names = ['Start', 'Waypoint 1', 'Waypoint 2', 'Goal']
            waypoint_name = waypoint_names[min(self.current_waypoint_idx, 3)]

            self.get_logger().info(
                f'Reached {waypoint_name}! '
                f'Position: ({self.current_x:.2f}, {self.current_y:.2f})'
            )

            self.current_waypoint_idx += 1

            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info(
                    '=== Navigation Complete! All waypoints reached. ==='
                )
                self.stop_robot()
                return

            # Brief pause between waypoints
            self.stop_robot()
            return

        # Log progress periodically
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 2.0:
                self.get_logger().info(
                    f'Navigating to waypoint {self.current_waypoint_idx}: '
                    f'distance={distance:.2f}m, '
                    f'pos=({self.current_x:.2f}, {self.current_y:.2f})'
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()

        # Navigate to current waypoint
        self.navigate_to_waypoint(self.current_waypoint_idx)

    def run_sensor_check(self, timeout: float = 10.0) -> bool:
        """Run a sensor check and report results."""
        self.get_logger().info(f'Starting sensor check ({timeout} seconds)...')

        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Early exit if all sensors found
            if self.lidar_received and self.imu_received:
                break

        # Report results
        self.get_logger().info('=== Sensor Check Results ===')
        self.get_logger().info(
            f'LiDAR:    {"OK" if self.lidar_received else "NOT RECEIVED"}'
        )
        self.get_logger().info(
            f'Camera:   {"OK" if self.camera_received else "NOT RECEIVED (optional)"}'
        )
        self.get_logger().info(
            f'IMU:      {"OK" if self.imu_received else "NOT RECEIVED"}'
        )
        self.get_logger().info(
            f'Odometry: {"OK" if self.odom_received else "NOT RECEIVED"}'
        )

        # Minimum required: LiDAR and position feedback
        all_ok = self.lidar_received and (self.imu_received or self.odom_received)
        return all_ok


def main(args=None):
    rclpy.init(args=args)

    tester = NavigationTester()

    # Run sensor check first
    if not tester.run_sensor_check():
        tester.get_logger().error(
            'Sensor check failed. Please verify robot configuration.'
        )
        tester.destroy_node()
        rclpy.shutdown()
        return

    tester.get_logger().info('Sensor check passed. Starting navigation test...')
    tester.get_logger().info(f'Starting at waypoint index {tester.current_waypoint_idx}')

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Navigation test interrupted by user')
    finally:
        tester.stop_robot()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
