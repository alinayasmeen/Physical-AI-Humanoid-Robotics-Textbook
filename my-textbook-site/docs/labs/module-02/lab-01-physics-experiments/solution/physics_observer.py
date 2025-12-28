#!/usr/bin/env python3
"""
Physics Observer - Module 2 Lab 1 Solution

Complete implementation of physics observation and analysis.

Usage:
    python3 physics_observer.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time


class PhysicsObserver(Node):
    """Observes physics behavior by tracking object pose and velocity."""

    def __init__(self):
        super().__init__('physics_observer')

        # Previous pose for velocity calculation
        self.prev_pose = None
        self.prev_time = None

        # Data storage
        self.observations = []
        self.starting_height = 5.0  # meters

        # Subscriber to object's pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/model/falling_cube/pose',
            self.pose_callback,
            10
        )

        # Timer to periodically report observations (every 0.5 seconds)
        self.report_timer = self.create_timer(0.5, self.report_callback)

        self.get_logger().info('Physics Observer started')
        self.get_logger().info('Subscribed to /model/falling_cube/pose')

    def pose_callback(self, msg):
        """Process incoming pose messages and calculate velocity."""
        current_time = self.get_clock().now()
        current_pose = msg.pose

        if self.prev_pose is not None and self.prev_time is not None:
            # Calculate time delta
            dt = (current_time - self.prev_time).nanoseconds / 1e9

            if dt > 0.001:  # Avoid division by very small dt
                # Calculate vertical velocity (z-axis)
                dz = current_pose.position.z - self.prev_pose.position.z
                vertical_velocity = dz / dt

                # Calculate fall distance from starting height
                fall_distance = self.starting_height - current_pose.position.z

                # Store observation
                observation = {
                    'time': current_time.nanoseconds / 1e9,
                    'height': current_pose.position.z,
                    'velocity': vertical_velocity,
                    'fall_distance': fall_distance,
                    'dt': dt
                }
                self.observations.append(observation)

        # Update previous values
        self.prev_pose = current_pose
        self.prev_time = current_time

    def report_callback(self):
        """Periodically report physics observations."""
        if len(self.observations) == 0:
            self.get_logger().info('Waiting for pose data...')
            return

        # Get latest observation
        latest = self.observations[-1]

        # Log observation data
        self.get_logger().info(
            f'Height: {latest["height"]:.3f} m | '
            f'Velocity: {latest["velocity"]:.3f} m/s | '
            f'Fall: {latest["fall_distance"]:.3f} m'
        )

    def analyze_physics(self):
        """Analyze collected observations and determine physics parameters."""
        if len(self.observations) < 10:
            self.get_logger().warn(
                f'Not enough observations ({len(self.observations)}) for analysis'
            )
            return None

        # Calculate accelerations from consecutive velocity measurements
        accelerations = []
        for i in range(1, len(self.observations)):
            prev = self.observations[i - 1]
            curr = self.observations[i]
            dt = curr['dt']

            if dt > 0.001:
                dv = curr['velocity'] - prev['velocity']
                acceleration = dv / dt
                accelerations.append(acceleration)

        if len(accelerations) == 0:
            return None

        # Estimate gravity (should be close to -9.8 m/s²)
        estimated_gravity = sum(accelerations) / len(accelerations)

        # Calculate average velocity
        velocities = [obs['velocity'] for obs in self.observations]
        average_velocity = sum(velocities) / len(velocities)

        # Calculate total fall time
        total_fall_time = (
            self.observations[-1]['time'] - self.observations[0]['time']
        )

        # Calculate theoretical gravity using kinematic equation
        # v² = v₀² + 2*a*d  =>  a = (v² - v₀²) / (2*d)
        if len(self.observations) > 2:
            v_initial = self.observations[0]['velocity']
            v_final = self.observations[-1]['velocity']
            distance = self.observations[-1]['fall_distance']

            if distance > 0.1:
                kinematic_gravity = (v_final**2 - v_initial**2) / (2 * distance)
            else:
                kinematic_gravity = estimated_gravity
        else:
            kinematic_gravity = estimated_gravity

        return {
            'estimated_gravity': estimated_gravity,
            'kinematic_gravity': kinematic_gravity,
            'average_velocity': average_velocity,
            'total_fall_time': total_fall_time,
            'num_samples': len(self.observations)
        }


def main():
    """Main entry point."""
    rclpy.init()

    observer = PhysicsObserver()

    try:
        print("=" * 50)
        print("Physics Observer - Module 2 Lab 1")
        print("=" * 50)
        print("\nCollecting physics observations for 10 seconds...")
        print("Make sure Gazebo is running with a falling object!\n")

        # Spin for a while to collect observations
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(observer, timeout_sec=0.1)

        # Analyze collected data
        print("\n" + "=" * 50)
        print("Physics Analysis Results")
        print("=" * 50)

        results = observer.analyze_physics()

        if results:
            print(f"Number of samples: {results['num_samples']}")
            print(f"Total observation time: {results['total_fall_time']:.2f} s")
            print(f"Average velocity: {results['average_velocity']:.2f} m/s")
            print(f"Estimated gravity (acceleration): {results['estimated_gravity']:.2f} m/s²")
            print(f"Kinematic gravity estimate: {results['kinematic_gravity']:.2f} m/s²")
            print(f"Expected Earth gravity: -9.81 m/s²")

            # Compare with expected
            error = abs(results['estimated_gravity'] - (-9.81))
            error_percent = (error / 9.81) * 100
            print(f"\nMeasurement error: {error:.2f} m/s² ({error_percent:.1f}%)")

            if error_percent < 10:
                print("Result: GOOD - Gravity measurement within 10% of expected")
            else:
                print("Result: CHECK - Large deviation from expected gravity")
                print("  - Verify simulation is running correctly")
                print("  - Check that gravity is set to -9.8 in world file")
        else:
            print("Analysis incomplete.")
            print("Troubleshooting:")
            print("  - Check that Gazebo simulation is running")
            print("  - Verify the falling_cube model exists")
            print("  - Check ros_gz bridge is active")

    except KeyboardInterrupt:
        print("\nObservation stopped by user")
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
