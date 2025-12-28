#!/usr/bin/env python3
"""
Physics Observer - Module 2 Lab 1 Starter Code

This script observes and analyzes physics behavior in Gazebo simulation.
Complete the TODOs to implement the physics observation functionality.

Prerequisites:
- ROS 2 Humble
- Gazebo Fortress with ros_gz bridge
- Running simulation with a falling object

Usage:
    python3 physics_observer.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
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

        # TODO 1: Create a subscriber to the object's pose topic
        # Hint: The topic is typically '/model/falling_cube/pose'
        # Message type: geometry_msgs/msg/PoseStamped
        # self.pose_sub = self.create_subscription(...)

        # TODO 2: Create a timer to periodically report observations
        # Hint: Use self.create_timer(period, callback)
        # self.report_timer = self.create_timer(...)

        self.get_logger().info('Physics Observer started')
        self.get_logger().info('Waiting for pose data...')

    def pose_callback(self, msg):
        """Process incoming pose messages and calculate velocity."""
        current_time = self.get_clock().now()
        current_pose = msg.pose

        if self.prev_pose is not None and self.prev_time is not None:
            # Calculate time delta
            dt = (current_time - self.prev_time).nanoseconds / 1e9

            if dt > 0:
                # TODO 3: Calculate vertical velocity (z-axis)
                # Hint: velocity = (current_z - previous_z) / dt
                # vertical_velocity = ...

                # TODO 4: Calculate fall distance from starting height
                # Assume starting height was 5.0 meters
                # fall_distance = ...

                # TODO 5: Store observation
                # observation = {
                #     'time': current_time.nanoseconds / 1e9,
                #     'height': current_pose.position.z,
                #     'velocity': vertical_velocity,
                #     'fall_distance': fall_distance
                # }
                # self.observations.append(observation)
                pass

        # Update previous values
        self.prev_pose = current_pose
        self.prev_time = current_time

    def report_callback(self):
        """Periodically report physics observations."""
        if len(self.observations) == 0:
            self.get_logger().info('No observations yet...')
            return

        # Get latest observation
        latest = self.observations[-1]

        # TODO 6: Log the observation data
        # Use self.get_logger().info() to print:
        # - Current height
        # - Vertical velocity
        # - Fall distance
        # self.get_logger().info(f'Height: {latest["height"]:.3f} m')
        # ...

    def analyze_physics(self):
        """Analyze collected observations and determine physics parameters."""
        if len(self.observations) < 10:
            self.get_logger().warn('Not enough observations for analysis')
            return None

        # TODO 7: Calculate average acceleration
        # Hint: Use consecutive velocity measurements
        # a = (v2 - v1) / dt
        # Compare with expected gravity (-9.8 m/s²)

        # TODO 8: Estimate gravity from observations
        # Use kinematic equation: v² = v₀² + 2*a*d
        # Solve for a given measured velocities and distances

        # TODO 9: Return analysis results
        # return {
        #     'estimated_gravity': ...,
        #     'average_velocity': ...,
        #     'total_fall_time': ...
        # }
        pass


def main():
    """Main entry point."""
    rclpy.init()

    observer = PhysicsObserver()

    try:
        # Spin for a while to collect observations
        print("Collecting physics observations for 10 seconds...")
        print("Make sure Gazebo is running with a falling object!")

        # Spin with timeout
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(observer, timeout_sec=0.1)

        # Analyze collected data
        print("\n--- Physics Analysis ---")
        results = observer.analyze_physics()

        if results:
            print(f"Estimated gravity: {results['estimated_gravity']:.2f} m/s²")
            print(f"Average velocity: {results['average_velocity']:.2f} m/s")
            print(f"Total fall time: {results['total_fall_time']:.2f} s")
        else:
            print("Analysis incomplete. Check that simulation is running.")

    except KeyboardInterrupt:
        print("\nObservation stopped by user")
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
