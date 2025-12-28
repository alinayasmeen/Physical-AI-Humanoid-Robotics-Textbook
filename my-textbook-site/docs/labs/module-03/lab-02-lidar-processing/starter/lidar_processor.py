#!/usr/bin/env python3
"""
Lab 2: LiDAR Point Cloud Processing - Starter Code
Module 3: Perception & Sensors

Complete the TODOs to process point cloud data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class LidarProcessor(Node):
    """ROS 2 node that processes LiDAR point clouds."""

    def __init__(self):
        super().__init__('lidar_processor')

        # Parameters for filtering
        self.ground_height = -0.3  # meters below sensor
        self.min_distance = 0.5    # meters
        self.max_distance = 10.0   # meters

        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10)

        self.get_logger().info('LiDAR processor started')
        self.frame_count = 0

    def lidar_callback(self, msg):
        """Process incoming point cloud data."""
        self.frame_count += 1

        # TODO 1: Convert PointCloud2 to numpy array
        # Hint: Use pc2.read_points() and iterate through points
        # Each point has (x, y, z) coordinates
        points = None  # Replace this

        if points is None or len(points) == 0:
            self.get_logger().warn('TODO 1: Implement point cloud conversion')
            return

        original_count = len(points)

        # TODO 2: Remove ground plane points
        # Keep only points where z > self.ground_height
        filtered_points = None  # Replace this

        if filtered_points is None:
            self.get_logger().warn('TODO 2: Implement ground removal')
            return

        after_ground_count = len(filtered_points)

        # TODO 3: Filter by distance
        # Keep points where distance from origin is between
        # self.min_distance and self.max_distance
        # Distance = sqrt(x^2 + y^2)
        final_points = None  # Replace this

        if final_points is None:
            self.get_logger().warn('TODO 3: Implement distance filtering')
            return

        final_count = len(final_points)

        # TODO 4: Compute statistics on final_points
        # - Minimum distance to any point
        # - Maximum height (z value)
        # - Centroid (mean x, y, z)
        stats = None  # Replace this

        if stats is None:
            self.get_logger().warn('TODO 4: Implement statistics')
            return

        # Log results every 10 frames
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'{original_count} -> {after_ground_count} -> {final_count} points, '
                f"closest: {stats['min_distance']:.2f}m"
            )

    def remove_ground(self, points):
        """Remove points below ground height threshold."""
        # Your implementation here
        pass

    def filter_by_distance(self, points):
        """Keep points within min/max distance range."""
        # Your implementation here
        pass

    def compute_statistics(self, points):
        """Compute statistics on point cloud."""
        # Your implementation here
        pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = LidarProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
