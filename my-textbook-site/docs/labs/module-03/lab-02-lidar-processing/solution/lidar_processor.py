#!/usr/bin/env python3
"""
Lab 2: LiDAR Point Cloud Processing - Solution
Module 3: Perception & Sensors

Complete implementation of point cloud processing.
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

        # SOLUTION 1: Convert PointCloud2 to numpy array
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        points = np.array(points_list)

        if len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return

        original_count = len(points)

        # SOLUTION 2: Remove ground plane points
        filtered_points = self.remove_ground(points)
        after_ground_count = len(filtered_points)

        # SOLUTION 3: Filter by distance
        final_points = self.filter_by_distance(filtered_points)
        final_count = len(final_points)

        # SOLUTION 4: Compute statistics
        if len(final_points) > 0:
            stats = self.compute_statistics(final_points)
        else:
            stats = {'min_distance': 0, 'max_height': 0, 'centroid': [0, 0, 0]}

        # Log results every 10 frames
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'{original_count} -> {after_ground_count} -> {final_count} points, '
                f"closest: {stats['min_distance']:.2f}m, "
                f"max_height: {stats['max_height']:.2f}m"
            )

    def remove_ground(self, points):
        """Remove points below ground height threshold."""
        mask = points[:, 2] > self.ground_height
        return points[mask]

    def filter_by_distance(self, points):
        """Keep points within min/max distance range."""
        if len(points) == 0:
            return points
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask = (distances > self.min_distance) & (distances < self.max_distance)
        return points[mask]

    def compute_statistics(self, points):
        """Compute statistics on point cloud."""
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        return {
            'count': len(points),
            'min_distance': float(np.min(distances)),
            'max_height': float(np.max(points[:, 2])),
            'centroid': np.mean(points, axis=0).tolist()
        }


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
