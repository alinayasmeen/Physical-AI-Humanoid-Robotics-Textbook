#!/usr/bin/env python3
"""
Lab 3: Multi-Sensor Fusion - Starter Code
Module 3: Perception & Sensors

Complete the TODOs to implement camera + LiDAR fusion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2


class SensorFusion(Node):
    """Fuses camera and LiDAR data."""

    def __init__(self):
        super().__init__('sensor_fusion')

        self.bridge = CvBridge()

        # Camera intrinsic matrix (example values - adjust for your camera)
        # [fx  0  cx]
        # [ 0 fy  cy]
        # [ 0  0   1]
        self.camera_matrix = np.array([
            [500.0,   0.0, 320.0],
            [  0.0, 500.0, 240.0],
            [  0.0,   0.0,   1.0]
        ])

        # TODO 1: Create synchronized subscribers
        # Hint: Use message_filters.Subscriber for Image and PointCloud2
        # Then create ApproximateTimeSynchronizer

        # Your code here

        self.get_logger().info('Sensor fusion node started')
        self.frame_count = 0

    def sync_callback(self, image_msg, lidar_msg):
        """Called when synchronized messages arrive."""
        self.frame_count += 1

        # TODO 2: Convert image and point cloud
        cv_image = None  # Convert image_msg to OpenCV
        points = None    # Convert lidar_msg to numpy

        if cv_image is None or points is None:
            self.get_logger().warn('TODO 2: Implement data conversion')
            return

        # TODO 3: Project LiDAR points to image coordinates
        # Transform points from LiDAR frame to camera frame (simplified)
        # Then project using camera matrix
        u, v, depths = None, None, None  # Replace this

        if u is None:
            self.get_logger().warn('TODO 3: Implement point projection')
            return

        # TODO 4: Overlay points on image
        fused_image = None  # Replace this

        if fused_image is None:
            self.get_logger().warn('TODO 4: Implement image overlay')
            return

        # Log every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'Fused {len(points)} points with image'
            )

    def pointcloud_to_numpy(self, msg):
        """Convert PointCloud2 to numpy array."""
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list)

    def project_points_to_image(self, points_3d, camera_matrix):
        """Project 3D points to 2D image coordinates."""
        # Filter points in front of camera
        # Project using K @ [x, y, z]
        # Normalize by z
        pass  # Your implementation here

    def overlay_points_on_image(self, image, u, v, depths):
        """Draw LiDAR points on image with depth coloring."""
        # For each valid point, draw colored circle
        # Color based on depth (red=close, blue=far)
        pass  # Your implementation here


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
