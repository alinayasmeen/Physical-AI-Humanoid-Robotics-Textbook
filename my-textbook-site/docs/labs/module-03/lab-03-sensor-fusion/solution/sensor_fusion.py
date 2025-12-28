#!/usr/bin/env python3
"""
Lab 3: Multi-Sensor Fusion - Solution
Module 3: Perception & Sensors

Complete implementation of camera + LiDAR fusion.
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

        # Camera intrinsic matrix (example values)
        self.camera_matrix = np.array([
            [500.0,   0.0, 320.0],
            [  0.0, 500.0, 240.0],
            [  0.0,   0.0,   1.0]
        ])

        # SOLUTION 1: Create synchronized subscribers
        self.image_sub = Subscriber(self, Image, '/camera/image_raw')
        self.lidar_sub = Subscriber(self, PointCloud2, '/lidar/points')

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=10,
            slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        # Publisher for fused image
        self.fused_pub = self.create_publisher(Image, '/fusion/image', 10)

        self.get_logger().info('Sensor fusion node started')
        self.frame_count = 0

    def sync_callback(self, image_msg, lidar_msg):
        """Called when synchronized messages arrive."""
        self.frame_count += 1

        # SOLUTION 2: Convert image and point cloud
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        points = self.pointcloud_to_numpy(lidar_msg)

        if len(points) == 0:
            return

        # SOLUTION 3: Project LiDAR points to image
        u, v, depths = self.project_points_to_image(points, self.camera_matrix)

        # SOLUTION 4: Overlay points on image
        fused_image = self.overlay_points_on_image(
            cv_image.copy(), u, v, depths)

        # Publish fused image
        fused_msg = self.bridge.cv2_to_imgmsg(fused_image, 'bgr8')
        fused_msg.header = image_msg.header
        self.fused_pub.publish(fused_msg)

        # Log every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'Fused {len(points)} points with image, '
                f'{len(u)} points visible'
            )

    def pointcloud_to_numpy(self, msg):
        """Convert PointCloud2 to numpy array."""
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list) if points_list else np.array([])

    def project_points_to_image(self, points_3d, camera_matrix):
        """Project 3D points to 2D image coordinates."""
        if len(points_3d) == 0:
            return np.array([]), np.array([]), np.array([])

        # Filter points in front of camera (z > 0)
        mask = points_3d[:, 2] > 0.1
        valid_points = points_3d[mask]

        if len(valid_points) == 0:
            return np.array([]), np.array([]), np.array([])

        # Project: [u*w, v*w, w] = K @ [x, y, z]
        # Assuming LiDAR frame aligned with camera (simplified)
        # In practice, you'd apply extrinsic transform first
        projected = camera_matrix @ valid_points.T
        projected = projected.T

        # Normalize by depth
        u = projected[:, 0] / projected[:, 2]
        v = projected[:, 1] / projected[:, 2]
        depths = valid_points[:, 2]

        return u, v, depths

    def overlay_points_on_image(self, image, u, v, depths):
        """Draw LiDAR points on image with depth coloring."""
        h, w = image.shape[:2]
        max_depth = 10.0

        for i in range(len(u)):
            x, y = int(u[i]), int(v[i])
            if 0 <= x < w and 0 <= y < h:
                # Color by depth: red=close, blue=far
                depth_normalized = min(depths[i], max_depth) / max_depth
                r = int(255 * (1 - depth_normalized))
                b = int(255 * depth_normalized)
                color = (b, 0, r)  # BGR format
                cv2.circle(image, (x, y), 2, color, -1)

        return image


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
