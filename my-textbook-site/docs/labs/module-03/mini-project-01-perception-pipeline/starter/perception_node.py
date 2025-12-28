#!/usr/bin/env python3
"""
Mini-Project 1: Perception Pipeline - Starter Code
Main pipeline node that orchestrates detection, fusion, and tracking.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2


class PerceptionPipeline(Node):
    """Complete perception pipeline integrating detection, fusion, and tracking."""

    def __init__(self):
        super().__init__('perception_pipeline')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('sync_tolerance', 0.1)
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value

        # TODO 1: Initialize detector
        # Load YOLO model or other detector
        self.detector = None

        # TODO 2: Initialize tracker
        # Create multi-object tracker
        self.tracker = None

        # TODO 3: Set up synchronized subscribers
        # Subscribe to /camera/image_raw and /lidar/points
        # Use ApproximateTimeSynchronizer

        # TODO 4: Create publisher for perception output
        # Publish DetectedObject3D messages or similar

        self.get_logger().info('Perception pipeline initialized')

    def synchronized_callback(self, image_msg, lidar_msg):
        """Process synchronized camera and LiDAR data."""

        # Convert inputs
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        points = self.pointcloud_to_numpy(lidar_msg)

        # TODO 5: Run object detection
        detections = []  # List of {bbox, class, confidence}

        # TODO 6: Perform 3D localization
        # For each detection, find corresponding LiDAR points
        # Calculate 3D centroid
        objects_3d = []

        # TODO 7: Update tracker
        # Associate detections with existing tracks
        tracked_objects = []

        # TODO 8: Publish results
        self.publish_objects(tracked_objects, image_msg.header)

        # Log status
        self.get_logger().info(f'Processed frame: {len(tracked_objects)} objects tracked')

    def pointcloud_to_numpy(self, msg):
        """Convert PointCloud2 to numpy array."""
        points = []
        for p in pc2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points) if points else np.array([]).reshape(0, 3)

    def detect_objects(self, image):
        """Run object detection on image."""
        # TODO: Implement detection logic
        pass

    def localize_3d(self, detection, points, camera_matrix):
        """Find 3D position of detected object using LiDAR."""
        # TODO: Implement 3D localization
        pass

    def publish_objects(self, objects, header):
        """Publish detected objects."""
        # TODO: Create and publish message
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
