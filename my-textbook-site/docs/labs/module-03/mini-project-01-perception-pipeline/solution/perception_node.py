#!/usr/bin/env python3
"""
Mini-Project 1: Perception Pipeline - Solution
Complete implementation of the perception pipeline.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from collections import defaultdict
import time


class SimpleTracker:
    """Multi-object tracker using centroid distance."""

    def __init__(self, max_distance=2.0, max_lost=10):
        self.tracks = {}  # track_id -> {position, class_id, lost_frames}
        self.next_id = 0
        self.max_distance = max_distance
        self.max_lost = max_lost

    def update(self, detections_3d):
        """Update tracks with new detections."""
        # Increment lost frames for all tracks
        for tid in self.tracks:
            self.tracks[tid]['lost_frames'] += 1

        matched_tracks = []
        unmatched_dets = list(range(len(detections_3d)))

        # Match detections to tracks
        for tid, track in list(self.tracks.items()):
            best_match = None
            best_dist = self.max_distance

            for i in unmatched_dets:
                det = detections_3d[i]
                dist = np.linalg.norm(
                    np.array(track['position']) - np.array(det['position']))
                if dist < best_dist:
                    best_dist = dist
                    best_match = i

            if best_match is not None:
                det = detections_3d[best_match]
                self.tracks[tid]['position'] = det['position']
                self.tracks[tid]['class_id'] = det['class_id']
                self.tracks[tid]['confidence'] = det['confidence']
                self.tracks[tid]['lost_frames'] = 0
                matched_tracks.append((tid, det))
                unmatched_dets.remove(best_match)

        # Create new tracks for unmatched detections
        for i in unmatched_dets:
            det = detections_3d[i]
            self.tracks[self.next_id] = {
                'position': det['position'],
                'class_id': det['class_id'],
                'confidence': det['confidence'],
                'lost_frames': 0
            }
            matched_tracks.append((self.next_id, det))
            self.next_id += 1

        # Remove lost tracks
        lost_ids = [tid for tid, t in self.tracks.items()
                   if t['lost_frames'] > self.max_lost]
        for tid in lost_ids:
            del self.tracks[tid]

        # Return tracked objects with IDs
        result = []
        for tid, det in matched_tracks:
            result.append({
                'track_id': tid,
                'position': det['position'],
                'class_id': det['class_id'],
                'confidence': det['confidence']
            })
        return result


class PerceptionPipeline(Node):
    """Complete perception pipeline."""

    def __init__(self):
        super().__init__('perception_pipeline')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('sync_tolerance', 0.1)
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value

        # Camera intrinsics (default values)
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # SOLUTION 1: Initialize detector
        self.net = None
        self.classes = []
        self.load_detector()

        # SOLUTION 2: Initialize tracker
        self.tracker = SimpleTracker(max_distance=2.0, max_lost=10)

        # SOLUTION 3: Set up synchronized subscribers
        self.image_sub = Subscriber(self, Image, '/camera/image_raw')
        self.lidar_sub = Subscriber(self, PointCloud2, '/lidar/points')

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=10,
            slop=self.sync_tolerance)
        self.sync.registerCallback(self.synchronized_callback)

        # SOLUTION 4: Create publisher
        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/markers', 10)

        # Performance tracking
        self.frame_count = 0
        self.total_time = 0

        self.get_logger().info('Perception pipeline initialized')

    def load_detector(self):
        """Load YOLO detector."""
        try:
            # Simplified: use color-based detection for demo
            self.classes = ['object']
            self.get_logger().info('Using simplified detector')
        except Exception as e:
            self.get_logger().warn(f'Could not load detector: {e}')

    def synchronized_callback(self, image_msg, lidar_msg):
        """Process synchronized data."""
        start_time = time.time()

        # Convert inputs
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        points = self.pointcloud_to_numpy(lidar_msg)

        if len(points) == 0:
            return

        # SOLUTION 5: Run object detection
        detections = self.detect_objects(cv_image)

        # SOLUTION 6: Perform 3D localization
        objects_3d = []
        for det in detections:
            position = self.localize_3d(det, points)
            if position is not None:
                objects_3d.append({
                    'position': position,
                    'class_id': det['class'],
                    'confidence': det['confidence'],
                    'bbox': det['bbox']
                })

        # SOLUTION 7: Update tracker
        tracked_objects = self.tracker.update(objects_3d)

        # SOLUTION 8: Publish results
        self.publish_markers(tracked_objects, image_msg.header)

        # Performance tracking
        elapsed = time.time() - start_time
        self.frame_count += 1
        self.total_time += elapsed

        if self.frame_count % 30 == 0:
            avg_fps = self.frame_count / self.total_time
            self.get_logger().info(
                f'Frame {self.frame_count}: {len(tracked_objects)} objects, '
                f'FPS: {avg_fps:.1f}')

    def pointcloud_to_numpy(self, msg):
        """Convert PointCloud2 to numpy array."""
        points = []
        for p in pc2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points) if points else np.array([]).reshape(0, 3)

    def detect_objects(self, image):
        """Simple color-based detection for demo."""
        detections = []

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'bbox': (x, y, w, h),
                    'class': 'object',
                    'confidence': 0.8
                })

        return detections

    def localize_3d(self, detection, points):
        """Find 3D position using LiDAR points."""
        if len(points) == 0:
            return None

        bbox = detection['bbox']
        x, y, w, h = bbox

        # Filter points in front of camera
        valid_mask = points[:, 2] > 0.1
        valid_points = points[valid_mask]

        if len(valid_points) == 0:
            return None

        # Project points to image
        projected = self.camera_matrix @ valid_points.T
        u = projected[0] / projected[2]
        v = projected[1] / projected[2]

        # Find points within bounding box
        in_bbox = (
            (u >= x) & (u <= x + w) &
            (v >= y) & (v <= y + h)
        )

        bbox_points = valid_points[in_bbox]

        if len(bbox_points) == 0:
            return None

        # Calculate centroid
        centroid = np.mean(bbox_points, axis=0)
        return centroid.tolist()

    def publish_markers(self, objects, header):
        """Publish visualization markers."""
        marker_array = MarkerArray()

        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'base_link'
            marker.id = obj['track_id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = obj['position'][0]
            marker.pose.position.y = obj['position'][1]
            marker.pose.position.z = obj['position'][2]

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


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
