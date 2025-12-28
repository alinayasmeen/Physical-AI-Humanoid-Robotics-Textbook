#!/usr/bin/env python3
"""
Lab 5: Perception-to-Control Integration - Solution
Module 3: Perception & Sensors

Complete implementation of target-following robot.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import math


class ControlState:
    """State machine states."""
    SEARCHING = 'searching'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    STOPPED = 'stopped'


class SimpleTracker:
    """Simple single-target tracker."""

    def __init__(self, max_distance=100):
        self.last_bbox = None
        self.max_distance = max_distance
        self.frames_lost = 0

    def update(self, detections, target_class):
        """Find and track target object."""
        # SOLUTION 2: Filter by target class
        candidates = []
        for det in detections:
            if len(det.results) > 0:
                if det.results[0].hypothesis.class_id == target_class:
                    candidates.append(det)

        if not candidates:
            self.frames_lost += 1
            return None

        if self.last_bbox is None:
            # Pick highest confidence
            best = max(candidates,
                      key=lambda d: d.results[0].hypothesis.score)
            self.last_bbox = best.bbox
            self.frames_lost = 0
            return best

        # Find closest to last position
        best_match = None
        best_dist = self.max_distance

        for det in candidates:
            dist = self.bbox_distance(self.last_bbox, det.bbox)
            if dist < best_dist:
                best_dist = dist
                best_match = det

        if best_match:
            self.last_bbox = best_match.bbox
            self.frames_lost = 0
            return best_match

        self.frames_lost += 1
        return None

    def bbox_distance(self, bbox1, bbox2):
        """Calculate center-to-center distance."""
        c1x = bbox1.center.position.x
        c1y = bbox1.center.position.y
        c2x = bbox2.center.position.x
        c2y = bbox2.center.position.y
        return math.sqrt((c1x - c2x)**2 + (c1y - c2y)**2)


class PerceptionController(Node):
    """Visual servoing controller."""

    def __init__(self):
        super().__init__('perception_controller')

        # Configuration
        self.target_class = 'person'
        self.image_width = 640
        self.image_height = 480

        # Control gains
        self.angular_gain = 0.003
        self.linear_gain = 0.0001
        self.desired_area = 25000

        # State
        self.state = ControlState.SEARCHING
        self.current_target = None
        self.tracker = SimpleTracker()

        # SOLUTION 1: Create subscribers and publishers
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Perception controller started')
        self.get_logger().info(f'Following: {self.target_class}')

    def detection_callback(self, msg):
        """Process detections and update tracking."""
        self.current_target = self.tracker.update(
            msg.detections, self.target_class)
        self.update_state()

    def update_state(self):
        """Update state machine."""
        # SOLUTION 4: State machine logic
        if self.tracker.frames_lost > 30:
            self.state = ControlState.SEARCHING
            self.tracker.last_bbox = None  # Reset tracker
        elif self.current_target is not None:
            bbox = self.current_target.bbox
            area = bbox.size_x * bbox.size_y

            if area > 40000:
                self.state = ControlState.STOPPED
            elif area > 20000:
                self.state = ControlState.APPROACHING
            else:
                self.state = ControlState.TRACKING

    def control_loop(self):
        """Generate velocity commands."""
        # SOLUTION 3: State-based control
        cmd = Twist()

        if self.state == ControlState.SEARCHING:
            cmd.angular.z = 0.3  # Rotate to search
        elif self.state == ControlState.STOPPED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_target is not None:
            cmd = self.compute_velocity(self.current_target.bbox)

        self.cmd_pub.publish(cmd)

        # Log state changes
        if hasattr(self, '_last_state') and self._last_state != self.state:
            self.get_logger().info(f'State: {self.state}')
        self._last_state = self.state

    def compute_velocity(self, bbox):
        """Visual servoing computation."""
        cmd = Twist()

        if bbox is None:
            return cmd

        cx = bbox.center.position.x
        area = bbox.size_x * bbox.size_y

        # Angular: turn toward target
        x_error = cx - self.image_width / 2
        cmd.angular.z = -self.angular_gain * x_error

        # Linear: maintain distance
        area_error = self.desired_area - area
        cmd.linear.x = self.linear_gain * area_error

        # Clamp
        cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
