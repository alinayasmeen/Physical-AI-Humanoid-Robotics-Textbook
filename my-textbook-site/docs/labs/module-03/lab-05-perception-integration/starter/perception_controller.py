#!/usr/bin/env python3
"""
Lab 5: Perception-to-Control Integration - Starter Code
Module 3: Perception & Sensors

Complete the TODOs to build a target-following robot.
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
        # TODO 2: Implement tracking logic
        # 1. Filter detections by target_class
        # 2. If no last_bbox, pick highest confidence
        # 3. Otherwise, find closest match to last_bbox
        # 4. Update frames_lost counter

        return None  # Replace with tracked detection

    def bbox_distance(self, bbox1, bbox2):
        """Calculate center-to-center distance between bboxes."""
        c1x = bbox1.center.position.x
        c1y = bbox1.center.position.y
        c2x = bbox2.center.position.x
        c2y = bbox2.center.position.y
        return math.sqrt((c1x - c2x)**2 + (c1y - c2y)**2)


class PerceptionController(Node):
    """Visual servoing controller that follows detected targets."""

    def __init__(self):
        super().__init__('perception_controller')

        # Configuration
        self.target_class = 'person'  # Class to follow
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

        # TODO 1: Create subscribers and publishers
        # Subscribe to /detections (Detection2DArray)
        # Publish to /cmd_vel (Twist)

        # Create timer for control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Perception controller started')
        self.get_logger().info(f'Following target class: {self.target_class}')

    def detection_callback(self, msg):
        """Process detections and update tracking."""
        # Update tracker with new detections
        self.current_target = self.tracker.update(
            msg.detections, self.target_class)

        # Update state machine
        self.update_state()

    def update_state(self):
        """Update state based on tracking status."""
        # TODO 4: Implement state machine
        # - SEARCHING: tracker.frames_lost > 30
        # - STOPPED: target very close (area > 40000)
        # - APPROACHING: target close (area > 20000)
        # - TRACKING: target visible but far
        pass

    def control_loop(self):
        """Generate and publish velocity commands."""
        # TODO 3: Compute velocity based on state and target
        cmd = Twist()

        if self.state == ControlState.SEARCHING:
            # Rotate to search for target
            cmd.angular.z = 0.3
        elif self.state == ControlState.STOPPED:
            # Stop when close enough
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_target is not None:
            # Visual servoing toward target
            cmd = self.compute_velocity(self.current_target.bbox)

        # Publish velocity command
        # self.cmd_pub.publish(cmd)  # Uncomment after creating publisher

        # Log state
        if hasattr(self, '_last_state') and self._last_state != self.state:
            self.get_logger().info(f'State: {self.state}')
        self._last_state = self.state

    def compute_velocity(self, bbox):
        """Compute velocity to follow target."""
        cmd = Twist()

        if bbox is None:
            return cmd

        # Get target center and size
        cx = bbox.center.position.x
        area = bbox.size_x * bbox.size_y

        # Angular: turn toward target
        x_error = cx - self.image_width / 2
        cmd.angular.z = -self.angular_gain * x_error

        # Linear: maintain distance (approach if far, back up if close)
        area_error = self.desired_area - area
        cmd.linear.x = self.linear_gain * area_error

        # Clamp velocities
        cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
