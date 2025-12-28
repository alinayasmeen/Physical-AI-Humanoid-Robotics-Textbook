#!/usr/bin/env python3
"""
Lab 4: Object Detection Pipeline - Starter Code
Module 3: Perception & Sensors

Complete the TODOs to build a YOLO-based detection pipeline.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    """ROS 2 node for YOLO-based object detection."""

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4

        # TODO 1: Load YOLO model and class names
        # Hint: Use cv2.dnn.readNetFromONNX('models/yolov5s.onnx')
        # Also load class names from 'models/coco.names'
        self.net = None
        self.classes = []

        if self.net is None:
            self.get_logger().warn('TODO 1: Load YOLO model')

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.detection_callback, 10)

        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

        self.viz_pub = self.create_publisher(
            Image, '/detections/image', 10)

        self.get_logger().info('Object detector started')

    def detection_callback(self, msg):
        """Process image and publish detections."""
        if self.net is None:
            return

        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # TODO 2: Preprocess image for YOLO
        # Hint: Use cv2.dnn.blobFromImage()
        blob = None

        if blob is None:
            self.get_logger().warn('TODO 2: Implement preprocessing')
            return

        # Run inference
        self.net.setInput(blob)
        outputs = self.net.forward()

        # TODO 3: Process outputs to extract boxes
        # Parse YOLO output format to get boxes, confidences, class_ids
        boxes, confidences, class_ids = [], [], []

        if len(boxes) == 0:
            self.get_logger().warn('TODO 3: Implement postprocessing')

        # TODO 4: Apply Non-Maximum Suppression
        # Hint: Use cv2.dnn.NMSBoxes()
        indices = []

        # TODO 5: Create and publish detection message
        # Use self.create_detection_msg()
        det_array = None

        if det_array is not None:
            self.detection_pub.publish(det_array)

        # Visualize detections
        viz_frame = self.draw_detections(
            frame.copy(), boxes, confidences, class_ids, indices)
        viz_msg = self.bridge.cv2_to_imgmsg(viz_frame, 'bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)

    def preprocess(self, image):
        """Prepare image for YOLO inference."""
        # Your implementation here
        pass

    def postprocess(self, outputs, image_shape):
        """Parse YOLO outputs to get boxes, confidences, class_ids."""
        # Your implementation here
        pass

    def create_detection_msg(self, boxes, confidences, class_ids, indices, header):
        """Create Detection2DArray message."""
        # Your implementation here
        pass

    def draw_detections(self, image, boxes, confidences, class_ids, indices):
        """Draw bounding boxes on image."""
        for i in indices:
            x, y, w, h = boxes[i]
            label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"

            # Draw box
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw label
            cv2.putText(image, label, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
