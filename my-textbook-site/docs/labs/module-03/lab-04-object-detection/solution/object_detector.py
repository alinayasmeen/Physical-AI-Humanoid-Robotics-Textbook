#!/usr/bin/env python3
"""
Lab 4: Object Detection Pipeline - Solution
Module 3: Perception & Sensors

Complete implementation of YOLO-based detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path


class ObjectDetector(Node):
    """ROS 2 node for YOLO-based object detection."""

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4
        self.input_size = (640, 640)

        # SOLUTION 1: Load YOLO model and class names
        model_path = Path(__file__).parent / 'models' / 'yolov5s.onnx'
        names_path = Path(__file__).parent / 'models' / 'coco.names'

        try:
            self.net = cv2.dnn.readNetFromONNX(str(model_path))
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.net = None

        try:
            with open(names_path, 'r') as f:
                self.classes = [line.strip() for line in f]
            self.get_logger().info(f'Loaded {len(self.classes)} classes')
        except FileNotFoundError:
            self.classes = [f'class_{i}' for i in range(80)]

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

        # SOLUTION 2: Preprocess image
        blob = self.preprocess(frame)

        # Run inference
        self.net.setInput(blob)
        outputs = self.net.forward()

        # SOLUTION 3: Process outputs
        boxes, confidences, class_ids = self.postprocess(outputs, frame.shape)

        # SOLUTION 4: Apply NMS
        indices = []
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, confidences,
                self.conf_threshold,
                self.nms_threshold
            )
            if len(indices) > 0:
                indices = indices.flatten()

        # SOLUTION 5: Create and publish detection message
        det_array = self.create_detection_msg(
            boxes, confidences, class_ids, indices, msg.header)
        self.detection_pub.publish(det_array)

        # Visualize
        viz_frame = self.draw_detections(
            frame.copy(), boxes, confidences, class_ids, indices)
        viz_msg = self.bridge.cv2_to_imgmsg(viz_frame, 'bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)

        self.get_logger().info(f'Detected {len(indices)} objects')

    def preprocess(self, image):
        """Prepare image for YOLO inference."""
        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1/255.0,
            size=self.input_size,
            swapRB=True,
            crop=False
        )
        return blob

    def postprocess(self, outputs, image_shape):
        """Parse YOLO outputs."""
        h, w = image_shape[:2]
        boxes, confidences, class_ids = [], [], []

        # YOLOv5 output shape: [1, 25200, 85]
        # 85 = 4 (box) + 1 (objectness) + 80 (classes)
        output = outputs[0]

        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = detection[4] * scores[class_id]

            if confidence > self.conf_threshold:
                # YOLO outputs normalized center_x, center_y, width, height
                cx = detection[0] * w
                cy = detection[1] * h
                bw = detection[2] * w
                bh = detection[3] * h

                # Convert to corner format
                x = int(cx - bw/2)
                y = int(cy - bh/2)

                boxes.append([x, y, int(bw), int(bh)])
                confidences.append(float(confidence))
                class_ids.append(int(class_id))

        return boxes, confidences, class_ids

    def create_detection_msg(self, boxes, confidences, class_ids, indices, header):
        """Create Detection2DArray message."""
        det_array = Detection2DArray()
        det_array.header = header

        for i in indices:
            det = Detection2D()
            det.header = header

            # Bounding box
            det.bbox.center.position.x = float(boxes[i][0] + boxes[i][2]/2)
            det.bbox.center.position.y = float(boxes[i][1] + boxes[i][3]/2)
            det.bbox.size_x = float(boxes[i][2])
            det.bbox.size_y = float(boxes[i][3])

            # Classification
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.classes[class_ids[i]]
            hyp.hypothesis.score = confidences[i]
            det.results.append(hyp)

            det_array.detections.append(det)

        return det_array

    def draw_detections(self, image, boxes, confidences, class_ids, indices):
        """Draw bounding boxes on image."""
        for i in indices:
            x, y, w, h = boxes[i]
            label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"

            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
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
