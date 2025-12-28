# Lab 4: Object Detection Pipeline

**Module**: 3 - Perception & Sensors
**Duration**: 120 minutes
**Difficulty**: Intermediate

## Objective

Implement a complete object detection pipeline using OpenCV DNN with YOLO, publishing detections as ROS 2 vision_msgs. You'll process camera images, run inference, filter results by confidence, and visualize detections.

## Prerequisites

- Completed Labs 1-3
- OpenCV 4.x with DNN module
- YOLO model files (provided or download instructions)
- `ros-humble-vision-msgs` package

## Learning Outcomes

By completing this lab, you will:
1. Load and run YOLO models using OpenCV DNN
2. Process model outputs to extract bounding boxes
3. Apply Non-Maximum Suppression (NMS)
4. Publish detections using vision_msgs

---

## Setup

### 1. Install Dependencies

```bash
# Install vision_msgs
sudo apt install ros-humble-vision-msgs

# Verify OpenCV DNN
python3 -c "import cv2; print(cv2.dnn)"
```

### 2. Download Model Files

```bash
cd starter/models

# Download YOLOv5s ONNX (or use provided)
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.onnx

# Download COCO class names
wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names
```

### 3. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src/labs/module-03/lab-04-object-detection
```

---

## Lab Steps

### Step 1: Load YOLO Model (20 min)

Complete TODO 1 - load the YOLO model:

```python
# Load ONNX model
self.net = cv2.dnn.readNetFromONNX('models/yolov5s.onnx')

# Set backend (CPU or CUDA)
self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Load class names
with open('models/coco.names', 'r') as f:
    self.classes = [line.strip() for line in f]
```

### Step 2: Preprocess Image (20 min)

Complete TODO 2 - prepare image for YOLO:

```python
def preprocess(self, image):
    # Resize and normalize
    blob = cv2.dnn.blobFromImage(
        image,
        scalefactor=1/255.0,
        size=(640, 640),
        swapRB=True,
        crop=False
    )
    return blob
```

### Step 3: Process Model Output (30 min)

Complete TODO 3 - parse YOLO outputs:

```python
def postprocess(self, outputs, image_shape, conf_threshold=0.5):
    h, w = image_shape[:2]
    boxes, confidences, class_ids = [], [], []

    for detection in outputs[0]:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence > conf_threshold:
            # YOLO outputs center_x, center_y, width, height
            cx, cy, bw, bh = detection[0:4]

            # Convert to corner format
            x = int((cx - bw/2) * w)
            y = int((cy - bh/2) * h)
            width = int(bw * w)
            height = int(bh * h)

            boxes.append([x, y, width, height])
            confidences.append(float(confidence))
            class_ids.append(class_id)

    return boxes, confidences, class_ids
```

### Step 4: Apply Non-Maximum Suppression (20 min)

Complete TODO 4 - remove duplicate detections:

```python
def apply_nms(self, boxes, confidences, nms_threshold=0.4):
    indices = cv2.dnn.NMSBoxes(
        boxes, confidences,
        score_threshold=0.5,
        nms_threshold=nms_threshold
    )
    return indices.flatten() if len(indices) > 0 else []
```

### Step 5: Publish Detections (20 min)

Complete TODO 5 - create ROS 2 detection messages:

```python
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose

def create_detection_msg(self, boxes, confidences, class_ids, indices, header):
    det_array = Detection2DArray()
    det_array.header = header

    for i in indices:
        det = Detection2D()
        det.header = header

        # Set bounding box
        det.bbox.center.position.x = boxes[i][0] + boxes[i][2]/2
        det.bbox.center.position.y = boxes[i][1] + boxes[i][3]/2
        det.bbox.size_x = float(boxes[i][2])
        det.bbox.size_y = float(boxes[i][3])

        # Set classification result
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.classes[class_ids[i]]
        hyp.hypothesis.score = confidences[i]
        det.results.append(hyp)

        det_array.detections.append(det)

    return det_array
```

### Step 6: Validate (10 min)

```bash
python3 starter/object_detector.py
python3 validate.py
```

---

## Deliverables

1. **Working Code**: Complete `starter/object_detector.py`
2. **Detection Output**: Screenshot showing bounding boxes on image
3. **Reflection Answers**: Complete questions below

---

## Reflection Questions

1. **Why do we use Non-Maximum Suppression?**
   - Consider: overlapping predictions, duplicate detections

2. **What is the trade-off between confidence threshold and recall?**
   - Think about: false positives, missed detections

3. **How would you optimize for edge deployment?**
   - Consider: model size, quantization, hardware acceleration

4. **Why does YOLO output center coordinates instead of corners?**
   - Think about: anchor boxes, prediction stability

---

## Validation

```bash
python3 validate.py
```

Expected output:
```
[PASS] object_detector.py exists
[PASS] Valid Python syntax
[PASS] YOLO model loading present
[PASS] Preprocessing implemented
[PASS] NMS applied
[PASS] Detection2DArray used
Lab 4 Complete!
```

---

## Next Steps

- **Lab 5**: Perception Integration - Connect detection to robot control
- **Module 4**: Motion Planning & Control
