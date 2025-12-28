# Lab 1: Image Subscriber and Basic Processing

**Module**: 3 - Perception & Sensors
**Duration**: 60 minutes
**Difficulty**: Beginner

## Objective

Build a ROS 2 node that subscribes to camera images, processes them using OpenCV, and publishes the results. You'll implement edge detection, color filtering, and learn the image transport pipeline.

## Prerequisites

- Completed Module 1 (ROS 2 fundamentals)
- Completed Module 2 (Simulation basics with camera sensors)
- Ubuntu 22.04 with ROS 2 Humble installed
- OpenCV 4.x installed (`pip install opencv-python`)
- cv_bridge package installed (`ros-humble-cv-bridge`)

## Learning Outcomes

By completing this lab, you will:
1. Subscribe to ROS 2 image topics using sensor_msgs/Image
2. Convert between ROS 2 Image messages and OpenCV arrays
3. Apply basic image processing operations (grayscale, blur, edges)
4. Publish processed images back to ROS 2 topics

---

## Setup

### 1. Verify Installation

```bash
# Check OpenCV version
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"

# Check cv_bridge is available
ros2 pkg list | grep cv_bridge

# Check sensor_msgs
ros2 interface show sensor_msgs/msg/Image
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-03/lab-01-image-subscriber .
cd lab-01-image-subscriber
```

### 3. Launch Simulation with Camera

Start Gazebo with a robot that has a camera sensor:

```bash
# Terminal 1: Launch Gazebo with camera-equipped robot
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Verify camera topic is publishing
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

---

## Lab Steps

### Step 1: Understand the Starter Code (10 min)

Open `starter/image_subscriber.py` and examine its structure:

```bash
cat starter/image_subscriber.py
```

The starter code provides:
- Basic ROS 2 node structure
- Empty callback function
- TODO markers for your implementation

**Your task**: Complete the TODOs to make the node functional.

### Step 2: Implement Image Conversion (15 min)

In the `image_callback` method, convert the ROS 2 Image to OpenCV format:

```python
# TODO 1: Convert ROS Image to OpenCV format
cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
```

Test your conversion:
```bash
# Terminal 1: Run your node
python3 starter/image_subscriber.py

# Terminal 2: Check for log output
ros2 topic echo /camera/image_raw --no-arr | head -20
```

### Step 3: Add Image Processing (15 min)

Implement the processing pipeline:

1. **Grayscale conversion**: Reduce 3 channels to 1
2. **Gaussian blur**: Reduce noise with 5x5 kernel
3. **Canny edge detection**: Find edges with thresholds 50, 150

```python
# TODO 2: Process the image
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blurred, 50, 150)
```

### Step 4: Publish Processed Image (10 min)

Convert the processed image back to ROS 2 format and publish:

```python
# TODO 3: Convert edges to BGR and publish
edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
out_msg = self.bridge.cv2_to_imgmsg(edges_bgr, 'bgr8')
out_msg.header = msg.header  # Preserve timestamp
self.processed_pub.publish(out_msg)
```

Verify publication:
```bash
# Check your processed topic is publishing
ros2 topic list | grep processed
ros2 topic hz /camera/image_processed
```

### Step 5: Visualize Results (5 min)

Use rqt_image_view to see both raw and processed images:

```bash
# Terminal: Launch image viewer
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw` and `/camera/image_processed` to compare.

### Step 6: Compare with Solution (5 min)

Review the complete solution:

```bash
# Compare your implementation
diff starter/image_subscriber.py solution/image_subscriber.py

# Run the solution
python3 solution/image_subscriber.py
```

---

## Deliverables

1. **Working Node**: `starter/image_subscriber.py` that:
   - Subscribes to `/camera/image_raw`
   - Converts to OpenCV format
   - Applies edge detection
   - Publishes to `/camera/image_processed`

2. **Screenshot**: Image showing raw vs processed frames side-by-side

3. **Answers**: Complete the reflection questions below

---

## Reflection Questions

1. **Why do we convert to grayscale before edge detection?**
   - Consider: computation cost, Canny algorithm requirements, information content

2. **What happens if you change the Canny thresholds (50, 150)?**
   - Experiment with: (10, 50), (100, 200), (150, 250)
   - Observe: edge sensitivity, noise handling

3. **Why do we preserve the original message header when publishing?**
   - Think about: timestamp synchronization, frame_id, downstream consumers

4. **How would you modify this node to detect only horizontal edges?**
   - Consider: Sobel operators, kernel orientation

---

## Validation

Run the validation script to verify your lab completion:

```bash
python3 validate.py
```

Expected output:
```
[PASS] image_subscriber.py exists in starter/
[PASS] Valid Python syntax
[PASS] cv_bridge import present
[PASS] Image subscriber created
[PASS] Image publisher created
[PASS] imgmsg_to_cv2 conversion used
[PASS] cv2_to_imgmsg conversion used
Lab 1 Complete!
```

---

## Troubleshooting

### ImportError: No module named 'cv_bridge'

```bash
# Install cv_bridge
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv
```

### No camera topic available

```bash
# Check if simulation is running
ros2 node list

# If no camera, spawn a simple robot with camera
ros2 run gazebo_ros spawn_entity.py -entity camera_robot -file /path/to/robot.sdf
```

### Image displays as black

- Check encoding matches: use 'bgr8' for color, 'mono8' for grayscale
- Verify image data is not all zeros
- Check camera is pointing at something

### Frame rate very slow

- Reduce image resolution in simulation
- Process every Nth frame instead of every frame
- Use threading for heavy processing

---

## Extension Challenges

1. **Color Detection**: Modify the node to detect red objects instead of edges
2. **Region of Interest**: Process only the bottom half of the image (for lane following)
3. **Multi-topic**: Subscribe to depth image and overlay on color image
4. **Parameters**: Make thresholds configurable via ROS 2 parameters

---

## Next Steps

After completing this lab, proceed to:
- **Lab 2**: LiDAR Processing - Work with point cloud data
- **Lesson 2**: Multi-Sensor Integration - Understanding different sensor types
