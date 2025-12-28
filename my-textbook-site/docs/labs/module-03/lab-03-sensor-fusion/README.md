# Lab 3: Multi-Sensor Fusion

**Module**: 3 - Perception & Sensors
**Duration**: 120 minutes
**Difficulty**: Intermediate

## Objective

Implement sensor fusion by synchronizing and combining data from camera and LiDAR sensors. You'll use ROS 2 message filters for time synchronization and learn to project 3D points onto 2D images.

## Prerequisites

- Completed Lab 1 (Image Subscriber)
- Completed Lab 2 (LiDAR Processing)
- Understanding of coordinate frames and TF2
- ROS 2 Humble with `message_filters` package

## Learning Outcomes

By completing this lab, you will:
1. Use message_filters for temporal synchronization
2. Understand coordinate frame transformations
3. Project LiDAR points onto camera images
4. Implement basic sensor fusion pipeline

---

## Setup

### 1. Verify Installation

```bash
# Check message_filters
python3 -c "from message_filters import Subscriber, ApproximateTimeSynchronizer; print('OK')"

# Check TF2
ros2 pkg list | grep tf2
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-03/lab-03-sensor-fusion .
cd lab-03-sensor-fusion
```

### 3. Launch Multi-Sensor Simulation

```bash
# Terminal 1: Launch Gazebo with camera + LiDAR robot
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Verify both topics
ros2 topic list | grep -E "(camera|lidar)"
```

---

## Lab Steps

### Step 1: Set Up Synchronized Subscribers (25 min)

Complete TODO 1 - create synchronized subscribers:

```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Create subscribers
self.image_sub = Subscriber(self, Image, '/camera/image_raw')
self.lidar_sub = Subscriber(self, PointCloud2, '/lidar/points')

# Create synchronizer with 0.1s tolerance
self.sync = ApproximateTimeSynchronizer(
    [self.image_sub, self.lidar_sub],
    queue_size=10,
    slop=0.1)
self.sync.registerCallback(self.sync_callback)
```

### Step 2: Process Synchronized Data (25 min)

Complete TODO 2 - handle synchronized callback:

```python
def sync_callback(self, image_msg, lidar_msg):
    # Convert image
    cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    # Convert point cloud
    points = self.pointcloud_to_numpy(lidar_msg)

    # Log synchronization
    self.get_logger().info(
        f'Synced: {len(points)} points with {cv_image.shape} image'
    )
```

### Step 3: Project Points to Image (30 min)

Complete TODO 3 - project 3D LiDAR points onto 2D camera image:

```python
def project_points_to_image(self, points_3d, camera_matrix):
    """Project 3D points to 2D image coordinates."""
    # Filter points in front of camera (z > 0)
    mask = points_3d[:, 2] > 0
    valid_points = points_3d[mask]

    # Project: [u, v, w] = K @ [x, y, z]
    projected = camera_matrix @ valid_points.T
    projected = projected.T

    # Normalize: u = u/w, v = v/w
    u = projected[:, 0] / projected[:, 2]
    v = projected[:, 1] / projected[:, 2]

    return u, v, valid_points[:, 2]  # x, y, depth
```

### Step 4: Overlay LiDAR on Image (25 min)

Complete TODO 4 - visualize fusion:

```python
def overlay_points_on_image(self, image, u, v, depths):
    """Draw LiDAR points on camera image with depth coloring."""
    # Normalize depths for coloring
    max_depth = 10.0
    for i in range(len(u)):
        x, y = int(u[i]), int(v[i])
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            # Color by depth (red=close, blue=far)
            intensity = int(255 * (1 - min(depths[i], max_depth) / max_depth))
            color = (intensity, 0, 255 - intensity)
            cv2.circle(image, (x, y), 2, color, -1)
    return image
```

### Step 5: Test and Validate (15 min)

```bash
# Run your implementation
python3 starter/sensor_fusion.py

# Compare with solution
python3 solution/sensor_fusion.py

# Validate
python3 validate.py
```

---

## Deliverables

1. **Working Code**: Complete `starter/sensor_fusion.py`
2. **Fusion Visualization**: Screenshot showing LiDAR points overlaid on image
3. **Reflection Answers**: Complete questions below

---

## Reflection Questions

1. **Why is time synchronization critical for sensor fusion?**
   - Consider: robot motion, temporal consistency

2. **What happens if camera and LiDAR have different frame rates?**
   - Think about: queue sizes, slop tolerance

3. **Why must we filter points with z less than or equal to 0 before projection?**
   - Consider: camera geometry, behind-camera points

4. **How would you improve this fusion for moving objects?**
   - Think about: motion compensation, prediction

---

## Validation

```bash
python3 validate.py
```

Expected output:
```
[PASS] sensor_fusion.py exists
[PASS] Valid Python syntax
[PASS] ApproximateTimeSynchronizer used
[PASS] sync_callback implemented
[PASS] Point projection logic present
[PASS] Image overlay implemented
Lab 3 Complete!
```

---

## Extension Challenges

1. **Depth-aware Detection**: Use LiDAR depth to filter camera detections
2. **Point Coloring**: Color LiDAR points with corresponding image pixels
3. **Obstacle Distance**: Calculate distance to detected objects
4. **Real-time Display**: Show fusion results in OpenCV window

---

## Next Steps

- **Lab 4**: Object Detection - Implement detection pipelines
- **Lesson 3**: Perception Pipelines - Complete perception systems
