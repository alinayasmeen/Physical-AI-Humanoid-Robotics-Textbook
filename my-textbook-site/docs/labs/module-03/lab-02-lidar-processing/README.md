# Lab 2: LiDAR Point Cloud Processing

**Module**: 3 - Perception & Sensors
**Duration**: 90 minutes
**Difficulty**: Intermediate

## Objective

Process LiDAR point cloud data in ROS 2, implementing ground plane removal, obstacle clustering, and distance filtering. You'll learn to work with PointCloud2 messages and numpy arrays for efficient 3D data processing.

## Prerequisites

- Completed Module 1 and 2
- Completed Lab 1 (Image Subscriber)
- Ubuntu 22.04 with ROS 2 Humble
- `sensor_msgs_py` package (included with ROS 2)

## Learning Outcomes

By completing this lab, you will:
1. Subscribe to PointCloud2 messages and convert to numpy arrays
2. Implement ground plane removal using height thresholding
3. Filter points by distance for obstacle detection
4. Calculate statistics on point cloud data

---

## Setup

### 1. Verify Installation

```bash
# Check sensor_msgs_py is available
python3 -c "import sensor_msgs_py.point_cloud2 as pc2; print('OK')"

# Check numpy
python3 -c "import numpy; print(f'NumPy: {numpy.__version__}')"
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-03/lab-02-lidar-processing .
cd lab-02-lidar-processing
```

### 3. Launch Simulation with LiDAR

```bash
# Terminal 1: Launch Gazebo with LiDAR-equipped robot
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Verify LiDAR topic
ros2 topic list | grep -E "(lidar|points|scan)"
ros2 topic hz /lidar/points
```

---

## Lab Steps

### Step 1: Understand Point Cloud Structure (15 min)

Examine PointCloud2 message structure:

```bash
# View message definition
ros2 interface show sensor_msgs/msg/PointCloud2

# Echo a single message
ros2 topic echo /lidar/points --once
```

Open `starter/lidar_processor.py` and review the structure.

### Step 2: Convert PointCloud2 to Numpy (20 min)

Complete TODO 1 - convert incoming point clouds to numpy arrays:

```python
# Use sensor_msgs_py for efficient conversion
import sensor_msgs_py.point_cloud2 as pc2

# In callback:
points_list = []
for point in pc2.read_points(msg, skip_nans=True):
    points_list.append([point[0], point[1], point[2]])
cloud = np.array(points_list)
```

### Step 3: Implement Ground Plane Removal (20 min)

Complete TODO 2 - remove ground points based on height:

```python
def remove_ground(points, ground_height=-0.3):
    """Remove points below ground_height threshold"""
    mask = points[:, 2] > ground_height
    return points[mask]
```

Test with different threshold values (-0.5, -0.3, 0.0).

### Step 4: Implement Distance Filtering (20 min)

Complete TODO 3 - keep only nearby obstacles:

```python
def filter_by_distance(points, min_dist=0.5, max_dist=10.0):
    """Keep points within distance range from sensor"""
    distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    mask = (distances > min_dist) & (distances < max_dist)
    return points[mask]
```

### Step 5: Calculate Statistics (10 min)

Complete TODO 4 - compute useful statistics:

```python
def compute_statistics(points):
    """Compute point cloud statistics"""
    return {
        'count': len(points),
        'min_distance': np.min(np.linalg.norm(points[:, :2], axis=1)),
        'max_height': np.max(points[:, 2]),
        'centroid': np.mean(points, axis=0)
    }
```

### Step 6: Validate and Compare (5 min)

```bash
# Run your implementation
python3 starter/lidar_processor.py

# Compare with solution
python3 solution/lidar_processor.py

# Run validation
python3 validate.py
```

---

## Deliverables

1. **Working Code**: Complete `starter/lidar_processor.py`
2. **Statistics Output**: Log showing filtered point counts
3. **Reflection Answers**: Answer questions below

---

## Reflection Questions

1. **Why remove points below a height threshold for ground removal?**
   - Consider: ground plane assumption, simplicity vs accuracy

2. **What are limitations of height-based ground removal?**
   - Think about: ramps, curbs, uneven terrain

3. **How would you detect obstacles vs free space?**
   - Consider: clustering, occupancy grids

4. **What happens with very dense point clouds (>100k points)?**
   - Think about: processing time, downsampling strategies

---

## Validation

```bash
python3 validate.py
```

Expected output:
```
[PASS] lidar_processor.py exists
[PASS] Valid Python syntax
[PASS] PointCloud2 import present
[PASS] read_points function used
[PASS] Ground removal implemented
[PASS] Distance filtering implemented
Lab 2 Complete!
```

---

## Extension Challenges

1. **Voxel Downsampling**: Reduce point density while preserving structure
2. **RANSAC Ground Plane**: Use RANSAC for robust ground detection
3. **Clustering**: Group nearby points into obstacle clusters
4. **Publish Filtered Cloud**: Output processed cloud to new topic

---

## Next Steps

- **Lab 3**: Sensor Fusion - Combine camera and LiDAR data
- **Lesson 3**: Perception Pipelines - Object detection and tracking
