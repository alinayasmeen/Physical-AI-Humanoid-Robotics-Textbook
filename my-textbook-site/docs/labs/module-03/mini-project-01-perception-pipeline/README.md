# Mini-Project 1: Complete Perception Pipeline

**Module**: 3 - Perception & Sensors
**Duration**: 3-4 hours
**Difficulty**: Advanced

## Overview

Build a complete perception pipeline that processes camera and LiDAR data to detect, track, and localize objects in 3D space. This project integrates all concepts from Module 3 into a cohesive system.

## Objectives

By completing this mini-project, you will:

1. Integrate camera and LiDAR data streams
2. Implement object detection with 3D localization
3. Build a tracking system that maintains object identity
4. Publish perception results for downstream consumers

## Prerequisites

- Completed Labs 1-5 of Module 3
- Understanding of coordinate frame transformations
- Familiarity with ROS 2 message types

## System Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Camera    │────▶│  Detector   │────▶│   Tracker   │
└─────────────┘     └─────────────┘     └─────────────┘
                           │                    │
                           ▼                    │
┌─────────────┐     ┌─────────────┐            │
│   LiDAR     │────▶│  3D Fusion  │◀───────────┘
└─────────────┘     └─────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │  Publisher  │──▶ /perception/objects
                    └─────────────┘
```

## Requirements

### Functional Requirements

1. **Input Processing**
   - Subscribe to `/camera/image_raw` (sensor_msgs/Image)
   - Subscribe to `/lidar/points` (sensor_msgs/PointCloud2)
   - Synchronize inputs within 100ms tolerance

2. **Object Detection**
   - Detect objects using YOLO or similar detector
   - Filter detections by confidence threshold (>0.5)
   - Apply Non-Maximum Suppression

3. **3D Localization**
   - Project LiDAR points to image coordinates
   - Associate detections with point cloud regions
   - Calculate 3D centroid for each detection

4. **Tracking**
   - Assign consistent IDs to objects across frames
   - Handle object entry/exit from scene
   - Maintain track history for trajectory analysis

5. **Output**
   - Publish to `/perception/objects` (custom message)
   - Include: class, confidence, 3D position, track_id, velocity

### Non-Functional Requirements

- Process at minimum 10 FPS
- Latency under 100ms from input to output
- Handle sensor failures gracefully

## Getting Started

### 1. Create ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create perception_pipeline --build-type ament_python --dependencies rclpy sensor_msgs vision_msgs geometry_msgs
```

### 2. Define Custom Message

Create `perception_msgs/msg/DetectedObject3D.msg`:

```
std_msgs/Header header
string class_id
float32 confidence
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
int32 track_id
float32 bbox_size_x
float32 bbox_size_y
float32 bbox_size_z
```

### 3. Implement Pipeline Nodes

The starter code provides a skeleton. Complete the TODOs:

- `perception_node.py` - Main pipeline orchestration
- `detector.py` - Object detection module
- `tracker.py` - Multi-object tracking
- `fusion.py` - Camera-LiDAR fusion

## Validation Criteria

Run the validation script:

```bash
python3 validate.py
```

Expected checks:
- [ ] Pipeline subscribes to required topics
- [ ] Detections published to output topic
- [ ] 3D positions within reasonable bounds
- [ ] Track IDs consistent across 10+ frames
- [ ] Processing rate >= 10 FPS
- [ ] No memory leaks over 5-minute run

## Deliverables

1. **Working Pipeline**: Complete implementation in `starter/`
2. **Demo Video**: 30-second recording showing object tracking
3. **Performance Report**: FPS, latency, accuracy metrics
4. **Documentation**: README with setup instructions

## Extension Challenges

1. **Semantic Segmentation**: Replace detection with segmentation
2. **Velocity Estimation**: Calculate object velocities from track history
3. **Occlusion Handling**: Predict positions during temporary occlusions
4. **Multi-Camera**: Extend to handle multiple camera views

## Resources

- [ROS 2 Message Filters Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Message-Filters.html)
- [vision_msgs Package](https://github.com/ros-perception/vision_msgs)
- [OpenCV DNN Module](https://docs.opencv.org/4.x/d2/d58/tutorial_table_of_content_dnn.html)
