# Mini-Project 1: Navigation Testbed

**Duration**: 3-4 hours
**Module**: Module 2 - The Digital Twin
**Prerequisites**: Labs 1-3 completed, understanding of Gazebo worlds and ROS 2 integration

## Overview

Build a complete Gazebo simulation environment for robot navigation testing. This project integrates all simulation concepts from Module 2: digital twin workflows, physics simulation, world creation, and sensor integration.

## Objectives

Create a navigation testbed with:
- Custom Gazebo world with varied terrain
- Multiple obstacle types (static and dynamic)
- Waypoint markers for navigation goals
- Sensor-equipped robot for autonomous navigation testing
- ROS 2 integration for control and feedback

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Navigation Testbed World                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                                                          │    │
│  │    [Start]  ──────▶  [Waypoint 1]  ──────▶  [Goal]      │    │
│  │       │                    │                   │         │    │
│  │       │    ┌──────┐       │    ┌──────┐      │         │    │
│  │       └───▶│ Obs1 │◀──────┘───▶│ Obs2 │◀─────┘         │    │
│  │            └──────┘            └──────┘                  │    │
│  │                                                          │    │
│  │    [Dynamic Obstacle]  ←→  Moving along path             │    │
│  │                                                          │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
│  Robot Sensors:                                                  │
│  - LiDAR (360° scan for obstacle detection)                     │
│  - Camera (visual navigation and waypoint detection)            │
│  - IMU (odometry and orientation)                               │
└─────────────────────────────────────────────────────────────────┘
```

## Requirements

### 1. World Design

Create a navigation testbed world (`navigation_testbed.world`) with:

**Terrain Features**:
- Flat ground area (20m x 20m minimum)
- Optional: Ramp sections with 5-10° incline
- Distinct visual texture for ground plane

**Static Obstacles**:
- At least 5 static obstacles (boxes, cylinders, walls)
- Varied sizes (0.3m to 2m)
- Collision properties configured

**Dynamic Obstacle**:
- One moving obstacle (optional, for advanced testing)
- Predictable movement pattern (back-and-forth)

**Waypoint Markers**:
- Visual markers (colored cylinders or spheres)
- Start position marker (green)
- Intermediate waypoints (yellow)
- Goal position marker (red)

### 2. Robot Configuration

Use the sensor-equipped robot from Lab 3, or configure a robot with:

```xml
<!-- Required sensors -->
<sensor name="lidar" type="gpu_lidar">
  <!-- 360° scan, 10Hz update rate -->
</sensor>

<sensor name="front_camera" type="camera">
  <!-- RGB camera for visual navigation -->
</sensor>

<sensor name="imu" type="imu">
  <!-- Orientation and acceleration data -->
</sensor>
```

### 3. ROS 2 Launch File

Create `navigation_testbed.launch.py`:

```python
# Must launch:
# - Gazebo with navigation_testbed.world
# - Robot spawned at start position
# - ros_gz bridges for all sensor topics
# - Optional: RViz2 for visualization
```

### 4. Test Script

Create `test_navigation.py` ROS 2 node:

```python
# Test script should:
# 1. Verify robot spawned correctly
# 2. Check all sensor topics are publishing
# 3. Send robot to each waypoint sequentially
# 4. Log navigation progress and sensor readings
# 5. Report success/failure for each waypoint
```

## Getting Started

### Setup Workspace

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python navigation_testbed \
    --dependencies rclpy geometry_msgs sensor_msgs
```

### Starter Files

Use the starter code in `starter/` directory:
- `worlds/navigation_testbed_template.sdf` - World template with TODOs
- `launch/navigation_testbed.launch.py` - Launch file template
- `navigation_testbed/test_navigation.py` - Test script template

### Running the Testbed

```bash
# Build the package
colcon build --packages-select navigation_testbed

# Launch the testbed
ros2 launch navigation_testbed navigation_testbed.launch.py

# In another terminal, run the test script
ros2 run navigation_testbed test_navigation
```

## Evaluation Criteria

| Criterion | Points |
|-----------|--------|
| World includes 5+ static obstacles | 15 |
| Waypoint markers visible and positioned correctly | 15 |
| Robot sensors configured and publishing | 20 |
| Launch file works without errors | 15 |
| Test script verifies sensor topics | 15 |
| Navigation between waypoints possible | 10 |
| Clean code and documentation | 10 |
| **Total** | **100** |

## Extensions (Bonus)

1. **Dynamic Obstacles**: Add moving obstacles with collision avoidance challenges
2. **Terrain Variation**: Include ramps, rough terrain, or deformable surfaces
3. **Multiple Robots**: Spawn and coordinate multiple robots
4. **Weather Effects**: Add fog, lighting changes, or particle effects
5. **Metrics Collection**: Log and visualize navigation performance metrics
6. **Path Planning Integration**: Integrate with Nav2 navigation stack

## Hints

- Use `<include>` tags in SDF to reuse obstacle models
- Set `<static>true</static>` for obstacles that shouldn't move
- Use `<visual>` without `<collision>` for waypoint markers (non-physical)
- Configure sensor update rates based on your hardware capability
- Test with `gz topic -l` to verify Gazebo topics before bridging

## Solution

See `solution/` directory for complete implementation including:
- `worlds/navigation_testbed.world` - Complete world file
- `launch/navigation_testbed.launch.py` - Working launch file
- `navigation_testbed/test_navigation.py` - Complete test script

## Reflection Questions

1. How does the choice of sensor update rate affect navigation performance vs. computational cost?
2. What physics parameters (friction, mass) most impact the robot's ability to navigate around obstacles?
3. How would you modify this testbed to evaluate different navigation algorithms fairly?
4. What sim-to-real gaps might you encounter when deploying navigation code from this testbed to a physical robot?
5. How could you automate the process of generating varied obstacle layouts for stress testing?
