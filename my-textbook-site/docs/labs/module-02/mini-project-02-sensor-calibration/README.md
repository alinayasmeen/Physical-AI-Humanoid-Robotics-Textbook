# Mini-Project 2: Sensor Calibration Simulator

**Duration**: 3-4 hours
**Module**: Module 2 - The Digital Twin
**Prerequisites**: Labs 1-3 completed, understanding of simulated sensors and ROS 2 topics

## Overview

Build a simulation environment for testing and calibrating multiple robot sensors. This project integrates sensor configuration, data pipeline concepts, and validation techniques from Module 2, creating a tool that can be used to tune sensor parameters before deployment.

## Objectives

Create a sensor calibration system with:
- Multiple configurable sensors (camera, LiDAR, IMU)
- Calibration targets and reference objects
- Data collection and analysis tools
- Sensor accuracy validation scripts
- ROS 2 integration for real-time calibration

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                 Sensor Calibration Environment                   │
│                                                                  │
│  ┌─────────────────┐        ┌─────────────────┐                 │
│  │ Calibration     │        │ Reference       │                 │
│  │ Target Board    │        │ Objects         │                 │
│  │ (Checkerboard)  │        │ (Known sizes)   │                 │
│  └─────────────────┘        └─────────────────┘                 │
│                                                                  │
│           ┌─────────────────────────────┐                       │
│           │      Robot with Sensors      │                       │
│           │  ┌───────┐ ┌───────┐ ┌───┐  │                       │
│           │  │Camera │ │ LiDAR │ │IMU│  │                       │
│           │  └───────┘ └───────┘ └───┘  │                       │
│           └─────────────────────────────┘                       │
│                        │                                         │
│                        ▼                                         │
│           ┌─────────────────────────────┐                       │
│           │   Calibration Node (ROS 2)   │                       │
│           │   - Collect sensor data      │                       │
│           │   - Compare to ground truth  │                       │
│           │   - Calculate error metrics  │                       │
│           │   - Generate calibration     │                       │
│           └─────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
```

## Requirements

### 1. Calibration Environment

Create a calibration world (`calibration_room.sdf`) with:

**Calibration Targets**:
- Checkerboard pattern for camera calibration (1m x 1m)
- Distance markers at known positions (1m, 2m, 3m, 5m)
- Reference cube with known dimensions (0.5m x 0.5m x 0.5m)

**Environment Features**:
- Clean, well-lit indoor environment
- Flat floor with distance grid markings
- Neutral colored walls for clear sensor readings
- Multiple target positions for comprehensive testing

### 2. Sensor Configuration

Configure sensors with adjustable parameters:

```yaml
# sensor_calibration_config.yaml
camera:
  resolution: [640, 480]
  fov: 1.047  # 60 degrees
  update_rate: 30
  noise:
    type: gaussian
    stddev: 0.02

lidar:
  samples: 360
  range_min: 0.1
  range_max: 10.0
  update_rate: 10
  noise:
    type: gaussian
    stddev: 0.01

imu:
  update_rate: 100
  noise:
    angular_velocity:
      stddev: 0.001
    linear_acceleration:
      stddev: 0.01
```

### 3. Calibration Node

Create `sensor_calibrator.py` ROS 2 node:

```python
# Node should:
# 1. Subscribe to all sensor topics
# 2. Collect calibration data samples
# 3. Compare measurements to known ground truth
# 4. Calculate error metrics (RMSE, mean error, std dev)
# 5. Generate calibration report
```

### 4. Validation Tests

Create tests to validate sensor accuracy:

**Camera Tests**:
- Detect checkerboard corners
- Verify FOV coverage
- Measure color accuracy

**LiDAR Tests**:
- Measure distance to known targets
- Verify scan coverage (360°)
- Calculate range accuracy at different distances

**IMU Tests**:
- Verify gravity vector (9.81 m/s²)
- Test orientation stability
- Measure drift over time

## Getting Started

### Setup Workspace

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python sensor_calibration \
    --dependencies rclpy sensor_msgs geometry_msgs cv_bridge numpy
```

### Starter Files

Use the starter code in `starter/` directory:
- `worlds/calibration_room_template.sdf` - Environment template
- `config/sensor_config_template.yaml` - Sensor configuration
- `sensor_calibration/calibrator_node.py` - Calibration node template
- `sensor_calibration/validation_tests.py` - Test script template

### Running the Calibration

```bash
# Build the package
colcon build --packages-select sensor_calibration

# Launch calibration environment
ros2 launch sensor_calibration calibration.launch.py

# Run calibration node
ros2 run sensor_calibration calibrator_node

# Run validation tests
ros2 run sensor_calibration validation_tests
```

## Evaluation Criteria

| Criterion | Points |
|-----------|--------|
| Calibration environment with targets | 15 |
| Camera calibration working | 20 |
| LiDAR accuracy validation | 20 |
| IMU gravity/orientation test | 15 |
| Error metrics calculated correctly | 15 |
| Calibration report generated | 10 |
| Clean code and documentation | 5 |
| **Total** | **100** |

## Extensions (Bonus)

1. **Interactive Calibration**: GUI for adjusting sensor parameters in real-time
2. **Multi-sensor Fusion**: Cross-validate sensors against each other
3. **Automated Calibration**: Implement automatic parameter tuning
4. **Export Calibration**: Save calibration data in standard formats (YAML, JSON)
5. **Temperature Simulation**: Model sensor drift with simulated temperature changes
6. **Noise Characterization**: Implement Allan variance analysis for IMU

## Hints

- Use OpenCV for checkerboard detection (`cv2.findChessboardCorners`)
- Store ground truth positions in a configuration file
- Collect multiple samples and average for better accuracy
- Use `numpy` for statistical calculations
- Log data with timestamps for time-series analysis
- Consider sensor synchronization for multi-sensor comparison

## Solution

See `solution/` directory for complete implementation including:
- `worlds/calibration_room.sdf` - Complete calibration environment
- `config/sensor_config.yaml` - Tuned sensor configuration
- `sensor_calibration/calibrator_node.py` - Working calibration node
- `sensor_calibration/validation_tests.py` - Complete test suite

## Calibration Report Format

Your calibration node should generate a report like:

```
=== Sensor Calibration Report ===
Date: 2024-01-15 14:30:00
Duration: 60 seconds
Samples collected: 600

--- Camera Calibration ---
Resolution: 640x480
Checkerboard detected: YES
Reprojection error: 0.23 pixels
FOV measured: 59.8° (expected: 60°)

--- LiDAR Calibration ---
Range accuracy (1m target): 0.012m RMSE
Range accuracy (3m target): 0.018m RMSE
Range accuracy (5m target): 0.025m RMSE
Angular resolution: 1.0°
Coverage: 360°

--- IMU Calibration ---
Gravity magnitude: 9.807 m/s² (expected: 9.81)
Orientation drift: 0.02°/min
Accelerometer noise: 0.008 m/s² std
Gyroscope noise: 0.0008 rad/s std

--- Overall Status ---
Camera: PASS
LiDAR: PASS
IMU: PASS
```

## Reflection Questions

1. Why is sensor calibration important before deploying a robot in the real world?
2. How do simulated sensor noise models compare to real sensor noise characteristics?
3. What additional calibration would you need for a multi-camera system?
4. How would you calibrate sensor timing/synchronization?
5. What is the sim-to-real gap for sensor calibration, and how can it be minimized?
