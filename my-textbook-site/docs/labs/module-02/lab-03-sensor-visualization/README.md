# Lab 3: Sensor Configuration and Visualization

**Module**: 2 - The Digital Twin
**Duration**: 90 minutes
**Difficulty**: Intermediate

## Objective

Configure simulated sensors (camera, LiDAR, IMU) in Gazebo and visualize sensor data using ROS 2 tools. By the end of this lab, you'll understand the complete sensor data pipeline from simulation to ROS 2 topics.

## Prerequisites

- Completed Lab 2 (Gazebo Worlds)
- Completed Module 2 Lesson 3 (Sensors & Unity)
- Ubuntu 22.04 with ROS 2 Humble
- Gazebo Fortress with ros_gz packages

## Learning Outcomes

By completing this lab, you will:
1. Configure camera sensors in SDF/URDF
2. Configure LiDAR sensors with customizable parameters
3. Configure IMU sensors with noise models
4. Bridge sensor topics from Gazebo to ROS 2
5. Visualize sensor data in RViz2
6. Process sensor messages in a Python node

---

## Setup

### 1. Install Required Packages

```bash
# Ensure ros_gz packages are installed
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-image

# Install RViz2 if not present
sudo apt install ros-humble-rviz2
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src/labs/module-02/lab-03-sensor-visualization
```

---

## Lab Steps

### Step 1: Review Sensor Configuration (15 min)

Examine the starter sensor configuration file:

```bash
cat starter/sensor_config.yaml
```

This YAML file defines sensor parameters you'll use in the SDF robot model.

### Step 2: Add Camera Sensor (20 min)

Add a camera sensor to a robot model. Create or edit an SDF file:

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.1 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="front_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <topic>camera/image_raw</topic>
</sensor>
```

**Bridge the camera topic:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### Step 3: Add LiDAR Sensor (20 min)

Add a 2D LiDAR sensor:

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.2 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
  <topic>scan</topic>
</sensor>
```

**Bridge the LiDAR topic:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### Step 4: Add IMU Sensor (15 min)

Add an IMU sensor with noise:

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.05 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <topic>imu/data</topic>
</sensor>
```

**Bridge the IMU topic:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU
```

### Step 5: Launch and Verify Topics (10 min)

Launch Gazebo with your sensor-equipped robot:

```bash
# Terminal 1: Launch simulation
gz sim -r your_world_with_robot.sdf

# Terminal 2: Run all bridges
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU

# Terminal 3: List topics
ros2 topic list
```

Expected topics:
- `/camera/image_raw`
- `/scan`
- `/imu/data`

### Step 6: Visualize in RViz2 (10 min)

Launch RViz2 and add displays:

```bash
ros2 run rviz2 rviz2
```

**Add Displays:**
1. Add → By topic → `/camera/image_raw` → Image
2. Add → By topic → `/scan` → LaserScan
3. Set Fixed Frame to `base_link` or appropriate frame

**Save your RViz config** for future use.

### Step 7: Process Sensors with Python (15 min)

Complete the sensor analyzer script:

```bash
# Run the solution to see expected behavior
python3 solution/sensor_analyzer.py

# Then implement your version in starter/
python3 starter/sensor_analyzer.py
```

The script should:
- Subscribe to all three sensor topics
- Log sensor data statistics
- Detect when sensors are publishing

---

## Deliverables

1. **Working Sensor Configuration**: Robot model with camera, LiDAR, and IMU
2. **RViz Configuration**: Saved `.rviz` file showing all sensors
3. **Sensor Analyzer Script**: Python node that processes all sensor data

---

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
[PASS] Sensor configuration file exists
[PASS] Solution script has valid syntax
[PASS] Solution handles Image messages
[PASS] Solution handles LaserScan messages
[PASS] Solution handles Imu messages
Lab 3 Complete!
```

---

## Reflection Questions

1. **Why add noise to simulated sensors?**
   - How does noise help with sim-to-real transfer?

2. **What happens if camera and LiDAR frames are misaligned?**
   - Consider sensor fusion scenarios

3. **How would you increase LiDAR resolution vs range?**
   - Trade-offs in sensor configuration

---

## Bonus Challenges

1. **Add Depth Camera**: Configure a depth camera with point cloud output
2. **Sensor Fusion**: Combine camera and LiDAR data in a single node
3. **Custom Noise Model**: Implement bias drift for the IMU

---

## Troubleshooting

### Topics not appearing
```bash
# Check Gazebo sensor plugins are loaded
gz topic -l

# Verify bridge is running
ros2 node list | grep bridge
```

### RViz shows no data
- Check that Fixed Frame matches sensor frame
- Verify topic names in displays
- Check that simulation is running (not paused)

### Sensor data looks wrong
- Verify sensor pose in SDF
- Check coordinate frame conventions
- Ensure physics is stepping (real-time factor > 0)

---

## Next Steps

After completing this lab:
- Review Module 2 quiz in Lesson 3
- Proceed to Module 3: Perception algorithms
- Use your sensor-equipped world for navigation testing
