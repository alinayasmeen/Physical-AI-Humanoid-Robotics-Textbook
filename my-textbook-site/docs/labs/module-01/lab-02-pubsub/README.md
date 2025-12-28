# Lab 2: Publisher-Subscriber Communication

**Duration**: 90 minutes
**Lesson**: [Lesson 3: Python Agents with rclpy](/docs/module-01-ros2/lesson-03-rclpy-agents)
**Prerequisites**: Lab 1 completed, ROS 2 workspace configured

## Objective

Implement topic-based communication between two nodes. Create a publisher that sends velocity commands and a subscriber that receives and logs them.

## Learning Outcomes

After completing this lab, you will be able to:
- Create publisher nodes using `create_publisher()`
- Create subscriber nodes using `create_subscription()`
- Use `geometry_msgs/Twist` for velocity commands
- Verify message exchange using ROS 2 CLI tools

## Architecture

```
┌─────────────────┐    /cmd_vel    ┌─────────────────┐
│ velocity_pub    │ ──────────────▶│ velocity_sub    │
│ (Publisher)     │   Twist msg    │ (Subscriber)    │
└─────────────────┘                └─────────────────┘
```

## Setup

### 1. Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python pubsub_demo \
    --dependencies rclpy geometry_msgs
cd pubsub_demo/pubsub_demo
```

## Tasks

### Task 1: Create Publisher (30 min)

Create `velocity_publisher.py` that:
- Creates a node named `'velocity_publisher'`
- Publishes to topic `/cmd_vel` with message type `Twist`
- Uses a 10 Hz timer (0.1 second period)
- Sends `linear.x = 0.5` and `angular.z = 0.1`

**Starter code provided in `starter/velocity_publisher.py`**

### Task 2: Create Subscriber (30 min)

Create `velocity_subscriber.py` that:
- Creates a node named `'velocity_subscriber'`
- Subscribes to topic `/cmd_vel`
- Logs received linear and angular velocities

**Starter code provided in `starter/velocity_subscriber.py`**

### Task 3: Configure Package (10 min)

Edit `setup.py` entry points:

```python
entry_points={
    'console_scripts': [
        'velocity_pub = pubsub_demo.velocity_publisher:main',
        'velocity_sub = pubsub_demo.velocity_subscriber:main',
    ],
},
```

### Task 4: Build and Test (20 min)

```bash
cd ~/ros2_ws
colcon build --packages-select pubsub_demo
source install/setup.bash

# Terminal 1: Run publisher
ros2 run pubsub_demo velocity_pub

# Terminal 2: Run subscriber
ros2 run pubsub_demo velocity_sub

# Terminal 3: Verify
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
✅ Publisher node syntax valid
✅ Subscriber node syntax valid
✅ Required ROS 2 patterns found
✅ All checks passed!
```

## Expected Results

1. Publisher outputs: "Publishing: linear=0.50, angular=0.10"
2. Subscriber outputs: "Received: linear=0.50, angular=0.10"
3. `ros2 topic hz /cmd_vel` shows ~10 Hz
4. `ros2 topic echo /cmd_vel` displays Twist messages

## Understanding the Code

### Message Types

The `Twist` message from `geometry_msgs` contains:
```
Vector3 linear   # x, y, z linear velocities
Vector3 angular  # x, y, z angular velocities
```

For ground robots:
- `linear.x` = forward/backward speed
- `angular.z` = rotation speed (yaw)

### QoS (Quality of Service)

The `10` parameter in `create_publisher()` sets queue depth. This affects:
- **Reliability**: How many messages to buffer
- **Latency**: Trade-off with real-time performance

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Subscriber doesn't receive | Check topic names match exactly |
| Import error | Install geometry_msgs: `sudo apt install ros-humble-geometry-msgs` |
| Messages delayed | Reduce queue depth or check network |

## Extensions (Optional)

1. Add a parameter to control velocity values
2. Implement a velocity ramp (gradual acceleration)
3. Add a third node that transforms velocities

## Next Steps

Continue to [Lab 3: Services](/labs/module-01/lab-03-services) to learn request-response patterns.
