# Mini-Project 2: Arm Controller

**Duration**: 3-4 hours
**Module**: Module 1 - The Robotic Nervous System
**Prerequisites**: Labs 1-4 completed

## Overview

Build a keyboard-controlled arm system using your 2-DOF URDF model from Lab 4. This project combines URDF modeling, ROS 2 control, and interactive input handling.

## Objectives

Create a complete arm control system:
- Load URDF model
- Keyboard teleop for joint control
- Joint limit enforcement
- Emergency stop functionality

## System Architecture

```
┌──────────────┐    /joint_commands    ┌──────────────┐
│   Keyboard   │ ──────────────────▶   │   Joint      │
│   Teleop     │                       │   Controller │
│   Node       │                       │   Node       │
└──────────────┘                       └──────────────┘
                                              │
                                              ▼
                                       /joint_states
                                              │
                                              ▼
                                       ┌──────────────┐
                                       │   RViz       │
                                       │   Display    │
                                       └──────────────┘
```

## Requirements

### 1. Joint Controller Node

Receives commands and publishes joint states:

```python
# Subscribe to: /joint_commands (Float64MultiArray)
# Publish to: /joint_states (JointState)
```

Requirements:
- Enforce joint limits from URDF
- Smooth motion (interpolate between positions)
- Publish joint states at 30 Hz
- Emergency stop zeros all velocities

### 2. Keyboard Teleop Node

Read keyboard input and send commands:

| Key | Action |
|-----|--------|
| `w` | Shoulder up (+0.1 rad) |
| `s` | Shoulder down (-0.1 rad) |
| `a` | Elbow up (+0.1 rad) |
| `d` | Elbow down (-0.1 rad) |
| `h` | Home position (all zeros) |
| `e` | Emergency stop |
| `q` | Quit |

### 3. Emergency Stop Service

Service to halt all motion:

```python
# Service: /emergency_stop (std_srvs/Trigger)
# Sets all joint velocities to zero
# Returns success status
```

### 4. Launch File

Launch entire system with one command:

```bash
ros2 launch arm_controller display.launch.py
```

Should start:
- Joint controller node
- Keyboard teleop node
- RViz with robot model
- robot_state_publisher

## Getting Started

### Setup Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python arm_controller \
    --dependencies rclpy std_msgs sensor_msgs std_srvs
mkdir -p arm_controller/urdf arm_controller/launch arm_controller/rviz
```

### Copy URDF

Copy your `simple_arm.urdf` from Lab 4:

```bash
cp ~/ros2_ws/src/urdf_demo/urdf/simple_arm.urdf \
   ~/ros2_ws/src/arm_controller/urdf/
```

### Starter Files

Use the starter code in `starter/` directory:
- `joint_controller.py` - Joint state publisher
- `keyboard_teleop.py` - Keyboard input handler
- `display.launch.py` - Launch file template

## Expected Behavior

1. Launch shows arm in RViz at home position
2. Pressing keys moves joints within limits
3. Motion is smooth (no jerky movements)
4. Emergency stop immediately halts motion
5. Joint limits are respected (visual feedback in terminal)

## Evaluation Criteria

| Criterion | Points |
|-----------|--------|
| Joint controller publishes valid JointState | 20 |
| Keyboard teleop sends correct commands | 20 |
| Joint limits enforced | 15 |
| Emergency stop works | 15 |
| Launch file starts all nodes | 15 |
| Smooth motion interpolation | 10 |
| Documentation | 5 |
| **Total** | **100** |

## Extensions (Bonus)

1. **Trajectory Recording**: Record and playback arm movements
2. **Gamepad Support**: Use joystick instead of keyboard
3. **End-Effector Control**: Control gripper position in Cartesian space
4. **Collision Warning**: Detect self-collision from joint positions
5. **GUI**: Create simple GUI with sliders for each joint

## Hints

### Reading Keyboard Input

```python
import sys
import select
import termios
import tty

def get_key():
    """Non-blocking keyboard read."""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
```

### Smooth Motion

```python
def interpolate(current, target, step=0.05):
    """Move current toward target by step amount."""
    if abs(target - current) < step:
        return target
    elif target > current:
        return current + step
    else:
        return current - step
```

### Publishing JointState

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ['shoulder', 'elbow']
msg.position = [shoulder_pos, elbow_pos]
msg.velocity = [0.0, 0.0]
msg.effort = [0.0, 0.0]
```

## Solution

See `solution/` directory for complete implementation.

## Reflection Questions

1. Why is smooth interpolation important for real robots?
2. What would happen without joint limit enforcement?
3. How would you add gravity compensation?
4. What's the difference between position and velocity control?
