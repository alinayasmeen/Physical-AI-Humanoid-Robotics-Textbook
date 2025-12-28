# Lab 1: Hello ROS 2

**Duration**: 60 minutes
**Lesson**: [Lesson 3: Python Agents with rclpy](/docs/module-01-ros2/lesson-03-rclpy-agents)
**Prerequisites**: ROS 2 Humble installed, Python 3.10+

## Objective

Create and run your first ROS 2 node. By the end of this lab, you'll understand the basic structure of a ROS 2 Python node and how to verify it's running correctly.

## Learning Outcomes

After completing this lab, you will be able to:
- Set up a ROS 2 workspace
- Create a ROS 2 Python package
- Write a minimal node that logs messages
- Verify the node is running using ROS 2 CLI tools

## Setup

### 1. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Create Package

```bash
ros2 pkg create --build-type ament_python hello_ros2 --dependencies rclpy
```

### 3. Navigate to Package

```bash
cd hello_ros2/hello_ros2
```

## Tasks

### Task 1: Write the Node (20 min)

Create a file `hello_node.py` in the `hello_ros2/hello_ros2/` directory.

**Requirements:**
- Create a class `HelloNode` that inherits from `Node`
- The node should be named `'hello_ros2_node'`
- Log "Hello, ROS 2!" when the node starts
- Use a timer to log "Still running..." every 2 seconds

**Starter code is provided in `starter/hello_node.py`**

### Task 2: Configure the Package (10 min)

Edit `setup.py` to add the entry point:

```python
entry_points={
    'console_scripts': [
        'hello_node = hello_ros2.hello_node:main',
    ],
},
```

### Task 3: Build and Run (15 min)

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select hello_ros2

# Source the workspace
source install/setup.bash

# Run the node
ros2 run hello_ros2 hello_node
```

### Task 4: Verify (15 min)

In a new terminal:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# List running nodes
ros2 node list

# Check node info
ros2 node info /hello_ros2_node
```

## Validation

Run the validation script to verify your implementation:

```bash
python3 validate.py
```

Expected output:
```
✅ Node 'hello_ros2_node' found
✅ Node is publishing logs
✅ All checks passed!
```

## Expected Results

1. `ros2 node list` shows `/hello_ros2_node`
2. Terminal displays "Hello, ROS 2!" followed by "Still running..." every 2 seconds
3. `ros2 node info /hello_ros2_node` shows node details

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Node not found | Ensure you sourced the workspace |
| Import error | Check rclpy is installed: `pip3 install rclpy` |
| Permission denied | Make the Python file executable: `chmod +x hello_node.py` |

## Extensions (Optional)

1. Add a parameter for the timer period
2. Log the current time with each message
3. Create a second node and run them simultaneously

## Next Steps

Continue to [Lab 2: Publisher-Subscriber](/labs/module-01/lab-02-pubsub) to learn topic-based communication.
