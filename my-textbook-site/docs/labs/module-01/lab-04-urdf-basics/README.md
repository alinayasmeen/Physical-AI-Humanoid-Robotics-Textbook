# Lab 4: URDF Basics

**Duration**: 120 minutes
**Lesson**: [Lesson 4: Humanoid URDF Fundamentals](/docs/module-01-ros2/lesson-04-urdf-fundamentals)
**Prerequisites**: Labs 1-3 completed, ROS 2 workspace configured

## Objective

Create a 2-DOF robotic arm URDF model and visualize it in RViz. This lab teaches the fundamentals of robot modeling that scale to complex humanoid systems.

## Learning Outcomes

After completing this lab, you will be able to:
- Write valid URDF XML files
- Define links with visual and collision geometry
- Create joints with proper limits and axes
- Visualize robot models in RViz
- Debug URDF parsing errors

## Architecture

```
2-DOF Arm Structure:

    [base_link]
         |
    (shoulder) - revolute joint, Y-axis
         |
    [upper_arm]
         |
     (elbow) - revolute joint, Y-axis
         |
    [lower_arm]
         |
    [end_effector]
```

## Setup

### 1. Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python urdf_demo \
    --dependencies rclpy urdf launch launch_ros
cd urdf_demo
mkdir -p urdf launch rviz
```

### 2. Install Visualization Dependencies

```bash
sudo apt install ros-humble-rviz2 ros-humble-joint-state-publisher-gui
```

## Tasks

### Task 1: Create Base URDF (30 min)

Create `urdf/simple_arm.urdf` with:
- `base_link`: A box (0.1 x 0.1 x 0.05 m)
- Material color: Gray (rgba: 0.5 0.5 0.5 1.0)

**Starter code provided in `starter/simple_arm.urdf`**

### Task 2: Add Shoulder Joint and Upper Arm (30 min)

Add to your URDF:
- `shoulder` joint: Revolute, Y-axis, limits ±90°
- `upper_arm` link: Cylinder (radius 0.02m, length 0.3m)

### Task 3: Add Elbow Joint and Lower Arm (30 min)

Add to your URDF:
- `elbow` joint: Revolute, Y-axis, limits 0° to 135°
- `lower_arm` link: Cylinder (radius 0.015m, length 0.2m)
- `end_effector` link: Sphere (radius 0.02m) - fixed joint

### Task 4: Create Launch File and Visualize (30 min)

Create `launch/display.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('urdf_demo')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf.rviz')]
        ),
    ])
```

Update `setup.py` to include data files:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/urdf', ['urdf/simple_arm.urdf']),
    ('share/' + package_name + '/launch', ['launch/display.launch.py']),
    ('share/' + package_name + '/rviz', ['rviz/urdf.rviz']),
],
```

Build and launch:

```bash
cd ~/ros2_ws
colcon build --packages-select urdf_demo
source install/setup.bash
ros2 launch urdf_demo display.launch.py
```

## Validation

Run the validation script:

```bash
python3 validate.py urdf/simple_arm.urdf
```

Expected output:
```
✅ URDF file parses successfully
✅ Found 4 links: base_link, upper_arm, lower_arm, end_effector
✅ Found 3 joints: shoulder, elbow, wrist
✅ Joint limits are valid
✅ All checks passed!
```

## Expected Results

1. RViz displays the arm model
2. Joint State Publisher GUI shows sliders for shoulder and elbow
3. Moving sliders animates the arm
4. TF tree shows correct parent-child relationships

### RViz Configuration

In RViz:
1. Add `RobotModel` display
2. Set Fixed Frame to `base_link`
3. Add `TF` display to see frames

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Invalid URDF" | Check XML syntax and joint parent/child references |
| Model not visible | Ensure Fixed Frame is set to `base_link` |
| Joints don't move | Verify joint types are `revolute`, not `fixed` |
| Launch fails | Check all data_files paths in setup.py |

## Understanding the Model

### Coordinate Frames

Each joint creates a new coordinate frame:
- `base_link` → origin of robot
- `shoulder` → rotates upper_arm around Y
- `elbow` → rotates lower_arm around Y

### Checking with TF

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF showing the transform tree.

## Extensions (Optional)

1. Add colors to differentiate arm segments
2. Add inertial properties for simulation
3. Create a 6-DOF arm (add shoulder pitch/roll, wrist)
4. Add collision geometry

## Complete URDF Reference

See `solution/simple_arm.urdf` for the complete working URDF.

## Next Steps

Continue to [Lesson 5: Simulation, Practice & Review](/docs/module-01-ros2/lesson-05-simulation-review) to learn how to connect URDF with simulation.
