# Lab 5: Perception-to-Control Integration

**Module**: 3 - Perception & Sensors
**Duration**: 150 minutes
**Difficulty**: Advanced

## Objective

Build a complete perception-to-action pipeline that detects objects, tracks them, and generates robot control commands. This capstone lab integrates all Module 3 concepts into a working target-following system.

## Prerequisites

- Completed Labs 1-4
- Understanding of visual servoing concepts
- ROS 2 geometry_msgs for velocity commands

## Learning Outcomes

By completing this lab, you will:
1. Integrate detection, tracking, and control in a single system
2. Implement simple visual servoing
3. Handle edge cases (lost target, multiple targets)
4. Test closed-loop perception-control behavior

---

## Setup

### 1. Verify Dependencies

```bash
# Check all required packages
python3 -c "import cv2, rclpy, numpy; print('Core packages OK')"
ros2 pkg list | grep -E "(vision_msgs|geometry_msgs)"
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src/labs/module-03/lab-05-perception-integration
```

### 3. Launch Robot Simulation

```bash
# Terminal 1: Launch robot with camera
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Spawn target objects
ros2 run gazebo_ros spawn_entity.py -entity target -file target.sdf
```

---

## Lab Steps

### Step 1: Detection Integration (30 min)

Complete TODO 1 - integrate object detector:

```python
class PerceptionController(Node):
    def __init__(self):
        super().__init__('perception_controller')

        # Detection subscriber
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10)

        # Velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Target class to follow
        self.target_class = 'person'
        self.current_target = None
```

### Step 2: Implement Simple Tracker (30 min)

Complete TODO 2 - track target across frames:

```python
class SimpleTracker:
    def __init__(self, max_distance=100):
        self.track_id = None
        self.last_bbox = None
        self.max_distance = max_distance
        self.frames_lost = 0

    def update(self, detections, target_class):
        """Find and track target object."""
        candidates = [d for d in detections
                     if d.class_id == target_class]

        if not candidates:
            self.frames_lost += 1
            return None

        if self.last_bbox is None:
            # Start new track with highest confidence
            best = max(candidates, key=lambda d: d.confidence)
            self.last_bbox = best.bbox
            self.frames_lost = 0
            return best

        # Find closest to last position
        best_match = None
        best_dist = self.max_distance

        for det in candidates:
            dist = self.bbox_distance(self.last_bbox, det.bbox)
            if dist < best_dist:
                best_dist = dist
                best_match = det

        if best_match:
            self.last_bbox = best_match.bbox
            self.frames_lost = 0
            return best_match

        self.frames_lost += 1
        return None
```

### Step 3: Visual Servoing Controller (40 min)

Complete TODO 3 - generate velocity commands:

```python
def compute_velocity(self, target_bbox, image_width=640, image_height=480):
    """Compute velocity commands to follow target."""
    cmd = Twist()

    if target_bbox is None:
        # Search behavior when target lost
        cmd.angular.z = 0.3  # Rotate to search
        return cmd

    # Get target center
    cx = target_bbox.center.x
    cy = target_bbox.center.y
    area = target_bbox.size_x * target_bbox.size_y

    # Angular velocity: turn toward target
    # Positive error = target on right = turn right (negative z)
    x_error = cx - image_width / 2
    cmd.angular.z = -0.003 * x_error

    # Linear velocity: maintain distance via bbox area
    desired_area = 25000  # Target area when at desired distance
    area_error = desired_area - area
    cmd.linear.x = 0.0001 * area_error

    # Clamp velocities
    cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
    cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

    return cmd
```

### Step 4: State Machine (30 min)

Complete TODO 4 - handle different states:

```python
class ControlState:
    SEARCHING = 'searching'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    STOPPED = 'stopped'

def update_state(self):
    """Update state based on tracking status."""
    if self.tracker.frames_lost > 30:
        self.state = ControlState.SEARCHING
    elif self.current_target is not None:
        area = (self.current_target.bbox.size_x *
                self.current_target.bbox.size_y)
        if area > 40000:  # Very close
            self.state = ControlState.STOPPED
        elif area > 20000:  # Close enough
            self.state = ControlState.APPROACHING
        else:
            self.state = ControlState.TRACKING
```

### Step 5: Test and Tune (20 min)

```bash
# Run your implementation
python3 starter/perception_controller.py

# In another terminal, move the target
ros2 topic pub /target/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# Observe robot following behavior
```

---

## Deliverables

1. **Working Code**: Complete `starter/perception_controller.py`
2. **Demo Video**: Record robot following target
3. **Parameter Tuning**: Document optimal gain values
4. **Reflection Answers**: Complete questions below

---

## Reflection Questions

1. **What happens when the target moves out of view?**
   - Observe and describe the search behavior

2. **How do the control gains affect following behavior?**
   - Test with 2x and 0.5x gains, describe differences

3. **What causes oscillation during following?**
   - Consider: control frequency, sensor latency, gain tuning

4. **How would you extend this for multiple targets?**
   - Think about: priority, switching logic, target selection

5. **What safety features should a real system have?**
   - Consider: collision avoidance, velocity limits, emergency stop

---

## Validation

```bash
python3 validate.py
```

Expected output:
```
[PASS] perception_controller.py exists
[PASS] Valid Python syntax
[PASS] Detection subscription present
[PASS] Velocity publisher present
[PASS] Visual servoing logic implemented
[PASS] State machine implemented
Lab 5 Complete! Module 3 Finished!
```

---

## Extension Challenges

1. **PID Control**: Replace P controller with PID for smoother motion
2. **Obstacle Avoidance**: Use LiDAR to avoid obstacles while following
3. **Multi-Target**: Track and switch between multiple targets
4. **Gesture Control**: Respond to detected hand gestures

---

## Congratulations!

You've completed Module 3: Perception & Sensors. You can now:
- Process camera images with OpenCV
- Work with LiDAR point clouds
- Fuse multiple sensor streams
- Detect and track objects
- Connect perception to robot control

**Next**: Module 4 - Motion Planning & Control
