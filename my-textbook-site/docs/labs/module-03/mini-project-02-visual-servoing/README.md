# Mini-Project 2: Visual Servoing Robot

**Module**: 3 - Perception & Sensors
**Duration**: 3-4 hours
**Difficulty**: Advanced

## Overview

Implement a visual servoing system that enables a robot to autonomously follow and interact with detected objects. The robot uses camera feedback to adjust its motion in real-time, demonstrating closed-loop perception-action control.

## Objectives

By completing this mini-project, you will:

1. Implement image-based visual servoing (IBVS)
2. Design a control system with appropriate gains
3. Handle edge cases (lost target, multiple targets, obstacles)
4. Integrate safety features for real-world deployment

## Prerequisites

- Completed Labs 1-5 of Module 3
- Understanding of PID control concepts
- Familiarity with ROS 2 Twist messages

## System Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Camera    │────▶│  Detector   │────▶│  Controller │
└─────────────┘     └─────────────┘     └─────────────┘
                                               │
                    ┌─────────────┐            │
                    │  Safety     │◀───────────┤
                    │  Monitor    │            │
                    └─────────────┘            │
                           │                   │
                           ▼                   ▼
                    ┌─────────────┐     ┌─────────────┐
                    │  Override   │────▶│  Robot      │
                    └─────────────┘     └─────────────┘
```

## Requirements

### Functional Requirements

1. **Target Detection**
   - Detect target object using color or shape
   - Track target across frames
   - Handle temporary occlusions

2. **Visual Servoing**
   - Center target in camera view (angular control)
   - Maintain desired distance (linear control)
   - Smooth motion without oscillation

3. **Behavior States**
   - SEARCHING: Rotate to find target
   - TRACKING: Follow moving target
   - APPROACHING: Move toward stationary target
   - HOLDING: Maintain position at desired distance
   - LOST: Target lost, return to search

4. **Safety Features**
   - Maximum velocity limits
   - Emergency stop on obstacle detection
   - Watchdog timer for sensor failures
   - Graceful degradation

### Non-Functional Requirements

- Control loop at minimum 10 Hz
- Response time under 200ms
- Stable following within 10cm accuracy

## Getting Started

### 1. Project Structure

```
mini-project-02-visual-servoing/
├── starter/
│   ├── visual_servo_node.py    # Main control node
│   ├── target_detector.py      # Target detection
│   ├── servo_controller.py     # Control algorithms
│   └── safety_monitor.py       # Safety systems
└── solution/
    └── ...
```

### 2. Control Algorithm

The visual servo controller uses proportional control:

```python
# Angular velocity (turn toward target)
error_x = target_x - image_center_x
omega_z = -K_angular * error_x

# Linear velocity (maintain distance)
error_area = desired_area - target_area
v_x = K_linear * error_area
```

### 3. State Machine

Implement the following state transitions:

```
SEARCHING ──[target found]──▶ TRACKING
TRACKING ──[target close]──▶ APPROACHING
TRACKING ──[target lost]──▶ LOST
APPROACHING ──[at distance]──▶ HOLDING
HOLDING ──[target moved]──▶ TRACKING
LOST ──[timeout]──▶ SEARCHING
```

## Implementation Tasks

### Task 1: Target Detection (45 min)

Complete `target_detector.py`:
- Implement color-based detection
- Add confidence filtering
- Return bounding box and centroid

### Task 2: Servo Controller (60 min)

Complete `servo_controller.py`:
- Implement proportional control
- Add velocity smoothing
- Handle edge cases

### Task 3: State Machine (45 min)

Complete `visual_servo_node.py`:
- Implement state transitions
- Add timeout handling
- Log state changes

### Task 4: Safety Monitor (30 min)

Complete `safety_monitor.py`:
- Velocity limiting
- Obstacle detection integration
- Emergency stop

### Task 5: Integration Testing (60 min)

- Test in simulation
- Tune control gains
- Verify safety features

## Validation Criteria

Run the validation script:

```bash
python3 validate.py
```

Expected checks:
- [ ] Robot centers target within 50 pixels
- [ ] Smooth velocity commands (no oscillation)
- [ ] State transitions work correctly
- [ ] Safety limits enforced
- [ ] Handles target loss gracefully
- [ ] Control loop runs at 10+ Hz

## Deliverables

1. **Working System**: Complete implementation
2. **Demo Video**: 60-second recording showing:
   - Target acquisition
   - Following behavior
   - Loss and recovery
3. **Tuning Report**: Document gain values and tuning process
4. **Safety Analysis**: Describe safety features implemented

## Extension Challenges

1. **PID Control**: Replace P with full PID controller
2. **Path Planning**: Plan path around obstacles to reach target
3. **Multiple Targets**: Prioritize and switch between targets
4. **Gesture Recognition**: Respond to hand gestures
5. **Formation Control**: Multiple robots following a leader

## Tuning Guide

### Angular Control (K_angular)
- Too low: Slow response, target drifts
- Too high: Oscillation, overshoot
- Start: 0.002, adjust by 0.001

### Linear Control (K_linear)
- Too low: Slow approach, undershoot
- Too high: Aggressive motion, overshoot
- Start: 0.0001, adjust by 0.00005

### Velocity Smoothing
- Use exponential moving average
- Alpha = 0.3 for smooth motion
- Alpha = 0.7 for responsive motion

## Resources

- [Visual Servoing Tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-ibvs.html)
- [ROS 2 Control Concepts](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Control.html)
- [PID Tuning Guide](https://pidtuner.com/)
