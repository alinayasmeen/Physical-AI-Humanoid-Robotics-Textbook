# Mini-Project 1: Robot Talker System

**Duration**: 2-3 hours
**Module**: Module 1 - The Robotic Nervous System
**Prerequisites**: Labs 1-3 completed

## Overview

Build a multi-robot communication system where robots exchange status information using ROS 2 topics and services. This project integrates all the communication concepts from Module 1.

## Objectives

Create a system with:
- Multiple robot status publishers
- A dashboard aggregator node
- Status query service
- Custom message handling

## System Architecture

```
┌──────────────┐     /robot_1/status     ┌──────────────┐
│  Robot 1     │ ─────────────────────▶  │              │
│  Status Node │                         │   Dashboard  │
└──────────────┘                         │   Node       │
                                         │              │
┌──────────────┐     /robot_2/status     │   Subscribes │
│  Robot 2     │ ─────────────────────▶  │   to all     │
│  Status Node │                         │   robots     │
└──────────────┘                         │              │
                                         │   Provides   │
┌──────────────┐     /robot_3/status     │   /query_    │
│  Robot 3     │ ─────────────────────▶  │   status     │
│  Status Node │                         │   service    │
└──────────────┘                         └──────────────┘
```

## Requirements

### 1. Robot Status Node

Each robot publishes status at 1 Hz:

```python
# Message content (use std_msgs/String with JSON)
{
    "robot_id": "robot_1",
    "status": "active",      # active, idle, error
    "battery": 85,           # 0-100
    "position": {"x": 1.5, "y": 2.0},
    "timestamp": "2024-01-15T10:30:00"
}
```

Requirements:
- Accept robot_id as ROS parameter
- Simulate battery drain (decrease 1% every 10 seconds)
- Change status to "error" when battery < 20%

### 2. Dashboard Node

Aggregates all robot statuses:

Requirements:
- Subscribe to `/robot_*/status` (use wildcard or multiple subscriptions)
- Maintain dictionary of latest status for each robot
- Log summary every 5 seconds
- Detect robots that haven't reported in 10+ seconds (mark as "offline")

### 3. Query Service

Service to query individual robot status:

```python
# Request
robot_id: "robot_1"

# Response
found: True
status_json: "{ ... full status JSON ... }"
```

## Getting Started

### Setup Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_talker \
    --dependencies rclpy std_msgs std_srvs
```

### Starter Files

Use the starter code in `starter/` directory:
- `robot_node.py` - Robot status publisher template
- `dashboard_node.py` - Dashboard subscriber template
- `query_service.py` - Status query service template

### Running the System

```bash
# Terminal 1: Launch 3 robots
ros2 run robot_talker robot_node --ros-args -p robot_id:=robot_1
ros2 run robot_talker robot_node --ros-args -p robot_id:=robot_2
ros2 run robot_talker robot_node --ros-args -p robot_id:=robot_3

# Terminal 2: Dashboard
ros2 run robot_talker dashboard_node

# Terminal 3: Query
ros2 service call /query_status robot_talker/srv/QueryStatus "{robot_id: 'robot_1'}"
```

## Evaluation Criteria

| Criterion | Points |
|-----------|--------|
| Robot nodes publish correct format | 20 |
| Dashboard aggregates all robots | 20 |
| Offline detection works | 15 |
| Query service returns correct data | 20 |
| Clean code and error handling | 15 |
| Documentation/comments | 10 |
| **Total** | **100** |

## Extensions (Bonus)

1. **Launch File**: Create launch file to start all nodes
2. **Custom Message**: Define `RobotStatus.msg` instead of JSON strings
3. **Action Server**: Add action for "send robot to charging station"
4. **Visualization**: Create simple terminal UI for dashboard
5. **Multi-machine**: Run robots on different machines (use ROS_DOMAIN_ID)

## Hints

- Use `json.dumps()` and `json.loads()` for message serialization
- `self.declare_parameter()` for ROS parameters
- `time.time()` for timestamps (convert to ISO format)
- Dictionary comprehension for status aggregation

## Solution

See `solution/` directory for complete implementation.

## Reflection Questions

1. Why use topics instead of services for robot status?
2. What happens if the dashboard node crashes and restarts?
3. How would you scale this to 100 robots?
4. What QoS settings would improve reliability?
