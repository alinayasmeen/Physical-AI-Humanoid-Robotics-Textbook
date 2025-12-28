# Lab 3: ROS 2 Services

**Duration**: 90 minutes
**Lesson**: [Lesson 3: Python Agents with rclpy](/docs/module-01-ros2/lesson-03-rclpy-agents)
**Prerequisites**: Lab 1 and Lab 2 completed

## Objective

Implement request-response communication using ROS 2 services. Create a service server that handles robot state queries and a client that sends requests.

## Learning Outcomes

After completing this lab, you will be able to:
- Create service servers using `create_service()`
- Create service clients using `create_client()`
- Use standard service types (`std_srvs`)
- Handle synchronous request-response patterns

## Architecture

```
┌─────────────────┐  request   ┌─────────────────┐
│ state_client    │ ─────────▶│ state_server    │
│ (Client)        │            │ (Server)        │
│                 │ ◀───────── │                 │
└─────────────────┘  response  └─────────────────┘
         │                              │
         └──────── /get_state ──────────┘
```

## Setup

### 1. Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python services_demo \
    --dependencies rclpy std_srvs
cd services_demo/services_demo
```

## Tasks

### Task 1: Create Service Server (35 min)

Create `state_server.py` that:
- Creates a node named `'robot_state_server'`
- Provides a service `/get_state` using `SetBool` type
- When `data=True`: Returns robot's "running" state
- When `data=False`: Returns robot's "stopped" state
- Tracks internal state and logs transitions

**Starter code provided in `starter/state_server.py`**

### Task 2: Create Service Client (35 min)

Create `state_client.py` that:
- Creates a node named `'robot_state_client'`
- Calls `/get_state` service
- Accepts command-line argument for request value
- Logs the response

**Starter code provided in `starter/state_client.py`**

### Task 3: Configure and Test (20 min)

Edit `setup.py`:

```python
entry_points={
    'console_scripts': [
        'state_server = services_demo.state_server:main',
        'state_client = services_demo.state_client:main',
    ],
},
```

Build and test:

```bash
cd ~/ros2_ws
colcon build --packages-select services_demo
source install/setup.bash

# Terminal 1: Run server
ros2 run services_demo state_server

# Terminal 2: Call service via CLI
ros2 service call /get_state std_srvs/srv/SetBool "{data: true}"

# Terminal 3: Run client
ros2 run services_demo state_client --ros-args -p request:=true
```

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
✅ Server node syntax valid
✅ Client node syntax valid
✅ Required service patterns found
✅ All checks passed!
```

## Expected Results

### Server Output
```
[INFO] Robot state server ready
[INFO] Received request: data=True
[INFO] Responding: success=True, message='Robot is running'
```

### Client Output
```
[INFO] Sending request: data=True
[INFO] Response: success=True, message='Robot is running'
```

### CLI Verification
```bash
$ ros2 service list
/get_state

$ ros2 service type /get_state
std_srvs/srv/SetBool
```

## Understanding the Code

### SetBool Service Type

The `std_srvs/srv/SetBool` service has:

**Request:**
```
bool data
```

**Response:**
```
bool success
string message
```

### Synchronous vs Asynchronous Clients

This lab uses asynchronous client calls (`call_async`). For simple scripts, you could use synchronous calls, but async is preferred in node callbacks to avoid blocking.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Service not found | Ensure server is running first |
| Timeout error | Check service name matches exactly |
| Import error | Install std_srvs: `sudo apt install ros-humble-std-srvs` |

## Extensions (Optional)

1. Add multiple services (start, stop, reset)
2. Implement service with custom message type
3. Add timeout handling in client
4. Create action-based alternative for comparison

## Next Steps

Continue to [Lab 4: URDF Basics](/labs/module-01/lab-04-urdf-basics) to learn robot modeling.
