# Lab 1: Physics Experiments in Gazebo

**Module**: 2 - The Digital Twin
**Duration**: 60 minutes
**Difficulty**: Beginner

## Objective

Observe and analyze robot behavior under different physics parameter settings in Gazebo simulation. By modifying gravity, friction, and mass properties, you'll develop intuition for how physics engines affect robot motion.

## Prerequisites

- Completed Module 1 (ROS 2 fundamentals)
- Completed Module 2 Lesson 1 (Digital Twin concepts)
- Ubuntu 22.04 with ROS 2 Humble installed
- Gazebo Fortress installed (`ros-humble-ros-gz`)

## Learning Outcomes

By completing this lab, you will:
1. Launch Gazebo with a simple robot model
2. Observe physics behavior under default settings
3. Modify gravity and observe changes
4. Modify friction coefficients and observe changes
5. Document observations for later reference

---

## Setup

### 1. Verify Installation

```bash
# Check Gazebo is installed
gz sim --version

# Check ros_gz bridge
ros2 pkg list | grep ros_gz
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src  # or your workspace
cp -r /path/to/labs/module-02/lab-01-physics-experiments .
cd lab-01-physics-experiments
```

---

## Lab Steps

### Step 1: Launch Default Simulation (10 min)

Launch Gazebo with a simple falling cube:

```bash
# Terminal 1: Launch Gazebo with default physics
gz sim -r starter/falling_cube.sdf
```

**Observe**:
- How fast does the cube fall?
- Does it bounce when hitting the ground?
- What happens when you nudge it (use Gazebo GUI to apply force)?

**Record your observations** in the table below.

### Step 2: Modify Gravity (15 min)

Edit the world file to change gravity settings:

```bash
# Open the starter file
nano starter/falling_cube.sdf
```

Find the `<gravity>` tag and try these values:

| Experiment | Gravity (m/s²) | Expected Behavior |
|------------|----------------|-------------------|
| Earth | 0 0 -9.8 | Normal fall speed |
| Moon | 0 0 -1.62 | Slow, floating fall |
| Mars | 0 0 -3.71 | Moderate fall speed |
| Jupiter | 0 0 -24.79 | Fast, heavy fall |
| Zero-G | 0 0 0 | Object floats |

For each setting:
1. Save the file
2. Restart Gazebo: `gz sim -r starter/falling_cube.sdf`
3. Observe and record behavior

### Step 3: Modify Friction (15 min)

Change the friction coefficient (`mu`) for the ground plane:

```xml
<!-- In the ground_plane model -->
<surface>
  <friction>
    <ode>
      <mu>0.1</mu>   <!-- Low friction (ice) -->
      <mu2>0.1</mu2>
    </ode>
  </friction>
</surface>
```

Test these friction values:

| Surface Type | mu Value | Expected Behavior |
|--------------|----------|-------------------|
| Ice | 0.1 | Object slides easily |
| Wood | 0.4 | Moderate sliding |
| Rubber on concrete | 1.0 | High grip, minimal sliding |
| Sticky | 2.0 | Almost no sliding |

### Step 4: Run Physics Observer Script (10 min)

Use the provided Python script to programmatically observe physics:

```bash
# Terminal 1: Launch Gazebo
gz sim -r starter/falling_cube.sdf

# Terminal 2: Run observer
python3 starter/physics_observer.py
```

The script will:
- Subscribe to the cube's pose
- Calculate fall velocity
- Report physics observations

### Step 5: Compare with Solution (10 min)

Review the solution files:

```bash
# Compare your observations with expected results
cat solution/expected_observations.md

# Run the complete solution script
python3 solution/physics_observer.py
```

---

## Deliverables

1. **Observation Table**: Complete the table below with your findings

| Parameter | Value | Observation |
|-----------|-------|-------------|
| Default gravity | -9.8 | |
| Moon gravity | -1.62 | |
| Low friction | 0.1 | |
| High friction | 1.0 | |

2. **Reflection Questions**: Answer these in your lab notebook

---

## Reflection Questions

1. **How would Moon gravity (1.62 m/s²) affect humanoid robot walking?**
   - Think about balance, step timing, energy consumption

2. **Why does increased gravity create more joint stress?**
   - Consider the relationship between weight and torque

3. **How might friction coefficient affect a wheeled robot vs a legged robot?**
   - Compare traction needs for different locomotion types

4. **What happens to simulation stability with very high friction values?**
   - Consider numerical precision and physics engine limitations

---

## Validation

Run the validation script to verify your lab completion:

```bash
python3 validate.py
```

Expected output:
```
[PASS] Physics observer script runs successfully
[PASS] Gravity experiments documented
[PASS] Friction experiments documented
[PASS] Reflection questions answered
Lab 1 Complete!
```

---

## Troubleshooting

### Gazebo won't launch
```bash
# Check if another instance is running
pkill -9 gz

# Clear cache
rm -rf ~/.gz/
```

### Bridge not connecting
```bash
# Verify ros_gz is installed
sudo apt install ros-humble-ros-gz
```

### Physics behaving unexpectedly
- Check that SDF version matches Gazebo version
- Ensure all required plugins are loaded
- Verify physics parameters are within reasonable ranges

---

## Next Steps

After completing this lab, proceed to:
- **Lab 2**: Gazebo Worlds - Create custom simulation environments
- **Lesson 2**: Gazebo Simulation Workflows - Deep dive into Gazebo configuration
