# Lab X: [Lab Title]

**Module**: 3 - Perception & Sensors
**Duration**: XX minutes
**Difficulty**: Beginner/Intermediate/Advanced

## Objective

[Clear statement of what students will accomplish in this lab]

## Prerequisites

- Completed Module 1 (ROS 2 fundamentals)
- Completed Module 2 (Simulation basics)
- Ubuntu 22.04 with ROS 2 Humble installed
- OpenCV 4.x installed (`pip install opencv-python`)
- cv_bridge package installed (`ros-humble-cv-bridge`)

## Learning Outcomes

By completing this lab, you will:
1. [Outcome 1]
2. [Outcome 2]
3. [Outcome 3]
4. [Outcome 4]

---

## Setup

### 1. Verify Installation

```bash
# Check OpenCV version
python3 -c "import cv2; print(cv2.__version__)"

# Check cv_bridge
ros2 pkg list | grep cv_bridge

# Check sensor_msgs
ros2 interface show sensor_msgs/msg/Image
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-03/lab-XX-name .
cd lab-XX-name
```

---

## Lab Steps

### Step 1: [First Step Title] (XX min)

[Instructions for step 1]

```python
# Example code
```

**Observe**:
- [What to observe]
- [Expected behavior]

### Step 2: [Second Step Title] (XX min)

[Instructions for step 2]

### Step 3: [Third Step Title] (XX min)

[Instructions for step 3]

### Step 4: Compare with Solution (XX min)

Review the solution files:

```bash
# Compare your implementation with the solution
diff starter/your_file.py solution/your_file.py

# Run the complete solution
python3 solution/your_file.py
```

---

## Deliverables

1. **Working Code**: Complete implementation in `starter/` directory
2. **Observation Notes**: Document your findings
3. **Reflection Answers**: Answer the questions below

---

## Reflection Questions

1. **[Question about concept]**
   - Consider: [hint]

2. **[Question about application]**
   - Think about: [hint]

3. **[Question about extension]**
   - Explore: [hint]

---

## Validation

Run the validation script to verify your lab completion:

```bash
python3 validate.py
```

Expected output:
```
[PASS] [Check 1]
[PASS] [Check 2]
[PASS] [Check 3]
Lab X Complete!
```

---

## Troubleshooting

### OpenCV import fails
```bash
pip install opencv-python opencv-python-headless
```

### cv_bridge not found
```bash
sudo apt install ros-humble-cv-bridge
```

### Camera topic not publishing
```bash
# Check available topics
ros2 topic list

# Check if camera node is running
ros2 node list
```

---

## Next Steps

After completing this lab, proceed to:
- **Lab X+1**: [Next lab title]
- **Lesson X**: [Related lesson]
