# Lab 2: Building Simulation Worlds

**Module**: 2 - The Digital Twin
**Duration**: 90 minutes
**Difficulty**: Intermediate

## Objective

Create a custom Gazebo simulation world with obstacles, terrain, and lighting. By the end of this lab, you'll have a reusable simulation environment for testing robot navigation.

## Prerequisites

- Completed Lab 1 (Physics Experiments)
- Completed Module 2 Lesson 2 (Gazebo Workflows)
- Ubuntu 22.04 with ROS 2 Humble
- Gazebo Fortress (`ros-humble-ros-gz`)

## Learning Outcomes

By completing this lab, you will:
1. Create SDF world files from scratch
2. Configure physics engine parameters
3. Add static obstacles (boxes, cylinders, spheres)
4. Set up lighting for realistic visualization
5. Test the world with a spawned robot

---

## Setup

### 1. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src/labs/module-02/lab-02-gazebo-worlds
```

### 2. Review Starter Files

```bash
ls starter/worlds/
# Should contain: empty_world.sdf
```

---

## Lab Steps

### Step 1: Understand the Empty World (10 min)

Open and review the empty world template:

```bash
cat starter/worlds/empty_world.sdf
```

Key sections to identify:
- `<world>` - Root element
- `<physics>` - Engine configuration
- `<light>` - Lighting
- `<model name="ground_plane">` - Floor

### Step 2: Add Physics Configuration (15 min)

Edit `starter/worlds/empty_world.sdf` to configure physics:

```xml
<physics name="custom_physics" type="ode">
  <!-- Step size: smaller = more accurate, slower -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor: 1.0 = real-time, >1.0 = faster -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- ODE specific settings -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

### Step 3: Add Obstacles (25 min)

Add at least 3 obstacles of different types:

**Box Obstacle:**
```xml
<model name="obstacle_box">
  <static>true</static>
  <pose>3 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Cylinder Obstacle:**
```xml
<model name="obstacle_cylinder">
  <static>true</static>
  <pose>0 3 0.75 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Sphere Obstacle:**
```xml
<model name="obstacle_sphere">
  <static>true</static>
  <pose>-2 2 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere><radius>0.5</radius></sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere><radius>0.5</radius></sphere>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Step 4: Configure Lighting (15 min)

Add realistic lighting with shadows:

```xml
<!-- Sun light -->
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.5 -1</direction>
</light>

<!-- Ambient light for fill -->
<light type="point" name="ambient_light">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 5 0 0 0</pose>
  <diffuse>0.3 0.3 0.3 1</diffuse>
  <specular>0.0 0.0 0.0 1</specular>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>
```

### Step 5: Launch and Test (15 min)

Launch your custom world:

```bash
gz sim -r starter/worlds/empty_world.sdf
```

**Verify:**
- [ ] World loads without errors
- [ ] All obstacles are visible
- [ ] Lighting creates shadows
- [ ] Physics behavior is stable

### Step 6: Spawn a Test Robot (10 min)

Test your world with a simple robot:

```bash
# Terminal 1: Launch world
gz sim -r starter/worlds/empty_world.sdf

# Terminal 2: Spawn a model (use built-in shapes)
gz service -s /world/custom_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<sdf version=\"1.8\"><model name=\"test_box\"><pose>0 0 2 0 0 0</pose><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>"'
```

Watch the box fall and interact with your world!

---

## Deliverables

1. **Custom World File**: `starter/worlds/empty_world.sdf` with:
   - Custom physics configuration
   - At least 3 different obstacle types
   - Proper lighting with shadows
   - Named appropriately (rename to `custom_world.sdf`)

2. **Screenshot**: Save a screenshot of your world in Gazebo

---

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
[PASS] SDF file is valid XML
[PASS] Physics configuration present
[PASS] At least 3 obstacles defined
[PASS] Lighting configuration present
[PASS] Ground plane present
Lab 2 Complete!
```

---

## Bonus Challenges

1. **Add a Wall**: Create a boundary wall around the environment
2. **Height Variation**: Add platforms or ramps at different heights
3. **Textured Ground**: Add a texture to the ground plane
4. **Moving Obstacle**: Create a non-static obstacle that can be pushed

---

## Troubleshooting

### World fails to load
- Check XML syntax (use `xmllint --noout file.sdf`)
- Verify SDF version matches Gazebo version
- Look for typos in tag names

### Obstacles don't appear
- Check `<pose>` values (x y z roll pitch yaw)
- Ensure `<visual>` section is defined
- Verify model is inside `<world>` tags

### Physics unstable
- Increase solver iterations
- Reduce step size
- Check for interpenetrating objects

---

## Next Steps

After completing this lab, proceed to:
- **Lab 3**: Sensor Visualization - Add sensors to your world
- **Lesson 3**: Sensors & Unity - Learn about sensor configuration
