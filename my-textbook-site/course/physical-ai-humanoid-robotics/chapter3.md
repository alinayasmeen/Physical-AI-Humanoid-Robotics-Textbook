# Chapter 3: Actuation and Manipulation

Actuation and manipulation are key capabilities that enable physical AI and humanoid robots to perform physical tasks and interact with their environment. This chapter delves into the mechanisms that generate motion and force, and how robots use these to achieve dextrous manipulation.

## Actuators
Actuators are the 'muscles' of a robot, converting energy into mechanical motion. Various types of actuators are used in robotics, each with its own characteristics:
- **Electric Motors**: Most common type, including DC motors, servo motors, and stepper motors.
  - Advantages: High precision, good control, clean operation.
  - Disadvantages: Can be bulky, require gearboxes for high torque.
- **Hydraulic Actuators**: Use pressurized fluid to generate large forces.
  - Advantages: High power density, can generate very large forces.
  - Disadvantages: Messy, require pumps and reservoirs, complex plumbing.
- **Pneumatic Actuators**: Use compressed air to generate motion.
  - Advantages: Simple, fast, clean, low cost.
  - Disadvantages: Difficult to control precisely, lower force density than hydraulics.
- **Smart Materials**: Emerging actuators like Shape Memory Alloys (SMAs) and Dielectric Elastomer Actuators (DEAs).

## Robot Manipulators
Robot manipulators are composed of a series of links connected by joints, designed to perform tasks involving grasping, moving, and orienting objects. Key aspects include:
- **End-Effectors**: The 'hand' of the robot, designed for specific tasks like grasping (grippers), welding, or painting.
  - **Grippers**: Mechanical devices that grasp objects. Can be parallel-jaw, three-finger, or specialized designs.
  - **Tools**: Integrated tools like drills, cameras, or sensors.
- **Degrees of Freedom (DoF)**: The number of independent parameters that define the configuration of a robot. Humanoid arms typically have many DoF for dexterity.
- **Workspace**: The volume of space that the robot's end-effector can reach.

## Manipulation Strategies
Effective manipulation requires sophisticated strategies to plan movements, grasp objects, and interact with the environment:
- **Grasping**: Planning contact points and forces to securely hold an object.
  - **Force Closure**: Ensuring that external forces cannot dislodge the object.
  - **Form Closure**: Geometric constraint preventing object movement.
- **Motion Planning**: Generating collision-free paths for the robot's end-effector and joints.
  - **Path Planning Algorithms**: e.g., RRT (Rapidly-exploring Random Tree), PRM (Probabilistic Roadmaps).
- **Compliant Control**: Allowing the robot to interact with the environment by yielding to forces, essential for tasks like assembly or polishing.
- **Teleoperation**: Human control of a robot, often used for complex or hazardous tasks.