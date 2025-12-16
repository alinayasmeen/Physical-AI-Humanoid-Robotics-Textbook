---
sidebar_position: 4
---

# Hardware Requirements

This course is technically demanding, sitting at the intersection of three heavy computational loads: **Physics Simulation** (e.g., Isaac Sim/Gazebo), **Visual Perception** (e.g., SLAM/Computer Vision), and **Generative AI** (e.g., LLMs/VLA). Successfully executing these tasks simultaneously demands significant computational resources, pushing typical consumer-grade hardware beyond its limits.

Because the capstone involves a "Simulated Humanoid," the primary investment must be in **High-Performance Workstations**. However, to truly fulfill the "Physical AI" promise and deploy intelligence into the real world, you also need **Edge Computing Kits** (brains without bodies) or specific robot hardware.

## 1. The "Digital Twin" Workstation (Required per Student)
This is the most critical component, serving as the primary development and simulation environment. NVIDIA Isaac Sim is an Omniverse application that fundamentally relies on **RTX** (Ray Tracing) capabilities for its physically accurate rendering and synthetic data generation. Standard laptops (MacBooks or non-RTX Windows machines) will not suffice as they lack the necessary dedicated RT cores and CUDA capabilities for these demanding workloads, leading to significant performance bottlenecks or outright incompatibility.

*   **GPU (The Bottleneck):** NVIDIA RTX 4070 Ti (12GB VRAM) or higher.
    *   **Why:** A powerful GPU with ample VRAM is paramount. You need high VRAM to load the complex USD (Universal Scene Description) assets for high-fidelity robot models and intricate environments, plus simultaneously run the large VLA (Vision-Language-Action) models for cognitive tasks. Insufficient VRAM will lead to crashes or extremely slow performance.
    *   **Ideal:** RTX 3090 or 4090 (24GB VRAM) allows for smoother "Sim-to-Real" training due to the ability to load larger models and datasets, and offers greater longevity for future advancements.
*   **CPU:** Intel Core i7 (13th Gen+) or AMD Ryzen 9.
    *   **Why:** While GPU is critical for AI, physics calculations (Rigid Body Dynamics) in simulators like Gazebo and Isaac Sim are heavily CPU-intensive. A robust multi-core CPU ensures that the simulated world operates at a realistic speed, preventing simulation bottlenecks.
*   **RAM:** 64 GB DDR5 (32 GB is the absolute minimum, but will crash during complex scene rendering and large model loading).
    *   **Why:** Running multiple demanding applications (OS, IDE, simulator, models) concurrently consumes vast amounts of RAM. 64GB provides a comfortable buffer for complex scenes and prevents system slowdowns or application failures.
*   **OS:** Ubuntu 22.04 LTS.
    *   **Note:** While Isaac Sim has Windows support, ROS 2 (Humble/Iron), the core robotics middleware, is natively and most robustly supported on Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free development and deployment experience, avoiding compatibility issues and leveraging the extensive ROS ecosystem.

## 2. The "Physical AI" Edge Kit
This kit is designed for deploying and testing AI models in a physical context, mimicking a robot's embedded system. It represents the "brain without a body" where students can understand real-world constraints. This kit covers Module 3 (Isaac ROS for inference) and Module 4 (VLA deployment).

*   **The Brain:** NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB).
    *   **Role:** These are the industry standard embedded platforms for embodied AI. Students will deploy their trained ROS 2 nodes and AI models here to understand resource constraints, power efficiency, and real-time performance differences compared to their powerful workstations.
*   **The Eyes (Vision):** Intel RealSense D435i or D455.
    *   **Role:** These depth cameras provide crucial RGB (Color) and Depth (Distance) data. This multi-modal input is essential for the VSLAM (Visual Simultaneous Localization and Mapping) and Perception modules, allowing the edge device to understand its surroundings.
*   **The Inner Ear (Balance):** Generic USB IMU (BNO055) (Often built into the RealSense D435i or Jetson boards, but a separate module helps teach IMU calibration).
    *   **Role:** An IMU provides inertial data (acceleration, angular velocity), critical for dead reckoning, balance control, and understanding the robot's own motion and orientation in 3D space.
*   **Voice Interface:** A simple USB Microphone/Speaker array (e.g., ReSpeaker) for the "Voice-to-Action" Whisper integration.
    *   **Role:** Enables natural language interaction with the robot, converting human speech into commands for the VLA system.

## 3. The Robot Lab
For the "Physical" part of the course, providing actual robot hardware offers invaluable experience, with three tiers of options depending on budget and educational goals.

### Option A: The "Proxy" Approach (Recommended for Budget)
This approach allows students to learn core robotics principles and software stack transferability without the high cost of a humanoid.
*   **Robot:** Unitree Go2 Edu (~$1,800 - $3,000).
    *   **Pros:** Highly durable, excellent ROS 2 support, and affordable enough to have multiple units for broader student access. The underlying software principles (navigation, perception, manipulation) transfer 90% effectively to humanoids.
    *   **Cons:** Not a biped (humanoid), so specific bipedal locomotion challenges cannot be directly addressed.

### Option B: The "Miniature Humanoid" Approach
Small, table-top humanoids offer a more direct, albeit scaled-down, experience with bipedal forms.
*   **Robot:** Unitree H1 is too expensive ($90k+), so consider Unitree G1 (~$16k) or Robotis OP3 (older, but stable, ~$12k). These provide a more robust platform than hobby-grade options.
*   **Budget Alternative:** Hiwonder TonyPi Pro (~$600).
    *   **Warning:** The cheap kits (e.g., Hiwonder) usually run on Raspberry Pi, which lacks the computational power and specific hardware acceleration to run NVIDIA Isaac ROS efficiently. You would use these primarily for kinematics (walking patterns) and visual demonstrations, relying on the Jetson kits for heavy AI processing.

### Option C: The "Premium" Lab (Sim-to-Real specific)
This option is for labs aiming to achieve direct deployment of advanced Capstone projects onto full-scale humanoid hardware.
*   **Robot:** Unitree G1 Humanoid.
    *   **Why:** It is one of the few commercially available humanoids that can actually walk dynamically and has an SDK open enough for students to inject their own ROS 2 controllers, making it suitable for advanced research and sim-to-real efforts.

## 4. Summary of Architecture
To teach this successfully, your lab infrastructure should thoughtfully integrate these components:

| Component    | Hardware                         | Function                                                                 |
| :----------- | :------------------------------- | :----------------------------------------------------------------------- |
| Sim Rig      | PC with RTX 4080 + Ubuntu 22.04  | Runs Isaac Sim, Gazebo, Unity; trains complex LLM/VLA models.             |
| Edge Brain   | Jetson Orin Nano                 | Runs the "Inference" stack; students deploy and test their code in real-time. |
| Sensors      | RealSense Camera + Lidar         | Connected to the Jetson to feed real-world data to the AI.              |
| Actuator     | Unitree Go2 or G1 (Shared)       | Receives motor commands from the Jetson, executing physical actions.     |

If access to RTX-enabled workstations is not feasible, the course structure would require significant modification to rely entirely on cloud-based instances (like AWS RoboMaker or NVIDIA's cloud delivery for Omniverse). However, this introduces substantial latency, increased operational cost complexity (OpEx over CapEx), and potential integration challenges for physical deployments.

## Option 2 High OpEx: The "Ether" Lab (Cloud-Native)
**Best for:** Rapid deployment, offering scalability and accessibility for students with weaker local machines, but with higher ongoing costs.

### 1. Cloud Workstations (AWS/Azure)
Instead of a capital expenditure (CapEx) on physical PCs, you incur operational expenditure (OpEx) by renting instances.
*   **Instance Type:** AWS g5.2xlarge (featuring NVIDIA A10G GPU, 24GB VRAM) or g6e.xlarge. These instances are selected for their powerful GPU capabilities, essential for running Isaac Sim and training AI models efficiently in the cloud.
*   **Software:** NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI). Accessing Isaac Sim via the cloud ensures all students have access to the necessary computational power regardless of local hardware.
*   **Cost Calculation:**
    *   Instance cost: ~$1.50/hour (utilizing a mix of spot and on-demand instances to optimize cost).
    *   Usage: 10 hours/week × 12 weeks = 120 hours.
    *   Storage (EBS volumes for saving environments and models): ~$25/quarter.
    *   **Total Cloud Bill:** Approximately ~$205 per quarter per student, assuming consistent usage.

### 2. Local "Bridge" Hardware
You cannot eliminate hardware entirely for "Physical AI" if real-world interaction is a goal.
*   **Edge AI Kits:** You still need the Jetson Kit for the physical deployment phase. This enables students to test their trained models on actual edge hardware, understanding real-world performance.
    *   **Cost:** $700 (One-time purchase).
*   **Robot:** You still need one physical robot (e.g., Unitree Go2 Standard) for the final demo and physical deployment tests.
    *   **Cost:** $3,000 (Unitree Go2 Standard).

## The Economy Jetson Student Kit
**Best for:** Learning ROS 2, Basic Computer Vision, and Sim-to-Real control in a more budget-friendly, hands-on manner. This kit provides an excellent entry point into embedded robotics.

| Component      | Model                                    | Price (Approx.) | Notes                                                                   |
| :------------- | :--------------------------------------- | :-------------- | :---------------------------------------------------------------------- |
| The Brain      | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249            | New official MSRP (Price dropped from ~$499). Capable of 40 TOPS, offering significant AI inference power. |
| The Eyes       | Intel RealSense D435i                    | $349            | Includes IMU (essential for robust SLAM and depth perception). Crucially, ensure it's the 'i' model. |
| The Ears       | ReSpeaker USB Mic Array v2.0             | $69             | A far-field microphone array, ideal for voice commands and integration with the VLA module (Module 4). |
| Wi-Fi          | (Included in Dev Kit)                    | $0              | The new "Super" kit includes the Wi-Fi module pre-installed for seamless connectivity. |
| Power/Misc     | SD Card (128GB) + Jumper Wires           | $30             | A high-endurance microSD card is required for the OS and persistent storage. |
| **TOTAL**      |                                          | **~$700 per kit** | This provides a comprehensive and capable platform for a significant portion of the course curriculum. |

## 3. The Latency Trap (Hidden Cost)
While simulating in the cloud works well, attempting to directly control a real robot from a cloud instance introduces significant and potentially dangerous latency. This delay can lead to unstable robot behavior, safety risks, and difficulty in fine-tuning real-time control.

**Solution:** Students are instructed to train their AI models and control policies in the cloud, download the resulting model weights (the learned intelligence), and then flash these trained models to the local Jetson kit for on-device inference and direct control of the physical robot. This "train in cloud, deploy on edge" paradigm mitigates latency issues and provides a robust workflow.
