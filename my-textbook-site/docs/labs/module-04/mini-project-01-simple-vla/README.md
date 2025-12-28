# Mini-Project 1: Simple Vision-Language-Action (VLA) System

## Overview
In this mini-project, you'll build a basic Vision-Language-Action (VLA) system that can understand simple spoken commands and execute them using a robot simulator. This project will help you understand the fundamental components of VLA systems and how they integrate.

## Learning Objectives
- Implement a basic VLA pipeline with speech recognition, vision, and action components
- Integrate ROS 2 with multimodal AI components
- Handle basic error cases and safety considerations
- Test the system with simple commands like "move forward" or "turn left"

## Prerequisites
- Completed Module 1-3 (ROS 2, Simulation, Perception)
- Basic Python programming skills
- Understanding of multimodal AI concepts from Module 4

## Estimated Time
4-6 hours

## Project Structure
```
mini-project-01-simple-vla/
├── README.md (this file)
├── starter/
│   ├── vla_pipeline.py
│   ├── speech_recognizer.py
│   └── action_executor.py
├── solution/
│   ├── vla_pipeline.py
│   ├── speech_recognizer.py
│   └── action_executor.py
└── validate.py
```

## Task Breakdown

### 1. Speech Recognition Component
Implement a basic speech recognition module that can:
- Capture audio input
- Convert speech to text
- Parse simple commands

### 2. Basic Action Execution
Implement ROS 2 action execution that can:
- Execute simple movement commands
- Handle basic navigation tasks
- Monitor execution status

### 3. Pipeline Integration
Connect all components into a working pipeline that:
- Processes spoken commands
- Executes appropriate actions
- Provides feedback to the user

## Getting Started
1. Navigate to the `starter/` directory
2. Implement the missing functionality in the provided files
3. Test your implementation with the `validate.py` script
4. Compare with the `solution/` directory when complete

## Validation
Run the validation script to check your implementation:
```bash
python validate.py
```

## Resources
- Review Module 4 lessons on VLA components
- Reference ROS 2 action client tutorials
- Consult speech recognition libraries like `speech_recognition`

## Submission
When complete, ensure your solution passes all validation checks and demonstrates the required functionality with a simple demonstration to a peer or instructor.