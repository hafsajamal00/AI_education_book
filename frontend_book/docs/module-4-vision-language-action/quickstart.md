---
sidebar_label: "Quickstart Guide - Vision-Language-Action (VLA)"
---

# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview

This quickstart guide will help you set up and run the Vision-Language-Action (VLA) module quickly. Follow these steps to get started with voice-controlled robotics using LLMs.

## Prerequisites

Before starting, ensure you have:

1. **ROS 2** installed (Humble Hawksbill or later recommended)
2. **Python 3.11** or later
3. **Node.js** 16.x or later
4. **OpenAI API key** (for Whisper and LLM functionality)
5. A simulation environment (Gazebo, Unity, or NVIDIA Isaac)

## Setting Up the Environment

### 1. Clone the Repository

```bash
git clone <repository-url>
cd hackathon-1
```

### 2. Install Docusaurus Dependencies

```bash
cd frontend_book
npm install
```

### 3. Set Up Python Environment

```bash
# Create virtual environment
python -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate

# Install required packages
pip install openai pyaudio rclpy
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with your OpenAI API key:

```bash
OPENAI_API_KEY=your_openai_api_key_here
```

## Running the VLA Module

### 1. Start the Documentation Server

```bash
cd frontend_book
npm start
```

This will start the Docusaurus development server at `http://localhost:3000`.

### 2. Set Up ROS 2 Environment

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source your workspace (if applicable)
source install/setup.bash
```

### 3. Launch the Simulation Environment

For Gazebo:
```bash
# Terminal 1
ros2 launch gazebo_ros empty_world.launch.py

# Terminal 2
ros2 run your_robot_simulation spawn_robot.launch.py
```

### 4. Run the VLA Pipeline

```bash
# In a new terminal, with the Python environment activated
python -m ros2_vla_integration.vla_pipeline
```

## Trying Your First Voice Command

1. Ensure all components are running (documentation, ROS 2, simulation)
2. Navigate to the VLA module in the documentation
3. Find the interactive voice command section (if available)
4. Speak a command like "move forward" or "turn left"
5. Observe the robot's response in the simulation

## Example Commands

Try these sample commands to test the system:

- "Move forward"
- "Turn left"
- "Go to the kitchen"
- "Pick up the red cube"
- "Go to the kitchen, find a cup, and bring it back"

## Troubleshooting

### Common Issues

1. **Voice commands not recognized**:
   - Check microphone permissions
   - Verify OpenAI API key is correctly set
   - Ensure a quiet environment for better recognition

2. **Robot not responding in simulation**:
   - Verify ROS 2 nodes are running
   - Check topic connections with `ros2 topic list`
   - Ensure the simulation environment is properly launched

3. **Cognitive planning errors**:
   - Check that the LLM API is accessible
   - Verify the instruction format is clear and unambiguous
   - Review safety constraints that might be blocking execution

### Checking System Status

```bash
# Check active ROS 2 nodes
ros2 node list

# Check active topics
ros2 topic list

# Check service servers
ros2 service list
```

## Next Steps

After completing the quickstart:

1. Work through Chapter 1 to understand voice-to-action systems
2. Proceed to Chapter 2 to learn cognitive planning with LLMs
3. Complete the capstone project in Chapter 3
4. Experiment with custom voice commands and complex tasks

## Additional Resources

- [Chapter 1: Voice-to-Action with OpenAI Whisper](../module-4-vision-language-action/chapter-1-voice-to-action)
- [Chapter 2: Cognitive Planning with LLMs](../module-4-vision-language-action/chapter-2-cognitive-planning)
- [Chapter 3: Capstone Project - The Autonomous Humanoid](../module-4-vision-language-action/chapter-3-capstone-project)
- [ROS 2 Documentation](https://docs.ros.org/)
- [OpenAI API Documentation](https://platform.openai.com/docs/)