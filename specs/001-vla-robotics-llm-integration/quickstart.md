# Quickstart Guide: Vision-Language-Action (VLA) Robotics Education Module

## Overview
This guide will help you set up and start using the Vision-Language-Action (VLA) Robotics Education Module. The module teaches the integration of LLMs with robotics for cognitive planning, voice commands, and autonomous task execution in humanoid robots.

## Prerequisites
- ROS 2 (Humble Hawksbill or later)
- Node.js 16.x or later
- Python 3.8 or later
- Access to simulation environment (Gazebo, Unity, or NVIDIA Isaac)
- OpenAI API key (for Whisper integration) or alternative STT system

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/vla-robotics-education.git
cd vla-robotics-education
```

### 2. Install Docusaurus
```bash
npm install
```

### 3. Install ROS 2 Dependencies
```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Install additional ROS 2 packages if needed
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root:
```
OPENAI_API_KEY=your_openai_api_key
ROS_DOMAIN_ID=42
```

## Running the Documentation Locally

### 1. Start the Docusaurus Server
```bash
npm start
```

This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without having to restart the server.

### 2. Navigate to the VLA Module
In your browser, go to:
```
http://localhost:3000/docs/module-4-vision-language-action
```

## Using the VLA Module

### Chapter 1: Voice-to-Action with OpenAI Whisper
1. Navigate to the Voice-to-Action chapter
2. Follow the instructions to set up voice command recognition
3. Test voice commands in the simulation environment:
   - Say "move forward" to command the robot
   - Say "pick up the red cube" to perform manipulation tasks

### Chapter 2: Cognitive Planning with LLMs
1. Navigate to the Cognitive Planning chapter
2. Learn how to provide complex natural language instructions
3. Observe how the system decomposes instructions into action sequences
4. Test multi-step instructions like "Go to the kitchen and bring me the blue cup"

### Chapter 3: Capstone Project - The Autonomous Humanoid
1. Complete the full pipeline integration
2. Test complete scenarios from voice command to task completion
3. Experiment with navigation, object recognition, and manipulation
4. Learn about safety checks and adaptive planning

## Integration with Simulation Environments

### Connecting to Gazebo
1. Launch your Gazebo simulation:
   ```bash
   ros2 launch your_robot_gazebo your_robot_world.launch.py
   ```
2. In a new terminal, start the rosbridge:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run rosbridge_server rosbridge_websocket
   ```
3. The documentation interface will automatically connect to the simulation

### Connecting to Unity or NVIDIA Isaac
Follow the specific connection instructions provided in each simulation environment's documentation.

## Running Tests
```bash
# Run unit tests
npm test

# Run integration tests with simulation
npm run test:integration
```

## Troubleshooting

### Common Issues
1. **Voice commands not recognized**: Ensure your microphone permissions are granted in the browser and that the OpenAI API key is valid.
2. **Simulation not connecting**: Check that rosbridge is running and the correct ports are accessible.
3. **Action sequences failing**: Verify that safety constraints are properly configured for your simulation environment.

### Getting Help
- Check the FAQ section in the documentation
- Review the troubleshooting guide in Chapter 1
- Contact your instructor or teaching assistant

## Next Steps
- Complete all three chapters of the VLA module
- Experiment with custom voice commands and instructions
- Explore advanced features in the simulation environments
- Contribute to the module by suggesting improvements or reporting issues