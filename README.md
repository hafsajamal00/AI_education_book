# Physical AI & Humanoid Robotics Learning Platform

This repository contains an educational resource for learning ROS 2 and robotics through a series of modules. Each module builds upon the previous one, providing a comprehensive learning path for robotics enthusiasts and professionals.

## Modules

The `frontend_book` directory contains 4 comprehensive modules:

### Module 1: The Robotic Nervous System (ROS 2)
- Chapter 1: ROS 2 Fundamentals
- Chapter 2: ROS 2 Communication
- Chapter 3: Robot Structure

### Module 2: The Digital Twin (Gazebo & Unity)
- Chapter 1: Digital Twins
- Chapter 2: Gazebo Physics
- Chapter 3: Unity Integration

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Chapter 1: NVIDIA Isaac Sim Fundamentals
- Chapter 2: Isaac ROS Perception & Navigation
- Chapter 3: Nav2 Bipedal Humanoid Movement

### Module 4: Vision-Language-Action (VLA)
- Chapter 1: Voice-to-Action with OpenAI Whisper
- Chapter 2: Cognitive Planning with LLMs
- Chapter 3: Capstone Project - The Autonomous Humanoid

## Vision-Language-Action (VLA) Module

The VLA module focuses on integrating Large Language Models (LLMs) with robotics for cognitive planning, voice commands, and autonomous task execution in humanoid robots.

### Features
- Voice command recognition using OpenAI Whisper
- Cognitive planning with LLMs for complex task decomposition
- Integration with ROS 2 for robot control
- Simulation testing in Gazebo, Unity, and NVIDIA Isaac
- Complete pipeline from voice command to action execution

### Prerequisites
- Students should have prior knowledge of ROS 2, Gazebo, Unity, and NVIDIA Isaac
- Python 3.11+ and Node.js 16+
- OpenAI API key for Whisper and LLM functionality

## Getting Started

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd ros2-educational-book
   ```

2. Install dependencies:
   ```bash
   cd frontend_book
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

4. The documentation will be available at `http://localhost:3000`

## Contributing

We welcome contributions to this educational resource. Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

Last updated: December 25, 2025