---
title: Chapter 3 - Robot Structure & Control Basics
sidebar_label: Chapter 3 - Robot Structure & Control Basics
description: "Understanding URDF for humanoid robots and bridging Python AI agents to ROS controllers"
---

# Chapter 3: Robot Structure & Control Basics

## URDF for Humanoid Robots

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It is used by ROS packages such as rviz, gazebo, and moveit to model the physical properties of a robot.

### Robot Description Elements

URDF defines several key elements for robot description:

#### Links
Links represent rigid bodies with physical properties like mass, visual representation, and collision properties. Each link has:
- Mass and inertia properties
- Visual representation for rendering
- Collision representation for physics simulation
- Material properties for visualization

#### Joints
Joints define the kinematic and dynamic relationships between links. Types of joints include:
- **Revolute**: Rotational joint with limits
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No movement between links
- **Floating**: 6 DOF with no limits
- **Planar**: Movement on a plane

#### Sensors and Actuators
URDF can also describe sensors and actuators attached to the robot:
- Camera sensors
- IMU sensors
- Force/torque sensors
- Joint actuators

## Links, Joints, Sensors, Actuators

### Links Example
```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.1" length="0.2" />
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.1" length="0.2" />
    </geometry>
  </collision>
</link>
```

### Joints Example
```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link" />
  <child link="wheel_link" />
  <origin xyz="0 0.1 0" rpy="0 0 0" />
  <axis xyz="0 1 0" />
</joint>
```

## How to Create and Interpret Robot Descriptions

### Creating a Simple Robot Description
1. Define the base link (root of the kinematic tree)
2. Add additional links for robot parts
3. Define joints connecting the links
4. Specify visual and collision properties
5. Add sensors and actuators if needed

### Interpreting Robot Descriptions
- Start from the base link and follow the kinematic chain
- Understand the degrees of freedom provided by each joint
- Identify sensor positions and types
- Recognize the kinematic structure (serial chain, parallel mechanisms, etc.)

## Bridging Python AI Agents to ROS Controllers

### Control Architecture
The connection between AI agents and robot controllers typically involves:
- High-level AI planning and decision making
- Motion planning and trajectory generation
- Low-level control for actuators
- Sensor data processing and feedback

### Implementation Example
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AIControllerBridge(Node):
    def __init__(self):
        super().__init__('ai_controller_bridge')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # AI decision making timer
        self.ai_timer = self.create_timer(0.1, self.ai_decision_callback)

        self.current_joint_states = None

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def ai_decision_callback(self):
        if self.current_joint_states is not None:
            # AI logic to determine desired joint positions
            desired_positions = self.ai_compute_desired_positions()

            # Create trajectory message
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.current_joint_states.name
            point = JointTrajectoryPoint()
            point.positions = desired_positions
            point.time_from_start = Duration(sec=1)
            traj_msg.points = [point]

            # Publish trajectory
            self.joint_cmd_publisher.publish(traj_msg)

    def ai_compute_desired_positions(self):
        # Placeholder for AI decision logic
        # In a real implementation, this would interface with
        # the AI agent to determine desired joint positions
        return [0.0] * len(self.current_joint_states.name)

def main(args=None):
    rclpy.init(args=args)
    bridge = AIControllerBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples of Connecting AI Agents to Robot Hardware

### Perception-Action Loop
1. AI agent receives sensor data (camera, lidar, joint states)
2. AI processes perception data to understand environment
3. AI makes decisions based on goals and current state
4. AI sends commands to robot controllers
5. Robot executes actions and provides feedback

### Learning from Interaction
- Reinforcement learning with real robot feedback
- Imitation learning from human demonstrations
- Sim-to-real transfer of learned behaviors

## Control Mechanisms and Data Flow

### Control Hierarchy
```
AI Agent (High Level)
    ↓ (goals, plans)
Motion Planner (Mid Level)
    ↓ (trajectories)
Controller (Low Level)
    ↓ (motor commands)
Robot Hardware
    ↑ (sensor data)
```

### Data Flow Considerations
- Real-time constraints for control loops
- Safety mechanisms and limits
- Sensor fusion for state estimation
- Communication latency and reliability