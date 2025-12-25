---
sidebar_position: 1
---

# TurtleBot Simulation

In this project, we'll simulate a TurtleBot in Gazebo and control it using ROS2. This is a great way to learn ROS2 concepts without needing physical hardware.

## Setup

First, make sure you have the necessary packages installed:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
```

Set the proper environment variables:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

## Launching the Simulation

To start the simulation:

```bash
# Terminal 1 - Launch Gazebo with TurtleBot
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2 - Launch the teleop node to control the robot
ros2 run turtlebot3_teleop teleop_keyboard
```

## Creating a Navigation Node

Let's create a simple node that moves the TurtleBot in a square pattern:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        self.move_square()

    def move_square(self):
        """Move the robot in a square pattern"""
        twist = Twist()
        
        for _ in range(4):  # Make a square
            # Move forward
            twist.linear.x = 0.5  # m/s
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            time.sleep(2)  # Move for 2 seconds
            
            # Turn
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # rad/s
            self.publisher.publish(twist)
            time.sleep(3.14)  # Turn for 90 degrees
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    square_mover = SquareMover()
    
    # Allow some time for the node to publish messages
    time.sleep(12)  # 4 sides * (2s forward + 3.14s turn) = ~20s, but we'll make it shorter
    
    square_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## RViz Visualization

To visualize the robot in RViz:

```bash
ros2 run rviz2 rviz2
```

In RViz, add the RobotModel display and set the TF topic to visualize the robot.

## Adding a Laser Scanner

To add obstacle detection, we can use the laser scanner:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Check if there's an obstacle in front
        front_distance = msg.ranges[len(msg.ranges)//2]  # middle range value
        
        twist = Twist()
        if front_distance > 1.0:  # No obstacle within 1 meter
            twist.linear.x = 0.5
        else:  # Obstacle detected, turn
            twist.angular.z = 0.5
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this project, we learned how to:

1. Set up a TurtleBot simulation in Gazebo
2. Control the robot using ROS2
3. Create custom nodes for robot behavior
4. Use sensors like laser scanners for navigation

This project provides a foundation for more complex robotics applications using ROS2.