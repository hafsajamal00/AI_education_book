---
sidebar_position: 2
---

# Autonomous Robot Project

In this advanced project, we'll build a complete autonomous robot system that can navigate in an environment, avoid obstacles, and reach specified goals.

## Project Overview

This project combines multiple ROS2 concepts to create a robot that can:

- Map its environment using SLAM
- Navigate to specified locations
- Avoid obstacles in real-time
- Integrate multiple sensors and actuators

## Prerequisites

Before starting this project, ensure you have:

- Navigation2 packages installed
- SLAM toolbox installed
- Basic understanding of TF, topics, and services

## System Architecture

The autonomous robot system consists of several key components:

1. **Hardware Abstraction Layer** - Interfaces with physical hardware
2. **Perception System** - Processes sensor data
3. **Mapping System** - Creates and updates environment map
4. **Localization System** - Determines robot position
5. **Planning System** - Plans paths to goals
6. **Control System** - Controls robot movement

## Launch File

Here's a launch file that brings up the complete autonomous system:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Enable SLAM mode'
    )
    
    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'maps',
            'turtlebot3_world.yaml'
        ),
        description='Full path to map file to load'
    )
    
    use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='true',
        description='Enable the lifecycle manager'
    )

    # Navigation launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(
                get_package_share_directory('nav2_bringup'),
                'params',
                'nav2_params.yaml'
            )
        }.items()
    )

    # SLAM launch file
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(
                get_package_share_directory('slam_toolbox'),
                'config',
                'mapper_params_online_async.yaml'
            )
        }.items()
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
        ]
    )

    return LaunchDescription([
        use_sim_time,
        slam,
        map_yaml_file,
        use_lifecycle_manager,
        nav2_bringup_launch,
        slam_toolbox_launch,
        lifecycle_manager
    ])
```

## Navigation Node

Here's a simple navigation node that sends goals to the Navigation2 system:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def navigate_to_pose(self, x, y, theta):
        """Send a navigation goal to the robot"""
        # Wait for the action server to be available
        self.nav_to_pose_client.wait_for_server()

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set the orientation (assuming theta is in radians)
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        # Send the goal
        self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self.get_logger().info(f'Navigation goal sent to ({x}, {y}, {theta})')

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the navigation action"""
        feedback = feedback_msg.feedback
        # Process feedback as needed
        self.get_logger().info('Distance remaining: {:.2f}'.format(
            feedback.distance_remaining))

def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    
    # Example: Navigate to a specific location
    navigator.navigate_to_pose(1.0, 1.0, 0.0)  # x=1.0, y=1.0, theta=0.0
    
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Obstacle Avoidance

For real-time obstacle avoidance, we can implement a simple reactive controller:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            QoSProfile(depth=10)
        )
        
        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10)
        )
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Initialize variables
        self.obstacle_detected = False
        self.safe_distance = 0.5  # meters

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Check for obstacles in front of the robot
        front_scan = msg.ranges[len(msg.ranges)//2 - 30:len(msg.ranges)//2 + 30]
        
        # Find minimum distance in the front sector
        min_distance = min(front_scan)
        
        self.obstacle_detected = min_distance < self.safe_distance

    def control_loop(self):
        """Main control loop for obstacle avoidance"""
        twist = Twist()
        
        if self.obstacle_detected:
            # Stop and turn to avoid obstacle
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info('Obstacle detected! Turning...')
        else:
            # Move forward safely
            twist.linear.x = 0.3  # m/s
            twist.angular.z = 0.0
            self.get_logger().info('Path clear. Moving forward...')
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the System

To run the complete autonomous robot system:

1. Launch the robot hardware interface (or simulation):
   ```bash
   ros2 launch your_robot_bringup robot.launch.py
   ```

2. Launch the navigation system:
   ```bash
   ros2 launch your_robot_nav autonomous_nav.launch.py
   ```

3. Run the navigation node to send goals:
   ```bash
   ros2 run your_robot_nav autonomous_navigator
   ```

## Testing and Validation

To test the autonomous robot:

1. Use RViz to visualize the map, robot position, and planned path
2. Send navigation goals using the `NavigateToPose` action
3. Monitor the robot's behavior in different scenarios
4. Validate that obstacle avoidance works correctly

## Summary

This project demonstrates how to build a complete autonomous robot system using ROS2. It combines multiple concepts like:

- Navigation and path planning
- SLAM for mapping
- Sensor processing
- Control systems
- Action-based communication

This forms a solid foundation for developing more sophisticated robotic applications.