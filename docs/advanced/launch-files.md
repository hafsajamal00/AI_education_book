---
sidebar_position: 3
---

# Launch Files

Launch files allow you to start multiple nodes at once with a single command. They also allow you to set parameters for your nodes and manage complex robot systems.

## Overview

Launch files in ROS2 are Python files that define how to launch a system. They provide:

- A way to start multiple nodes with a single command
- Parameter configuration for nodes
- Conditional launching based on arguments
- Process management (starting, stopping, monitoring)

## Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_talker',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='my_listener',
        )
    ])
```

## Launch Arguments

You can pass arguments to launch files:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Use the launch argument
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        use_sim_time,
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_talker',
            parameters=[{'use_sim_time': use_sim_time_config}]
        )
    ])
```

## Conditional Launch

Launch files support conditional execution:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare argument
    enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera node'
    )
    
    camera_enabled = LaunchConfiguration('enable_camera')
    
    return LaunchDescription([
        enable_camera,
        Node(
            package='camera_package',
            executable='camera_node',
            name='camera',
            condition=IfCondition(camera_enabled)
        )
    ])
```

## Including Other Launch Files

You can include other launch files in your launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include another launch file
    other_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('other_package'),
                'launch',
                'other_launch_file.launch.py'
            )
        )
    )
    
    return LaunchDescription([
        other_launch_file
    ])
```

## Running Launch Files

To run a launch file:

```bash
ros2 launch package_name launch_file_name.py
```

With arguments:

```bash
ros2 launch package_name launch_file_name.py use_sim_time:=true
```

## Best Practices

- Organize launch files by functionality
- Use descriptive names for launch files
- Make use of arguments for flexibility
- Include error handling in your launch files
- Document your launch files well