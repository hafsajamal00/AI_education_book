---
sidebar_position: 1
---

# TF Transforms

TF (Transform) is a package that lets the user keep track of multiple coordinate frames over time. It maintains the relationship between coordinate frames in a tree structure buffered in time.

## Overview

TF transforms allow you to:

- Keep track of the position and orientation of objects over time
- Transform points, vectors, etc. between coordinate frames
- Represent the relationship between different parts of a robot

## Basic TF Usage

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_frame)

    def broadcast_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Quaternion for rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
```

## TF Lookup

To look up transforms between frames:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'target_frame',
                'source_frame',
                rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().info(f'Could not transform: {e}')
            return None
```

## Best Practices

- Use descriptive names for your frames
- Publish transforms at a consistent rate
- Use the appropriate QoS settings for your use case
- Remember that TF is designed for slowly changing transforms

## Command Line Tools

Useful command line tools for working with TF:

```bash
# View the TF tree
ros2 run tf2_tools view_frames

# Echo transforms
ros2 run tf2_ros tf2_echo source_frame target_frame

# View TF in RViz
rviz2
```