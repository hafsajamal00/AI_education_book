---
sidebar_position: 1
---

# Nodes

In ROS2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program and are designed to be modular.

## Creating a Node

Here's a simple example of creating a node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running a Node

To run the node:

```bash
ros2 run package_name executable_name
```

## Node Parameters

Nodes can be configured with parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')
        my_param = self.get_parameter('my_parameter').value
```

## Best Practices

- Keep nodes focused on a single responsibility
- Use meaningful names for nodes
- Properly handle cleanup in the node destructor
- Use logging to help with debugging