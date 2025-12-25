---
sidebar_position: 2
---

# Basic Concepts

Understanding the fundamental concepts of ROS2 is crucial before diving into development.

## Nodes

A node is a process that performs computation. Nodes are combined together into a graph to perform various tasks. In ROS2, nodes are designed to be modular and distributed.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello from my_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Packages

Packages are the software organization unit of ROS. Each package can contain libraries, executables, scripts, or other artifacts. A package must contain at least a `package.xml` manifest file.

## Workspaces

A workspace is a folder containing ROS2 packages. The standard structure includes:

- `src/` - source code
- `build/` - build artifacts
- `install/` - installation directory
- `log/` - log files

## Next Steps

Now that you understand the basic concepts, continue to learn about ROS2 nodes, topics, and services in the next sections.