---
sidebar_position: 2
---

# Topics

Topics are named buses over which nodes exchange messages. They enable asynchronous message passing between nodes.

## Publisher-Subscriber Model

ROS2 uses a publisher-subscriber model where:

- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can use the same topic

## Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
```

## Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Quality of Service (QoS)

QoS settings allow you to configure how messages are delivered:

```python
from rclpy.qos import QoSProfile

qos_profile = QoSProfile(depth=10)
# or use predefined profiles
from rclpy.qos import qos_profile_sensor_data
```

## Command Line Tools

Useful command line tools for working with topics:

```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"
```