---
title: Chapter 2 - ROS 2 Communication Model
sidebar_label: Chapter 2 - ROS 2 Communication Model
description: "Understanding ROS 2 communication patterns: nodes, topics, services, and Python integration"
---

# Chapter 2: ROS 2 Communication Model

## Nodes, Topics, and Services

ROS 2 uses a distributed computing model where computation is spread across potentially many devices working together. This is achieved through a graph architecture of nodes and the messages they exchange.

### Nodes
Nodes are the fundamental unit of execution in ROS 2. They are processes that perform computation. Nodes written in different programming languages can be distributed across different machines and still work together as a single system.

### Topics and Publishers/Subscribers
Topics are named buses over which nodes exchange messages. The communication is based on a publish/subscribe pattern where publishers send messages to topics without knowing who (if anyone) is subscribed.

### Services
Services provide a request/response communication pattern. A service client sends a request message to a service server, which processes the request and returns a response message.

## Message Passing and Coordination

Communication in ROS 2 is designed to be:
- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Asynchronous**: Publishers and subscribers can operate at different rates
- **Typed**: All messages have defined types
- **Serialized**: Messages are serialized for transport
- **Transport-agnostic**: Multiple transport options available (TCP, UDP, shared memory)

## Python Integration using rclpy

rclpy is the Python client library for ROS 2. It provides a Python API for ROS 2 concepts such as:
- Nodes
- Publishers
- Subscribers
- Services
- Parameters
- Actions

### Basic Example

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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

1. Create a simple publisher and subscriber pair
2. Implement a service client and server
3. Use parameters to configure node behavior
4. Create a more complex message type

## Code Examples with Explanations

### Subscriber Example
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

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solutions and Best Practices

### Best Practices for Communication:
- Use appropriate Quality of Service (QoS) settings
- Design message types carefully
- Consider message frequency and size
- Implement proper error handling
- Use appropriate naming conventions

### Common Communication Scenarios:
- Sensor data publishing
- Command execution
- State synchronization
- Event notification