---
sidebar_position: 2
title: ROS Nodes
---

# ROS Nodes

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. They communicate with other nodes using topics, services, actions, and parameters.

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the concept of ROS nodes and their role in a robotic system
- Create ROS nodes using Python and the rclpy library
- Implement basic node functionality for humanoid robotics applications

## What is a ROS Node?

A ROS node is an executable process that uses ROS client libraries (like rclpy for Python) to communicate with other nodes. Nodes can publish messages to topics, subscribe to topics to receive messages, provide services, call services, and more.

In the context of humanoid robotics, nodes might handle:
- Sensor data processing (IMU, cameras, joint encoders)
- Actuator control (motor commands)
- High-level decision making (path planning, behavior selection)
- Perception systems (object detection, SLAM)

## Creating a Basic ROS Node

Let's create a simple ROS node that publishes messages about the humanoid robot's status:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidStatusPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_status_publisher')
        self.publisher_ = self.create_publisher(String, 'humanoid_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Humanoid robot status: Operational - {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_status_publisher = HumanoidStatusPublisher()

    try:
        rclpy.spin(humanoid_status_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_status_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Structure Explained

The above example demonstrates the basic structure of a ROS node:

1. **Import statements**: Import the necessary ROS 2 libraries
2. **Node class**: Inherit from `rclpy.node.Node` to create your node
3. **Initialization**: Set up publishers, subscribers, timers, etc. in `__init__`
4. **Main function**: Initialize ROS 2, create the node, and start spinning

## Running the Node

To run this node:

1. Save the code in a file (e.g., `humanoid_status_publisher.py`)
2. Make sure your ROS 2 environment is sourced
3. Run the node: `python3 humanoid_status_publisher.py`

## Best Practices for Humanoid Robotics Nodes

When creating nodes for humanoid robotics applications:

- **Modularity**: Design nodes to perform a single, well-defined function
- **Robustness**: Handle errors gracefully and ensure the node can recover from failures
- **Real-time considerations**: For critical control tasks, consider real-time performance requirements
- **Resource efficiency**: Humanoid robots often have limited computational resources

## Exercise

Create a ROS node that publishes joint position commands for a humanoid robot. The node should publish messages to a topic called `joint_commands` with appropriate message types.

## Summary

ROS nodes form the foundation of any ROS 2 system. Understanding how to create, structure, and manage nodes is crucial for developing complex humanoid robotics applications. In the next lesson, we'll explore how nodes communicate with each other using topics and services.