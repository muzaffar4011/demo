---
sidebar_position: 4
title: rclpy Bridge
---

# rclpy Bridge

The `rclpy` package is the Python client library for ROS 2. It provides the Python API for developing ROS 2 packages and enables Python developers to create ROS 2 nodes, publishers, subscribers, services, and more.

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the role of rclpy in ROS 2 Python development
- Create ROS 2 nodes using rclpy
- Implement publishers, subscribers, services, and clients with rclpy
- Bridge Python agents with ROS 2 for humanoid robotics applications

## Introduction to rclpy

`rclpy` is the official Python client library for ROS 2. It provides Python bindings for the ROS 2 middleware (RCL - ROS Client Library), allowing Python developers to interact with the ROS 2 ecosystem. For humanoid robotics applications, rclpy enables the integration of complex Python-based AI algorithms with the ROS 2 communication framework.

### Key Components of rclpy

- **Node**: The basic execution unit that can communicate with other nodes
- **Publisher**: Sends messages to topics
- **Subscriber**: Receives messages from topics
- **Service Server**: Provides services that other nodes can call
- **Service Client**: Calls services provided by other nodes
- **Timer**: Executes callbacks at regular intervals
- **Parameter**: Handles node configuration parameters

## Basic Node Structure with rclpy

Here's the standard structure for a ROS 2 node using rclpy:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Initialize publishers, subscribers, services, etc. here

    def destroy_node(self):
        # Cleanup operations before node destruction
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    my_robot_node = MyRobotNode()

    try:
        rclpy.spin(my_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers with rclpy

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Command {self.counter}: Move forward'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()

    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command here

def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()

    try:
        rclpy.spin(command_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        command_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and Clients with rclpy

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    future = minimal_client.send_request(1, 2)

    try:
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced rclpy Features for Humanoid Robotics

### Working with Custom Message Types

```python
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

class HumanoidStateNode(Node):
    def __init__(self):
        super().__init__('humanoid_state_node')

        # Publisher for humanoid state
        self.state_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            'humanoid_controller/state',
            10
        )

        # Subscriber for joint commands
        self.joint_subscriber = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.publish_state)  # 50Hz

    def joint_command_callback(self, msg):
        # Process joint commands
        self.get_logger().info(f'Received {len(msg.name)} joint commands')

    def publish_state(self):
        # Publish current humanoid state
        msg = JointTrajectoryControllerState()
        # Fill in the message with current state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Handling

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('step_height', 0.1)
        self.declare_parameter('walking_speed', 0.5)
        self.declare_parameter('max_torque', 100.0)

        # Get parameter values
        self.step_height = self.get_parameter('step_height').value
        self.walking_speed = self.get_parameter('walking_speed').value
        self.max_torque = self.get_parameter('max_torque').value

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'step_height' and param.type == ParameterType.PARAMETER_DOUBLE:
                self.step_height = param.value
                self.get_logger().info(f'Step height updated to: {self.step_height}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for rclpy in Humanoid Robotics

1. **Error Handling**: Always implement proper error handling in your callbacks
2. **Resource Management**: Clean up resources in the destroy_node method
3. **Threading**: Be aware of threading implications when using rclpy
4. **Performance**: Optimize message publishing rates based on real-time requirements
5. **Logging**: Use appropriate log levels (info, warn, error, debug)

## Exercise

Create a Python node that bridges a simple AI decision-making algorithm with ROS 2. The node should:
1. Subscribe to sensor data topics
2. Make decisions based on the sensor input
3. Publish commands to actuator topics
4. Use parameters to configure the AI behavior

## Summary

The rclpy library provides Python developers with powerful tools to create ROS 2 nodes for humanoid robotics applications. By understanding its core components and best practices, you can effectively bridge Python-based AI algorithms with the ROS 2 communication framework, enabling sophisticated robotic behaviors.