---
sidebar_position: 3
title: Topics and Services
---

# Topics and Services

ROS 2 provides two primary communication patterns: topics (for asynchronous, broadcast communication) and services (for synchronous, request-response communication). Understanding these patterns is crucial for designing effective robotic systems.

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the differences between topics and services
- Implement publishers and subscribers for topics
- Create and use services for request-response communication
- Apply appropriate communication patterns to humanoid robotics scenarios

## Topics: Publisher-Subscriber Pattern

Topics enable asynchronous, one-to-many communication between nodes. Publishers send messages to a topic, and any number of subscribers can receive those messages. This pattern is ideal for continuous data streams like sensor readings or robot state information.

### Example: Sensor Data Topic

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HumanoidSensorPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

    def publish_sensor_data(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.1, 0.2, 0.3]  # Example positions in radians
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = HumanoidSensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscribing to Topics

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Response Pattern

Services provide synchronous, one-to-one communication where a client sends a request and waits for a response. This pattern is ideal for actions that require confirmation or computation, like requesting a specific robot pose or executing a complex behavior.

### Example: Robot Pose Service

First, define the service interface in a `.srv` file (e.g., `SetPose.srv`):

```
# Request
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
---
# Response
bool success
string message
```

Then implement the service server:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using a standard service for simplicity

class HumanoidPoseService(Node):
    def __init__(self):
        super().__init__('humanoid_pose_service')
        self.srv = self.create_service(
            SetBool,
            'set_humanoid_pose',
            self.set_pose_callback)

    def set_pose_callback(self, request, response):
        # In a real implementation, this would set the humanoid's pose
        self.get_logger().info(f'Setting pose to: {request.data}')

        response.success = True
        response.message = 'Pose set successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    pose_service = HumanoidPoseService()

    try:
        rclpy.spin(pose_service)
    except KeyboardInterrupt:
        pass
    finally:
        pose_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And the service client:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class PoseClientAsync(Node):
    def __init__(self):
        super().__init__('pose_client')
        self.cli = self.create_client(SetBool, 'set_humanoid_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, pose_data):
        self.req.data = pose_data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    pose_client = PoseClientAsync()

    response = pose_client.send_request(True)
    pose_client.get_logger().info(f'Response: {response.success}, {response.message}')

    pose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## When to Use Topics vs Services

### Use Topics when:
- Broadcasting information to multiple subscribers
- Continuous data streams (sensors, robot state)
- Real-time requirements (no blocking)
- Data doesn't require acknowledgment

### Use Services when:
- Requesting specific actions or computations
- Need confirmation of completion
- One-to-one communication is sufficient
- Request-response pattern fits the use case

## Humanoid Robotics Communication Patterns

In humanoid robotics, common communication patterns include:

- **Topics**: Joint states, sensor data, robot pose, camera feeds
- **Services**: Set robot pose, execute behavior, calibration, emergency stop
- **Actions**: Complex tasks that take time (walking, grasping) with feedback

## Exercise

Create a ROS 2 node that publishes IMU sensor data to a topic called `imu_data` and another node that subscribes to this topic to monitor the humanoid's orientation.

## Summary

Topics and services provide the fundamental communication mechanisms in ROS 2. Understanding when and how to use each pattern is essential for creating robust and efficient humanoid robotics applications. Topics are ideal for continuous data streams, while services are perfect for request-response interactions.