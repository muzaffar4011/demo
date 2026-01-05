---
sidebar_position: 4
title: rclpy برج
---

# rclpy برج

`rclpy` package ROS 2 کے لیے Python کلائنٹ لائبریری ہے۔ یہ ROS 2 packages تیار کرنے کے لیے Python API فراہم کرتا ہے اور Python ڈویلپرز کو ROS 2 نوڈز، publishers، subscribers، سروسز، اور مزید بنانے کے قابل بناتا ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ROS 2 Python ڈویلپمنٹ میں rclpy کے کردار کو سمجھنا
- rclpy کا استعمال کرتے ہوئے ROS 2 نوڈز بنانا
- rclpy کے ساتھ publishers، subscribers، سروسز، اور کلائنٹس کو لاگو کرنا
- ہیومینوئڈ روبوٹکس ایپلی کیشنز کے لیے Python ایجنٹس کو ROS 2 کے ساتھ جوڑنا

## rclpy کا تعارف

`rclpy` ROS 2 کے لیے سرکاری Python کلائنٹ لائبریری ہے۔ یہ ROS 2 middleware (RCL - ROS Client Library) کے لیے Python bindings فراہم کرتا ہے، Python ڈویلپرز کو ROS 2 ecosystem کے ساتھ تعامل کرنے کی اجازت دیتا ہے۔ ہیومینوئڈ روبوٹکس ایپلی کیشنز کے لیے، rclpy پیچیدہ Python-based AI الگورتھمز کو ROS 2 مواصلاتی فریم ورک کے ساتھ انضمام کو ممکن بناتا ہے۔

### rclpy کے اہم اجزاء

- **Node**: بنیادی execution unit جو دوسرے نوڈز کے ساتھ بات چیت کر سکتا ہے
- **Publisher**: ٹاپکس پر میسجز بھیجتا ہے
- **Subscriber**: ٹاپکس سے میسجز وصول کرتا ہے
- **Service Server**: سروسز فراہم کرتا ہے جنہیں دوسرے نوڈز کال کر سکتے ہیں
- **Service Client**: دوسرے نوڈز کی فراہم کردہ سروسز کو کال کرتا ہے
- **Timer**: باقاعدہ وقفوں پر callbacks کو execute کرتا ہے
- **Parameter**: نوڈ configuration پیرامیٹرز کو handle کرتا ہے

## rclpy کے ساتھ بنیادی نوڈ کی ساخت

یہاں rclpy استعمال کرتے ہوئے ROS 2 نوڈ کے لیے معیاری ساخت ہے:

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

## rclpy کے ساتھ Publishers اور Subscribers

### Publisher مثال

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

### Subscriber مثال

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

## rclpy کے ساتھ سروسز اور کلائنٹس

### Service Server مثال

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

### Service Client مثال

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

## ہیومینوئڈ روبوٹکس کے لیے اعلیٰ درجے کی rclpy خصوصیات

### Custom Message Types کے ساتھ کام کرنا

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

## ہیومینوئڈ روبوٹکس میں rclpy کے لیے بہترین طریقے

1. **Error Handling**: اپنے callbacks میں ہمیشہ مناسب error handling لاگو کریں
2. **Resource Management**: destroy_node method میں وسائل کو صاف کریں
3. **Threading**: rclpy استعمال کرتے وقت threading implications سے آگاہ رہیں
4. **Performance**: ریئل-ٹائم ضروریات کی بنیاد پر میسج publishing rates کو optimize کریں
5. **Logging**: مناسب log levels استعمال کریں (info، warn، error، debug)

## مشق

ایک Python نوڈ بنائیں جو ایک سادہ AI فیصلہ سازی الگورتھم کو ROS 2 کے ساتھ جوڑتا ہے۔ نوڈ کو:
1. سینسر ڈیٹا ٹاپکس کی سبسکرائب کرنی چاہیے
2. سینسر input کی بنیاد پر فیصلے کرنے چاہئیں
3. actuator ٹاپکس پر کمانڈز شائع کرنی چاہئیں
4. AI رویے کو configure کرنے کے لیے پیرامیٹرز استعمال کرنے چاہئیں

## خلاصہ

rclpy لائبریری Python ڈویلپرز کو ہیومینوئڈ روبوٹکس ایپلی کیشنز کے لیے ROS 2 نوڈز بنانے کے لیے طاقتور ٹولز فراہم کرتی ہے۔ اس کے بنیادی اجزاء اور بہترین طریقوں کو سمجھ کر، آپ Python-based AI الگورتھمز کو ROS 2 مواصلاتی فریم ورک کے ساتھ مؤثر طریقے سے جوڑ سکتے ہیں، پیچیدہ روبوٹک رویوں کو ممکن بناتے ہیں۔

