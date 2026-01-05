---
sidebar_position: 3
title: ٹاپکس اور سروسز
---

# ٹاپکس اور سروسز

ROS 2 دو بنیادی مواصلاتی پیٹرنز فراہم کرتا ہے: ٹاپکس (asynchronous، broadcast مواصلات کے لیے) اور سروسز (synchronous، request-response مواصلات کے لیے)۔ ان پیٹرنز کو سمجھنا موثر روبوٹک سسٹمز ڈیزائن کرنے کے لیے بہت اہم ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ٹاپکس اور سروسز کے درمیان فرق کو سمجھنا
- ٹاپکس کے لیے publishers اور subscribers کو لاگو کرنا
- Request-response مواصلات کے لیے سروسز بنانا اور استعمال کرنا
- ہیومینوئڈ روبوٹکس منظرناموں میں مناسب مواصلاتی پیٹرنز لاگو کرنا

## ٹاپکس: Publisher-Subscriber پیٹرن

ٹاپکس نوڈز کے درمیان asynchronous، one-to-many مواصلات کو ممکن بناتے ہیں۔ Publishers ایک ٹاپک پر میسجز بھیجتے ہیں، اور کسی بھی تعداد میں subscribers ان میسجز کو وصول کر سکتے ہیں۔ یہ پیٹرن مسلسل ڈیٹا سٹریمز جیسے سینسر ریڈنگز یا روبوٹ کی حالت کی معلومات کے لیے مثالی ہے۔

### مثال: سینسر ڈیٹا ٹاپک

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

### ٹاپکس کی سبسکرائب کرنا

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

## سروسز: Request-Response پیٹرن

سروسز synchronous، one-to-one مواصلات فراہم کرتی ہیں جہاں ایک کلائنٹ ایک درخواست بھیجتا ہے اور جواب کا انتظار کرتا ہے۔ یہ پیٹرن ان اعمال کے لیے مثالی ہے جن کے لیے تصدیق یا کمپیوٹیشن کی ضرورت ہوتی ہے، جیسے کسی مخصوص روبوٹ pose کی درخواست کرنا یا پیچیدہ رویے کو انجام دینا۔

### مثال: روبوٹ Pose سروس

پہلے، ایک `.srv` فائل میں سروس انٹرفیس کی تعریف کریں (مثلاً، `SetPose.srv`):

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

پھر سروس سرور کو لاگو کریں:

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

اور سروس کلائنٹ:

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

## کب ٹاپکس استعمال کریں vs سروسز

### ٹاپکس استعمال کریں جب:
- متعدد subscribers کو معلومات broadcast کرنا
- مسلسل ڈیٹا سٹریمز (سینسرز، روبوٹ کی حالت)
- ریئل-ٹائم ضروریات (no blocking)
- ڈیٹا کو acknowledgment کی ضرورت نہیں

### سروسز استعمال کریں جب:
- مخصوص اعمال یا کمپیوٹیشنز کی درخواست کرنا
- تکمیل کی تصدیق کی ضرورت
- One-to-one مواصلات کافی ہے
- Request-response پیٹرن use case کے مطابق ہے

## ہیومینوئڈ روبوٹکس مواصلاتی پیٹرنز

ہیومینوئڈ روبوٹکس میں، عام مواصلاتی پیٹرنز شامل ہیں:

- **ٹاپکس**: جوائنٹ states، سینسر ڈیٹا، روبوٹ pose، کیمرہ feeds
- **سروسز**: روبوٹ pose سیٹ کریں، رویے کو انجام دیں، calibration، emergency stop
- **ایکشنز**: پیچیدہ کام جو وقت لیتے ہیں (walking، grasping) feedback کے ساتھ

## مشق

ایک ROS 2 نوڈ بنائیں جو `imu_data` نامی ٹاپک پر IMU سینسر ڈیٹا شائع کرتا ہے اور ایک اور نوڈ جو اس ٹاپک کی سبسکرائب کرتا ہے تاکہ ہیومینوئڈ کی orientation کو مانیٹر کر سکے۔

## خلاصہ

ٹاپکس اور سروسز ROS 2 میں بنیادی مواصلاتی میکانزم فراہم کرتے ہیں۔ ہر پیٹرن کو کب اور کیسے استعمال کرنا ہے یہ سمجھنا مضبوط اور موثر ہیومینوئڈ روبوٹکس ایپلی کیشنز بنانے کے لیے ضروری ہے۔ ٹاپکس مسلسل ڈیٹا سٹریمز کے لیے مثالی ہیں، جبکہ سروسز request-response تعاملات کے لیے بہترین ہیں۔

