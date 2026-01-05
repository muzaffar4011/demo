---
sidebar_position: 5
title: Sensors
---

# Simulation میں Sensor Integration

سینسرز ڈیجیٹل ٹوئنز کے اہم اجزاء ہیں، ہیومینوئڈ روبوٹکس ایپلی کیشنز میں perception، navigation، اور control کے لیے ضروری ڈیٹا فراہم کرتے ہیں۔ یہ سبق simulation environments میں مختلف sensor types کے انضمام کا احاطہ کرتا ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ہیومینوئڈ روبوٹ simulation میں مختلف قسم کے سینسرز کو انضمام کرنا
- Realistic ڈیٹا generation کے لیے sensor parameters configure کرنا
- Simulation میں sensor fusion techniques لاگو کرنا
- Sensor ڈیٹا accuracy اور timing validate کرنا

## Sensor Simulation کا تعارف

ڈیجیٹل ٹوئن environments میں، سینسرز کو اپنے حقیقی دنیا کے counterparts کی درست simulation کرنی چاہیے:
- **LiDAR**: فاصلہ پیمائش اور environment mapping
- **Cameras**: Visual perception اور object recognition
- **IMUs**: Orientation اور acceleration کے لیے inertial measurement
- **Force/Torque sensors**: Contact forces اور joint loads
- **GPS**: Global positioning (outdoor روبوٹس کے لیے)

ہیومینوئڈ روبوٹس کے لیے، sensor placement اہم ہے:
- Balance اور stability control
- Environment perception اور navigation
- Human-robot interaction
- Human environments میں محفوظ operation

## LiDAR Sensors

### Configuration Example

```xml
<model name="humanoid_robot">
  <link name="lidar_link">
    <sensor name="lidar_2d" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -π -->
            <max_angle>3.14159</max_angle>    <!-- π -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
    </sensor>
  </link>
</model>
```

## Camera Sensors

### RGB Camera Configuration

```xml
<sensor name="rgb_camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## IMU Sensors

### Configuration for Humanoid Balance

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0005</bias_stddev>
        </noise>
      </x>
      <!-- Similar for y and z -->
    </angular_velocity>
    <linear_acceleration>
      <!-- Configuration similar to angular_velocity -->
    </linear_acceleration>
  </imu>
</sensor>
```

## Sensor Placement for Humanoid Robots

### Head-Mounted Sensors
- **Stereo cameras**: Depth perception اور object recognition کے لیے
- **Microphones**: Voice interaction اور sound localization کے لیے
- **Infrared sensors**: قریبی range obstacle detection کے لیے

### Body-Mounted Sensors
- **IMUs**: Balance control کے لیے torso میں، motion tracking کے لیے limbs میں
- **Force sensors**: Balance کے لیے feet میں، manipulation کے لیے hands میں
- **Tactile sensors**: Contact feedback کے لیے hands اور feet میں

## Sensor Fusion in Simulation

### Example: IMU and Camera Fusion

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for IMU and camera
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(Vector3, 'fused_orientation', 10)

    def imu_callback(self, msg):
        self.latest_imu = msg
        if hasattr(self, 'latest_image'):
            self.perform_fusion()

    def camera_callback(self, msg):
        self.latest_image = msg
        if self.latest_imu:
            self.perform_fusion()

    def perform_fusion(self):
        # Simple sensor fusion example
        if self.latest_imu:
            orientation = self.latest_imu.orientation
            fused_data = Vector3()
            fused_data.x = orientation.x
            fused_data.y = orientation.y
            fused_data.z = orientation.z
            self.fused_pub.publish(fused_data)

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

### Sensor Update Rates
- **High-rate sensors** (IMU, encoders): 100-1000 Hz
- **Medium-rate sensors** (cameras): 30-60 Hz
- **Low-rate sensors** (GPS): 1-10 Hz

### Computational Load
- **LiDAR**: اعلیٰ computational cost، خاص طور پر 3D
- **Cameras**: Resolution اور processing پر منحصر moderate سے high
- **IMU**: کم computational cost
- **Force/Torque**: کم computational cost

## مشق

ایک ہیومینوئڈ روبوٹ ماڈل بنائیں جس میں شامل ہو:
- Environment mapping کے لیے LiDAR sensor
- Depth perception کے لیے Stereo cameras
- Balance control کے لیے IMU sensors
- Ground contact detection کے لیے feet میں Force/torque sensors
- IMU اور camera ڈیٹا کو ملا کر ایک سادہ sensor fusion node لاگو کریں

## خلاصہ

Sensor integration ہیومینوئڈ روبوٹکس میں realistic ڈیجیٹل ٹوئن ڈویلپمنٹ کے لیے بہت اہم ہے۔ Sensor parameters، placement، اور fusion techniques کی مناسب configuration simulation environments میں مؤثر perception اور control کو ممکن بناتی ہے۔ اگلا ماڈیول NVIDIA Isaac استعمال کرتے ہوئے AI-Robot Brain systems کا احاطہ کرے گا۔

