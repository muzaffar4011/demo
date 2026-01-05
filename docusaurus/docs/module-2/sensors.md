---
sidebar_position: 5
title: Sensors
---

# Sensor Integration in Simulation

Sensors are crucial components of digital twins, providing the data needed for perception, navigation, and control in humanoid robotics applications. This lesson covers the integration of various sensor types in simulation environments.

## Learning Objectives

After completing this lesson, you will be able to:
- Integrate different types of sensors in humanoid robot simulation
- Configure sensor parameters for realistic data generation
- Implement sensor fusion techniques in simulation
- Validate sensor data accuracy and timing

## Introduction to Sensor Simulation

In digital twin environments, sensors must accurately simulate their real-world counterparts:
- **LiDAR**: Distance measurement and environment mapping
- **Cameras**: Visual perception and object recognition
- **IMUs**: Inertial measurement for orientation and acceleration
- **Force/Torque sensors**: Contact forces and joint loads
- **GPS**: Global positioning (for outdoor robots)

For humanoid robots, sensor placement is critical for:
- Balance and stability control
- Environment perception and navigation
- Human-robot interaction
- Safe operation in human environments

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

### 3D LiDAR Configuration

```xml
<sensor name="lidar_3d" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-2.35619</min_angle>  <!-- -135° -->
        <max_angle>2.35619</max_angle>   <!-- 135° -->
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>   <!-- -15° -->
        <max_angle>0.2618</max_angle>    <!-- 15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
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

### Depth Camera Configuration

```xml
<sensor name="depth_camera" type="depth">
  <camera name="depth_cam">
    <horizontal_fov>1.0472</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
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
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0005</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0005</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.001</bias_mean>
          <bias_stddev>0.005</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.001</bias_mean>
          <bias_stddev>0.005</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.001</bias_mean>
          <bias_stddev>0.005</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Force/Torque Sensors

### Joint Force/Torque Sensors

```xml
<sensor name="left_ankle_ft" type="force_torque">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

## GPS Sensors (for outdoor humanoid robots)

```xml
<sensor name="gps_sensor" type="gps">
  <always_on>1</always_on>
  <update_rate>1</update_rate>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.2</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.3</stddev>
        </noise>
      </vertical>
    </position_sensing>
  </gps>
</sensor>
```

## Sensor Placement for Humanoid Robots

### Head-Mounted Sensors
- **Stereo cameras**: For depth perception and object recognition
- **Microphones**: For voice interaction and sound localization
- **Infrared sensors**: For close-range obstacle detection

### Body-Mounted Sensors
- **IMUs**: In torso for balance control, in limbs for motion tracking
- **Force sensors**: In feet for balance, in hands for manipulation
- **Tactile sensors**: In hands and feet for contact feedback

### Joint Sensors
- **Encoders**: For precise joint angle measurement
- **Torque sensors**: For force control and safety

## Sensor Fusion in Simulation

### Example: IMU and Camera Fusion

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for IMU and camera
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(Vector3, 'fused_orientation', 10)

        self.bridge = CvBridge()
        self.latest_imu = None

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
            # Extract orientation from IMU
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

## Sensor Calibration in Simulation

### Simulated Calibration Process

```xml
<sensor name="calibrated_camera" type="camera">
  <camera name="calib_cam">
    <intrinsics>
      <fx>525.0</fx>
      <fy>525.0</fy>
      <cx>319.5</cx>
      <cy>239.5</cy>
    </intrinsics>
    <distortion>
      <k1>0.0</k1>
      <k2>0.0</k2>
      <k3>0.0</k3>
      <p1>0.0</p1>
      <p2>0.0</p2>
    </distortion>
  </camera>
</sensor>
```

## Performance Considerations

### Sensor Update Rates
- **High-rate sensors** (IMU, encoders): 100-1000 Hz
- **Medium-rate sensors** (cameras): 30-60 Hz
- **Low-rate sensors** (GPS): 1-10 Hz

### Computational Load
- **LiDAR**: High computational cost, especially 3D
- **Cameras**: Moderate to high depending on resolution and processing
- **IMU**: Low computational cost
- **Force/Torque**: Low computational cost

## Sensor Validation Techniques

### Data Quality Checks
1. **Range validation**: Ensure sensor readings are within expected ranges
2. **Temporal consistency**: Check for sudden jumps in sensor data
3. **Cross-validation**: Compare readings from redundant sensors
4. **Physical plausibility**: Verify readings match physical constraints

### Simulation vs. Reality Comparison
- Compare noise characteristics
- Validate timing and synchronization
- Check for systematic biases

## Common Sensor Issues in Simulation

### Noise Modeling
- **Underestimated noise**: Leads to overconfident perception
- **Overestimated noise**: Reduces sensor effectiveness
- **Non-Gaussian noise**: Can cause filter divergence

### Synchronization
- **Timestamp accuracy**: Critical for sensor fusion
- **Update rate mismatches**: Can cause integration issues
- **Latency modeling**: Important for realistic control

## Advanced Sensor Techniques

### Multi-sensor Arrays
For redundancy and enhanced perception:

```xml
<!-- Stereo camera setup -->
<sensor name="left_camera" type="camera">
  <!-- Configuration -->
</sensor>
<sensor name="right_camera" type="camera">
  <!-- Configuration with baseline offset -->
</sensor>
```

### Dynamic Sensor Configuration
Change sensor parameters based on robot state:

```xml
<gazebo>
  <plugin name="dynamic_sensor_plugin" filename="libDynamicSensorPlugin.so">
    <topic>sensor_config</topic>
    <default_rate>30</default_rate>
    <active_rate>60</active_rate>
  </plugin>
</gazebo>
```

## Exercise

Create a humanoid robot model with:
- LiDAR sensor for environment mapping
- Stereo cameras for depth perception
- IMU sensors for balance control
- Force/torque sensors in feet for ground contact detection
- Implement a simple sensor fusion node that combines IMU and camera data

## Summary

Sensor integration is crucial for realistic digital twin development in humanoid robotics. Proper configuration of sensor parameters, placement, and fusion techniques enables effective perception and control in simulation environments. The next module will cover the AI-Robot Brain systems using NVIDIA Isaac.