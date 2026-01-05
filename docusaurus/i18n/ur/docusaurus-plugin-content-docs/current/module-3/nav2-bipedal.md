---
sidebar_position: 4
title: Nav2 Bipedal Planning
---

# Bipedal Path Planning کے لیے Nav2

Navigation 2 (Nav2) ROS 2 navigation stack ہے جو path planning اور navigation capabilities فراہم کرتا ہے۔ ہیومینوئڈ روبوٹس کے لیے، Nav2 کو bipedal locomotion patterns اور منفرد mobility constraints handle کرنے کے لیے special configuration کی ضرورت ہوتی ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ہیومینوئڈ روبوٹ navigation کے لیے Nav2 configure کرنا
- Bipedal locomotion کے لیے path planning algorithms adapt کرنا
- ہیومینوئڈ walking patterns کے لیے custom controllers لاگو کرنا
- Nav2 کو ہیومینوئڈ-specific perception systems کے ساتھ انضمام کرنا

## ہیومینوئڈ Navigation چیلنجز کا تعارف

Bipedal navigation wheeled روبوٹس کے مقابلے میں منفرد چیلنجز پیش کرتا ہے:
- **Balance constraints**: مخصوص foot placements تک محدود
- **Dynamic stability**: مسلسل balance control کی ضرورت
- **Footstep planning**: Discrete footstep locations plan کرنا ضروری
- **Terrain adaptability**: سیڑھیاں، ڈھلوانیں، اور uneven surfaces handle کرنے کی ضرورت
- **Human-aware navigation**: Humans کے ارد گرد محفوظ طریقے سے navigate کرنا ضروری

## ہیومینوئڈ روبوٹس کے لیے Nav2 Architecture

### Custom Costmap Configuration

ہیومینوئڈ روبوٹس کو specialized costmaps کی ضرورت ہوتی ہے جو غور کرتے ہیں:
- Bipedal locomotion کے لیے traversable terrain
- Step height اور width constraints
- Balance اور stability requirements

```yaml
# Costmap configuration for humanoid robot
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "map"
        transform_tolerance: 0.2

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
```

## Footstep Planning for Bipedal Robots

### Custom Footstep Planner

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.footstep_pub = self.create_publisher(
            Path,
            '/footstep_plan',
            10
        )

        # Humanoid-specific parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters
        self.step_height = 0.05 # meters

    def path_callback(self, msg):
        """Convert continuous path to discrete footsteps"""
        footstep_path = Path()
        footstep_path.header = msg.header

        # Generate footsteps along the path
        footsteps = self.generate_footsteps(msg.poses)

        for footstep in footsteps:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = footstep
            footstep_path.poses.append(pose_stamped)

        self.footstep_pub.publish(footstep_path)

    def generate_footsteps(self, path_poses):
        """Generate discrete footsteps from continuous path"""
        footsteps = []
        # Implementation details...
        return footsteps
```

## Custom Controllers for Humanoid Walking

### Walking Pattern Generator

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class HumanoidWalkingController(Node):
    def __init__(self):
        super().__init__('humanoid_walking_controller')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # Walking parameters
        self.step_frequency = 1.0  # steps per second
        self.step_length = 0.3     # meters
        self.step_height = 0.05    # meters

        self.timer = self.create_timer(0.02, self.generate_walking_pattern)
        self.current_vel = Twist()
        self.phase = 0.0

    def cmd_vel_callback(self, msg):
        """Update walking velocity"""
        self.current_vel = msg

    def generate_walking_pattern(self):
        """Generate walking joint commands based on velocity"""
        # Implementation details...
        pass
```

## Human-Aware Navigation

### Social Navigation Parameters

```yaml
# Social navigation configuration
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "voxel_layer", "social_layer", "inflation_layer"]

      social_layer:
        plugin: "nav2_social_layer::SocialLayer"
        enabled: True
        observation_sources: people
        people:
          topic: /people_positions
          max_obstacle_height: 2.0
```

## Terrain Adaptation for Humanoid Navigation

### Stair and Slope Navigation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import numpy as np

class TerrainAnalyzer(Node):
    def __init__(self):
        super().__init__('terrain_analyzer')

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        self.stair_pub = self.create_publisher(
            Bool,
            '/terrain/stairs_detected',
            10
        )

    def pointcloud_callback(self, msg):
        """Analyze terrain for stairs and slopes"""
        # Implementation details...
        pass
```

## مشق

ایک ہیومینوئڈ روبوٹ کے لیے Nav2 configure کریں جس میں شامل ہو:
- Bipedal navigation کے لیے custom costmap layers
- Continuous paths کو discrete steps میں تبدیل کرنے والا footstep planner
- Appropriate joint commands generate کرنے والا walking controller
- سیڑھیاں اور ڈھلوان detection کے لیے terrain analysis

## خلاصہ

Nav2 کو ہیومینوئڈ روبوٹ navigation کے لیے significant customization کی ضرورت ہوتی ہے، بشمول specialized costmaps، footstep planning، اور walking controllers۔ مناسب configuration پیچیدہ environments میں محفوظ اور مؤثر bipedal navigation کو ممکن بناتی ہے۔ اگلا ماڈیول ہیومینوئڈ روبوٹس کے لیے Vision-Language-Action systems کا احاطہ کرے گا۔

