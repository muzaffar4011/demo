---
sidebar_position: 4
title: Nav2 Bipedal Planning
---

# Nav2 for Bipedal Path Planning

Navigation 2 (Nav2) is the ROS 2 navigation stack that provides path planning and navigation capabilities. For humanoid robots, Nav2 requires special configuration to handle bipedal locomotion patterns and unique mobility constraints.

## Learning Objectives

After completing this lesson, you will be able to:
- Configure Nav2 for humanoid robot navigation
- Adapt path planning algorithms for bipedal locomotion
- Implement custom controllers for humanoid walking patterns
- Integrate Nav2 with humanoid-specific perception systems

## Introduction to Humanoid Navigation Challenges

Bipedal navigation presents unique challenges compared to wheeled robots:
- **Balance constraints**: Limited to specific foot placements
- **Dynamic stability**: Requires continuous balance control
- **Footstep planning**: Must plan discrete footstep locations
- **Terrain adaptability**: Need to handle stairs, slopes, and uneven surfaces
- **Human-aware navigation**: Must navigate safely around humans

## Nav2 Architecture for Humanoid Robots

### Custom Costmap Configuration

Humanoid robots need specialized costmaps that consider:
- Traversable terrain for bipedal locomotion
- Step height and width constraints
- Balance and stability requirements

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
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.50
        inflate_to_robot_footprint: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      use_sim_time: true
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_range: 2.5
          raytrace_range: 3.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.30
```

## Footstep Planning for Bipedal Robots

### Custom Footstep Planner

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
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
        self.step_height = 0.05 # meters (for stepping over obstacles)

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

        if len(path_poses) < 2:
            return footsteps

        # Start with current position
        current_pos = path_poses[0].pose.position
        footsteps.append(path_poses[0].pose)

        # Generate footsteps along the path
        for i in range(1, len(path_poses)):
            target_pos = path_poses[i].pose.position

            # Calculate direction and distance
            dx = target_pos.x - current_pos.x
            dy = target_pos.y - current_pos.y
            distance = np.sqrt(dx*dx + dy*dy)

            # Generate intermediate footsteps if needed
            if distance > self.step_length:
                num_steps = int(distance / self.step_length)

                for j in range(1, num_steps + 1):
                    ratio = j / num_steps
                    step_x = current_pos.x + ratio * dx
                    step_y = current_pos.y + ratio * dy

                    step_pose = path_poses[i].pose
                    step_pose.position.x = step_x
                    step_pose.position.y = step_y
                    footsteps.append(step_pose)

            current_pos = target_pos

        return footsteps

def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlanner()

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

## Custom Controllers for Humanoid Walking

### Walking Pattern Generator

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
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

        # Timer for walking pattern generation
        self.timer = self.create_timer(0.02, self.generate_walking_pattern)  # 50Hz

        self.current_vel = Twist()
        self.phase = 0.0

    def cmd_vel_callback(self, msg):
        """Update walking velocity"""
        self.current_vel = msg

    def generate_walking_pattern(self):
        """Generate walking joint commands based on velocity"""
        joint_state = JointState()
        joint_state.name = [
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
        ]

        # Generate walking pattern based on desired velocity
        # This is a simplified example - real implementation would be more complex
        joint_positions = self.calculate_joint_positions()
        joint_state.position = joint_positions

        # Add timing information
        joint_state.header.stamp = self.get_clock().now().to_msg()

        self.joint_cmd_pub.publish(joint_state)

    def calculate_joint_positions(self):
        """Calculate joint positions for walking pattern"""
        # Simplified walking pattern calculation
        # In practice, this would use inverse kinematics and dynamic balance

        # Calculate phase based on time and step frequency
        self.phase += 0.02 * self.step_frequency * 2 * np.pi

        # Calculate walking pattern based on phase
        left_leg = self.calculate_leg_position('left', self.phase)
        right_leg = self.calculate_leg_position('right', self.phase + np.pi)

        # Combine with torso and arm positions
        positions = left_leg + [0.0, 0.0, 0.0] + right_leg + [0.0, 0.0, 0.0]

        return positions

    def calculate_leg_position(self, side, phase):
        """Calculate joint positions for a leg based on walking phase"""
        # Simplified leg position calculation
        # Real implementation would include balance control
        hip_pitch = 0.1 * np.sin(phase) if side == 'left' else 0.1 * np.sin(phase)
        knee = 0.2 * np.cos(phase)
        ankle = -0.1 * np.sin(phase)

        return [0.0, 0.0, hip_pitch, knee, ankle, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidWalkingController()

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

## Behavior Trees for Humanoid Navigation

### Custom Behavior Tree Nodes

```xml
<!-- Example behavior tree for humanoid navigation -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="6" name="NavigateRecovery">
            <PipelineSequence name="NavigateWithReplanning">
                <RateController hz="1.0">
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                </RateController>
                <SmoothPath path="{path}" output_path="{smoothed_path}" max_vel="0.3" max_acc="0.5"/>
                <TruncatePath path="{smoothed_path}" distance="1.0" output_path="{truncated_path}"/>
                <RemovePassedGoals goal="{goal}" path="{truncated_path}" output_goal="{updated_goal}"/>
                <FollowPath path="{truncated_path}" controller_id="FollowPath"/>
            </PipelineSequence>
            <ReactiveFallback name="RecoveryFallback">
                <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
                <BackUp backup_dist="0.30" backup_speed="0.05" name="Backup"/>
                <Spin spin_dist="1.57" name="Spin"/>
                <Wait wait_duration="5" name="Wait"/>
            </ReactiveFallback>
        </RecoveryNode>
    </BehaviorTree>
</root>
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
          clearing: False
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.5
```

## Terrain Adaptation for Humanoid Navigation

### Stair and Slope Navigation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from scipy import ndimage

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

        self.slope_pub = self.create_publisher(
            Bool,
            '/terrain/slope_detected',
            10
        )

    def pointcloud_callback(self, msg):
        """Analyze terrain for stairs and slopes"""
        # Convert PointCloud2 to numpy array
        # This is a simplified example
        terrain_data = self.process_pointcloud(msg)

        # Analyze for stairs
        stairs_detected = self.detect_stairs(terrain_data)
        self.stair_pub.publish(Bool(data=stairs_detected))

        # Analyze for slopes
        slope_detected = self.detect_slope(terrain_data)
        self.slope_pub.publish(Bool(data=slope_detected))

    def detect_stairs(self, terrain_data):
        """Detect stairs in terrain data"""
        # Simple stair detection algorithm
        # In practice, this would use more sophisticated methods
        if terrain_data is not None:
            # Look for step-like patterns in elevation data
            elevation_changes = np.diff(terrain_data, axis=0)
            step_count = np.sum(np.abs(elevation_changes) > 0.15)  # 15cm steps
            return step_count > 2
        return False

    def detect_slope(self, terrain_data):
        """Detect slopes in terrain data"""
        if terrain_data is not None:
            # Calculate terrain gradient
            grad_x, grad_y = np.gradient(terrain_data)
            slope_angle = np.arctan(np.sqrt(grad_x**2 + grad_y**2))
            max_slope = np.max(slope_angle)
            # Return True if slope is above 15 degrees
            return np.rad2deg(max_slope) > 15
        return False

def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalyzer()

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

## Performance Optimization for Humanoid Navigation

### Real-time Considerations

```yaml
# Optimized parameters for real-time humanoid navigation
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Lower frequency for complex planning
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0      # Higher frequency for balance
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller/MppiController"
      time_steps: 20
      control_frequency: 20.0
      batch_size: 2000
      model_dt: 0.05
      wheels_separation: 0.3
      wheels_radius: 0.1
```

## Exercise

Configure Nav2 for a humanoid robot with:
- Custom costmap layers for bipedal navigation
- Footstep planner that converts continuous paths to discrete steps
- Walking controller that generates appropriate joint commands
- Terrain analysis for stairs and slope detection

## Summary

Nav2 requires significant customization for humanoid robot navigation, including specialized costmaps, footstep planning, and walking controllers. Proper configuration enables safe and effective bipedal navigation in complex environments. The next lesson will cover the Vision-Language-Action systems for humanoid robots.