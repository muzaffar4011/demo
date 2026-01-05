---
sidebar_position: 3
title: Collisions
---

# Collision Detection and Handling

Collision detection is a critical component of physics simulation for humanoid robots, ensuring realistic interactions with the environment and preventing parts of the robot from passing through obstacles or itself.

## Learning Objectives

After completing this lesson, you will be able to:
- Implement accurate collision geometries for humanoid robot models
- Configure collision detection parameters for optimal performance
- Handle self-collision and environment collision scenarios
- Optimize collision detection for real-time humanoid robot simulation

## Introduction to Collision Detection

Collision detection in robotic simulation involves:
- **Broad phase**: Quick elimination of non-colliding pairs
- **Narrow phase**: Precise collision detection for potentially colliding pairs
- **Contact generation**: Determining contact points, normals, and penetration depth

For humanoid robots, collision detection is particularly challenging due to:
- Complex articulated structures with many potential collision pairs
- Close proximity of limbs during natural movements
- Need for accurate contact points for balance and manipulation

## Collision Geometries

### Simple Geometries
For performance, simple geometries are preferred:

```xml
<!-- Box collision for torso -->
<collision name="torso_collision">
  <geometry>
    <box>
      <size>0.2 0.15 0.4</size>
    </box>
  </geometry>
</collision>

<!-- Cylinder collision for limbs -->
<collision name="upper_arm_collision">
  <geometry>
    <cylinder>
      <length>0.3</length>
      <radius>0.06</radius>
    </cylinder>
  </geometry>
</collision>

<!-- Sphere collision for simplified elements -->
<collision name="head_collision">
  <geometry>
    <sphere>
      <radius>0.1</radius>
    </sphere>
  </geometry>
</collision>
```

### Mesh Collisions
For high accuracy, mesh collisions can be used, but they are computationally expensive:

```xml
<collision name="detailed_hand_collision">
  <geometry>
    <mesh>
      <uri>model://humanoid/meshes/hand_collision.dae</uri>
    </mesh>
  </geometry>
</collision>
```

## Self-Collision Avoidance

Humanoid robots must avoid self-collision during movement:

```xml
<link name="left_upper_arm">
  <collision name="upper_arm_collision">
    <geometry>
      <cylinder>
        <length>0.3</length>
        <radius>0.05</radius>
      </cylinder>
    </geometry>
    <!-- Disable self-collision with adjacent links -->
    <self_collide>false</self_collide>
  </collision>
</link>

<!-- For complex self-collision scenarios -->
<joint name="shoulder" type="revolute">
  <physics>
    <ode>
      <provide_feedback>true</provide_feedback>
    </ode>
  </physics>
</joint>
```

## Collision Parameters and Optimization

### Contact Parameters
Fine-tune contact behavior for realistic interactions:

```xml
<collision name="foot_collision">
  <geometry>
    <box>
      <size>0.15 0.1 0.05</size>
    </box>
  </geometry>
  <surface>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1000000000000.0</kp>
        <kd>1000000000000.0</kd>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

### Collision Detection Settings
Configure global collision detection parameters:

```xml
<world name="humanoid_world">
  <physics type="ode">
    <!-- Collision detection parameters -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>

    <!-- Contact parameters -->
    <ode>
      <solver>
        <type>quick</type>
        <iters>1000</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.000001</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</world>
```

## Collision Filtering

Prevent unnecessary collision checks between specific links:

```xml
<!-- In URDF/Xacro, use collision groups -->
<gazebo reference="left_upper_arm">
  <collision>
    <surface>
      <contact>
        <collide_without_contact>0</collide_without_contact>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Environment Collision Detection

### Ground Plane
Proper ground collision is essential for humanoid locomotion:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
    </visual>
  </link>
</model>
```

### Obstacle Detection
Configure collision properties for environmental objects:

```xml
<model name="obstacle_box">
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.5</mu>
            <mu2>0.5</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
      </surface>
    </collision>
  </link>
</model>
```

## Performance Optimization

### Collision Simplification
Balance accuracy with performance:

1. **Use simple geometries** where possible (boxes, cylinders, spheres)
2. **Adjust collision mesh resolution** for complex shapes
3. **Disable collision** for links that don't need it during certain operations
4. **Use bounding volume hierarchies** for complex models

### Simulation Parameters
Tune for humanoid robot simulation:

- **Time step**: 0.001s for accurate collision detection
- **Solver iterations**: Higher for stable contact (50-200)
- **Contact surface layer**: Prevents jitter (0.001-0.01)
- **Max correcting velocity**: Limits contact response (1-100)

## Common Collision Issues in Humanoid Simulation

### Penetration
- **Cause**: Large time steps or insufficient solver iterations
- **Solution**: Reduce time step, increase solver iterations

### Jittering
- **Cause**: Inadequate contact stabilization
- **Solution**: Adjust ERP (Error Reduction Parameter) and CFM (Constraint Force Mixing)

### Self-Collision Conflicts
- **Cause**: Aggressive joint movements
- **Solution**: Implement joint limits and collision filters

## Advanced Collision Techniques

### Custom Contact Sensors
Implement custom collision detection for specific applications:

```xml
<sensor name="contact_sensor" type="contact">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <contact>
    <collision>foot_collision</collision>
  </contact>
  <plugin name="contact_plugin" filename="libContactPlugin.so">
    <topic>contact_info</topic>
  </plugin>
</sensor>
```

### Dynamic Collision Environments
For complex scenarios:

```xml
<model name="movable_object">
  <link name="link">
    <!-- Physics properties for dynamic objects -->
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1</iyy>
        <iyz>0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

## Exercise

Create a humanoid robot model with appropriate collision geometries for all links. Test the model in Gazebo with various obstacles to ensure proper collision detection and response.

## Summary

Collision detection is essential for realistic humanoid robot simulation. Proper configuration of collision geometries, parameters, and optimization techniques ensures accurate and efficient simulation. In the next lesson, we'll explore rendering and visualization aspects of digital twins.