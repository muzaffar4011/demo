---
sidebar_position: 5
title: URDF for Humanoids
---

# URDF for Humanoids

Unified Robot Description Format (URDF) is an XML format for representing a robot model. URDF is used in ROS to describe the physical and kinematic properties of a robot, including its joints, links, and visual/inertial properties. For humanoid robots, URDF is crucial for simulation, visualization, and kinematic calculations.

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the structure and components of URDF files
- Create URDF models for humanoid robots
- Define joints, links, and materials for humanoid kinematics
- Use Xacro to simplify complex humanoid URDF models

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. A URDF file contains information about:
- Robot kinematics (joints and links)
- Physical properties (mass, inertia, collision properties)
- Visual properties (appearance for visualization)
- Materials and colors

For humanoid robots, URDF is essential for:
- Simulation in Gazebo
- Robot visualization in RViz
- Kinematic calculations (forward/inverse kinematics)
- Collision detection

## Basic URDF Structure

A minimal URDF file has a single base link with no joints:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
</robot>
```

## URDF Components for Humanoid Robots

### Links
Links represent rigid bodies in the robot. For a humanoid, these include:
- Torso
- Head
- Arms (upper arm, lower arm, hand)
- Legs (upper leg, lower leg, foot)
- Various joint components

### Joints
Joints connect links and define their movement. For humanoid robots, common joints include:
- Revolute (rotational)
- Continuous (unlimited rotation)
- Prismatic (linear motion)
- Fixed (no motion)

## Example: Simple Humanoid Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="1"/>
  </joint>

</robot>
```

## Humanoid Joint Types and Configuration

For humanoid robots, specific joint configurations are important:

### Revolute Joints with Limits
```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_leg"/>
  <origin xyz="0 -0.1 -0.1"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Mimic Joints for Symmetry
Mimic joints are useful for symmetric parts like both arms:
```xml
<joint name="right_elbow" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_lower_arm"/>
  <origin xyz="0 0 -0.3"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="3.14" effort="100" velocity="1"/>
  <mimic joint="left_elbow" multiplier="1" offset="0"/>
</joint>
```

## Using Xacro for Complex Humanoid Models

Xacro (XML Macros) helps manage complex URDF models by allowing macros, properties, and mathematical expressions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.4" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="leg_length" value="0.5" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- Macros for repeated elements -->
  <xacro:macro name="simple_arm" params="side reflect">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
        <material name="grey"/>
        <origin rpy="0 1.57 0" xyz="0 0 ${-arm_length/2}"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
        <origin rpy="0 1.57 0" xyz="0 0 ${-arm_length/2}"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_yaw" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${0.15 * reflect} 0 ${torso_height/4}"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 ${torso_height}"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 ${torso_height/2 + 0.05}"/>
  </joint>

  <!-- Use the macro to create both arms -->
  <xacro:simple_arm side="left" reflect="1"/>
  <xacro:simple_arm side="right" reflect="-1"/>

</robot>
```

## URDF Validation and Testing

To validate your URDF files:

1. **Check syntax**: Use `check_urdf` command
   ```bash
   check_urdf /path/to/your/robot.urdf
   ```

2. **Visualize**: Use `rviz` to visualize your robot
   ```bash
   ros2 run rviz2 rviz2
   ```

3. **Test in Gazebo**: Load your robot in simulation to verify kinematics

## Common URDF Issues in Humanoid Models

1. **Incorrect origins**: Make sure joint origins are correctly positioned
2. **Mass properties**: Set realistic masses and inertias for physics simulation
3. **Joint limits**: Set appropriate limits to match physical constraints
4. **Collision vs visual**: Both collision and visual elements are needed for simulation

## Exercise

Create a simplified URDF model for a humanoid robot with:
- A torso and head
- Two arms with shoulder and elbow joints
- Two legs with hip and knee joints
- Use Xacro to define properties and macros to reduce redundancy

## Summary

URDF is fundamental to humanoid robotics in ROS, enabling accurate simulation, visualization, and kinematic calculations. Using Xacro helps manage complex humanoid models with many repeated elements. Proper URDF models are essential for effective humanoid robot development and testing in simulation environments.