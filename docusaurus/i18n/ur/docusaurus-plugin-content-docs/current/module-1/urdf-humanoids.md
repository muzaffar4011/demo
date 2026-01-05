---
sidebar_position: 5
title: ہیومینوئڈز کے لیے URDF
---

# ہیومینوئڈز کے لیے URDF

Unified Robot Description Format (URDF) روبوٹ ماڈل کی نمائندگی کے لیے ایک XML format ہے۔ URDF کا استعمال ROS میں روبوٹ کی جسمانی اور kinematic خصوصیات کو بیان کرنے کے لیے کیا جاتا ہے، بشمول اس کے joints، links، اور visual/inertial خصوصیات۔ ہیومینوئڈ روبوٹس کے لیے، URDF simulation، visualization، اور kinematic calculations کے لیے بہت اہم ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- URDF فائلوں کی ساخت اور اجزاء کو سمجھنا
- ہیومینوئڈ روبوٹس کے لیے URDF ماڈلز بنانا
- ہیومینوئڈ kinematics کے لیے joints، links، اور materials کی تعریف کرنا
- پیچیدہ ہیومینوئڈ URDF ماڈلز کو آسان بنانے کے لیے Xacro استعمال کرنا

## URDF کا تعارف

URDF (Unified Robot Description Format) ایک XML-based format ہے جو ROS میں روبوٹ ماڈلز کو بیان کرنے کے لیے استعمال ہوتا ہے۔ ایک URDF فائل میں معلومات شامل ہوتی ہیں:
- روبوٹ kinematics (joints اور links)
- جسمانی خصوصیات (mass، inertia، collision properties)
- Visual properties (visualization کے لیے ظاہری شکل)
- Materials اور colors

ہیومینوئڈ روبوٹس کے لیے، URDF ضروری ہے:
- Gazebo میں simulation
- RViz میں روبوٹ visualization
- Kinematic calculations (forward/inverse kinematics)
- Collision detection

## بنیادی URDF ساخت

ایک کم سے کم URDF فائل میں joints کے بغیر ایک واحد base link ہوتا ہے:

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

## ہیومینوئڈ روبوٹس کے لیے URDF اجزاء

### Links
Links روبوٹ میں rigid bodies کی نمائندگی کرتے ہیں۔ ہیومینوئڈ کے لیے، ان میں شامل ہیں:
- Torso
- Head
- Arms (upper arm، lower arm، hand)
- Legs (upper leg، lower leg، foot)
- مختلف joint components

### Joints
Joints links کو جوڑتے ہیں اور ان کی حرکت کی تعریف کرتے ہیں۔ ہیومینوئڈ روبوٹس کے لیے، عام joints شامل ہیں:
- Revolute (rotational)
- Continuous (unlimited rotation)
- Prismatic (linear motion)
- Fixed (no motion)

## مثال: سادہ ہیومینوئڈ روبوٹ URDF

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

## ہیومینوئڈ Joint Types اور Configuration

ہیومینوئڈ روبوٹس کے لیے، مخصوص joint configurations اہم ہیں:

### Limits کے ساتھ Revolute Joints
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

### Symmetry کے لیے Mimic Joints
Mimic joints symmetric parts جیسے دونوں بازوؤں کے لیے مفید ہیں:
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

## پیچیدہ ہیومینوئڈ ماڈلز کے لیے Xacro استعمال کرنا

Xacro (XML Macros) macros، properties، اور mathematical expressions کی اجازت دے کر پیچیدہ URDF ماڈلز کو منظم کرنے میں مدد کرتا ہے:

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

## URDF Validation اور Testing

اپنی URDF فائلوں کو validate کرنے کے لیے:

1. **Syntax چیک کریں**: `check_urdf` کمانڈ استعمال کریں
   ```bash
   check_urdf /path/to/your/robot.urdf
   ```

2. **Visualize**: اپنے روبوٹ کو visualize کرنے کے لیے `rviz` استعمال کریں
   ```bash
   ros2 run rviz2 rviz2
   ```

3. **Gazebo میں ٹیسٹ**: kinematics verify کرنے کے لیے simulation میں اپنا روبوٹ لوڈ کریں

## ہیومینوئڈ ماڈلز میں عام URDF مسائل

1. **غلط origins**: یقینی بنائیں کہ joint origins صحیح طور پر positioned ہیں
2. **Mass properties**: physics simulation کے لیے realistic masses اور inertias سیٹ کریں
3. **Joint limits**: جسمانی constraints سے میل کھانے کے لیے مناسب limits سیٹ کریں
4. **Collision vs visual**: simulation کے لیے collision اور visual دونوں elements ضروری ہیں

## مشق

ایک سادہ URDF ماڈل بنائیں ہیومینوئڈ روبوٹ کے لیے جس میں شامل ہو:
- ایک torso اور head
- shoulder اور elbow joints کے ساتھ دو بازو
- hip اور knee joints کے ساتھ دو ٹانگیں
- redundancy کم کرنے کے لیے properties اور macros کی تعریف کے لیے Xacro استعمال کریں

## خلاصہ

URDF ROS میں ہیومینوئڈ روبوٹکس کے لیے بنیادی ہے، درست simulation، visualization، اور kinematic calculations کو ممکن بناتا ہے۔ Xacro استعمال کرنا بہت سے repeated elements کے ساتھ پیچیدہ ہیومینوئڈ ماڈلز کو منظم کرنے میں مدد کرتا ہے۔ مناسب URDF ماڈلز simulation environments میں مؤثر ہیومینوئڈ روبوٹ ڈویلپمنٹ اور testing کے لیے ضروری ہیں۔

