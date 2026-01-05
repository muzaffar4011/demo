---
sidebar_position: 3
title: Collisions
---

# Collision Detection اور Handling

Collision detection ہیومینوئڈ روبوٹس کے لیے physics simulation کا ایک اہم جزو ہے، environment کے ساتھ realistic interactions کو یقینی بناتا ہے اور روبوٹ کے parts کو obstacles یا خود سے گزرنے سے روکتا ہے۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ہیومینوئڈ روبوٹ ماڈلز کے لیے درست collision geometries لاگو کرنا
- بہترین performance کے لیے collision detection parameters configure کرنا
- Self-collision اور environment collision scenarios handle کرنا
- ریئل-ٹائم ہیومینوئڈ روبوٹ simulation کے لیے collision detection optimize کرنا

## Collision Detection کا تعارف

روبوٹک simulation میں collision detection شامل ہے:
- **Broad phase**: Non-colliding pairs کی تیزی سے elimination
- **Narrow phase**: ممکنہ colliding pairs کے لیے precise collision detection
- **Contact generation**: Contact points، normals، اور penetration depth کا تعین

ہیومینوئڈ روبوٹس کے لیے، collision detection خاص طور پر چیلنجنگ ہے کیونکہ:
- بہت سے ممکنہ collision pairs کے ساتھ پیچیدہ articulated structures
- قدرتی movements کے دوران limbs کی قریبی proximity
- Balance اور manipulation کے لیے درست contact points کی ضرورت

## Collision Geometries

### سادہ Geometries
Performance کے لیے، سادہ geometries ترجیح دی جاتی ہیں:

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
اعلیٰ accuracy کے لیے، mesh collisions استعمال کی جا سکتی ہیں، لیکن وہ computationally expensive ہیں:

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

ہیومینوئڈ روبوٹس کو movement کے دوران self-collision سے بچنا چاہیے:

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

## Collision Parameters اور Optimization

### Contact Parameters
Realistic interactions کے لیے contact behavior کو fine-tune کریں:

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
Global collision detection parameters configure کریں:

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

## Environment Collision Detection

### Ground Plane
ہیومینوئڈ locomotion کے لیے مناسب ground collision ضروری ہے:

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

## Performance Optimization

### Collision Simplification
Accuracy کے ساتھ performance کا توازن:

1. **سادہ geometries استعمال کریں** جہاں ممکن ہو (boxes، cylinders، spheres)
2. **پیچیدہ shapes کے لیے collision mesh resolution adjust کریں**
3. **مخصوص operations کے دوران collision disable کریں** links کے لیے جن کی ضرورت نہیں
4. **پیچیدہ ماڈلز کے لیے bounding volume hierarchies استعمال کریں**

### Simulation Parameters
ہیومینوئڈ روبوٹ simulation کے لیے tune کریں:

- **Time step**: درست collision detection کے لیے 0.001s
- **Solver iterations**: مستحکم contact کے لیے زیادہ (50-200)
- **Contact surface layer**: Jitter کو روکتا ہے (0.001-0.01)
- **Max correcting velocity**: Contact response کو محدود کرتا ہے (1-100)

## ہیومینوئڈ Simulation میں عام Collision Issues

### Penetration
- **وجہ**: بڑے time steps یا ناکافی solver iterations
- **حل**: Time step کم کریں، solver iterations بڑھائیں

### Jittering
- **وجہ**: ناکافی contact stabilization
- **حل**: ERP (Error Reduction Parameter) اور CFM (Constraint Force Mixing) adjust کریں

### Self-Collision Conflicts
- **وجہ**: Aggressive joint movements
- **حل**: Joint limits اور collision filters لاگو کریں

## مشق

تمام links کے لیے مناسب collision geometries کے ساتھ ایک ہیومینوئڈ روبوٹ ماڈل بنائیں۔ مختلف obstacles کے ساتھ Gazebo میں ماڈل کو test کریں تاکہ مناسب collision detection اور response کو یقینی بنایا جا سکے۔

## خلاصہ

Collision detection realistic ہیومینوئڈ روبوٹ simulation کے لیے ضروری ہے۔ Collision geometries، parameters، اور optimization techniques کی مناسب configuration درست اور efficient simulation کو یقینی بناتی ہے۔ اگلے سبق میں، ہم ڈیجیٹل ٹوئنز کے rendering اور visualization پہلوؤں کو دریافت کریں گے۔

