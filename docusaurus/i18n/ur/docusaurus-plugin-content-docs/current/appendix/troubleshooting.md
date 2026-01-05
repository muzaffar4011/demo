---
sidebar_position: 1
title: Troubleshooting Guide
---

# Troubleshooting Guide

یہ guide Physical AI & Humanoid Robotics systems تیار کرنے اور operate کرنے کے دوران سامنے آنے والے عام مسائل کے حل فراہم کرتی ہے۔

## ROS 2 Issues

### عام ROS 2 Problems

#### 1. Nodes Communicate نہیں کر رہے
**مسئلہ**: ROS 2 nodes ایک دوسرے کے ساتھ communicate نہیں کر رہے۔
**حل**:
- چیک کریں کہ nodes ایک ہی ROS domain پر ہیں: `echo $ROS_DOMAIN_ID`
- اگر multiple machines پر چل رہے ہیں تو network configuration verify کریں
- یقینی بنائیں کہ RMW implementation تمام nodes میں consistent ہے
- ROS communication کو block کرنے والے firewall issues چیک کریں

```bash
# Check available nodes
ros2 node list

# Check topics
ros2 topic list

# Check services
ros2 service list
```

#### 2. Topic Echo کوئی ڈیٹا نہیں دکھاتا
**مسئلہ**: `ros2 topic echo` کوئی messages نہیں دکھاتا حالانکہ publisher چل رہا ہے۔
**حل**:
- چیک کریں کہ publisher اور subscriber کے درمیان QoS profiles match کرتے ہیں
- Topic names verify کریں کہ identical ہیں (case-sensitive)
- یقینی بنائیں کہ nodes ایک ہی domain پر ہیں
- چیک کریں کہ publisher واقعی publish کر رہا ہے (`ros2 topic hz` استعمال کریں)

#### 3. Communication میں High Latency
**مسئلہ**: Messages delayed یا dropped ہو رہی ہیں۔
**حل**:
- Message frequency یا size کم کریں
- Appropriate QoS settings استعمال کریں (reliable vs best-effort)
- Network bandwidth اور latency چیک کریں
- Local communication کے لیے shared memory استعمال کرنے پر غور کریں

### Performance Issues

#### 1. High CPU Usage
**مسئلہ**: ROS 2 nodes excessive CPU resources استعمال کر رہے ہیں۔
**حل**:
- Timer callback frequencies کم کریں
- Efficiency کے لیے callback functions optimize کریں
- Threading appropriately استعمال کریں
- Bottlenecks identify کرنے کے لیے code profile کریں

#### 2. Memory Leaks
**مسئلہ**: وقت کے ساتھ memory usage بڑھ رہی ہے۔
**حل**:
- Subscriptions، publishers، اور timers کو properly clean up کریں
- جہاں مناسب ہو weak references استعمال کریں
- `htop` یا ROS 2 tools کے ساتھ memory usage monitor کریں

## Isaac Sim Issues

### Simulation Problems

#### 1. Poor Simulation Performance
**مسئلہ**: Isaac Sim سست چل رہا ہے یا low frame rates کے ساتھ۔
**حل**:
- Scene complexity کم کریں
- Rendering quality settings کم کریں
- Simplified collision meshes استعمال کریں
- GPU utilization اور drivers چیک کریں

#### 2. Physics Instability
**مسئلہ**: روبوٹ floor سے گزر رہا ہے یا unrealistic physics behavior۔
**حل**:
- Physics time step adjust کریں (عام طور پر ہیومینوئڈ روبوٹس کے لیے 0.001s)
- Solver iterations بڑھائیں
- Mass اور inertia properties verify کریں
- Joint limits اور stiffness چیک کریں

## Navigation Issues

### Nav2 Problems

#### 1. Path Planning Fails
**مسئلہ**: Nav2 paths plan نہیں کر سکتا۔
**حل**:
- Costmap configuration verify کریں
- Map data چیک کریں
- Planner parameters adjust کریں
- Robot footprint صحیح طور پر configured ہے verify کریں

#### 2. Robot Stuck یا Oscillating
**مسئلہ**: روبوٹ stuck ہو جاتا ہے یا oscillate کرتا ہے۔
**حل**:
- Controller parameters tune کریں
- Recovery behaviors enable کریں
- Costmap inflation radius adjust کریں
- Path smoothing parameters optimize کریں

## Voice اور AI Integration Issues

### Speech Recognition Problems

#### 1. Poor Recognition Accuracy
**مسئلہ**: Speech recognition غلط results دیتا ہے۔
**حل**:
- Microphone quality اور placement چیک کریں
- Background noise کم کریں
- Whisper model parameters tune کریں
- Audio preprocessing improve کریں

#### 2. LLM Planning Errors
**مسئلہ**: LLM planning incorrect یا unsafe plans generate کرتا ہے۔
**حل**:
- Prompts improve کریں اور safety constraints شامل کریں
- Response validation implement کریں
- Fallback mechanisms شامل کریں
- Planning output review کریں قبل execution کے

## Hardware Issues

### Sensor Problems

#### 1. Sensor Data Missing یا Incorrect
**مسئلہ**: Sensors ڈیٹا نہیں دیتے یا incorrect ڈیٹا دیتے ہیں۔
**حل**:
- Sensor connections verify کریں
- Calibration procedures چیک کریں
- Sensor drivers update کریں
- ROS 2 topics monitor کریں

#### 2. Motor Control Issues
**مسئلہ**: Motors respond نہیں کرتے یا incorrect behavior دکھاتے ہیں۔
**حل**:
- Motor power اور connections verify کریں
- Control parameters tune کریں
- Joint limits چیک کریں
- Safety systems verify کریں

## مشق

عام مسائل کا سامنا کرتے وقت:
1. Error messages carefully پڑھیں
2. System logs چیک کریں
3. Step-by-step debugging approach استعمال کریں
4. Community resources اور documentation consult کریں

## خلاصہ

یہ troubleshooting guide عام مسائل کے حل فراہم کرتی ہے۔ اگر آپ کو مزید مدد کی ضرورت ہے، تو specific module documentation یا community forums سے رجوع کریں۔

