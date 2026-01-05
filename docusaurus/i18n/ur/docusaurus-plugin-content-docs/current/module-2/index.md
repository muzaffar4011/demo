---
sidebar_position: 1
title: ماڈیول 2 - ڈیجیٹل ٹوئن (Gazebo & Unity)
---

# ماڈیول 2: ڈیجیٹل ٹوئن (Gazebo & Unity)

فزیکل AI اور ہیومینوئڈ روبوٹکس: مجسم ذہانت کے ماڈیول 2 میں خوش آمدید۔ اس ماڈیول میں، آپ Gazebo اور Unity استعمال کرتے ہوئے ہیومینوئڈ روبوٹس کے لیے ڈیجیٹل ٹوئنز بنانے کے بارے میں سیکھیں گے، بشمول physics simulation، collision detection، اور sensor integration۔

## سیکھنے کے مقاصد

اس ماڈیول کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- روبوٹکس میں ڈیجیٹل ٹوئنز کے تصور اور اہمیت کو سمجھنا
- درست collision detection کے ساتھ physics simulations سیٹ اپ کرنا
- realistic visualization کے لیے high-fidelity rendering لاگو کرنا
- simulation میں مختلف سینسرز (LiDAR، depth cameras، IMUs) کو انضمام کرنا
- ہیومینوئڈ روبوٹ testing کے لیے simulation environments configure کرنا

## جائزہ

ایک ڈیجیٹل ٹوئن ایک جسمانی سسٹم کا virtual replica ہے جسے حقیقی hardware پر تعینات کرنے سے پہلے simulation، testing، اور validation کے لیے استعمال کیا جا سکتا ہے۔ ہیومینوئڈ روبوٹکس میں، ڈیجیٹل ٹوئنز ضروری ہیں:
- محفوظ virtual environments میں روبوٹ رویوں کی testing
- hardware damage کے خطرے کے بغیر control algorithms کی validation
- AI سسٹمز کے لیے synthetic training data پیدا کرنا
- روبوٹ ڈیزائنز اور تعاملات کا prototyping

یہ ماڈیول دو بنیادی simulation environments پر توجہ مرکوز کرتا ہے: Gazebo، جو ROS کے ساتھ tight integration رکھتا ہے اور روبوٹکس تحقیق میں وسیع پیمانے پر استعمال ہوتا ہے، اور Unity، جو زیادہ realistic visualization اور human-robot interaction studies کے لیے high-fidelity graphics فراہم کرتا ہے۔

## پیشگی شرائط

اس ماڈیول کو شروع کرنے سے پہلے، آپ کے پاس ہونا چاہیے:
- ماڈیول 1 مکمل (ROS 2 fundamentals)
- Physics concepts کی بنیادی سمجھ (forces، collisions، gravity)
- 3D coordinate systems اور transformations سے واقفیت

## ماڈیول کی ساخت

یہ ماڈیول کئی اسباق میں تقسیم ہے:
- Physics simulation: سمجھنا کہ simulated physics کیسے کام کرتی ہے
- Collision detection: درست collision models لاگو کرنا
- Rendering: realistic visual representations بنانا
- Sensors: virtual environments میں مختلف روبوٹ سینسرز کی simulation

آئیے روبوٹک environments میں physics simulation کو دریافت کرنے سے شروع کریں۔

