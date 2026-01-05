---
sidebar_position: 2
title: ROS نوڈز
---

# ROS نوڈز

ROS 2 میں، ایک نوڈ ایک پروسیس ہے جو کمپیوٹیشن انجام دیتا ہے۔ نوڈز ROS 2 سسٹم کے بنیادی بلڈنگ بلاکس ہیں۔ وہ ٹاپکس، سروسز، ایکشنز، اور پیرامیٹرز کا استعمال کرتے ہوئے دوسرے نوڈز کے ساتھ بات چیت کرتے ہیں۔

## سیکھنے کے مقاصد

اس سبق کو مکمل کرنے کے بعد، آپ قابل ہوں گے:
- ROS نوڈز کے تصور اور روبوٹک سسٹم میں ان کے کردار کو سمجھنا
- Python اور rclpy لائبریری کا استعمال کرتے ہوئے ROS نوڈز بنانا
- ہیومینوئڈ روبوٹکس ایپلی کیشنز کے لیے بنیادی نوڈ فنکشنلٹی کو لاگو کرنا

## ROS نوڈ کیا ہے؟

ایک ROS نوڈ ایک قابل عمل پروسیس ہے جو ROS کلائنٹ لائبریریز (جیسے Python کے لیے rclpy) کا استعمال کرتے ہوئے دوسرے نوڈز کے ساتھ بات چیت کرتا ہے۔ نوڈز ٹاپکس پر میسجز شائع کر سکتے ہیں، میسجز وصول کرنے کے لیے ٹاپکس کی سبسکرائب کر سکتے ہیں، سروسز فراہم کر سکتے ہیں، سروسز کو کال کر سکتے ہیں، اور مزید۔

ہیومینوئڈ روبوٹکس کے تناظر میں، نوڈز یہ ہینڈل کر سکتے ہیں:
- سینسر ڈیٹا پروسیسنگ (IMU، کیمرے، جوائنٹ انکوڈرز)
- Actuator کنٹرول (موٹر کمانڈز)
- اعلیٰ درجے کی فیصلہ سازی (پاتھ پلاننگ، رویے کا انتخاب)
- ادراکی سسٹمز (آبجیکٹ ڈیٹیکشن، SLAM)

## ایک بنیادی ROS نوڈ بنانا

آئیے ایک سادہ ROS نوڈ بنائیں جو ہیومینوئڈ روبوٹ کی حیثیت کے بارے میں میسجز شائع کرتا ہے:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidStatusPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_status_publisher')
        self.publisher_ = self.create_publisher(String, 'humanoid_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Humanoid robot status: Operational - {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_status_publisher = HumanoidStatusPublisher()

    try:
        rclpy.spin(humanoid_status_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_status_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## نوڈ کی ساخت کی وضاحت

مندرجہ بالا مثال ایک ROS نوڈ کی بنیادی ساخت کو ظاہر کرتی ہے:

1. **Import statements**: ضروری ROS 2 لائبریریز کو import کریں
2. **Node class**: اپنا نوڈ بنانے کے لیے `rclpy.node.Node` سے وراثت لیں
3. **Initialization**: `__init__` میں publishers، subscribers، timers، وغیرہ سیٹ اپ کریں
4. **Main function**: ROS 2 کو initialize کریں، نوڈ بنائیں، اور spinning شروع کریں

## نوڈ چلانا

اس نوڈ کو چلانے کے لیے:

1. کوڈ کو ایک فائل میں محفوظ کریں (مثلاً، `humanoid_status_publisher.py`)
2. یقینی بنائیں کہ آپ کا ROS 2 ماحول sourced ہے
3. نوڈ چلائیں: `python3 humanoid_status_publisher.py`

## ہیومینوئڈ روبوٹکس نوڈز کے لیے بہترین طریقے

جب ہیومینوئڈ روبوٹکس ایپلی کیشنز کے لیے نوڈز بناتے ہیں:

- **Modularity**: نوڈز کو ایک واحد، اچھی طرح سے تعریف شدہ فنکشن انجام دینے کے لیے ڈیزائن کریں
- **Robustness**: غلطیوں کو نرمی سے ہینڈل کریں اور یقینی بنائیں کہ نوڈ ناکامیوں سے بحال ہو سکتا ہے
- **Real-time considerations**: اہم کنٹرول کاموں کے لیے، ریئل-ٹائم کارکردگی کی ضروریات پر غور کریں
- **Resource efficiency**: ہیومینوئڈ روبوٹس میں اکثر محدود کمپیوٹیشنل وسائل ہوتے ہیں

## مشق

ایک ROS نوڈ بنائیں جو ہیومینوئڈ روبوٹ کے لیے جوائنٹ پوزیشن کمانڈز شائع کرتا ہے۔ نوڈ کو `joint_commands` نامی ٹاپک پر مناسب میسج اقسام کے ساتھ میسجز شائع کرنے چاہئیں۔

## خلاصہ

ROS نوڈز کسی بھی ROS 2 سسٹم کی بنیاد بناتے ہیں۔ نوڈز کو بنانے، ساخت دینے، اور منظم کرنے کا طریقہ سمجھنا پیچیدہ ہیومینوئڈ روبوٹکس ایپلی کیشنز تیار کرنے کے لیے بہت اہم ہے۔ اگلے سبق میں، ہم دریافت کریں گے کہ نوڈز ٹاپکس اور سروسز کا استعمال کرتے ہوئے ایک دوسرے کے ساتھ کیسے بات چیت کرتے ہیں۔

