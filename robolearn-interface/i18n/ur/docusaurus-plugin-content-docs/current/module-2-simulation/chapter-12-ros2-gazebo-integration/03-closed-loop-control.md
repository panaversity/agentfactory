---
id: lesson-12-3-closed-loop-control
title: 'Lesson 12.3: Closed-Loop Control'
sidebar_position: 3
sidebar_label: 12.3 Closed-Loop Control
description: Implementing feedback control loops between ROS 2 and Gazebo
duration_minutes: 75
proficiency_level: B1
layer: L2
hardware_tier: 1
learning_objectives:
  - Implement velocity control via cmd_vel topic
  - Subscribe to sensor feedback from simulation
  - Create a simple reactive behavior using sensor data
  - Visualize robot state in RViz2 with real-time feedback
skills:
  - ros2-gazebo-bridge
  - control-loops
cognitive_load:
  new_concepts: 8
tier_1_path: TheConstruct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 12.3: بند لوپ کنٹرول (Closed-Loop Control)

## روبوٹ کنٹرول کے تین حصے

ایک روبوٹ تبھی کارآمد ہوتا ہے جب وہ **دنیا پر عمل کرتا ہے**۔ لیکن فیڈ بیک کے بغیر عمل اندھا ہوتا ہے۔ روبوٹ کو سینس کرنا ہوتا ہے، فیصلہ کرنا ہوتا ہے، پھر عمل کرنا ہوتا ہے—یہ عمل بار بار، اتنی تیزی سے ہونا چاہیے کہ تبدیلیوں پر ردعمل ظاہر کر سکے۔

یہ **کنٹرول لوپ** ہے:

```
┌─────────────┐
│   سینَس     │ ← سینسر پڑھیں (LIDAR، کیمرہ، IMU)
└──────┬──────┘
       │
       ↓
┌─────────────┐
│   فیصلہ     │ ← حساب لگائیں کہ کیا کرنا ہے
└──────┬──────┘
       │
       ↓
┌─────────────┐
│    عمل     │ ← حرکت کا حکم دیں
└──────┬──────┘
       │
       └─────→ [تیز فریکوئنسی پر دہرائیں]
```

**مثال: رکاوٹ سے بچاؤ (Obstacle Avoidance)**
1. **سینَس**: LIDAR نے 0.5 میٹر آگے رکاوٹ کی اطلاع دی
2. **فیصلہ**: "رکاوٹ قریب ہے، حرکت روک دو"
3. **عمل**: `/cmd_vel` پر صفر ویلوسٹی (velocity) شائع کریں
4. **نتیجہ**: ٹکراؤ سے پہلے روبوٹ رک جاتا ہے

یہ تینوں مراحل مل کر کام کرنے چاہئیں۔ سینسر والا روبوٹ لیکن کنٹرول لاجک کے بغیر ایک مسافر ہے۔ کنٹرول لاجک والا روبوٹ لیکن سینسر کے بغیر اندھا ہے۔

---

## حصہ 1: ویلوسٹی کمانڈز شائع کرنا (Publishing Velocity Commands)

آپ کا روبوٹ تب حرکت کرتا ہے جب آپ `/cmd_vel` ٹاپک پر کچھ شائع (publish) کرتے ہیں۔ میسج کی قسم `geometry_msgs/msg/Twist` ہوتی ہے:

```python
from geometry_msgs.msg import Twist

def cmd_vel_message(linear_x, angular_z):
    """ایک ویلوسٹی کمانڈ بنائیں"""
    msg = Twist()
    msg.linear.x = linear_x    # آگے کی ویلوسٹی (m/s)
    msg.linear.y = 0.0         # سائیڈ وائز (عام طور پر ڈفرنشل ڈرائیو کے لیے صفر)
    msg.linear.z = 0.0         # عمودی (پہیوں والے روبوٹ کے لیے عام طور پر صفر)
    msg.angular.x = 0.0        # رول گردش
    msg.angular.y = 0.0        # پچ گردش
    msg.angular.z = angular_z  # یاو گردش (rad/s)
    return msg
```

**عام پیٹرنز (Common patterns)**:

```python
# آگے بڑھیں
move_forward = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))

# وہیں گھومیں
turn_left = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.5))

# آگے بائیں طرف مڑیں
curve = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.2))

# ایمرجنسی سٹاپ
stop = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
```

**مسلسل شائع کرنا (Publishing continuously)**:

```python
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 10 Hz پر شائع کریں
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.speed = 0.5  # m/s
        self.angular = 0.0  # rad/s

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.angular
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: v={self.speed}, w={self.angular}')

    def stop(self):
        """ایمرجنسی سٹاپ۔"""
        self.speed = 0.0
        self.angular = 0.0
```

**اہم نکتہ**: آپ کو ویلوسٹی کمانڈز **مسلسل شائع** کرنی ہوں گی۔ اگر آپ ایک بار شائع کرتے ہیں اور رک جاتے ہیں، تو روبوٹ صرف ایک بار اس حکم پر عمل کرتا ہے، پھر رک جاتا ہے۔ مسلسل حرکت کے لیے، 10-50 Hz پر ٹائمر لوپ میں شائع کریں۔

---

## حصہ 2: سینسر فیڈ بیک کی سبسکرائب کرنا (Subscribing to Sensor Feedback)

فیصلے کرنے کے لیے، آپ کو یہ پڑھنا ہوگا کہ روبوٹ کیا محسوس کر رہا ہے۔ LIDAR اسکین اور کیمرہ کی تصاویر ROS 2 ٹاپکس کے ذریعے آتی ہیں۔

### LIDAR فیڈ بیک: رکاوٹ کا پتہ لگانا

LIDAR فاصلے کی پیمائش کے ساتھ `sensor_msgs/msg/LaserScan` شائع کرتا ہے:

```python
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LIDAR ٹاپک
            self.lidar_callback,
            10
        )
        self.min_distance = float('inf')

    def lidar_callback(self, msg: LaserScan):
        """جب بھی LIDAR شائع کرتا ہے تو یہ کال ہوتا ہے۔"""
        # msg.ranges فاصلوں کی ایک فہرست ہے [0...360 ڈگری]
        # سامنے کا حصہ عام طور پر درمیانی انڈیکس ہوتا ہے، کنارے کنارے ہوتے ہیں

        # کم سے کم فاصلہ تلاش کریں (روبوٹ کے سب سے قریب رکاوٹ)
        valid_ranges = [r for r in msg.ranges if 0 < r < msg.range_max]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')

        # نتیجہ شائع کریں
        if self.min_distance < 0.5:
            self.get_logger().warn(f'رکاوٹ قریب ہے: {self.min_distance:.2f}m')
        else:
            self.get_logger().info(f'آگے صاف ہے: {self.min_distance:.2f}m')
```

**LaserScan ڈھانچہ**:
```
msg.angle_min = -π       # شروع کا زاویہ (ریڈیئنز)
msg.angle_max = π        # اختتام کا زاویہ
msg.angle_increment = Δθ # فی نمونہ ریڈیئنز
msg.ranges = [r0, r1, r2, ...]  # ہر زاویے پر فاصلہ
```

**مثال**: اگر ایک LIDAR 180 ڈگری کو 180 نمونوں کے ساتھ اسکین کرتا ہے، تو ہر نمونہ 1 ڈگری کا فاصلہ رکھتا ہے۔

### کیمرہ فیڈ بیک: ویژن پروسیسنگ

کیمرہ کی تصاویر `sensor_msgs/msg/Image` کے طور پر شائع ہوتی ہیں۔ پروسیسنگ میں عام طور پر OpenCV استعمال ہوتا ہے:

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.red_detected = False

    def image_callback(self, msg: Image):
        """آنے والے کیمرہ فریم کو پروسیس کریں۔"""
        # ROS Image میسج کو OpenCV فارمیٹ میں تبدیل کریں
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # سرخ رنگ کا پتہ لگائیں
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # اگر سرخ پکسلز موجود ہیں، تو سرخ چیز کا پتہ چلا
        self.red_detected = cv2.countNonZero(mask) > 100

        if self.red_detected:
            self.get_logger().info('سرخ چیز کا پتہ چلا!')
```

---

## حصہ 3: فیصلہ سازی کی منطق (Decision Logic)

ایک بار جب آپ سینس کر لیتے ہیں، تو آپ کو فیصلہ کرنا ہوتا ہے۔ سادہ ردعمل (reactive) منطق اچھی طرح کام کرتی ہے:

**اگر-تب کے اصول (If-Then rules)**:
```python
if obstacle_distance < 0.5:
    command = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
    # رک جاؤ
elif obstacle_distance < 1.0:
    command = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.0))
    # رفتار کم کرو
else:
    command = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))
    # پوری رفتار سے آگے بڑھو
```

**سٹیٹ مشینز (State machines)** (زیادہ پیچیدہ رویے):

```python
class RobotStateMachine:
    STATE_EXPLORING = 0
    STATE_OBSTACLE_DETECTED = 1
    STATE_BACKING_UP = 2

    def __init__(self):
        self.state = self.STATE_EXPLORING
        self.time_in_state = 0

    def update(self, obstacle_distance, dt):
        """سینسر ان پٹ کی بنیاد پر سٹیٹ کو اپ ڈیٹ کریں۔"""
        self.time_in_state += dt

        if self.state == self.STATE_EXPLORING:
            if obstacle_distance < 0.5:
                self.state = self.STATE_BACKING_UP
                self.time_in_state = 0

        elif self.state == self.STATE_BACKING_UP:
            if self.time_in_state > 2.0:  # 2 سیکنڈ کے لیے پیچھے ہٹیں
                self.state = self.STATE_EXPLORING
                self.time_in_state = 0

        # سٹیٹ کی بنیاد پر ویلوسٹی کمانڈ واپس کریں
        if self.state == self.STATE_EXPLORING:
            return Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))
        else:
            return Twist(linear=Vector3(x=-0.3), angular=Vector3(z=0.0))
```

---

## مکمل مثال: رکاوٹ پر رکنا (Stop-on-Obstacle)

یہاں ایک مکمل نوڈ ہے جو LIDAR پڑھتا ہے، رکاوٹ کا پتہ لگاتا ہے، اور روبوٹ کو روک دیتا ہے:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class StopOnObstacle(Node):
    def __init__(self):
        super().__init__('stop_on_obstacle')

        # پبلشرز اور سبسکرائبرز
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # 10 Hz پر ویلوسٹی شائع کریں
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # سٹیٹ
        self.min_distance = 2.0  # ابتدائی: کچھ بھی نہیں پایا گیا
        self.desired_velocity = 0.5

    def lidar_callback(self, msg: LaserScan):
        """LIDAR اسکین پر عمل کریں۔"""
        # سامنے کے فاصلے حاصل کریں (اسکین کا درمیانی نصف حصہ)
        front_ranges = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
        valid = [r for r in front_ranges if 0 < r < msg.range_max]

        self.min_distance = min(valid) if valid else 2.0
        self.get_logger().info(f'کم سے کم فاصلہ: {self.min_distance:.2f}m')

    def publish_velocity(self):
        """سینسر ڈیٹا کی بنیاد پر ویلوسٹی کمانڈ شائع کریں۔"""
        msg = Twist()

        # فیصلہ سازی کی منطق: اگر رکاوٹ قریب ہو تو رک جاؤ
        if self.min_distance < 0.5:
            msg.linear.x = 0.0
            self.get_logger().warn('رکاوٹ! رک رہا ہے۔')
        elif self.min_distance < 1.0:
            msg.linear.x = 0.2
            self.get_logger().info('رکاوٹ قریب آ رہی ہے، رفتار کم کر رہا ہوں۔')
        else:
            msg.linear.x = self.desired_velocity
            self.get_logger().debug('سب صاف ہے، آگے بڑھ رہا ہوں۔')

        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)

def main():
    rclpy.init()
    node = StopOnObstacle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C پر ایمرجنسی سٹاپ
        msg = Twist()
        node.vel_publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**یہ کیا کرتا ہے**:
1. **کال بیک**: LIDAR ڈیٹا `lidar_callback()` کو متحرک کرتا ہے، `min_distance` کو اپ ڈیٹ کرتا ہے۔
2. **ٹائمر**: ہر 0.1 سیکنڈ میں، `publish_velocity()` چلتا ہے۔
3. **فیصلہ**: اگر رکاوٹ < 0.5m ہے، تو ویلوسٹی کو 0 پر سیٹ کریں
4. **شائع کریں**: روبوٹ کو حکم بھیجیں۔

**نتیجہ**: روبوٹ آگے بڑھتا ہے، رکاوٹ دیکھنے پر رک جاتا ہے۔

---

## RViz2 میں بصری بنانا (Visualizing in RViz2)

اپنے کنٹرول لوپ کو ڈیبگ کرنے کے لیے، جو ہو رہا ہے اسے دیکھیں:

**RViz2 لانچ کریں**:
```bash
ros2 launch robot_launcher display.launch.py
```

**بصریات شامل کریں**:
1. **TF** ٹیب: روبوٹ کی ساخت اور ٹرانسفارمز
2. **LaserScan** ٹیب: LIDAR سے پوائنٹ کلاؤڈ
3. **Image** ٹیب: خام کیمرہ فیڈ
4. **Velocity Vector** (کسٹم مارکر): شائع شدہ cmd_vel دکھائیں

**ڈیبگ ورک فلو**:
- روبوٹ کے حرکت کرتے وقت LIDAR بصریات دیکھیں
- تصدیق کریں کہ روبوٹ کے رکنے سے پہلے رکاوٹ پوائنٹ کلاؤڈ میں ظاہر ہوتی ہے
- شائع شدہ احکامات دیکھنے کے لیے `ros2 topic echo` کے ساتھ cmd_vel ٹاپک چیک کریں

---

## مشق: رکاوٹ سے بچاؤ کا نفاذ کریں (Implement Obstacle Avoidance)

**مقصد**: ایک ایسا روبوٹ بنائیں جو آگے بڑھے، رکاوٹیں آنے پر رک جائے۔

**سیٹ اپ**:
1. سبق 12.2 سے پیدا شدہ روبوٹ کے ساتھ Gazebo
2. LIDAR سینسر ترتیب دیا گیا (باب 11 سے)
3. ros_gz_bridge جو `/cmd_vel` کو میپ کرتا ہے

**مرحلہ 1: کنٹرول نوڈ بنائیں**
```python
# nodes/stop_on_obstacle.py کے طور پر محفوظ کریں
# [اوپر سے مکمل کوڈ استعمال کریں]
```

**مرحلہ 2: لانچ فائل میں شامل کریں**
```python
# spawn_robot.launch.py میں، یہ شامل کریں:
control_node = Node(
    package='robot_controller',
    executable='stop_on_obstacle.py'
)
```

**مرحلہ 3: سب کچھ لانچ کریں**
```bash
ros2 launch robot_launcher spawn_robot.launch.py
```

**مرحلہ 4: تجربہ کریں**
- روبوٹ آگے بڑھتا ہے
- Gazebo میں ایک رکاوٹ (دیوار، ڈبہ) شامل کریں
- روبوٹ رکاوٹ کا پتہ لگاتا ہے اور رک جاتا ہے

**کامیابی**: روبوٹ حقیقی وقت میں سینس-فیصلہ-عمل لوپ کا مظاہرہ کرتا ہے۔

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: ChatGPT یا Claude کھولیں اور اپنے کنٹرول رویے کو بہتر بنانے کے بارے میں پوچھیں۔

**پرامپٹ 1** (ایڈوانسڈ رکاوٹ سے بچاؤ):
```
میرا موجودہ رکاوٹ سے بچاؤ صرف رک جاتا ہے۔ میں چاہتا ہوں کہ روبوٹ یہ کرے:
1. بائیں طرف رکاوٹ کا پتہ لگائیں
2. دائیں مڑیں اور اس کے گرد نیویگیٹ کریں
3. اصل سمت دوبارہ شروع کریں

اس کو کون سی فیصلہ سازی کی منطق سنبھالتی ہے؟ کیا مجھے سٹیٹ مشین استعمال کرنی چاہیے؟
کوڈ ڈھانچہ دکھائیں۔
```

**پرامپٹ 2** (حدود کو ایڈجسٹ کرنا):
```
میرے روبوٹ کی رکاوٹ کا پتہ لگانے کی حد 0.5m ہے۔ یہ سمولیشن میں کام کرتا ہے
لیکن یا تو بہت جارحانہ یا بہت بزدلانہ محسوس ہوتا ہے۔ میں اس حد کو منظم طریقے سے
کیسے ایڈجسٹ کروں؟ شروع کرنے کے لیے ایک اچھی جگہ کیا ہے؟
```

**پرامپٹ 3** (ملٹی سینسر فیوژن):
```
میں LIDAR اور کیمرہ ڈیٹا کو یکجا کرنا چاہتا ہوں: LIDAR رکاوٹوں کا پتہ لگاتا ہے،
کیمرہ شناخت کرتا ہے کہ یہ ایک متحرک شے (شخص) ہے یا جامد دیوار۔
ہر ایک کے لیے مختلف ردعمل ظاہر کریں۔ میں ایک فیصلہ سازی کے نوڈ میں دو سینسر اسٹریمز کو
کیسے یکجا کروں؟
```

**متوقع نتائج**:
- AI ایسے کنٹرول پیٹرن تجویز کرتا ہے جن پر آپ نے غور نہیں کیا (سٹیٹ مشینز، ہیسٹیرسس)
- AI آپ کے مخصوص ماحول کے لیے پیرامیٹرز کو ایڈجسٹ کرنے میں مدد کرتا ہے
- AI سینسر فیوژن تکنیکوں کی وضاحت کرتا ہے

**حفاظتی نوٹ**: سمولیشن میں، جارحانہ رویوں کا تجربہ کرنا محفوظ ہے۔ حقیقی روبوٹ پر تعینات کرنے سے پہلے، تمام حدود کو پوری طرح سے درست کریں۔

---

## اگلے اقدامات

آپ نے کنٹرول لوپ کو بند کر دیا ہے: سینس، فیصلہ، عمل۔ اگلے سبق میں، آپ ان پیٹرنز کو دوبارہ استعمال کے قابل مہارتوں میں تبدیل کریں گے جو منصوبوں میں بڑھتی ہیں۔

**اس سبق سے کیا سامنے آیا**: کنٹرول لوپ خود مختار نظاموں کا دل ہیں۔ بغیر فیصلہ سازی کی منطق کے سینسر صرف ڈیٹا اسٹریم ہیں۔ بغیر سینسر کے فیصلہ سازی کی منطق صرف اندازہ لگانا ہے۔ یہ سب مل کر روبوٹ کو اپنی دنیا پر ذہانت سے ردعمل ظاہر کرنے کے قابل بناتے ہیں۔

---

## کلیدی تصورات کا چیک پوائنٹ

آگے بڑھنے سے پہلے، اس بات کی تصدیق کریں کہ آپ یہ سمجھتے ہیں:

- **Twist میسج**: ویلوسٹی کمانڈز کیسے شائع کریں۔
- **سینسر سبسکرپشنز**: LIDAR اسکین اور کیمرہ کی تصاویر کیسے پڑھیں۔
- **فیصلہ سازی کی منطق**: اگر-تب کے اصول اور سٹیٹ مشینز۔
- **کنٹرول لوپ فریکوئنسی**: 10-50 Hz عام کیوں ہے۔
- **بصری بنانا**: رویے کو ڈیبگ کرنے کے لیے RViz2 کا استعمال۔

اگر یہ واضح ہیں، تو آپ سبق 12.4 کے لیے تیار ہیں۔