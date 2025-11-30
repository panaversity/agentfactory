---
id: lesson-4-3-writing-subscriber
title: 'Lesson 4.3: Writing a Subscriber'
sidebar_position: 3
sidebar_label: 4.3 Writing Subscriber
description: Write your first ROS 2 subscriber node using rclpy with message callbacks.
chapter: 4
lesson: 3
duration_minutes: 60
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 2
learning_objectives:
  - Write a ROS 2 subscriber node using rclpy
  - Implement message callbacks
  - Understand the subscriber queue depth parameter
  - Test pub/sub communication between two nodes
  - Process and respond to incoming messages
skills:
  - ros2-publisher-subscriber
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 4.3: سبسکرائبر لکھنا (Writing a Subscriber)

پچھلے سبق میں، آپ نے ایک پبلشر لکھا — ایک ایسا نوڈ جو ڈیٹا بھیجتا ہے۔ پبلشر تب تک کارآمد نہیں ہوتے جب تک کوئی سن رہا نہ ہو۔ اس سبق میں، آپ ایک **سبسکرائبر** لکھیں گے — ایک ایسا نوڈ جو ایک ٹاپک سے پیغامات وصول کرتا ہے۔

سبسکرائبر کو پبلشر کی کانفرنس میں بیٹھے سامعین کی طرح سمجھیں۔ سامعین اسپیکر کی بات سنتے ہیں اور اس پر ردعمل ظاہر کرتے ہیں۔ اسپیکر کو اس بات سے کوئی فرق نہیں پڑتا کہ کون سن رہا ہے — سامعین بس آتے ہیں اور سنتے ہیں۔

اس سبق کے اختتام تک، آپ کے پاس ایک پبلشر اور سبسکرائبر آپس میں بات چیت کر رہے ہوں گے، اور آپ سمجھ جائیں گے کہ موصولہ پیغامات کو کیسے پراسیس کیا جاتا ہے۔

---

## سبسکرائبر پیٹرن (The Subscriber Pattern)

تمام ROS 2 سبسکرائبر ایک ہی پیٹرن پر عمل کرتے ہیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

آئیے ہر حصے کو سمجھتے ہیں۔

---

## کوڈ کو سمجھنا (Understanding the Code)

### امپورٹس (Imports)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

پبلشر جیسا ہی — ہمیں rclpy، Node، اور String پیغامات کی ضرورت ہے۔

### سبسکرائبر بنانا (Creating the Subscriber)

```python
self.subscription = self.create_subscription(
    String,           # پیغام کی قسم (Message type)
    'topic',          # ٹاپک کا نام (Topic name) (پبلشر سے ملنا چاہیے)
    self.listener_callback,  # پیغام آنے پر کال کرنے والا فنکشن
    10)               # کیو ڈیپتھ (Queue depth)
```

یہ ایک سبسکرپشن بناتا ہے جو:
- **String پیغامات سنتا ہے** (پیغام کی قسم)
- **'topic' نامی ٹاپک پر سنتا ہے** (جس پر پبلشر بھیج رہا ہے، اس سے ملنا ضروری ہے)
- **جب بھی کوئی پیغام آتا ہے تو self.listener_callback() کو کال کرتا ہے**
- **کیو ڈیپتھ 10 رکھتا ہے** (اگر پیغامات آپ کے پراسیس کرنے کی رفتار سے تیز آئیں تو 10 تک بفر ہو جائیں گے)

### کال بیک (The Callback)

```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

جب کوئی پیغام آتا ہے تو یہ فنکشن خود بخود کال ہوتا ہے۔ `msg` پیرامیٹر میں موصولہ پیغام ہوتا ہے۔ آپ `msg.data` کے ذریعے ڈیٹا تک رسائی حاصل کر سکتے ہیں۔

### وہ لائن جو عجیب لگتی ہے (The Line That Looks Odd)

```python
self.subscription  # prevent unused variable warning
```

یہ پائتھن لینٹرز (linters) کی ایک عجیب عادت ہے۔ اس لائن کے بغیر، کچھ IDEs یہ وارننگ دے سکتے ہیں کہ "آپ نے self.subscription بنایا لیکن اسے استعمال نہیں کیا"۔ حقیقت میں، ROS 2 اسے اندرونی طور پر استعمال کر رہا ہوتا ہے — سبسکرپشن بیک گراؤنڈ میں سنتا رہتا ہے۔ یہ لائن اس جھوٹی وارننگ کو روکتی ہے۔

---

## سبسکرائبر فائل بنانا (Creating the Subscriber File)

آئیے اسے اپنے پیکیج میں بناتے ہیں۔

**مرحلہ 1: سبسکرائبر فائل بنائیں**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
cat > minimal_subscriber.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**مرحلہ 2: ایگزیکیوٹیبل شامل کرنے کے لیے setup.py کو اپ ڈیٹ کریں**

`~/ros2_ws/src/my_first_package/setup.py` میں ترمیم کریں:

```python
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
        'minimal_subscriber = my_first_package.minimal_subscriber:main',  # یہ لائن شامل کریں
    ],
},
```

**مرحلہ 3: ورک اسپیس کو بلڈ کریں**

```bash
cd ~/ros2_ws
colcon build
```

**مرحلہ 4: ورک اسپیس کو سورس کریں**

```bash
source ~/ros2_ws/install/setup.bash
```

---

## پبلشر + سبسکرائبر کا تجربہ کرنا (Testing Publisher + Subscriber)

اب آپ کے پاس پبلشر اور سبسکرائبر دونوں ہیں۔ آئیے انہیں ایک ساتھ ٹیسٹ کریں۔

**ٹرمینل 1: پبلشر چلائیں**

```bash
ros2 run my_first_package minimal_publisher
```

آؤٹ پٹ:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
...
```

**ٹرمینل 2: سبسکرائبر چلائیں**

```bash
ros2 run my_first_package minimal_subscriber
```

آؤٹ پٹ:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
...
```

کامیابی! پبلشر پیغامات بھیج رہا ہے، اور سبسکرائبر انہیں وصول کر رہا ہے۔ آؤٹ پٹ دیکھیں — سبسکرائبر پبلشر کے بھیجے گئے ہر پیغام کو وصول کر رہا ہے۔

**ٹرمینل 3: سسٹم کا معائنہ کریں**

```bash
ros2 node list
```

آؤٹ پٹ:

```
/minimal_publisher
/minimal_subscriber
```

دونوں نوڈز چل رہے ہیں۔

```bash
ros2 topic list
```

آؤٹ پٹ:

```
/topic
```

ایک ٹاپک ہے (پبلشر اور سبسکرائبر ایک ہی ٹاپک پر ہیں)۔

```bash
ros2 topic info /topic
```

آؤٹ پٹ:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

بالکل — ایک پبلشر اور ایک سبسکرائبر ایک ہی ٹاپک پر۔

---

## پیغام کے بہاؤ کو سمجھنا (Understanding Message Flow)

ROS 2 کے اندر یہ ہوتا ہے:

1. **پبلشر شائع کرتا ہے**: نوڈ 1 `publisher.publish(msg)` کال کرتا ہے
2. **ROS 2 مڈل ویئر**: ROS 2 میسج بروکر اسے وصول کرتا ہے اور محفوظ کرتا ہے
3. **سبسکرائبر وصول کرتا ہے**: ROS 2 نوڈ 2 میں `listener_callback(msg)` کو کال کرتا ہے
4. **پیغام کو پراسیس کریں**: نوڈ 2 کا کال بیک چلتا ہے اور ڈیٹا لاگ کرتا ہے

یہ خود بخود ہوتا ہے — آپ نوڈز کے درمیان دستی طور پر پیغامات پاس نہیں کرتے۔ مڈل ویئر (DDS، ROS 2 کا بنیادی مواصلاتی نظام) اس کا خیال رکھتا ہے۔

---

## پیغامات پراسیس کرنا: ایک زیادہ پیچیدہ سبسکرائبر (Processing Messages: A More Complex Subscriber)

سادہ سبسکرائبر صرف سن کر لاگ کرتا ہے۔ آئیے اسے کچھ زیادہ دلچسپ کام کروائیں — ڈیٹا کو پراسیس کریں۔

**ایک زیادہ پیچیدہ سبسکرائبر بنائیں:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ProcessingSubscriber(Node):
    def __init__(self):
        super().__init__('processing_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.message_count = 0

    def listener_callback(self, msg):
        self.message_count += 1

        # پیغام لاگ کریں
        self.get_logger().info(f'Message #{self.message_count}: {msg.data}')

        # "Hello World: X" سے نمبر نکالیں
        parts = msg.data.split(': ')
        if len(parts) == 2:
            try:
                number = int(parts[1])
                self.get_logger().info(f'  → Number extracted: {number}')

                # نمبر کی بنیاد پر ردعمل ظاہر کریں
                if number % 5 == 0:
                    self.get_logger().warning(f'  → Milestone! Number {number} is divisible by 5')
            except ValueError:
                self.get_logger().error(f'  → Could not parse number from {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    processing_subscriber = ProcessingSubscriber()
    rclpy.spin(processing_subscriber)
    processing_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

یہ سبسکرائبر:
1. **پیغامات کو گنتا ہے** (ایک چلتا ہوا کاؤنٹر رکھتا ہے)
2. **پیغام کو پارس کرتا ہے** (نمبر نکالتا ہے)
3. **شرط کے ساتھ ردعمل ظاہر کرتا ہے** (ہر 5 پیغامات پر ایک وارننگ لاگ کرتا ہے)

جب آپ اسے پبلشر کے ساتھ چلاتے ہیں، تو آؤٹ پٹ یہ ہو جاتا ہے:

```
[INFO] [processing_subscriber]: Message #1: Hello World: 0
[INFO] [processing_subscriber]:   → Number extracted: 0
[WARNING] [processing_subscriber]:   → Milestone! Number 0 is divisible by 5
[INFO] [processing_subscriber]: Message #2: Hello World: 1
[INFO] [processing_subscriber]:   → Number extracted: 1
...
```

---

## متعدد سبسکرائبرز کو جوڑنا (Connecting Multiple Subscribers)

ایک پبلشر متعدد سبسکرائبرز کو بھیج سکتا ہے۔ آئیے اسے ٹیسٹ کریں۔

**ٹرمینل 1: پبلشر چلائیں**

```bash
ros2 run my_first_package minimal_publisher
```

**ٹرمینل 2: سادہ سبسکرائبر چلائیں**

```bash
ros2 run my_first_package minimal_subscriber
```

**ٹرمینل 3: پراسیسنگ سبسکرائبر چلائیں**

```bash
ros2 run my_first_package processing_subscriber  # آپ کو یہ ایگزیکیوٹیبل setup.py میں شامل کرنا ہوگا
```

دونوں سبسکرائبرز ایک جیسے پیغامات بیک وقت وصول کرتے ہیں۔ پبلشر کو اس بات سے کوئی فرق نہیں پڑتا کہ کتنے سبسکرائبر سن رہے ہیں۔

**اس سے تصدیق کریں:**

```bash
ros2 topic info /topic
```

آؤٹ پٹ:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 2
```

ایک پبلشر، دو سبسکرائبر، ایک ٹاپک۔

---

## کیو ڈیپتھ کو سمجھنا (Understanding Queue Depth)

`create_subscription()` میں موجود `10` کیو ڈیپتھ ہے۔ یہاں اس کا مطلب ہے:

**اگر سبسکرائبر سست ہو:**

```
پیغام آتا ہے → سبسکرائبر مصروف ہے → ROS 2 پیغام کو کیو (قطار) میں لگا دیتا ہے
پیغام آتا ہے → اب بھی مصروف ہے → کیو میں اب 2 پیغامات ہیں
...
پیغام آتا ہے → کیو بھر جاتی ہے (10 پیغامات) → سب سے پرانا پیغام ڈراپ ہو جاتا ہے
```

**ریئل ٹائم سینسرز** (کیمرہ، LIDAR) کے لیے، پرانے پیغامات کو ڈراپ کرنا ٹھیک ہے — آپ کو صرف تازہ ترین کی پرواہ ہوتی ہے۔ لہذا کیو ڈیپتھ 1-2 ہو سکتی ہے۔

**نازک احکامات** (نیویگیشن گولز، حفاظتی سٹاپ) کے لیے، آپ کبھی بھی پیغامات کو ڈراپ نہیں کرنا چاہتے۔ لہذا کیو ڈیپتھ 100 یا اس سے زیادہ ہو سکتی ہے۔

فی الحال، ایک مناسب ڈیفالٹ کے طور پر `10` استعمال کریں۔

---

## سبسکرائبر کے مسائل کو ڈیبگ کرنا (Debugging Subscriber Issues)

**سبسکرائبر کو پیغامات موصول نہیں ہو رہے؟**

1. پبلشر چل رہا ہے یا نہیں چیک کریں: `ros2 node list`
2. ٹاپک کا نام مماثل ہے یا نہیں چیک کریں: `ros2 topic list`
3. پیغام کی قسم مماثل ہے یا نہیں چیک کریں: `ros2 topic info /topic`

**کال بیک کال نہیں ہو رہا؟**

1. تصدیق کریں کہ پبلشر بھیج رہا ہے: `ros2 topic echo /topic`
2. اپنے کال بیک میں کسی خرابی (exception) کی جانچ کریں (کوئی بھی غلطی لاگ ہو جائے گی)
3. یقینی بنائیں کہ آپ `rclpy.spin()` کال کر رہے ہیں — اس کے بغیر، کال بیک کبھی نہیں چلتے

**پیغامات آہستہ آ رہے ہیں؟**

یہ اکثر متوقع ہوتا ہے۔ نیٹ ورک کی تاخیر، سسٹم کا بوجھ، اور پیغام کا سائز سب رفتار کو متاثر کرتے ہیں۔ پیغام کی اصل شرح (rate) جانچنے کے لیے `ros2 topic hz /topic` کے ساتھ متوقع رویے کی تصدیق کریں۔

---

## AI کے ساتھ کوشش کریں (Try With AI)

آپ کے پاس ایک کام کرنے والا سبسکرائبر ہے۔ آئیے اسے AI کی مدد سے بہتر بنائیں۔

**اپنے AI سے پوچھیں:**

> "میرے پاس ایک ROS 2 سبسکرائبر ہے جو String پیغامات پراسیس کرتا ہے۔ میں اس میں مضبوطی شامل کرنا چاہتا ہوں: خراب پیغامات کے لیے ایرر ہینڈلنگ، اگر پبلشر بھیجنا بند کر دے (نوڈ خاموش ہو جائے) تو ٹائم آؤٹ، اور اعداد و شمار (کتنے پیغامات موصول ہوئے، پیغامات کے درمیان اوسط وقت)۔ کوڈ دکھائیں۔"

**متوقع نتیجہ:** AI تجویز کرے گا:
- کال بیک میں Try/except بلاکس
- ایک ٹائم آؤٹ ٹائمر جو اگر X سیکنڈ کے اندر کوئی پیغام نہ آئے تو فائر ہو
- اعداد و شمار کو ٹریک کرنا (پیغام کی تعداد، ٹائم سٹیمپ)
- اعداد و شمار کو وقفے وقفے سے پرنٹ کرنے کا ایک طریقہ

**AI کو چیلنج کریں:**

> "میں ٹائمر کا استعمال کرتے ہوئے ٹائم آؤٹ کا طریقہ لاگو کر رہا ہوں۔ کیا یہ غیر موثر نہیں ہوگا اگر پبلشر کثرت سے بھیج رہا ہو لیکن پھر 5 منٹ کے لیے رک جائے؟ کیا اس کا کوئی بہتر طریقہ ہے؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- کال بیک میں ٹائمر کو ری سیٹ کریں (صرف خاموشی کو گنیں)
- ٹریڈ آف: زیادہ پیچیدہ کوڈ لیکن بہتر کارکردگی
- متبادل: صرف آخری پیغام کے ٹائم سٹیمپ کو ٹریک کریں اور اسے وقفے وقفے سے چیک کریں۔

**مل کر بہتری لائیں:**

> "میں ٹائمر-ری سیٹ کا طریقہ لاگو کرتا ہوں۔ لیکن میں یہ بھی چاہتا ہوں کہ جب ٹائم آؤٹ پہلی بار ہو تو صرف ایک وارننگ لاگ ہو، بار بار نہیں۔ آپ اس طرح کی بار بار وارننگز کو روکنے کے لیے کیا تجویز کریں گے؟"

**متوقع نتیجہ:** AI آپ کو یہ دکھائے گا کہ کیسے:
- اسٹیٹ (has_warned فلیگ) کو ٹریک کریں
- پہلی بار ٹائم آؤٹ ہونے پر اسے True پر سیٹ کریں
- پیغام آنے پر اسے ری سیٹ کریں
- وارننگ صرف تب لاگ کریں جب اسٹیٹ تبدیل ہوا ہو

یہ تھری رولز پیٹرن (Three Roles pattern) کا عملی استعمال ہے — AI آپ کو پیٹرن سکھاتا ہے، آپ اپنی حدود کے ساتھ اسے گائیڈ کرتے ہیں، اور آپ ایک اچھے حل پر متفق ہوتے ہیں۔

---

## مشقیں (Exercises)

1. **سبسکرائبر کو تبدیل کریں تاکہ وہ موصولہ کل پیغامات کو گنے اور ہر 10 پیغامات پر گنتی پرنٹ کرے۔**
2. **ایک سبسکرائبر بنائیں جو عددی ڈیٹا (جیسے درجہ حرارت کی قیمتیں) نکالے اور پراسیس کرے۔**
3. **1 پبلشر کو بیک وقت 3 مختلف سبسکرائبرز کے ساتھ چلائیں۔**

---

## غور کریں (Reflect)

ان سوالات پر غور کریں:

1. **پولنگ (polling) کے مقابلے میں کال بیک پر مبنی ڈیزائن کیوں بہتر ہے؟** ایک لوپ میں پیغامات چیک کرنے میں کیا غلطی ہو سکتی ہے؟

2. **اگر آپ کا کال بیک سست ہو تو کیا ہوتا ہے؟** اگر پراسیسنگ میں اشاعت کے وقفے سے زیادہ وقت لگتا ہے، تو کیا ہو سکتا ہے؟

3. **ڈی کپلنگ (Decoupling) سسٹم ڈیزائن کو کیسے فائدہ پہنچاتی ہے؟** پبلشر کو سبسکرائبرز کے وجود کا علم نہیں ہوتا۔ یہ فائدہ کیوں ہے؟

اب آپ پب/سب پیٹرن کو سمجھ گئے ہیں — جو ROS 2 میں بنیادی مواصلاتی طریقہ کار ہے۔ اگلے سبق میں، آپ زیادہ پیچیدہ نظام بنانے کے لیے AI تعاون کے ساتھ سب کچھ یکجا کریں گے۔