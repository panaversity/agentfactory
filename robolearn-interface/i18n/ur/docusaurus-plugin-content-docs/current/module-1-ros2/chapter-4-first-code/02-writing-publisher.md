---
id: lesson-4-2-writing-publisher
title: 'Lesson 4.2: Writing a Publisher'
sidebar_position: 2
sidebar_label: 4.2 Writing Publisher
description: Write your first ROS 2 publisher node using rclpy with timer callbacks.
chapter: 4
lesson: 2
duration_minutes: 60
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 2
learning_objectives:
  - Write a ROS 2 publisher node using rclpy
  - Understand the Node class and lifecycle (init → spin → shutdown)
  - Implement periodic publishing with timer callbacks
  - Verify publication using ros2 topic echo
  - Extend publisher code based on feedback
skills:
  - ros2-publisher-subscriber
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 4.2: پبلشر لکھنا (Writing a Publisher)

اب آپ کے پاس ایک ورک اسپیس (workspace) اور ایک پیکیج (package) ہے۔ اب وقت ہے کہ آپ اپنا پہلا ROS 2 کوڈ لکھیں: ایک **پبلشر** — ایک ایسا نوڈ (node) جو ایک ٹاپک (topic) پر ڈیٹا بھیجتا ہے۔

پبلشر کو ایک کانفرنس میں اسپیکر کی طرح سمجھیں۔ وہ اسٹیج پر کھڑا ہوتا ہے اور معلومات نشر کرتا ہے۔ سامعین میں کوئی بھی (سبسکرائبرز) سن سکتا ہے۔ پبلشر کو اس بات کی پرواہ نہیں ہوتی کہ کون سن رہا ہے یا کتنے لوگ اسے سن رہے ہیں — وہ بس بولتا رہتا ہے۔

اس سبق میں، آپ ایک سادہ پبلشر لکھیں گے جو ہر 0.5 سیکنڈ میں ایک ٹیکسٹ میسج بھیجے گا۔ آپ کوڈ دیکھیں گے، سمجھیں گے کہ یہ کیسے کام کرتا ہے، اسے ٹیسٹ کریں گے، اور پھر فیڈ بیک کی بنیاد پر اسے تبدیل کریں گے۔

---

## پبلشر پیٹرن (The Publisher Pattern)

تمام ROS 2 پبلشرز ایک ہی پیٹرن پر عمل کرتے ہیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

آئیے اسے لائن بہ لائن سمجھتے ہیں۔

---

## کوڈ کو سمجھنا (Understanding the Code)

### امپورٹس (Imports)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- **rclpy**: ROS 2 پائپھن لائبریری۔ ROS کی ہر چیز یہاں سے شروع ہوتی ہے۔
- **Node**: بنیادی کلاس (base class)۔ ہر ROS 2 نوڈ `Node` سے وراثت (inherit) لیتا ہے۔
- **String**: ایک میسج کی قسم (message type)۔ یہ خاص قسم صرف ٹیکسٹ ڈیٹا پر مشتمل ہوتی ہے۔

### نوڈ کلاس بنانا (Creating the Node Class)

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

آپ ایک کلاس بناتے ہیں جو `Node` سے وراثت لیتی ہے۔ `super().__init__()` کال ROS 2 کو نوڈ کا نام بتاتی ہے: 'minimal_publisher'۔ یہ نام `ros2 node list` میں نظر آئے گا۔

### پبلشر بنانا (Creating the Publisher)

```python
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

یہ لائن ایک پبلشر بناتی ہے جو:
- **String میسجز بھیجتا ہے** (قسم)
- **'topic' نامی ٹاپک پر بھیجتا ہے** (چینل کا نام)
- **10 کی قطار کی گہرائی (queue depth) رکھتا ہے** (کتنے غیر بھیجے گئے میسجز کو محفوظ کیا جائے)

آخر میں انڈرسکور (`publisher_`) پائپھن میں ایک رواج ہے — یہ پڑھنے والوں کو بتاتا ہے کہ یہ ایک اندرونی خاصیت (internal attribute) ہے۔

### ٹائمر بنانا (Creating the Timer)

```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
```

یہ ایک ٹائمر بناتا ہے جو ہر 0.5 سیکنڈ میں `timer_callback()` کو کال کرتا ہے۔ اسے ایسے سمجھیں کہ "ہر 0.5 سیکنڈ میں، کچھ کرو۔"

### کال بیک (The Callback)

```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.i}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
```

یہ ہر 0.5 سیکنڈ میں کال ہوتا ہے:
1. **ایک میسج بنائیں**: `msg = String()`
2. **ڈیٹا بھریں**: `msg.data = ...`
3. **اسے پبلش کریں**: `self.publisher_.publish(msg)`
4. **لاگ کریں**: `self.get_logger().info(...)` کنسول پر پرنٹ کرتا ہے
5. **کاؤنٹر بڑھائیں**: `self.i += 1`

### مین فنکشن (The Main Function)

```python
def main(args=None):
    rclpy.init(args=args)              # ROS 2 شروع کریں
    minimal_publisher = MinimalPublisher()  # نوڈ بنائیں
    rclpy.spin(minimal_publisher)      # نوڈ چلائیں (ہمیشہ کے لیے بلاک کریں)
    minimal_publisher.destroy_node()   # صفائی کریں
    rclpy.shutdown()                   # ROS 2 بند کریں
```

یہ معیاری ROS 2 سٹارٹ اپ ترتیب ہے:
1. **rclpy.init()**: ROS 2 کو سیٹ اپ کریں۔
2. **اپنا نوڈ بنائیں**
3. **rclpy.spin()**: ہمیشہ کے لیے چلائیں۔ یہ وہ جگہ ہے جہاں ٹائمر کال بیک ہوتے ہیں۔
4. **صفائی**: جب spin() ختم ہوتا ہے (مثلاً، صارف Ctrl+C دباتا ہے)، تو اچھے طریقے سے صفائی کریں۔

---

## پبلشر فائل بنانا (Creating the Publisher File)

اب ہم اپنے پیکیج میں یہ فائل بناتے ہیں۔

**مرحلہ 1: اپنے پیکیج میں جائیں**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
```

نوٹ کریں کہ دو `my_first_package` ڈائریکٹریز ہیں — باہر والی فولڈر ہے، اندر والی پائپھن پیکیج ہے۔

**مرحلہ 2: پبلشر فائل بنائیں**

```bash
cat > minimal_publisher.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**مرحلہ 3: ایگزیکیوٹیبل کو package.xml میں شامل کریں**

بیرونی `my_first_package` فولڈر میں `setup.py` میں ترمیم کریں:

```python
# یہ سیکشن تلاش کریں:
entry_points={
    'console_scripts': [
    ],
},

# اس فہرست کے اندر یہ لائن شامل کریں:
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
    ],
},
```

یہ ROS 2 کو بتاتا ہے کہ ایک ایگزیکیوٹیبل ہے جس کا نام `minimal_publisher` ہے جو `minimal_publisher.py` سے `main()` فنکشن چلاتا ہے۔

**مرحلہ 4: ورک اسپیس کو بلڈ کریں**

```bash
cd ~/ros2_ws
colcon build
```

**مرحلہ 5: ورک اسپیس کو سورس کریں**

```bash
source ~/ros2_ws/install/setup.bash
```

---

## پبلشر چلانا (Running the Publisher)

**ٹرمینل 1: پبلشر شروع کریں**

```bash
ros2 run my_first_package minimal_publisher
```

آپ کو یہ آؤٹ پٹ دیکھنا چاہیے:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

نوڈ چل رہا ہے اور پبلش کر رہا ہے! لیکن یہ کیسے یقینی بنائیں کہ ڈیٹا واقعی بھیجا جا رہا ہے؟ آئیے چیک کرتے ہیں۔

**ٹرمینل 2: ٹاپک سنیں**

```bash
ros2 topic echo /topic
```

آپ کو یہ دیکھنا چاہیے:

```
data: Hello World: 0
---
data: Hello World: 1
---
data: Hello World: 2
---
```

`---` سے الگ کی گئی ہر لائن ایک میسج ہے۔ پبلشر کامیابی سے ڈیٹا بھیج رہا ہے!

**ٹرمینل 3: ٹاپک کا معائنہ کریں**

```bash
ros2 topic info /topic
```

آؤٹ پٹ:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1 (from ros2 topic echo)
```

یہ تصدیق کرتا ہے:
- 1 پبلشر ہے (آپ کا نوڈ)
- 1 سبسکرائبر ہے (ros2 topic echo کمانڈ)

---

## پبلشر میں ترمیم (Modifying the Publisher)

اب جب آپ کے پاس ایک کام کرنے والا پبلشر ہے، تو آئیے اسے تبدیل کریں۔ میسج کا وقفہ 0.5 سیکنڈ سے تبدیل کر کے 2 سیکنڈ کر دیں:

**minimal_publisher.py میں ترمیم کریں:**

```python
timer_period = 2.0  # Changed from 0.5 to 2.0
```

**دوبارہ بلڈ کریں:**

```bash
cd ~/ros2_ws && colcon build
```

**پبلشر دوبارہ شروع کریں:**

```bash
ros2 run my_first_package minimal_publisher
```

اب یہ ہر 2 سیکنڈ میں پبلش کرتا ہے بجائے 0.5 سیکنڈ کے۔ آپ `ros2 topic echo /topic` سے اس کی تصدیق کر سکتے ہیں۔

---

## پبلشر کو بڑھانا (Extending the Publisher)

آئیے اسے مزید حقیقت پسندانہ بناتے ہیں۔ ایک حقیقی سینسر میں متعدد فیلڈز ہو سکتے ہیں: ٹائم سٹیمپ، ویلیو، اور اعتماد (confidence)۔

**ایک نیا ورژن بنائیں:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.reading = 0.0

    def timer_callback(self):
        msg = String()
        # سینسر ڈیٹا کی سمولیشن: درجہ حرارت سیلسیس میں
        self.reading += 0.1
        msg.data = f'temperature: {self.reading:.2f}C'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sensor reading: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

یہ ایک درجہ حرارت سینسر کی سمولیشن کرتا ہے جو آہستہ آہستہ درجہ حرارت بڑھا رہا ہے۔

---

## اہم نکات (Key Insights)

**ٹائمنگ:** `create_timer()` کال بیک باقاعدہ وقفوں پر بلایا جاتا ہے۔ کال بیکس کبھی بھی ایک ساتھ نہیں بلائے جاتے — ROS 2 اس بات کو یقینی بناتا ہے کہ وہ ایک وقت میں ایک ہی چلیں۔

**لاگنگ:** تشخیصی آؤٹ پٹ کے لیے ہمیشہ `self.get_logger()` استعمال کریں۔ ROS نوڈز میں کبھی بھی `print()` استعمال نہ کریں۔ لاگر ROS 2 کے لاگنگ سسٹم کے ساتھ مربوط ہوتا ہے۔

**نامकरण:** نوڈ کے نام اہم ہیں۔ وہ `ros2 node list` اور `rqt_graph` میں نظر آتے ہیں۔ `node1` کی بجائے `sensor_publisher` جیسے وضاحتی نام استعمال کریں۔

**خدمت کا معیار (Quality of Service - QoS):** `create_publisher()` میں `10` قطار کی گہرائی (queue depth) ہے۔ اگر سبسکرائبر پیچھے رہ جاتا ہے، تو ROS 2 آخری 10 میسجز کو محفوظ رکھتا ہے۔ حقیقی وقت کے نظام کے لیے، یہ نمبر مختلف ہو سکتا ہے۔

---

## AI کے ساتھ کوشش کریں (Try With AI)

آپ کے پاس ایک کام کرنے والا پبلشر ہے۔ اب آئیے اسے بہتر بنانے کے لیے AI کا استعمال کریں۔

**اپنے AI سے پوچھیں:**

> "میرے پاس ایک سادہ ROS 2 پبلشر ہے جو ہر 0.5 سیکنڈ میں String میسجز بھیجتا ہے۔ میں اسے پروڈکشن کے قابل بنانا چاہتا ہوں۔ غلطی سے بچاؤ (error handling)، لاگنگ کی ترتیب (logging configuration)، اور اچھے طریقے سے بند ہونے (graceful shutdown) کے لیے مجھے کیا شامل کرنا چاہیے؟ مخصوص کوڈ بہتری بتائیں۔"

**متوقع نتیجہ:** AI تجویز کرے گا:
- غلطی سے بچاؤ کے لیے try/except بلاکس شامل کریں۔
- لاگنگ کی سطحیں (INFO, DEBUG, WARNING) ترتیب دیں۔
- باہر نکلنے پر پبلشنگ روکنے کے لیے ایک شٹ ڈاؤن ہک شامل کریں۔
- کوڈ کی وضاحت کے لیے docstrings شامل کریں۔

**AI کی تجاویز کو چیلنج کریں:**

> "اچھی بات ہے۔ میرے پبلشر کو کم بینڈوتھ والے حالات میں کام کرنے کی ضرورت ہے جہاں ہم فی سیکنڈ صرف 5 میسجز بھیج سکتے ہیں۔ آپ اس پابندی کو کوڈ میں کیسے شامل کریں گے؟ کیا یہ ایک سخت حد ہونی چاہیے یا ایک قابل ترتیب پیرامیٹر (configurable parameter)؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- پبلشنگ کو ہر Nویں کال بیک پر چھوڑنے کے لیے ایک کاؤنٹر استعمال کریں۔
- OR مشروط پبلشنگ کے ساتھ ایک چھوٹا ٹائمر پیریڈ استعمال کریں۔
- اسے package.xml میں یا ros2 param set کے ذریعے ایک قابل ترتیب پیرامیٹر بنائیں۔

**مل کر کام کریں:**

> "اچھا نکتہ ہے۔ آئیے قابل ترتیب پیرامیٹر کے طریقے کو لاگو کریں۔ دکھائیں کہ میں 'publish_rate' پیرامیٹر کیسے شامل کروں جسے صارف runtime پر `ros2 param set` کا استعمال کرکے سیٹ کر سکے۔"

**متوقع نتیجہ:** AI آپ کو دکھائے گا کہ کیسے:
- `__init__` میں پیرامیٹر کا اعلان کریں۔
- کال بیک میں اسے پڑھیں۔
- runtime پر اسے تبدیل کرنے کی اجازت دیں۔

اس بات چیت کے ذریعے، AI نے پیٹرن تجویز کیے، آپ نے پابندیاں فراہم کیں، اور مل کر آپ ایک بہتر حل پر پہنچے۔

---

## مشقیں (Exercises)

1. **پبلشر کو تبدیل کریں تاکہ وہ ہر بار 10 کے بجائے 10 کے اضافے سے گنتی بھیجے** (مثلاً: 0, 10, 20, 30...)
2. **اسی پیکیج میں ایک دوسرا پبلشر نوڈ بنائیں جو مختلف ڈیٹا بھیجے۔**
3. **دونوں پبلشرز کے بھیجنے کی تصدیق کے لیے `ros2 topic info` استعمال کریں۔**

---

## غور کریں (Reflect)

ان سوالات پر غور کریں:

1. **نوڈ لوپ کے بجائے ٹائمر کیوں استعمال کرتا ہے؟** `while True: publish()` کے مقابلے میں `create_timer()` کا کیا فائدہ ہے؟

2. **اگر کوئی سبسکرائب نہ کر رہا ہو تو میسجز کا کیا ہوتا ہے؟** کیا پبلشر کو پتہ ہوتا ہے؟ کیا اسے پرواہ کرنی چاہیے؟

3. **آپ پبلش ریٹ کو کیسے تبدیل کریں گے؟** کون سی کوڈ تبدیلیاں درکار ہیں، اور بہتر طریقہ کیا ہو گا؟

اگلے سبق میں، آپ اس پبلشر سے بھیجے گئے ڈیٹا کو سننے کے لیے ایک سبسکرائبر لکھیں گے۔