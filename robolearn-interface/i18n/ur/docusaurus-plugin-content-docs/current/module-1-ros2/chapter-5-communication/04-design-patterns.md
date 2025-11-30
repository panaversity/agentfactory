---
id: design-patterns
title: 'Lesson 5.4: Communication Design Patterns'
sidebar_label: 5.4 Design Patterns
sidebar_position: 4
chapter: 5
lesson: 4
duration_minutes: 75
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 6
learning_objectives:
  - Design communication architectures using topics and services
  - Apply decision frameworks for topics vs services
  - Compose multi-node systems with mixed communication patterns
  - Debug communication issues in complex systems
  - Refine communication architecture through AI iteration
skills:
  - ros2-service-pattern
  - ros2-custom-interfaces
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct) with multiple terminal windows
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 5.4: مواصلات کے ڈیزائن پیٹرن (Communication Design Patterns)

آپ اب ٹاپکس (مسلسل اسٹریمز)، سروسز (درخواست/جواب)، اور کسٹم انٹرفیسز (ٹائپ-سیف مواصلات) بنانا سیکھ چکے ہیں۔ لیکن آپ انہیں ایک حقیقی نظام میں کیسے یکجا کرتے ہیں؟

اس سبق میں، آپ **مواصلاتی فن تعمیر کے پیٹرن** سیکھیں گے—یعنی حقیقی روبوٹس کس طرح ایسے نظام ڈیزائن کرتے ہیں جو متعدد نوڈز کو مربوط کرتے ہیں، جن میں سے ہر ایک کا ایک مخصوص کام ہوتا ہے۔ آپ دیکھیں گے کہ ناسا روور کی مواصلات کیسے ڈیزائن کرتا ہے، گودام کے روبوٹس سینسر ڈیٹا کو کیسے منظم کرتے ہیں، اور ہیومنائڈ روبوٹس جسمانی کنٹرول کو کیسے منظم کرتے ہیں۔

اہم نکتہ: **اچھا فن تعمیر پوشیدہ ہوتا ہے۔ آپ اس پر تب تک توجہ نہیں دیتے جب تک کہ کچھ ٹوٹ نہ جائے۔**

---

## فیصلہ سازی کا فریم ورک: ٹاپکس بمقابلہ سروسز (دوبارہ جائزہ)

آئیے اس فیصلے کو باقاعدہ شکل دیں جو آپ بدیہی طور پر کر رہے تھے:

```
شروع: کیا ہمیں ڈیٹا بھیجنے کی ضرورت ہے؟
  └─ ہاں
       └─ کیا وصول کنندہ کو جواب/تصدیق کی ضرورت ہے؟
            ├─ نہیں → ٹاپک استعمال کریں (فائر اینڈ بھول جائیں)
            └─ ہاں
                 └─ کیا بھیجنے والا انتظار کرتے ہوئے بلاک ہو سکتا ہے؟
                      ├─ ہاں → سروس استعمال کریں (ہم آہنگ/Synchronous)
                      └─ نہیں → ایکشن استعمال کریں (ماڈیول 2)
```

**حقیقی مثالیں:**

| منظر نامہ | پیٹرن | کیوں |
|----------|---------|-----|
| کیمرہ 30x/سیکنڈ کی شرح سے تصاویر شائع کرتا ہے | ٹاپک | مسلسل، جواب کی ضرورت نہیں |
| بیٹری ہر 2 سیکنڈ میں اسٹیٹس شائع کرتی ہے | ٹاپک | مسلسل اسٹریم |
| روبوٹ کو 1 میٹر آگے بڑھائیں | سروس | تصدیق کی ضرورت ہے، نسبتاً نایاب |
| روبوٹ کو فعال/غیر فعال کریں | سروس | جواب کے ساتھ کمانڈ، غیر متواتر |
| اوڈومیٹری (پوزیشن اپ ڈیٹس) شائع کریں | ٹاپک | مسلسل، متعدد سبسکرائبرز |
| موجودہ بیٹری لیول حاصل کریں | سروس | استفسار کا جواب، آن ڈیمانڈ |

**روبوٹکس کے لیے فیصلہ سازی کا درخت:**

```
سینسر ریڈنگ؟ → ٹاپک
حرکت کی کمانڈ؟ → سروس
ہنگامی اسٹاپ؟ → سروس (تیز، ہم آہنگ)
ٹیلی میٹری ڈیٹا؟ → ٹاپک
کنفیگریشن تبدیلی؟ → سروس
موٹر اسپیڈ اپ ڈیٹس؟ → ٹاپک
چیز اٹھائیں (آن/آف)؟ → سروس
```

---

## فن تعمیر کا پیٹرن 1: پرت دار مواصلات (Layered Communication) (ہب اور اسپاک)

زیادہ تر روبوٹس میں ایک **مرکزی کوآرڈینیٹر** ہوتا ہے جو خصوصی ماڈیولز سے بات کرتا ہے۔

```
                    ┌─────────────┐
                    │  مرکزی       │
                    │  کوآرڈینیٹر │
                    └──────┬──────┘
            ┌───────┬──────┼──────┬────────┐
            │       │      │      │        │
        سبسکرائب سبسکرائب سبسکرائب شائع کریں
        حالت پر سینسر پر بیٹری پر
            │       │      │      │
         ┌──▼──┐ ┌─▼──┐ ┌─▼──┐ ┌─▼───┐
         │موٹر │ │بازو │ │سر  │ │پاؤں │
         │کنٹرول│ │کنٹرول│ │کنٹرول│ │کنٹرول │
         └─────┘ └────┘ └────┘ └─────┘
```

**پیٹرن:**
- مرکزی نوڈ تمام ماڈیولز سے اسٹیٹس سبسکرائب کرتا ہے
- مرکزی نوڈ سروسز کے ذریعے مخصوص ماڈیولز کو کمانڈ بھیجتا ہے
- ماڈیولز ٹاپکس کے ذریعے مسلسل سینسر ڈیٹا شائع کرتے ہیں
- ہنگامی اسٹاپ کے لیے وقف شدہ ہائی-پرائیورٹی ٹاپک استعمال ہوتا ہے

**فائدہ:** کوآرڈینیٹر کے پاس مکمل تصویر ہوتی ہے۔ ماڈیولز آزاد ہوتے ہیں۔

**نقصان:** ہائی فریکوئنسی اپ ڈیٹس کے لیے مرکزی نوڈ رکاوٹ بن جاتا ہے۔

---

## فن تعمیر کا پیٹرن 2: تقسیم شدہ Pub/Sub (Many-to-Many)

جب آپ کو مرکزی کوآرڈینیٹر کی ضرورت نہ ہو:

```
┌────────────┐    ┌─────────────┐    ┌──────────┐
│  سینسرز   │    │  منصوبہ بندی │    │  ایکچویٹرز
│            │    │             │    │
│ شائع کرتے ہیں: │    │ سبسکرائب کرتے ہیں: │    │ سبسکرائب کرتے ہیں:
│ - کیمرہ    │────▶ - کیمرہ    │    │ - motion_cmd
│ - لیڈر    │    │ - لیڈر     │    │
│ - آئی ایم یو │    │             │    │ شائع کرتے ہیں:
│            │    │ شائع کرتے ہیں: │    │ - motor_state
│            │    │ - motion_cmd────▶
│            │    │ - اشارہ    │    │
│            │    │             │    │
└────────────┘    └─────────────┘    └──────────┘
```

**پیٹرن:**
- ہر نوڈ اپنا آؤٹ پٹ ڈیٹا شائع کرتا ہے
- ہر نوڈ وہ ڈیٹا سبسکرائب کرتا ہے جس کی اسے ضرورت ہوتی ہے
- کوئی مرکزی کوآرڈینیٹر نہیں
- نوڈز ڈھیلے طریقے سے جڑے ہوئے ہیں (loosely coupled)

**فائدہ:** اچھی طرح سے اسکیل ہوتا ہے، کوئی بھی نوڈ آ یا جا سکتا ہے، کوئی رکاوٹ نہیں۔

**نقصان:** ڈیبگ کرنا مشکل (پیچیدہ باہمی روابط)۔

---

## فن تعمیر کا پیٹرن 3: درخواست/جواب کلسٹرز (Request/Response Clusters)

منفرد کاموں کے لیے سروسز استعمال کریں:

```
┌──────────────────────────────────────┐
│          روبوٹ مین لوپ             │
│                                      │
│  1. /get_sensor_reading کو کال کریں     │
│  2. /plan_motion کو کال کریں           │
│  3. /execute_command کو کال کریں        │
│  4. /report_status کو کال کریں         │
└────┬─────────┬────────────┬──────────┘
     │         │            │
     │         │            │
   سروس     سروس        سروس
     │         │            │
  ┌──▼──┐  ┌──▼──┐      ┌──▼──┐
  │سینسر│  │منصوبہ کار │موٹر │
  │سرور│  │سرور    │سرور│
  └──────┘  └────────┘  └─────┘
```

**پیٹرن:**
- مین لوپ سروس کالز کے ذریعے منظم کرتا ہے
- سروسز منفرد کام انجام دیتی ہیں
- ہم آہنگ، متعین (deterministic)

**فائدہ:** واضح کنٹرول فلو، آپریشنز کو ترتیب دینا آسان۔

**نقصان:** سست (سروسز جواب کا انتظار کرتی ہیں)۔

---

## حقیقی مثال: دو-نوڈ مواصلات

آئیے دو نوڈز کے ساتھ ایک مکمل نظام بناتے ہیں:
- **سینسر نوڈ**: سینسر ریڈنگز شائع کرتا ہے اور استفسارات کا جواب دیتا ہے
- **کنٹرول نوڈ**: سینسرز کو سبسکرائب کرتا ہے اور انہیں کمانڈ دیتا ہے

### انٹرفیس پیکیج (مشترکہ)

انٹرفیسز بنائیں (`my_robot_interfaces`):

**msg/SensorReading.msg:**
```
builtin_interfaces/Time timestamp
string sensor_name
float64 value
```

**srv/GetSensorData.srv:**
```
string sensor_id
---
float64 latest_value
string status
```

**srv/SetMotorSpeed.srv:**
```
string motor_id
float64 speed_rpm
---
bool success
string message
```

### سینسر نوڈ

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorReading
from my_robot_interfaces.srv import GetSensorData

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # مسلسل سینسر ڈیٹا شائع کریں
        self.publisher_ = self.create_publisher(
            SensorReading, 'sensor_data', 10)

        # استفسارات کا جواب دیں
        self.service = self.create_service(
            GetSensorData,
            'query_sensor',
            self.query_callback)

        # سینسر ڈیٹا کی نقالی کریں
        self.temperature = 25.0
        self.timer = self.create_timer(1.0, self.publish_sensor)

    def publish_sensor(self):
        """مسلسل سینسر ریڈنگز شائع کریں۔"""
        msg = SensorReading()
        msg.timestamp.sec = 0  # آسان بنایا گیا
        msg.sensor_name = 'temperature'
        self.temperature += 0.1  # بتدریج اضافہ کی نقالی کریں
        msg.value = self.temperature

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: temperature={msg.value:.1f}')

    def query_callback(self, request, response):
        """سینسر استفسارات کا جواب دیں۔"""
        self.get_logger().info(f'Query for sensor: {request.sensor_id}')

        if request.sensor_id == 'temperature':
            response.latest_value = self.temperature
            response.status = 'OK'
        else:
            response.latest_value = 0.0
            response.status = 'UNKNOWN_SENSOR'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کنٹرول نوڈ

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorReading
from my_robot_interfaces.srv import GetSensorData, SetMotorSpeed

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # مسلسل سینسر اسٹریم کو سبسکرائب کریں
        self.subscription = self.create_subscription(
            SensorReading,
            'sensor_data',
            self.sensor_callback,
            10)

        # آن ڈیمانڈ استفسارات کے لیے کلائنٹس بنائیں
        self.sensor_client = self.create_client(
            GetSensorData, 'query_sensor')
        self.motor_client = self.create_client(
            SetMotorSpeed, 'set_motor')

        # کنٹرول لوپ
        self.timer = self.create_timer(2.0, self.control_loop)
        self.latest_temp = 0.0

    def sensor_callback(self, msg):
        """اسٹریم شدہ سینسر ڈیٹا کو ہینڈل کریں۔"""
        self.latest_temp = msg.value
        self.get_logger().debug(
            f'Received {msg.sensor_name}: {msg.value:.1f}')

    def control_loop(self):
        """ہر 2 سیکنڈ میں مرکزی کنٹرول منطق۔"""
        self.get_logger().info(
            f'Control loop: latest temp = {self.latest_temp:.1f}')

        # فیصلہ: اگر درجہ حرارت زیادہ ہے، تو موٹر کو سست کریں
        if self.latest_temp > 30.0:
            self.command_motor('motor_0', 50.0)  # 50 RPM تک کم کریں
        else:
            self.command_motor('motor_0', 100.0)  # نارمل رفتار

    def command_motor(self, motor_id, speed_rpm):
        """سروس کے ذریعے موٹر کمانڈ بھیجیں۔"""
        if not self.motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Motor service unavailable')
            return

        request = SetMotorSpeed.Request()
        request.motor_id = motor_id
        request.speed_rpm = speed_rpm

        future = self.motor_client.call_async(request)
        future.add_done_callback(self.motor_response_callback)

    def motor_response_callback(self, future):
        """موٹر کمانڈ کے جواب کو ہینڈل کریں۔"""
        response = future.result()
        self.get_logger().info(f'Motor response: {response.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### موٹر سروس نوڈ

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetMotorSpeed

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.service = self.create_service(
            SetMotorSpeed,
            'set_motor',
            self.motor_callback)

        self.motor_speeds = {}
        self.get_logger().info('Motor service ready')

    def motor_callback(self, request, response):
        """موٹر کمانڈز پر عمل کریں۔"""
        self.motor_speeds[request.motor_id] = request.speed_rpm

        self.get_logger().info(
            f'Set {request.motor_id} to {request.speed_rpm} RPM')

        response.success = True
        response.message = f'Motor set to {request.speed_rpm} RPM'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### جانچ کرنا (Testing)

**ٹرمینل 1: سینسر نوڈ**
```bash
ros2 run my_first_package sensor_node
```

**ٹرمینل 2: موٹر نوڈ**
```bash
ros2 run my_first_package motor_node
```

**ٹرمینل 3: کنٹرول نوڈ**
```bash
ros2 run my_first_package control_node
```

آپ دیکھیں گے:
- سینسر مسلسل درجہ حرارت شائع کر رہا ہے
- کنٹرول اسے وصول کرتا ہے اور درجہ حرارت کی بنیاد پر موٹر کو ایڈجسٹ کرتا ہے
- موٹر سروس کمانڈز پر جواب دیتی ہے

---

## مواصلاتی مسائل کی ڈیبگنگ

### مسئلہ 1: "سروس دستیاب نہیں" (Service Not Available)

```python
# علامات: کال فوری طور پر ناکام ہو جاتی ہے
# وجہ: سروس چل نہیں رہی
# حل: چیک کریں کہ یہ لانچ ہے
ros2 service list
# آپ کی سروس دیکھنی چاہیے
```

### مسئلہ 2: "پیغام کی قسم کا بے میل ہونا" (Message Type Mismatch)

```python
# علامات: RuntimeError جو پیغام کی اقسام کے بارے میں ہو
# وجہ: ٹاپک پر غلط قسم شائع کرنا
msg = String()  # غلط
msg = RobotStatus()  # صحیح

# حل: پبلشر اور سبسکرائبر کی اقسام کی تصدیق کریں
```

### مسئلہ 3: "سست جواب" (Slow Response)

```python
# علامات: سروس کو 2+ سیکنڈ لگتے ہیں
# وجہ: کال بیک بھاری کمپیوٹیشن کر رہا ہے
def callback(self, request, response):
    response.value = expensive_calculation()  # بلاک کرتا ہے!
    return response

# حل: مہنگا کام غیر مطابقت پذیر (asynchronously) کریں۔
self.executor.submit(expensive_calculation, response)
```

### مسئلہ 4: "مردہ نوڈز" (Dead Nodes)

```python
# علامات: ایک نوڈ کریش ہو جاتا ہے، نظام ہمیشہ کے لیے رک جاتا ہے
# وجہ: دوسرے نوڈز ہمیشہ کے لیے انتظار کر رہے ہیں
# حل: ہمیشہ ٹائم آؤٹ استعمال کریں
cli.wait_for_service(timeout_sec=2.0)

# یا try/except استعمال کریں
try:
    response = cli.call(request)
except Exception:
    self.get_logger().error('Service failed')
```

### ویژولائزیشن ٹولز

```bash
# تمام نوڈز اور روابط دیکھیں
rqt_graph

# تمام ٹاپکس دیکھیں
ros2 topic list

# تمام سروسز دیکھیں
ros2 service list

# ایک ٹاپک کی نگرانی کریں
ros2 topic echo /topic_name

# سروس کال ریٹ کی نگرانی کریں
ros2 service call /service_name ServiceType "{field: value}"
```

---

## ڈیزائن کے غلط پیٹرن (Antipatterns) (کیا نہیں کرنا چاہیے)

### غلط پیٹرن 1: مسلسل ڈیٹا کے لیے سروس کا استعمال

```python
# برا: سروس ہر 0.1 سیکنڈ میں درجہ حرارت شائع کر رہی ہے
self.client.call(GetTemperature)  # جواب کا انتظار کرتے ہوئے بلاک ہو جاتا ہے
self.client.call(GetTemperature)  # دوبارہ بلاک ہو جاتا ہے
```

**حل:** اس کے بجائے ٹاپک استعمال کریں۔ سروسز نایاب کالز کے لیے ہیں۔

### غلط پیٹرن 2: گھنٹی دار سروسز (Nested Services)

```python
# برا: سروس ہینڈلر دوسری سروس کو کال کرتا ہے
def callback(self, request, response):
    # یہ دوسری سروس کا انتظار کرتے ہوئے بلاک ہو جاتا ہے!
    response = self.other_client.call(other_request)  # خطرناک!
    return response
```

**حل:** ڈیٹا کے بہاؤ کے لیے ٹاپکس استعمال کریں، سروسز صرف سادہ استفسارات کے لیے۔

### غلط پیٹرن 3: ہر چیز کو ہم آہنگ بنانا (Synchronous Everything)

```python
# برا: مین لوپ بالترتیب 5 سروسز کو کال کرتا ہے
self.service1.call()  # 100ms بلاک کریں
self.service2.call()  # 100ms بلاک کریں
self.service3.call()  # 100ms بلاک کریں
# کل: ہر لوپ تکرار کے لیے 300ms
```

**حل:** غیر مطابقت پذیر کالز اور کال بیکس استعمال کریں، یا ٹاپکس کے ساتھ دوبارہ ڈھانچہ بنائیں۔

### غلط پیٹرن 4: ہارڈکوڈڈ ٹاپک کے نام

```python
# برا: ٹاپک کا نام کوڈ میں چھپا ہوا ہے
self.create_publisher(Type, '/robot/sensor/temp', 10)
# نام بدلنا یا دوبارہ ترتیب دینا مشکل ہے

# اچھا: پیرامیٹرائزڈ
self.declare_parameter('sensor_topic', '/robot/sensor/temp')
topic = self.get_parameter('sensor_topic').value
self.create_publisher(Type, topic, 10)
```

---

## اچھے فن تعمیر کے لیے کلیدی اصول

1. **واضح ملکیت (Clear Ownership)**: ہر ٹاپک/سروس کا مالک ایک ہی پبلشر/سروس ہوتا ہے
2. **ڈھیلی جوڑ (Loose Coupling)**: نوڈز دوسروں کے اندرونی نفاذ پر منحصر نہیں ہوتے
3. **شائستہ انحطاط (Graceful Degradation)**: نظام غائب نوڈز کو ہینڈل کرتا ہے (ٹائم آؤٹ، فال بیکس)
4. **قابل مشاہدہ (Observable)**: بصری بنانے اور ڈیبگ کرنے کے لیے ROS 2 ٹولز استعمال کریں
5. **قابل جانچ (Testable)**: نوڈز کو موک کریں، مواصلات کو تنہائی میں جانچیں۔

---

## AI کے ساتھ کوشش کریں

آپ کے پاس ایک کام کرنے والا ملٹی-نوڈ نظام ہے۔ آئیے اسے فن تعمیر کے لحاظ سے بہتر بنائیں۔

**اپنے AI سے پوچھیں:**

> "میرے پاس سینسر، کنٹرول، اور موٹر نوڈز ہیں جو ٹاپکس اور سروسز کے ذریعے بات چیت کر رہے ہیں۔ جیسے جیسے روبوٹ زیادہ پیچیدہ ہوتا جاتا ہے، میں ایک منصوبہ بندی کا نوڈ، ایک حفاظتی مانیٹر، اور ایک لاگنگ سروس شامل کرنا چاہتا ہوں۔ ان میں سے ہر ایک کہاں فٹ ہونا چاہیے؟ کون سے ٹاپکس/سروسز شامل کیے جانے چاہئیں؟ مواصلاتی خاکہ بنائیں اور ڈیزائن کی منطق کی وضاحت کریں۔"

**متوقع نتیجہ:** AI تجویز کرے گا:
- حفاظتی مانیٹر ایک الگ ہائی پرائیورٹی لسنر کے طور پر
- منصوبہ بندی کا نوڈ سینسرز کو سبسکرائب کرے، موشن پلان شائع کرے
- ایک لاگر سروس جسے متعدد نوڈز کال کر سکتے ہیں
- ہر انتخاب کے لیے منطق کی وضاحت

**ڈیزائن کو چیلنج کریں:**

> "کیا ہوگا اگر منصوبہ بندی کا نوڈ سست ہو جائے؟ کیا کنٹرول نوڈ کو منصوبہ بندی کا انتظار کرنا چاہیے یا پرانے منصوبے کے ساتھ آگے بڑھنا چاہیے؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- غیر مطابقت پذیر پیٹرن تاکہ کنٹرول بلاک نہ ہو
- فال بیک رویے
- ٹائم آؤٹ ہینڈلنگ
- تازگی بمقابلہ جوابدہی کے درمیان مفادات کا ٹکراؤ (Tradeoffs)

**مل کر تکرار کریں:**

> "سمجھ گیا۔ مجھے ایک کنٹرول لوپ کا کوڈ دکھائیں جو: (1) اسٹیٹس ٹاپک شائع کرتا ہے، (2) منصوبہ بندی کی سروس کو غیر مطابقت پذیر طریقے سے کال کرتا ہے، (3) موٹر سروس کو صرف اس وقت کال کرتا ہے جب محفوظ ہو، (4) منصوبہ بندی دستیاب نہ ہونے پر فال بیک رکھتا ہو۔ مکمل نفاذ بشمول غلطی ہینڈلنگ کے ساتھ۔"

یہ تعاون کے ذریعے فن تعمیر کے ڈیزائن کو ظاہر کرتا ہے۔

---

## مشقیں

1. **دو-نوڈ نظام کو بڑھائیں** ایک تیسرے "حفاظتی مانیٹر" نوڈ کے ساتھ جو سینسر ڈیٹا کو سبسکرائب کرتا ہے اور انتباہات شائع کرتا ہے۔
2. **ایک پیرامیٹر سروس شامل کریں** جو آپ کو دوبارہ شروع کیے بغیر موٹر اسپیڈ کی حدود کو تبدیل کرنے دے۔
3. **ایک لانچ فائل بنائیں** جو تینوں نوڈز کو ایک ساتھ شروع کرے۔
4. `rqt_graph` کے ساتھ **مواصلات کا بصری جائزہ** لیں۔
5. **غلطی ہینڈلنگ کو نافذ کریں** تاکہ اگر موٹر سروس دستیاب نہ ہو تو کنٹرول نوڈ جاری رہے۔

---

## غور و فکر

سنگ بنیاد (سبق 6) سے پہلے، ان پر غور کریں:

- آپ اپنے ڈیزائن میں ٹاپک بمقابلہ سروس کب استعمال کریں گے؟
- 5+ نوڈز والے نظام کو آپ کیسے ڈیبگ کرتے ہیں؟
- ملٹی-نوڈ نظاموں میں سب سے زیادہ کیا ٹوٹتا ہے؟

**اگلا باب (باب 6): نظام بنانا**

آپ نے تمام مواصلاتی پیٹرن سیکھ لیے ہیں۔ اگلا آپ سیکھیں گے کہ متعدد نوڈز کو کیسے لانچ کیا جائے، بڑے منصوبوں کو کیسے منظم کیا جائے، اور حقیقی روبوٹ نظام کیسے بنائے جائیں جو اسکیل ہوں۔