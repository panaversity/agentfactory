---
id: custom-messages
title: 'Lesson 5.3: Custom Message and Service Definitions'
sidebar_label: 5.3 Custom Interfaces
sidebar_position: 3
chapter: 5
lesson: 3
duration_minutes: 75
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 5
learning_objectives:
  - Understand why custom message types are needed
  - Create .msg and .srv interface files
  - Configure CMakeLists.txt and package.xml for interfaces
  - Build and use custom interfaces in nodes
  - Design interfaces that communicate intent clearly
skills:
  - ros2-custom-interfaces
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 5.3: کسٹم میسج اور سروس کی تعریفیں

اب تک آپ نے بلٹ ان میسج ٹائپس استعمال کی ہیں: `String`، `SetBool`، `AddTwoInts`۔ یہ سادہ کیسز کے لیے کام کرتی ہیں۔ لیکن کیا ہو اگر آپ کو کچھ زیادہ پیچیدہ بھیجنے کی ضرورت ہو؟

تصور کریں کہ ایک روبوٹ اپنی حالت (status) کی رپورٹ کرتا ہے۔ اسے یہ بھیجنے کی ضرورت ہے:
- بیٹری لیول (float)
- درجہ حرارت (float)
- فعال کام (string)
- جوائنٹ اینگلز (floats کا array)
- ٹائم سٹیمپ (بِلٹ اِن وقت)

ان سب کو پیک کرنے کے لیے `String` استعمال کرنا بہت خراب ہوگا۔ آپ کو سٹرنگز کو پارس کرنا پڑے گا، کوئی ٹائپ سیفٹی نہیں ہوگی، اور غلطیوں کا امکان ہوگا۔

**کسٹم میسجز** آپ کو ڈیٹا سٹرکچر کی تعریف کرنے دیتے ہیں: "ایک `RobotStatus` میں ایک فلوٹ بیٹری، فلوٹ درجہ حرارت، سٹرنگ ٹاسک، ایرے جوائنٹ_پوزیشنز، اور ٹائم سٹیمپ شامل ہے"۔ پھر ROS 2 خود بخود پائپھن کلاسز تیار کرتا ہے۔

یہ سبق آپ کو سکھائے گا:
1. `.msg` فائلیں بنانا (میسج کی تعریفیں)
2. `.srv` فائلیں بنانا (سروس ریکویسٹ/رسپانس کی تعریفیں)
3. بلڈ فائلوں کو کنفیگر کرنا تاکہ ROS 2 کوڈ تیار کر سکے
4. اپنے نوڈز میں کسٹم ٹائپس استعمال کرنا

---

## کسٹم میسجز کیوں اہم ہیں؟

### کسٹم میسجز کے بغیر (برا)

```python
# پبلشر جو "battery:85.5,temp:45.2,task:moving" بھیج رہا ہے
msg = String()
msg.data = "battery:85.5,temp:45.2,task:moving"
self.publisher_.publish(msg)

# سبسکرائبر جو وصول کر رہا ہے اور پارس کر رہا ہے
if msg.data:
    parts = msg.data.split(',')
    battery = float(parts[0].split(':')[1])  # برا پارسنگ
    temp = float(parts[1].split(':')[1])
    task = parts[2].split(':')[1]
```

مسائل:
- پارسنگ میں غلطیوں کا امکان
- کوئی ٹائپ سیفٹی نہیں
- اس بات کی کوئی دستاویز نہیں کہ کون سے فیلڈ موجود ہیں
- توسیع کرنا مشکل (نیا فیلڈ شامل کریں = موجودہ کوڈ ٹوٹ جائے گا)

### کسٹم میسجز کے ساتھ (اچھا)

```python
# پبلشر
msg = RobotStatus()
msg.battery = 85.5
msg.temperature = 45.2
msg.task = "moving"
self.publisher_.publish(msg)

# سبسکرائبر جو وصول کر رہا ہے
self.get_logger().info(
    f'Battery: {msg.battery}, Temp: {msg.temperature}, Task: {msg.task}')
```

فوائد:
- ٹائپ سیف (پائپھن جانتا ہے کہ `battery` ایک فلوٹ ہے)
- سیلف-ڈاکومنٹنگ (واضح فیلڈ کے نام)
- IDE آٹو کمپلیٹ کام کرتا ہے
- توسیع کرنا آسان

---

## کسٹم میسج بنانا

آئیے `RobotStatus` میسج بناتے ہیں۔ سب سے پہلے، آپ کو انٹرفیس کے لیے وقف ایک پیکیج درکار ہے۔

### مرحلہ 1: ایک انٹرفیس پیکیج بنائیں

ROS 2 کنونشن: انٹرفیس پیکیجز کو ایگزیکیوٹیبل پیکیجز سے الگ رکھا جاتا ہے۔ ایک نیا پیکیج بنائیں:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_interfaces
cd my_robot_interfaces
```

یہ بناتا ہے:
```
my_robot_interfaces/
├── CMakeLists.txt
├── package.xml
├── src/          (آپ اسے ڈیلیٹ کر سکتے ہیں)
└── include/      (آپ اسے ڈیلیٹ کر سکتے ہیں)
```

میسج اور سروس فولڈرز شامل کریں:

```bash
mkdir msg
mkdir srv
```

### مرحلہ 2: میسج فائل بنائیں

`msg/RobotStatus.msg` بنائیں:

```
# File: msg/RobotStatus.msg
# روبوٹ کی حالت کی رپورٹنگ کے لیے کسٹم میسج

# معیاری ٹائم سٹیمپ
builtin_interfaces/Time stamp

# بنیادی حالت
bool is_active
float64 battery_level    # رینج: 0.0 سے 100.0

# سینسر ریڈنگز
float64 temperature      # سیلسیس
float64[] joint_angles   # ریڈینز میں جوائنٹ پوزیشنز کا ایرے

# موجودہ کام
string current_task
```

ہر لائن ایک فیلڈ ہے۔ فارمیٹ: `type name` جس کے ساتھ اختیاری `# comment`

**دستیاب ڈیٹا ٹائپس:**
- `bool`, `byte`, `char` (ایک حرف)
- `int8`, `int16`, `int32`, `int64` (سائنڈ)
- `uint8`, `uint16`, `uint32`, `uint64` (ان سائنڈ)
- `float32`, `float64`
- `string` (UTF-8 ٹیکسٹ)
- `builtin_interfaces/Time` (ٹائم سٹیمپ)
- `geometry_msgs/Point`, `geometry_msgs/Pose` (معیاری جیومیٹری)

**ایریز (Arrays):**
- `float64[]` — غیر محدود ایرے (کسی بھی سائز کا)
- `float64[4]` — فکسڈ ایرے (بالکل 4 ایلیمنٹس)
- `float64[<=10]` — باؤنڈڈ ایرے (زیادہ سے زیادہ 10 ایلیمنٹس)

### مرحلہ 3: ایک کسٹم سروس بنائیں

اب روبوٹ کو کنٹرول کرنے کے لیے ایک سروس بنائیں. اسے ایک کمانڈ لینی چاہیے اور کامیابی/ناکامی واپس کرنی چاہیے۔

`srv/RobotCommand.srv` بنائیں:

```
# ریکویسٹ: جو کلائنٹ بھیجتا ہے
string command_type      # مثال: "start", "stop", "pause"
float64 parameter        # مثال: سپیڈ فیکٹر

---
# رسپانس: جو سرور واپس بھیجتا ہے
bool success
string message           # فیڈ بیک یا ایرر میسج
float64 execution_time   # کتنا وقت لگا (سیکنڈز میں)
```

`---` ریکویسٹ (اوپر) کو رسپانس (نیچے) سے الگ کرتا ہے۔

### مرحلہ 4: CMakeLists.txt کو اپ ڈیٹ کریں

`CMakeLists.txt` کو ROS 2 کو آپ کی .msg اور .srv فائلوں سے کوڈ تیار کرنے کے لیے بتانے کی ضرورت ہے۔

`CMakeLists.txt` میں ترمیم کریں:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# انحصار تلاش کریں (Find dependencies)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# انٹرفیسز تیار کریں (Generate interfaces)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/RobotCommand.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# ament بلڈ سسٹم کے لیے ضروری
ament_package()
```

**اہم حصے:**
- `find_package(rosidl_default_generators REQUIRED)` — کوڈ جنریشن کو فعال کرتا ہے
- `rosidl_generate_interfaces()` — .msg/.srv فائلوں سے پائپھن/C++ کوڈ تیار کرتا ہے
- `DEPENDENCIES builtin_interfaces geometry_msgs` — ان پیکیجز کی فہرست جن پر آپ کے انٹرفیس منحصر ہیں۔

### مرحلہ 5: package.xml کو اپ ڈیٹ کریں

بلڈ انحصار (build dependencies) کو ظاہر کرنے کے لیے `package.xml` میں ترمیم کریں:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.1</version>
  <description>Custom interfaces for robot communication</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

**نازک لائن:**
```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

یہ ROS 2 بلڈ سسٹم کو بتاتا ہے کہ یہ ایک انٹرفیس پیکیج ہے۔

### مرحلہ 6: انٹرفیس پیکیج کو بلڈ کریں

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

اگر یہ کامیاب ہو جاتا ہے، تو اب آپ کے پاس ہے:
- `from my_robot_interfaces.msg import RobotStatus`
- `from my_robot_interfaces.srv import RobotCommand`

تصدیق کریں:

```bash
ros2 interface show my_robot_interfaces/msg/RobotStatus
```

آؤٹ پٹ:

```
builtin_interfaces/Time stamp
bool is_active
float64 battery_level
float64 temperature
float64[] joint_angles
string current_task
```

---

## نوڈز میں کسٹم میسجز استعمال کرنا

اب ایک نوڈ بنائیں جو `RobotStatus` کو پبلش کرتا ہے:

اپنے ایگزیکیوٹیبل پیکیج میں `robot_status_publisher.py` بنائیں:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import RobotStatus
from builtin_interfaces.msg import Time

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_pub')
        self.publisher_ = self.create_publisher(
            RobotStatus, 'robot_status', 10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.battery = 100.0

    def publish_status(self):
        """ہر سیکنڈ میں روبوٹ کی حالت شائع کریں۔"""
        msg = RobotStatus()

        # ٹائم سٹیمپ سیٹ کریں (سادہ کیا گیا—عام طور پر get_clock() استعمال کریں گے)
        msg.stamp.sec = 0  # آپ اسے ROS کلاک سے حاصل کریں گے
        msg.stamp.nanosec = 0

        # دیگر فیلڈز سیٹ کریں
        msg.is_active = True
        msg.battery_level = self.battery
        msg.temperature = 42.5
        msg.joint_angles = [0.1, 0.2, 0.3]  # لسٹ ایرے بن جاتی ہے
        msg.current_task = "moving"

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: battery={msg.battery_level}, temp={msg.temperature}')

        # بیٹری ڈرین کی سمولیشن
        self.battery -= 0.5

def main(args=None):
    rclpy.init(args=args)
    pub = RobotStatusPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

اور ایک سبسکرائبر:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import RobotStatus

class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_sub')
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10)

    def status_callback(self, msg):
        """جب اسٹیٹس میسج آتا ہے تو کال کیا جاتا ہے۔"""
        self.get_logger().info(
            f'Battery: {msg.battery_level:.1f}%, '
            f'Temp: {msg.temperature:.1f}C, '
            f'Task: {msg.current_task}')

def main(args=None):
    rclpy.init(args=args)
    sub = RobotStatusSubscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**اہم:** آپ کو اپنے انٹرفیس پیکیج سے امپورٹ کرنا ہوگا:

```python
from my_robot_interfaces.msg import RobotStatus
```

---

## کسٹم سروسز استعمال کرنا

ایک نوڈ بنائیں جو `RobotCommand` سروس استعمال کرتا ہے:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import RobotCommand
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.service = self.create_service(
            RobotCommand,
            'control_robot',
            self.command_callback)

    def command_callback(self, request, response):
        """روبوٹ کمانڈز کو ہینڈل کریں۔"""
        start_time = time.time()

        self.get_logger().info(f'Command: {request.command_type}, param: {request.parameter}')

        if request.command_type == 'start':
            response.success = True
            response.message = 'Robot started'
        elif request.command_type == 'stop':
            response.success = True
            response.message = 'Robot stopped'
        else:
            response.success = False
            response.message = f'Unknown command: {request.command_type}'

        response.execution_time = time.time() - start_time
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

اسے کال کریں:

```python
from my_robot_interfaces.srv import RobotCommand

cli = node.create_client(RobotCommand, 'control_robot')

request = RobotCommand.Request()
request.command_type = 'start'
request.parameter = 0.5

future = cli.call_async(request)
```

---

## بلڈ پروسیجر (مکمل)

جب آپ کے پاس کسٹم انٹرفیس ہوں:

```bash
# 1. سب سے پہلے انٹرفیس پیکیج کو بلڈ کریں
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces

# 2. نئے انٹرفیسز استعمال کرنے کے لیے سورس کریں
source install/setup.bash

# 3. ان انٹرفیسز کو استعمال کرنے والے پیکیجز کو بلڈ کریں
colcon build --packages-select my_first_package

# 4. دوبارہ سورس کریں (نیا تیار شدہ کوڈ دستیاب ہے)
source install/setup.bash
```

**ترتیب اہم ہے:** انٹرفیس پیکیج کو اس سے پہلے بلڈ ہونا چاہیے جو اسے استعمال کرتا ہے۔

---

## میسجز کے لیے ڈیزائن کے اصول

**میسج ڈیزائن چیک لسٹ:**
1. ✅ تمام فیلڈز کے واضح نام اور تبصرے ہوں
2. ✅ مستقل یونٹس (میٹر، ریڈین، ہرتز، وغیرہ)
3. ✅ معیاری ٹائپس کا استعمال کریں جب ممکن ہو (geometry_msgs, sensor_msgs)
4. ✅ اگر وقت حساس ہو تو ٹائم سٹیمپ شامل کریں
5. ✅ ویلیو رینجز کی دستاویز بنائیں (مثلاً، بیٹری 0.0-100.0)
6. ✅ توسیع پذیری پر غور کریں (بعد میں نئے فیلڈز کے لیے جگہ)

**برا میسج ڈیزائن:**
```
float64 value1
float64 value2
string status
```
کسی کو معلوم نہیں کہ یہ کیا مطلب رکھتے ہیں۔ یونٹس کیا ہیں؟

**اچھا میسج ڈیزائن:**
```
float64 battery_level        # رینج: 0.0-100.0 فیصد
float64 motor_temperature    # سیلسیس
string operational_status    # اختیارات: idle, moving, charging, error
```
واضح، سیلف-ڈاکومنٹنگ، توسیع پذیر۔

---

## اہم نکات

**انٹرفیس پیکیجز کو الگ رکھیں:**
انٹرفیسز کو ان کے اپنے پیکیج میں رکھیں۔ جو نوڈز انہیں استعمال کرتے ہیں وہ انٹرفیس پیکیج پر منحصر ہوتے ہیں۔

**بلڈ آرڈر:**
سب سے پہلے انٹرفیسز، پھر باقی سب کچھ۔

**معیاری انحصار:**
جب بھی دستیاب ہو `geometry_msgs`، `sensor_msgs`، `builtin_interfaces` سے معیاری ٹائپس استعمال کریں۔

**ایریز بمقابلہ فیلڈز:**
- `float64[]` متغیر سائز کے ایریز کے لیے (جوائنٹ پوزیشنز روبوٹ کے لحاظ سے مختلف ہوتی ہیں)
- `float64[4]` فکسڈ سائز کے لیے جب تعداد کبھی تبدیل نہیں ہوتی

---

## AI کے ساتھ کوشش کریں

آپ کے پاس کسٹم میسجز اور سروسز ہیں۔ آئیے انہیں بڑھاتے ہیں۔

**اپنے AI سے پوچھیں:**

> "میں نے RobotStatus اور RobotCommand انٹرفیس بنائے ہیں۔ اب میں ایک تیسرا انٹرفیس شامل کرنا چاہتا ہوں: 'SensorReading' جو کیپچر کرے: ٹائم سٹیمپ، sensor_id (int)، reading_type (string جیسے 'temperature', 'pressure')، value (float64)، اور confidence (0.0-1.0)۔ مجھے .msg فائل دکھائیں اور ڈیزائن کی وضاحت کریں۔"

**متوقع نتیجہ:** AI فراہم کرے گا:
- مناسب ٹائپس کے ساتھ مکمل .msg فائل
- ہر فیلڈ کی وضاحت کرنے والے تبصرے
- منتخب کردہ ٹائپس کے لیے منطق

**ڈیزائن کو چیلنج کریں:**

> "اچھا، لیکن کیا کانفیڈنس کو 0-1 کے درمیان فلوٹ ہونا چاہیے، یا کیا مجھے RELIABLE, MODERATE, UNRELIABLE جیسے اینم (enum) استعمال کرنا چاہیے؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- فلوٹ: زیادہ اظہار خیال، درست مقدار کا تعین
- اینم: آسان کلائنٹس، تشریح کی ضرورت نہیں
- آپ کے استعمال کے کیس کے لیے فائدے اور نقصانات

**تکرار (Iterate):**

> "ٹھیک ہے، میں ایک 'quality' سٹرنگ فیلڈ شامل کرنا چاہتا ہوں جو کانفیڈنس سے زیادہ چیزیں کیپچر کرے—جیسے 'sensor_warm_up'، 'noisy_environment'، وغیرہ۔ اپ ڈیٹ شدہ انٹرفیس دکھائیں۔"

یہ AI کے ذریعے انٹرفیس ڈیزائن پیٹرن سکھانے، آپ کے ذریعے ضروریات کو بہتر بنانے، اور بہترین ڈھانچے پر تعاون کرنے کا عمل ہے۔

---

## مشقیں (Exercises)

1. **ایک `SensorReading` میسج بنائیں** جس میں یہ شامل ہوں: ٹائم سٹیمپ، سینسر_آئی ڈی، ریڈنگ_ٹائپ، ویلیو، کانفیڈنس
2. **ایک `RobotState` سروس بنائیں** جو کوئی ریکویسٹ نہ لے اور موجودہ بیٹری، درجہ حرارت، اور فعال کام واپس کرے۔
3. **دونوں کو بلڈ کریں** اور `ros2 interface show` کے ساتھ تصدیق کریں۔
4. **ایک پبلشر بنائیں** جو ہر 2 سیکنڈ میں سینسر ریڈنگز بھیجے۔
5. **ایک کلائنٹ بنائیں** جو ہر 1 سیکنڈ میں روبوٹ کی حالت کے بارے میں پوچھ گچھ کرے اور نتائج کو لاگ کرے۔

---

## غور و فکر (Reflection)

اگلے سبق سے پہلے، ان پر غور کریں:

- انٹرفیس پیکیجز ایگزیکیوٹیبل پیکیجز سے الگ کیوں ہوتے ہیں؟
- آپ `float64` کے بجائے `builtin_interfaces/Time` کا استعمال کب کریں گے؟
- آپ میسجز کو اس طرح کیسے ڈیزائن کرتے ہیں کہ وہ توسیع پذیر ہوں (بعد میں نئے فیلڈز شامل کیے جا سکیں)?

اگلے سبق میں، ہم سب کچھ ایک ساتھ لائیں گے: ٹاپکس، سروسز، اور کسٹم انٹرفیسز۔ آپ پیچیدہ ملٹی-نوڈ سسٹمز کے لیے ڈیزائن پیٹرن سیکھیں گے۔