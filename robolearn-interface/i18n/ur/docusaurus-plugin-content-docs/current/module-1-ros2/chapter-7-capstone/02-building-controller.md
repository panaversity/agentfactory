---
id: lesson-7-2-building-controller
title: 'Lesson 7.2: Building the Controller'
sidebar_position: 2
sidebar_label: 7.2 Building the Controller
description: Implement multi-node system from specification using skills from Chapters 4-6.
duration_minutes: 90
proficiency_level: B1
layer: L4
hardware_tier: 1
learning_objectives:
  - Compose multi-node systems from existing patterns
  - Create custom messages and service definitions
  - Implement stateful service servers
  - Coordinate multiple publishers in a launch system
  - Debug integration issues systematically
---


# سبق 7.2: کنٹرولر بنانا

## اب نفاذ کریں (Implement Now)

آپ کے پاس اپنی تفصیلات (specification) موجود ہے۔ یہ واضح، غیر مبہم، اور جانچ کے قابل ہے۔ اب آپ اس پر عمل درآمد (implement) کرتے ہیں۔

یہ اچھی خبر ہے: **آپ نے ہر حصہ پہلے ہی بنا لیا ہے۔** یہ نئے تصورات نہیں ہیں۔ یہ ترکیب (composition) ہے—یعنی جو آپ نے ابواب 4 سے 6 میں سیکھا، اسے ایک مکمل نظام میں جوڑنا۔

- **باب 4**: پبلشر/سبسکرائبر نوڈز ✅
- **باب 5**: سروس سرورز اور کسٹم پیغامات ✅
- **باب 6**: پیرامیٹرز اور لانچ فائلز ✅

آپ موجودہ مہارتوں کو ایک نئے نظام میں منظم کر رہے ہیں۔ یہی پیشہ ور روبوٹکس ڈویلپمنٹ ہے۔

---

## نفاذ کے مراحل (Implementation Phases)

### مرحلہ 1: پیکیج اور انٹرفیس بنائیں

**آپ کا کام**: پیکیج کا ڈھانچہ بنائیں اور کسٹم پیغامات کی تعریف کریں۔

**باب 5** سے یاد رکھیں:
- کسٹم پیغامات `robot_controller/msg/TurtleStatus.msg` میں جائیں گے
- سروسز `robot_controller/srv/NavigateTo.srv` میں جائیں گی

```bash
# ورک اسپیس اور پیکیج بنائیں
mkdir -p turtle_ws/src
cd turtle_ws/src
ros2 pkg create robot_controller --build-type ament_cmake

# میسج اور سروس ڈائریکٹریز بنائیں
mkdir -p robot_controller/msg robot_controller/srv
```

**TurtleStatus.msg** (سٹیٹس پبلش کرنے کے لیے کسٹم میسج):
```
float32 x
float32 y
float32 theta
float32 vel_x
float32 vel_y
float32 battery
```

**NavigateTo.srv** (نیویگیشن کی درخواستوں کے لیے سروس):
```
float32 x
float32 y
float32 theta
---
bool success
float32 time_taken
```

پیغامات کو مرتب (compile) کرنے کے لیے `CMakeLists.txt` کو اپ ڈیٹ کریں:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleStatus.msg"
  "srv/NavigateTo.srv"
)

ament_package()
```

### مرحلہ 2: نیویگیٹر نوڈ کا نفاذ کریں

**آپ کا کام**: ایک پائتھن نوڈ بنائیں جو سروس کے ذریعے اہداف قبول کرے اور ٹرٹل کو کمانڈ دے۔

**باب 4 اور 5** سے یاد رکھیں:
- نوڈز `rclpy` سے `Node` بیس کلاس استعمال کرتے ہیں
- سروسز `Service` آبجیکٹ استعمال کرتی ہیں
- پبلشر پیغامات بھیجتے ہیں

**robot_controller/navigator_node.py**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from your_package.srv import NavigateTo  # صحیح پیکیج کا نام استعمال کریں
import math
import time

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')

        # ٹرٹل کو کنٹرول کرنے کے لیے پبلشر
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # نیویگیشن کے اہداف کے لیے سروس سرور
        self.navigate_srv = self.create_service(
            NavigateTo,
            '/navigate_to',
            self.navigate_callback
        )

        # موجودہ حالت کو ٹریک کرنا
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.get_logger().info('Navigator node started')

    def navigate_callback(self, request, response):
        """نیویگیشن کے اہداف کی درخواستوں کو ہینڈل کریں"""
        self.get_logger().info(
            f'هدف موصول ہوا: x={request.x}, y={request.y}, theta={request.theta}'
        )

        start_time = time.time()

        # ہدف تک فاصلہ حساب کریں (سادہ بنایا گیا)
        dx = request.x - self.current_x
        dy = request.y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # ہدف کی طرف بڑھیں (سادہ لکیری حرکت)
        # حقیقی نفاذ میں، یہ زیادہ نفیس ہوگا
        msg = Twist()
        msg.linear.x = 0.2  # 0.2 m/s پر آگے بڑھیں

        # فاصلہ/رفتار سیکنڈز کے لیے حرکت کریں (تقریباً)
        move_time = distance / 0.2 if distance > 0 else 0.0
        end_time = time.time() + move_time

        while time.time() < end_time:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)

        # روکیں
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)

        # حالت کو اپ ڈیٹ کریں (حقیقی نظام میں، یہ اوڈومیٹری سے آئے گا)
        self.current_x = request.x
        self.current_y = request.y
        self.current_theta = request.theta

        # جواب
        elapsed = time.time() - start_time
        response.success = distance < 0.5  # 0.5m کے اندر "کامیابی" ہے
        response.time_taken = elapsed

        self.get_logger().info(f'هدف {elapsed:.2f}s میں مکمل ہوا')
        return response

def main(args=None):
    rclpy.init(args=args)
    navigator = NavigatorNode()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**اہم نکات**:
- سروس کال بیک ہدف کی درخواستوں کو ہینڈل کرتا ہے
- پبلشر ویلوسٹی کمانڈز بھیجتا ہے (باب 4 سے)
- جواب کامیابی اور `time_taken` واپس کرتا ہے (باب 5 سے)
- حقیقی نفاذ اوڈومیٹری پڑھے گا؛ یہ سادہ ورژن حالت کو ٹریک کرتا ہے

### مرحلہ 3: سٹیٹس مانیٹر نوڈ کا نفاذ کریں

**آپ کا کام**: ایک نوڈ بنائیں جو روبوٹ کی حالت کو مسلسل پبلش کرے۔

**باب 4** سے پبلشر پیٹرن یاد رکھیں.

**robot_controller/status_monitor_node.py**:
```python
import rclpy
from rclpy.node import Node
from your_package.msg import TurtleStatus  # کسٹم میسج
import random

class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor')

        # سٹیٹس کے لیے پبلشر
        self.status_pub = self.create_publisher(
            TurtleStatus,
            '/robot/status',
            10
        )

        # ہر 100ms (10 Hz) پر پبلش کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.1, self.publish_status)

        # نقلی حالت (حقیقی نظام میں، یہ اوڈومیٹری سے آئے گا)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.battery = 100.0

        self.get_logger().info('Status Monitor started')

    def publish_status(self):
        """ہر 100ms پر سٹیٹس میسج پبلش کریں"""
        msg = TurtleStatus()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.vel_x = self.vel_x
        msg.vel_y = self.vel_y
        msg.battery = self.battery

        self.status_pub.publish(msg)
        self.get_logger().debug(f'سٹیٹس: ({self.x:.2f}, {self.y:.2f})')

        # بیٹری کے ختم ہونے کی نقل کریں
        self.battery = max(0.0, self.battery - 0.1)

def main(args=None):
    rclpy.init(args=args)
    monitor = StatusMonitorNode()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**اہم نکات**:
- باب 5 سے کسٹم میسج `TurtleStatus`
- ٹائمر 10 Hz پبلیکیشن کو یقینی بناتا ہے (باب 6 کے پیٹرن سے ہر 100ms)
- حالت کا انتظام (پوزیشن، ویلوسٹی، بیٹری کو ٹریک کرتا ہے)

### مرحلہ 4: رکاوٹ ڈیٹیکٹر نوڈ کا نفاذ کریں

**آپ کا کام**: ایک نوڈ بنائیں جو رکاوٹ کا پتہ لگانے کی نقل کرے۔

**robot_controller/obstacle_detector_node.py**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # رکاوٹ ڈیٹا کے لیے پبلشر (موک سینسر)
        self.obstacle_pub = self.create_publisher(
            LaserScan,
            '/obstacles',
            10
        )

        # ہر 200ms (5 Hz) پر پبلش کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.2, self.publish_obstacles)

        self.scan_counter = 0
        self.get_logger().info('Obstacle Detector started')

    def publish_obstacles(self):
        """موک رکاوٹ اسکین پبلش کریں"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = -1.5708  # -90 ڈگری
        msg.angle_max = 1.5708   # +90 ڈگری
        msg.angle_increment = 0.0174  # ~1 ڈگری
        msg.time_increment = 0.0
        msg.range_min = 0.0
        msg.range_max = 10.0

        # کچھ "پتہ لگانے" کی نقل کریں
        ranges = []
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        for i in range(num_readings):
            # رینڈم فاصلے (زیادہ تر کچھ نہ پانے کی نقل کریں،
            # کبھی کبھار رکاوٹیں)
            if random.random() > 0.95:  # رکاوٹ کا 5% موقع
                ranges.append(0.5)  # 0.5m پر رکاوٹ
            else:
                ranges.append(10.0)  # کوئی رکاوٹ نہیں (زیادہ سے زیادہ رینج)

        msg.ranges = ranges
        self.obstacle_pub.publish(msg)

        self.scan_counter += 1
        if self.scan_counter % 5 == 0:  # ہر 5 اسکین پر لاگ کریں
            self.get_logger().debug(f'اسکین #{self.scan_counter} پبلش ہوا')

def main(args=None):
    rclpy.init(args=args)
    detector = ObstacleDetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**اہم نکات**:
- معیاری ROS 2 میسج `LaserScan` استعمال کرتا ہے (باب 5 کے علم سے)
- 5 Hz پر پبلش کرتا ہے (باب 6 سے 200ms وقفے)
- سسٹم ٹیسٹنگ کے لیے کبھی کبھار رکاوٹوں کی نقل کرتا ہے (5% موقع)

### مرحلہ 5: لانچ فائل بنائیں

**آپ کا کام**: پیرامیٹرز کے ساتھ تمام نوڈز شروع کرنے کے لیے ایک لانچ فائل بنائیں۔

**باب 6** سے لانچ فائل کا ڈھانچہ یاد رکھیں۔

**robot_controller/launch/start_system.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # نیویگیٹر نوڈ
        Node(
            package='robot_controller',
            executable='navigator',
            name='navigator',
            output='screen',
            parameters=[
                {'goal_timeout': 5.0},
            ]
        ),

        # سٹیٹس مانیٹر نوڈ
        Node(
            package='robot_controller',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
            parameters=[
                {'publish_rate': 10},
            ]
        ),

        # رکاوٹ ڈیٹیکٹر نوڈ
        Node(
            package='robot_controller',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[
                {'scan_rate': 5},
            ]
        ),
    ])
```

**setup.py** انٹری پوائنٹس (تاکہ `ros2 launch` ایگزیکیوٹیبلز تلاش کر سکے):
```python
entry_points={
    'console_scripts': [
        'navigator=robot_controller.navigator_node:main',
        'status_monitor=robot_controller.status_monitor_node:main',
        'obstacle_detector=robot_controller.obstacle_detector_node:main',
    ],
},
```

---

## پیکیج بنانا اور نوڈز کا تجربہ کرنا (Building and Testing Nodes)

### پیکیج بنائیں

```bash
cd ~/turtle_ws
colcon build --packages-select robot_controller
source install/setup.bash
```

### انفرادی نوڈز کا تجربہ کریں

**ٹرمینل 1: turtlesim شروع کریں**
```bash
ros2 run turtlesim turtlesim_node
```

**ٹرمینل 2: لانچ فائل شروع کریں**
```bash
cd ~/turtle_ws
ros2 launch robot_controller start_system.launch.py
```

**ٹرمینل 3: نیویگیشن سروس کو کال کریں**
```bash
ros2 service call /navigate_to robot_controller/srv/NavigateTo "{x: 5.0, y: 5.0, theta: 0.0}"
```

**ٹرمینل 4: سٹیٹس ٹاپک کی نگرانی کریں**
```bash
ros2 topic echo /robot/status
```

**ٹرمینل 5: رکاوٹوں کی نگرانی کریں**
```bash
ros2 topic echo /obstacles
```

---

## عام انضمام کے مسائل اور ڈیبگنگ (Common Integration Issues & Debugging)

### مسئلہ 1: "کسٹم میسج نہیں مل رہا"
**علامت**: امپورٹ کی خرابی `robot_controller.srv.NavigateTo not found`
**حل**:
1. دوبارہ بنائیں: `colcon build --packages-select robot_controller`
2. سیٹ اپ سورس کریں: `source install/setup.bash`
3. CMakeLists.txt میں `rosidl_generate_interfaces` شامل ہے یا نہیں، چیک کریں

### مسئلہ 2: "سروس کال کا وقت ختم ہو جاتا ہے" (Service call times out)
**علامت**: `ros2 service call` ہینگ ہو جاتا ہے
**حل**:
1. چیک کریں کہ نوڈ چل رہا ہے: `ros2 node list`
2. چیک کریں کہ سروس موجود ہے: `ros2 service list`
3. سروس کال بیک میں لاگنگ شامل کریں تاکہ معلوم ہو کہ اسے کال کیا جا رہا ہے یا نہیں
4. ٹائم آؤٹ بڑھائیں: `ros2 service call /navigate_to ... --timeout 10.0`

### مسئلہ 3: "ٹاپک پبلش نہیں ہو رہا"
**علامت**: `ros2 topic echo /robot/status` کوئی آؤٹ پٹ نہیں دکھاتا
**حل**:
1. چیک کریں کہ پبلشر نوڈ چل رہا ہے: `ros2 node list`
2. ٹاپک کا نام چیک کریں: `ros2 topic list`
3. چیک کریں کہ فریکوئنسی بہت سست تو نہیں ہے (شاید پبلشر چل رہا ہے لیکن ڈیٹا پرانا ہے)
4. پبلش کال بیک میں لاگنگ شامل کریں

### مسئلہ 4: "ٹرٹل حرکت نہیں کرتا"
**علامت**: سروس کامیابی واپس کرتی ہے لیکن ٹرٹل ساکن رہتا ہے
**حل**:
1. تصدیق کریں کہ `turtlesim_node` چل رہا ہے اور `/turtle1/cmd_vel` پر سن رہا ہے
2. دستی طور پر ٹیسٹ کریں: `ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"`
3. ویلوسٹی کی مقدار چیک کریں (ٹیسٹنگ کے لیے 0.2 m/s معیاری ہے)
4. ویلوسٹی کمانڈز کے درمیان تاخیر شامل کریں

---

## تخلیق بمقابلہ ترکیب (Composition vs Creation)

نوٹ کریں: **آپ نے نئے تصورات تخلیق نہیں کیے۔** آپ نے موجودہ تصورات کو ترکیب کیا:

| باب سے | تصور | استعمال ہوا |
|--------------|---------|---------|
| 4 | پبلشر/سبسکرائبر | نیویگیٹر (pub)، سٹیٹس مانیٹر (pub)، رکاوٹ ڈیٹیکٹر (pub) |
| 5 | سروس پیٹرن | نیویگیٹر سروس سرور (/navigate_to) |
| 5 | کسٹم پیغامات | TurtleStatus میسج |
| 6 | لانچ فائلز | start_system.launch.py جس میں تمام 3 نوڈز شامل ہیں |
| 6 | پیرامیٹرز | goal_timeout, publish_rate, scan_rate |

یہ **لایه 3 انٹیلی جنس ڈیزائن** ہے: آپ دوبارہ استعمال کے قابل اجزاء لے رہے ہیں اور انہیں جوڑ رہے ہیں۔

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: نفاذ کے دوران مسائل کو ڈیبگ کرنے میں مدد کے لیے ایک چیٹ AI استعمال کریں۔

**ڈیبگنگ پرامپٹ**:
```
میں ایک ملٹی-نوڈ ROS 2 سسٹم بنا رہا ہوں۔ یہاں میری تفصیلات ہیں:
[Lesson 7.1 سے اپنی تفصیلات پیسٹ کریں]

جب میں نوڈز چلاتا ہوں تو مجھے یہ خرابی ملتی ہے:
[خرابی کا پیغام پیسٹ کریں]

کیا غلط ہے؟ میں اسے کیسے ٹھیک کروں؟
```

**کوڈ جائزہ پرامپٹ**:
```
یہ میرا navigator_node.py ہے:
[کوڈ پیسٹ کریں]

کیا یہ درست طریقے سے نافذ کرتا ہے:
1. میری تفصیلات سے /navigate_to سروس؟
2. ٹرٹل کو ویلوسٹی کمانڈز پبلش کرنا؟
3. جواب میں کامیابی اور time_taken واپس کرنا؟

کوئی کیڑے یا بہتری؟
```

**متوقع نتیجہ**:
- تمام 3 نوڈز بغیر کسی خرابی کے مرتب ہوتے ہیں
- لانچ فائل تمام نوڈز کو بیک وقت شروع کرتی ہے
- سروسز اور ٹاپکس `ros2 service list` اور `ros2 topic list` کے ساتھ نظر آتے ہیں
- آپ نیویگیشن سروس کو کال کر سکتے ہیں اور ٹرٹل کو رد عمل دیتے ہوئے دیکھ سکتے ہیں

**حفاظتی نوٹ**: آپ کا AI ایسی خصوصیات تجویز کر سکتا ہے جو تفصیلات سے باہر ہوں (حقیقی راستہ منصوبہ بندی، رکاوٹ سے بچنا، کائنےمیٹک حدود)۔ اپنے سبق 7.1 کے غیر اہداف کو یاد رکھیں۔ اسے سادہ رکھیں—باب 4-6 کے بنیادی pub/sub/service پیٹرن پر توجہ دیں۔

---

**اگلا قدم**: [سبق 7.3: جانچ اور توثیق →](./03-testing-validation.md)