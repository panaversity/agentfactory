---
id: lesson-6-2-launch-files
title: 'Lesson 6.2: Launch Files (Multi-Node Startup)'
sidebar_position: 2
sidebar_label: 6.2 Launch Files
description: >-
  Learn how to start multiple ROS 2 nodes together and pass parameters using
  Python launch files.
duration_minutes: 60
proficiency_level: B1
layer: L3
hardware_tier: 1
learning_objectives:
  - Write Python launch files that start multiple nodes
  - Pass parameters to nodes via launch files
  - Use YAML configuration files for complex systems
  - Implement launch arguments for flexible deployment
  - Understand launch file execution order and dependencies
---


# سبق 6.2: لانچ فائلز — ایک سے زیادہ نوڈز والے سسٹم شروع کرنا

تصور کریں کہ آپ نے 10 نوڈز کے ساتھ ایک روبوٹ سسٹم بنایا ہے:
- 3 سینسر پبلشرز (کیمرہ، LIDAR، IMU)
- 2 کنٹرولرز (موشن، آرم)
- 3 مانیٹرز (بیٹری، تھرمل، ڈائیگناسٹکس)
- 2 سیفٹی نوڈز (ایمرجنسی سٹاپ، ٹکراؤ سے بچاؤ)

ہر نوڈ کو الگ الگ ٹرمینلز میں انفرادی طور پر لانچ کرنا افراتفری کا باعث بنے گا:
```bash
# ٹرمینل 1
ros2 run sensor_pkg camera_node --ros-args --log-level debug

# ٹرمینل 2
ros2 run sensor_pkg lidar_node --ros-args --log-level debug

# ٹرمینل 3
ros2 run sensor_pkg imu_node

# ... وغیرہ
```

**لانچ فائلز** اس مسئلے کو حل کرتی ہیں۔ ایک واحد لانچ فائل ایک ہی کمانڈ سے تمام نوڈز شروع کر سکتی ہے، ہر نوڈ کو پیرامیٹرز پاس کر سکتی ہے، اور ابتدائی سیٹ اپ کو مربوط (coordinate) کر سکتی ہے۔

## لانچ فائل کیا ہے؟

ROS 2 لانچ فائل ایک پائپھن اسکرپٹ ہے جو:
1. شروع کیے جانے والے نوڈز کا ایک سیٹ متعین (define) کرتی ہے
2. ہر نوڈ کے لیے پیرامیٹرز کی وضاحت کرتی ہے
3. اختیاری طور پر ٹاپک کے ناموں کو دوبارہ نقشہ (remap) کرتی ہے
4. نوڈ کے آغاز اور بند ہونے کو مربوط کرتی ہے

لانچ فائلیں سسٹم کو اعلانیہ (declaratively) بیان کرنے کے لیے `launch` اور `launch_ros` پیکجز استعمال کرتی ہیں۔

## بنیادی لانچ فائل ڈھانچہ

### کم سے کم مثال: دو نوڈز

ایک فائل بنائیں: `my_robot_pkg/launch/simple_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # نوڈ 1: پبلشر
        Node(
            package='my_first_package',
            executable='minimal_publisher',
            name='sensor_publisher',
            output='screen',
        ),
        # نوڈ 2: سبسکرائبر
        Node(
            package='my_first_package',
            executable='minimal_subscriber',
            name='data_processor',
            output='screen',
        ),
    ])
```

**یہ کیا کرتا ہے:**
- `package='my_first_package'`: کون سا ROS 2 پیکج ایگزیکیوٹیبل رکھتا ہے
- `executable='minimal_publisher'`: داخلی نقطہ کا نام (پیکج کے setup.py سے)
- `name='sensor_publisher'`: جب یہ چلتا ہے تو نوڈ کا نام (ایگزیکیوٹیبل نام سے مختلف ہو سکتا ہے)
- `output='screen'`: لاگز کو ٹرمینل پر پرنٹ کریں (خاموش موڈ کے بجائے)

**اسے چلائیں:**
```bash
ros2 launch my_first_package simple_system.launch.py
```

**متوقع آؤٹ پٹ:**
```
[minimal_publisher-1] [INFO] [minimal_publisher]: Starting publisher
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[minimal_publisher-1] [INFO] [minimal_publisher]: Publishing: Hello World: 2
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 2"
```

دونوں نوڈز ایک ساتھ شروع ہوتے ہیں (ان کے انڈیکس کے ساتھ پیش کیے گئے)، فوری طور پر بات چیت کرتے ہیں، اور دونوں ایک ہی ٹرمینل پر لاگ کرتے ہیں۔ پبلشر شائع کرتا رہتا ہے، سبسکرائبر وصول کرتا رہتا ہے۔

## پیرامیٹرز کے ساتھ لانچ فائلز

لانچ فائلوں کی اصل طاقت سٹارٹ اپ پر نوڈز کو پیرامیٹرز پاس کرنا ہے۔

### مختلف پیرامیٹرز کے ساتھ ملٹی-نوڈ سسٹم

بنائیں: `my_robot_pkg/launch/robot_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # تیز سینسر: اعلی اشاعت کی شرح
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 30.0,  # کیمرے کے لیے 30 Hz
                'robot_name': 'my_robot',
                'sensor_type': 'camera',
            }],
        ),

        # سست سینسر: کم اشاعت کی شرح
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='lidar_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,  # LIDAR کے لیے 10 Hz
                'robot_name': 'my_robot',
                'sensor_type': 'lidar',
            }],
        ),

        # ڈیٹا پروسیسر: دونوں سینسر استعمال کرتا ہے
        Node(
            package='my_first_package',
            executable='data_processor',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'timeout': 5.0,
            }],
        ),
    ])
```

**اہم مشاہدات:**
- ایک ہی ایگزیکیوٹیبل (`configurable_node`) دو بار چلتا ہے جس کے نام اور پیرامیٹرز مختلف ہوتے ہیں
- ہر نوڈ انسٹنس کا اپنا پیرامیٹر سیٹ ہوتا ہے
- پیرامیٹرز کو `parameters` آرگومنٹ میں ڈکشنریوں کی فہرست کے طور پر پاس کیا جاتا ہے

**اسے چلائیں:**
```bash
ros2 launch my_first_package robot_system.launch.py
```

**پیرامیٹر تفویض کی تصدیق کریں:**
```bash
ros2 param list /camera_publisher
# آؤٹ پٹ: /camera_publisher:
#   publish_rate
#   robot_name
#   sensor_type

ros2 param get /camera_publisher publish_rate
# آؤٹ پٹ: publish_rate: 30.0

ros2 param get /lidar_publisher publish_rate
# آؤٹ پٹ: publish_rate: 10.0
```

## کنفیگریشن فائلوں (YAML) کے ساتھ لانچ فائلز

پیچیدہ سسٹم کے لیے، لانچ فائل میں پیرامیٹرز کو ہارڈ کوڈ کرنا گڑبڑ ہے۔ اس کے بجائے، YAML کنفیگریشن فائلوں کا استعمال کریں۔

### YAML کنفیگریشن فائل

بنائیں: `my_robot_pkg/config/robot_params.yaml`

```yaml
# روبوٹ کے وسیع کنفیگریشن
robot_name: "physical_ai_bot"
robot_id: 001

# کیمرہ نوڈ کنفیگریشن
camera_publisher:
  ros__parameters:
    publish_rate: 30.0
    resolution: "1920x1080"
    encoding: "RGB8"
    sensor_type: "camera"

# LIDAR نوڈ کنفیگریشن
lidar_publisher:
  ros__parameters:
    publish_rate: 10.0
    angle_range: 360.0
    max_distance: 50.0
    sensor_type: "lidar"

# سینسر فیوژن کنفیگریشن
sensor_fusion:
  ros__parameters:
    timeout: 5.0
    fusion_mode: "weighted"
```

### YAML استعمال کرنے والی لانچ فائل

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # کانفیگ فائل کا پتہ لگائیں
    config_dir = os.path.join(
        get_package_share_directory('my_first_package'),
        'config'
    )
    config_file = os.path.join(config_dir, 'robot_params.yaml')

    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='camera_publisher',
            output='screen',
            parameters=[config_file],  # YAML سے لوڈ کریں
        ),
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='lidar_publisher',
            output='screen',
            parameters=[config_file],  # YAML سے لوڈ کریں
        ),
        Node(
            package='my_first_package',
            executable='data_processor',
            name='sensor_fusion',
            output='screen',
            parameters=[config_file],  # YAML سے لوڈ کریں
        ),
    ])
```

**فائدہ:** لانچ فائل میں ترمیم کیے بغیر پیرامیٹرز تبدیل کریں۔ بس YAML میں ترمیم کریں اور دوبارہ لانچ کریں۔

## لانچ فائلوں کو دلائل (Arguments) کے ساتھ لچکدار بنانا

کیا ہوگا اگر آپ چاہتے ہیں کہ ایک ہی لانچ فائل مختلف طریقوں (سیمولیشن بمقابلہ حقیقت، ڈیبگ بمقابلہ پروڈکشن) میں کام کرے؟

### دلائل کے ساتھ لانچ فائل

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # لانچ دلائل کا اعلان کریں
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='روبوٹ کا نام'
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='سیمولیشن میں چلائیں (true) یا حقیقی (false)'
    )

    return LaunchDescription([
        robot_name_arg,
        sim_mode_arg,

        Node(
            package='my_first_package',
            executable='robot_controller',
            name='controller',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'simulation_mode': LaunchConfiguration('sim_mode'),
                'safety_enabled': True,
            }],
        ),
    ])
```

**ڈیفالٹ دلائل کے ساتھ چلائیں:**
```bash
ros2 launch my_first_package robot_system.launch.py
# استعمال کرتا ہے: robot_name=default_robot, sim_mode=true
```

**کسٹم دلائل کے ساتھ چلائیں:**
```bash
ros2 launch my_first_package robot_system.launch.py robot_name:=my_bot sim_mode:=false
# استعمال کرتا ہے: robot_name=my_bot, sim_mode=false
```

## لانچ فائلوں کے لیے اپنا پیکج تیار کرنا

لانچ فائلوں کے کام کرنے کے لیے، آپ کو ROS 2 کو یہ بتانا ہوگا کہ انہیں کہاں تلاش کرنا ہے۔ یہ `setup.py` میں کیا جاتا ہے۔

### کم سے کم setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # معیاری ROS 2 فائلیں
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # لانچ فائلوں کو شامل کریں
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # کانفیگ فائلوں کو شامل کریں
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = my_first_package.minimal_publisher:main',
            'minimal_subscriber = my_first_package.minimal_subscriber:main',
            'configurable_node = my_first_package.configurable_node:main',
            'data_processor = my_first_package.data_processor:main',
        ],
    },
)
```

**اہم حصے:**
- `data_files` میں `glob` پیٹرن ROS 2 کو `launch/` ڈائریکٹری سے تمام `.launch.py` فائلوں کو شامل کرنے کا کہتے ہیں
- کانفیگ فائلوں کے لیے بھی یہی `config/` ڈائریکٹری میں

**لانچ فائلیں شامل کرنے کے بعد:**
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_package
source install/setup.bash
```

## گائیڈڈ پریکٹس: مکمل ملٹی-نوڈ سسٹم

آئیے ایک چھوٹا روبوٹ سسٹم لانچ کنفیگریشن بناتے ہیں جس میں:
- 2 سینسر پبلشرز (مختلف شرحوں پر)
- 1 ڈیٹا پروسیسر
- قابل ترتیب پیرامیٹرز

### مرحلہ 1: پیکج ڈھانچہ بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_demo
cd robot_demo
mkdir -p launch config
```

### مرحلہ 2: نوڈز بنائیں

`robot_demo/robot_demo/` میں تین سادہ پائپھن فائلیں بنائیں:

**sensor_node.py** (جعلی سینسر ڈیٹا شائع کرتا ہے):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('sensor_name', 'sensor1')

        rate = self.get_parameter('publish_rate').value
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 42.0
        sensor_name = self.get_parameter('sensor_name').value
        self.get_logger().info(f'{sensor_name} published: {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SensorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**processor_node.py** (سینسر ڈیٹا کو سبسکرائب کرتا ہے):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        self.subscription = self.create_subscription(Float32, 'sensor_data', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Processed: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProcessorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### مرحلہ 3: لانچ فائل بنائیں

`robot_demo/launch/robot_demo.launch.py` بنائیں:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_demo',
            executable='sensor_node',
            name='temperature_sensor',
            output='screen',
            parameters=[{
                'publish_rate': 5.0,
                'sensor_name': 'Temperature',
            }],
        ),
        Node(
            package='robot_demo',
            executable='sensor_node',
            name='humidity_sensor',
            output='screen',
            parameters=[{
                'publish_rate': 2.0,
                'sensor_name': 'Humidity',
            }],
        ),
        Node(
            package='robot_demo',
            executable='processor_node',
            name='data_processor',
            output='screen',
        ),
    ])
```

### مرحلہ 4: بنائیں اور چلائیں

```bash
cd ~/ros2_ws
colcon build --packages-select robot_demo
source install/setup.bash
ros2 launch robot_demo robot_demo.launch.py
```

**متوقع آؤٹ پٹ:**
```
[sensor_node-1] [INFO] [temperature_sensor]: Temperature published: 42.0
[sensor_node-2] [INFO] [humidity_sensor]: Humidity published: 42.0
[processor_node-1] [INFO] [processor_node]: Processed: 42.0
[processor_node-1] [INFO] [processor_node]: Processed: 42.0
```

## آزادانہ مشق

### مشق 1: پیرامیٹر کنفیگریشن

ایک `config/sensors.yaml` فائل بنائیں جس میں سینسر کے مخصوص پیرامیٹرز ہوں، اور لانچ فائل کو اسے لوڈ کرنے کے لیے تبدیل کریں۔

### مشق 2: دلائل کے ساتھ لانچ

لانچ فائل کو تبدیل کریں تاکہ وہ ایک `robot_mode` دلیل (سیمولیشن/حقیقی) قبول کرے اور اسے نوڈز پر پاس کرے۔

### مشق 3: پیچیدہ سسٹم

ایک ایسا سسٹم بنائیں جس میں:
- 3 سینسر نوڈز (مختلف شرحوں پر)
- 2 پروسیسر (ہر ایک مختلف سینسر سے سبسکرائب کرتا ہے)
- 1 ایگریگیٹر (دونوں پروسیسر سے سبسکرائب کرتا ہے)

ہر نوڈ کے کردار کو ایک تبصرہ (comment) میں دستاویز کریں۔

## AI کے ساتھ کوشش کریں

**سیٹ اپ:** اپنے AI کے ساتھ ملٹی-نوڈ لانچ سسٹم کو ڈیزائن اور ڈیبگ کرنے پر کام کریں۔

**پرامپٹ سیٹ:**

```
Prompt 1: "میرے پاس 5 نوڈز ہیں جنہیں ایک ساتھ شروع کرنے کی ضرورت ہے، لیکن وہ ایک دوسرے پر منحصر ہیں (نوڈ A کو نوڈ B سے پہلے شروع ہونا چاہیے)۔ میں لانچ فائل میں اسے کیسے ترتیب دوں؟ کیا میں آغاز کے ترتیب (startup order) کی وضاحت کر سکتا ہوں؟"

Prompt 2: "میں ایک ہی نوڈ ایگزیکیوٹیبل کو مختلف پیرامیٹرز کے ساتھ کئی بار چلانا چاہتا ہوں۔ لانچ فائل کو ڈپلیکیشن سے بچانے کے لیے سیٹ اپ کرنے کا بہترین طریقہ کیا ہے؟"

Prompt 3: "جب میں اپنا سسٹم لانچ کرتا ہوں، تو ایک نوڈ شروع ہونے میں ناکام ہو جاتا ہے۔ میں اس کی ڈیبگنگ کیسے کروں گا؟ کون سے ٹولز یہ شناخت کرنے میں مدد کرتے ہیں کہ کون سا نوڈ کریش ہوا اور کیوں؟"
```

**متوقع نتائج:**
- AI لانچ فائل کے عمل درآمد کے ترتیب (execution order) کی وضاحت کرتا ہے (عام طور پر متوازی، لیکن انحصار کی وضاحت کی جا سکتی ہے)
- AI ڈپلیکیشن کو کم کرنے کے لیے پیرامیٹر ٹیمپلیٹنگ پیٹرن دکھاتا ہے
- AI ڈیبگنگ ورک فلو دکھاتا ہے (ros2 launch ور بوسیٹی، ros2 node list، لاگز)

**اختیاری توسیع:**
ایک لانچ فائل بنائیں جو:
- 3+ نوڈز کے ساتھ ایک سسٹم شروع کرے
- 3+ قابل ترتیب پیرامیٹرز رکھے
- طریقوں کے درمیان سوئچ کرنے کے لیے کمانڈ لائن دلائل قبول کرے
- ڈیبگنگ کے لیے لاگنگ کنفیگریشن شامل کرے

**حفاظتی نوٹ:** روبوٹ سسٹم لانچ کرتے وقت، جسمانی ہارڈ ویئر چلانے سے پہلے تصدیق کریں کہ تمام نوڈز کامیابی سے شروع ہو گئے ہیں اور بات چیت کر رہے ہیں۔ کنیکٹیویٹی کی تصدیق کے لیے `ros2 node list` اور `rqt_graph` استعمال کریں۔