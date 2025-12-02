---
id: lesson-12-2-spawning-robots
title: 'Lesson 12.2: Spawning Robots from ROS 2'
sidebar_position: 2
sidebar_label: 12.2 Spawning Robots
description: Spawning robot models into Gazebo from ROS 2 launch files
duration_minutes: 60
proficiency_level: B1
layer: L2
hardware_tier: 1
learning_objectives:
  - Use ros_gz spawn service to add robots to simulation
  - Convert URDF to SDF for Gazebo spawning
  - Integrate robot spawning into ROS 2 launch files
  - Manage robot_description parameter and joint state publishing
skills:
  - ros2-gazebo-bridge
  - launch-files
cognitive_load:
  new_concepts: 7
tier_1_path: TheConstruct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 12.2: ROS 2 سے روبوٹس کو اسپان کرنا (Spawning Robots from ROS 2)

## جامد بمقابلہ متحرک مسئلہ (The Static vs Dynamic Problem)

باب 10 میں، آپ نے GUI کا استعمال کرتے ہوئے پہلے سے رکھے گئے روبوٹس کے ساتھ Gazebo دنیاؤں کو ڈیزائن کیا۔ یہ سنگل روبوٹ منظرناموں کے لیے کام کرتا ہے، لیکن جیسے جیسے پیچیدگی بڑھتی ہے، یہ مسائل پیدا کرتا ہے:

- **مختلف روبوٹس کی جانچ**: ہر روبوٹ ویرینٹ کے لیے مختلف دنیا کی فائل لانچ کریں۔
- **کثیر روبوٹ منظرنامے**: دنیا کی فائل میں دستی طور پر متعدد کاپیاں کا انتظام کریں۔
- **خودکار جانچ**: بیچ ٹیسٹنگ کے لیے آسانی سے 10 روبوٹس کو شروع نہیں کیا جا سکتا۔
- **تقسیم**: دنیا کی فائلوں میں روبوٹ کی تفصیلات شامل ہوتی ہیں، جس سے وہ بڑی اور جڑی ہوئی ہو جاتی ہیں۔

**بہتر طریقہ**: روبوٹ کو ایک بار (URDF میں) بیان کریں، پھر اسے ROS 2 سروس کے ذریعے متحرک طور پر اسپان کریں۔ یہ خدشات کو الگ کرتا ہے: دنیایں ماحول کو بیان کرتی ہیں، روبوٹس کی ضرورت پڑنے پر اسپان ہوتے ہیں۔

`ros_gz spawn_entity` سروس متحرک اسپاننگ کے لیے آپ کا آلہ ہے۔

---

## متحرک اسپاننگ کیسے کام کرتی ہے

جب آپ ROS 2 کے ذریعے روبوٹ کو اسپان کرتے ہیں:

```
ROS 2 لانچ فائل
    ↓
پیرامیٹر سے URDF لوڈ کریں
    ↓
(اگر ضرورت ہو تو) URDF کو SDF میں تبدیل کریں
    ↓
spawn_entity ROS 2 سروس کو کال کریں
    ↓
Gazebo دنیا میں روبوٹ بناتا ہے
    ↓
برج ROS 2 ٹاپکس کو جوڑتا ہے
    ↓
روبوٹ احکامات قبول کرتا ہے اور سینسر ڈیٹا شائع کرتا ہے
```

یہ سب ایک ہی ROS 2 لانچ کمانڈ سے ہوتا ہے۔ کوئی دستی GUI کارروائی نہیں۔

---

## مرحلہ 1: روبوٹ کی تفصیل لوڈ کریں

آپ کا روبوٹ `robot_description` نامی ROS 2 پیرامیٹر کے طور پر دستیاب ہونا چاہیے. اس پیرامیٹر میں روبوٹ کا URDF (یا SDF) ایک سٹرنگ کے طور پر ہوتا ہے۔

### آپشن A: لانچ میں فائل سے لوڈ کریں

**پائیتھن لانچ فائل** (`launch/spawn_robot.launch.py`):
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # robot_description پیکیج اور URDF فائل تلاش کریں
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "urdf",
        "robot.urdf"
    ])

    # URDF فائل پڑھیں اور اسے پیرامیٹر کے طور پر بے نقاب کریں
    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()

    # لانچ کی تفصیل بنائیں
    ld = LaunchDescription()

    # robot_description پیرامیٹر سیٹ کریں (تمام نوڈز کو شائع کیا جائے گا)
    ld.add_action(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_desc
        }]
    ))

    return ld
```

### آپشن B: robot_state_publisher استعمال کریں (تجویز کردہ)

`robot_state_publisher` نوڈ یہ کام آپ کے لیے کرتا ہے:

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                'robot_description': open(
                    PathJoinSubstitution([
                        FindPackageShare("robot_description"),
                        "urdf",
                        "robot.urdf"
                    ])
                ).read()
            }]
        )
    ])
```

**`robot_state_publisher` کیا کرتا ہے**:
1. `robot_description` پیرامیٹر سے URDF پڑھتا ہے
2. `/tf` ٹاپک پر شائع کرتا ہے (تخصیص کے بدلے)
3. `/joint_states` (جوائنٹ پوزیشنز) کو سبسکرائب کرتا ہے
4. RViz اور دیگر ٹولز کو روبوٹ کی ساخت نشر کرتا ہے

---

## مرحلہ 2: اسپان سروس کو کال کریں

ایک بار جب `robot_description` دستیاب ہو جائے، تو اسپان سروس کو کال کریں:

**سروس کا نام**: `/spawn_entity`

**سروس کی تعریف** (`ros_gz_msgs/srv/SpawnEntity.srv`):
```
# درخواست (Request)
string name                  # منفرد روبوٹ کا نام
string xml                   # URDF/SDF بطور سٹرنگ
string world_name            # جس دنیا میں اسپان کرنا ہے
geometry_msgs/Pose initial_pose  # پوزیشن اور اورینٹیشن
---
# جواب (Response)
bool success
string statusMessage
```

**پائیتھن لانچ فائل میں**:
```python
from ros_gz_sim.srv import SpawnEntity
from geometry_msgs.msg import Pose

# سروس کال بنائیں
def spawn_robot():
    client = rclpy.client.Client(SpawnEntity)
    request = SpawnEntity.Request()
    request.name = "my_robot"
    request.xml = robot_desc_string
    request.world_name = "default"
    request.initial_pose = Pose(position=Point(x=0.0, y=0.0, z=1.0))

    future = client.call_async(request)
    rclpy.spin_until_future_complete(client, future)

    if future.result().success:
        print("Robot spawned successfully!")
    else:
        print(f"Spawn failed: {future.result().statusMessage}")
```

---

## مرحلہ 3: لانچ فائل میں ضم کریں

**مکمل لانچ فائل** (`launch/spawn_robot.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # روبوٹ کی تفصیل کا پیکیج حاصل کریں
    robot_pkg = FindPackageShare("robot_description")
    urdf_file = PathJoinSubstitution([robot_pkg, "urdf", "robot.urdf"])

    # URDF مواد پڑھیں
    urdf_file_path = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "robot.urdf"
    )

    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # نوڈ 1: robot_state_publisher (TF فریمز شائع کرتا ہے)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': ''
        }]
    )

    # نوڈ 2: ros_gz_bridge (ROS 2 ٹاپکس کو Gazebo میں ترجمہ کرتا ہے)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry"
        ],
        output="screen"
    )

    # نوڈ 3: Gazebo میں روبوٹ اسپان کریں
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="spawn",
        arguments=[
            "-name", "my_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-file", urdf_file_path
        ],
        output="screen"
    )

    return LaunchDescription([
        rsp_node,
        bridge_node,
        spawn_robot
    ])
```

**اس کی وضاحت**:
1. **robot_state_publisher**: URDF پڑھتا ہے، ٹرانسفارمز شائع کرتا ہے
2. **ros_gz_bridge**: ROS 2 ٹاپکس کو Gazebo میں ترجمہ کرتا ہے
3. **spawn** نوڈ: روبوٹ کو دنیا میں شامل کرنے کے لیے اسپان سروس کو کال کرتا ہے

---

## URDF سے SDF میں تبدیلی

Gazebo SDF فارمیٹ کو ترجیح دیتا ہے، لیکن `robot_state_publisher` آپ کے لیے خود بخود URDF کو تبدیل کرتا ہے۔ یہاں بتایا گیا ہے کہ کیا ہوتا ہے:

**ان پٹ (URDF)**:
```xml
<robot name="robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="wheel"/>
    <limit lower="0" upper="0" effort="100" velocity="1"/>
  </joint>
</robot>
```

**خودکار تبدیلی** (اندرونی طور پر، آپ SDF نہیں لکھتے):
- URDF لنکس SDF لنکس بن جاتے ہیں
- URDF جوائنٹس SDF جوائنٹس بن جاتے ہیں
- جڑتا کے پیرامیٹرز فزکس خصوصیات میں تبدیل ہو جاتے ہیں
- ٹکراؤ اور بصری ہندسی شکل برقرار رہتی ہے

آپ URDF ایک بار لکھتے ہیں، Gazebo تبدیلی کو سنبھالتا ہے۔

---

## پوزیشن اور اورینٹیشن پیرامیٹرز

اسپان کرتے وقت، بتائیں کہ روبوٹ کہاں ظاہر ہوتا ہے:

**پوزیشن** (میٹر میں x، y، z):
```python
request.initial_pose.position.x = 0.0
request.initial_pose.position.y = 0.0
request.initial_pose.position.z = 0.5  # زمین سے 0.5m اوپر
```

**اورینٹیشن** (کواٹرنیئن):
```python
from math import sin, cos, pi

# z-axis کے گرد 45 ڈگری گھمائیں
theta = pi / 4
request.initial_pose.orientation.x = 0
request.initial_pose.orientation.y = 0
request.initial_pose.orientation.z = sin(theta / 2)
request.initial_pose.orientation.w = cos(theta / 2)
```

**عام اورینٹیشنز**:
- +X کی طرف منہ: کوآٹ [0, 0, 0, 1]
- +Y کی طرف منہ: کوآٹ [0, 0, 0.707, 0.707]
- -X کی طرف منہ: کوآٹ [0, 0, 1, 0]
- -Y کی طرف منہ: کوآٹ [0, 0, 0.707, -0.707]

یا ایک کواٹرنیئن لائبریری استعمال کریں:
```python
from tf_transformations import quaternion_from_euler

quat = quaternion_from_euler(0, 0, pi/4)  # رول، پچ، یاو
request.initial_pose.orientation.x = quat[0]
request.initial_pose.orientation.y = quat[1]
request.initial_pose.orientation.z = quat[2]
request.initial_pose.orientation.w = quat[3]
```

---

## جوائنٹ اسٹیٹ پبلشنگ

جب آپ کوئی روبوٹ اسپان کرتے ہیں، تو یہ تمام جوائنٹس صفر پوزیشن پر شروع ہوتے ہیں۔ RViz میں روبوٹ کو درست طریقے سے دیکھنے کے لیے، آپ کو جوائنٹ اسٹیٹس شائع کرنے کی ضرورت ہے۔

**آپشن A: Gazebo سے**
Gazebo میں روبوٹ خود بخود اپنی جوائنٹ اسٹیٹس شائع کرتا ہے۔ RViz میں `/joint_states` ٹاپک کو سبسکرائب کریں۔

**آپشن B: دستی پبلشنگ**
اگر آپ جوائنٹس کو دستی طور پر کنٹرول کر رہے ہیں، تو جوائنٹ اسٹیٹس شائع کریں:

```python
from sensor_msgs.msg import JointState
from rclpy.node import Node
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joints)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left', 'wheel_right', 'arm_joint']
        msg.position = [0.0, 0.0, 0.5]  # موجودہ پوزیشنز
        msg.velocity = [0.0, 0.0, 0.0]  # موجودہ ویلوسٹیز
        msg.effort = [0.0, 0.0, 0.0]    # لگائی گئی قوتیں

        self.publisher.publish(msg)
```

---

## مشق: ایک روبوٹ اسپان کریں

**ہدف**: ros_gz اسپان سروس کا استعمال کرتے ہوئے Gazebo میں ایک روبوٹ اسپان کریں۔

**سیٹ اپ**:
1. ایک روبوٹ URDF تیار رکھیں (باب 9 سے)
2. ایک خالی دنیا کے ساتھ Gazebo چل رہا ہو
3. robot_state_publisher + برج + اسپان نوڈز کے ساتھ لانچ فائل بنائیں

**مرحلہ 1: لانچ فائل بنائیں**
```python
# launch/spawn_robot.launch.py
# [اوپر سے مکمل لانچ فائل استعمال کریں]
```

**مرحلہ 2: لانچ کریں**
```bash
ros2 launch robot_launcher spawn_robot.launch.py
```

**متوقع آؤٹ پٹ**:
```
[robot_state_publisher-1]: Publishing robot state
[parameter_bridge-2]: Starting ros_gz_bridge
[spawn-3]: Spawning robot 'my_robot' at [0.0, 0.0, 0.5]
[spawn-3]: Robot spawned successfully!
```

**مرحلہ 3: Gazebo میں تصدیق کریں**
- روبوٹ مخصوص پوزیشن پر دنیا میں ظاہر ہوتا ہے
- Gazebo سمولیشن روبوٹ ماڈل دکھاتا ہے

**مرحلہ 4: RViz2 میں تصدیق کریں**
```bash
ros2 launch robot_launcher display.launch.py
```
- روبوٹ لنکس کے لیے TF فریمز ظاہر ہوتے ہیں
- RViz میں روبوٹ ماڈل دکھاتا ہے

**کامیابی**: روبوٹ Gazebo (فزکس) اور RViz (ویژولائزیشن) دونوں میں مطابقت پذیر حالت کے ساتھ ظاہر ہوتا ہے۔

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: ChatGPT یا Claude کھولیں اور اپنے مخصوص روبوٹ کو اسپان کرنے کے بارے میں پوچھیں۔

**پرامپٹ 1** (اسپان کی ناکامی کو ڈیبگ کرنا):
```
My robot won't spawn in Gazebo. The error message is:
"Spawn failed: Could not find model"

The URDF loads fine when I view it in RViz. What could be wrong
with the spawn call? Walk me through the checklist.
```

**پرامپٹ 2** (متعدد روبوٹس کو اسپان کرنا):
```
I want to spawn 5 instances of the same robot in different positions:
(0, 0), (1, 1), (2, 0), (1, 2), (0, 2)

Show me how to modify the spawn launch file to spawn multiple robots
with unique names at each position.
```

**پرامپٹ 3** (URDF سے SDF میں تبدیلی):
```
I'm getting warnings about inertia values during spawn. My URDF has
inertia properties but Gazebo says some values are near zero.
What inertia properties are critical for spawning, and what are
the minimum reasonable values?
```

**متوقع نتائج**:
- AI اسپان کی ناکامیوں کو ڈیبگ کرنے میں مدد کرتا ہے (غائب پیرامیٹرز، نحو کی غلطیاں)
- AI متعدد روبوٹس کو اسپان کرنے کے لیے پیٹرن دکھاتا ہے
- AI وضاحت کرتا ہے کہ کون سی URDF خصوصیات Gazebo اسپاننگ کو متاثر کرتی ہیں

**حفاظتی نوٹ**: فوری طور پر حرکت کا حکم نہ دیں (اسپان کرنے کے فوراً بعد صفر ویلوسٹی سے شروع کریں)۔ Gazebo کو کشش ثقل کی سمولیشن کرنے اور روبوٹ کو مستحکم ہونے کا وقت دیں۔

---

## اگلے اقدامات

اب آپ لانچ فائلوں سے روبوٹس کو متحرک طور پر اسپان کرتے ہیں۔ اگلے سبق میں، آپ ان روبوٹس کو کمانڈ کریں گے اور ان کے سینسرز کو پڑھیں گے، کنٹرول لوپ کو بند کریں گے۔

**اس سبق سے کیا سامنے آیا**: روبوٹ کی تعریف (URDF) کو روبوٹ کے نفاذ (اسپان سروس) سے الگ کرنے سے لچکدار، دوبارہ استعمال کے قابل سمولیشن منظرناموں کو فعال کیا جاتا ہے۔

---

## کلیدی تصورات کا چیک پوائنٹ

آگے بڑھنے سے پہلے، تصدیق کریں کہ آپ سمجھتے ہیں:

- **robot_description پیرامیٹر**: روبوٹ URDF کو کیسے لوڈ اور شائع کیا جائے
- **robot_state_publisher**: یہ کیا شائع کرتا ہے اور اس کی ضرورت کیوں ہے
- **اسپان سروس**: اسے کیسے کال کریں اور اسے کن پیرامیٹرز کی ضرورت ہے
- **پوزیشن/اورینٹیشن**: روبوٹ کہاں اسپان ہوتا ہے یہ کیسے بیان کریں
- **لانچ فائل انٹیگریشن**: تمام اجزاء کو ایک ساتھ لانا

اگر یہ واضح ہیں، تو آپ سبق 12.3 کے لیے تیار ہیں۔