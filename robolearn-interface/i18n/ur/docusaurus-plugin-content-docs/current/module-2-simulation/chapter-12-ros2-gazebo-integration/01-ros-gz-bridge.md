---
id: lesson-12-1-ros-gz-bridge
title: 'Lesson 12.1: The ros_gz Bridge'
sidebar_position: 1
sidebar_label: 12.1 ros_gz Bridge
description: Connecting Gazebo topics to ROS 2 using ros_gz_bridge
duration_minutes: 75
proficiency_level: B1
layer: L2
hardware_tier: 1
learning_objectives:
  - Explain the ros_gz_bridge architecture and purpose
  - Configure topic bridging using command-line syntax
  - Create YAML configuration files for multiple topic bridges
  - Verify bridge connectivity and debug common issues
skills:
  - ros2-gazebo-bridge
cognitive_load:
  new_concepts: 8
tier_1_path: TheConstruct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 12.1: ros_gz_bridge

## کنکشن کا مسئلہ (The Connection Problem)

Gazebo ایک فزکس سمولیشن چلاتا ہے۔ ROS 2 روبوٹکس کے لیے ایک مڈل ویئر ہے۔ وہ مختلف زبانیں بولتے ہیں:

**Gazebo** اینٹیٹیز (entities)، ماڈلز (models)، اور فزکس پراپرٹیز کے لحاظ سے سوچتا ہے۔ یہ Gazebo-مقامی میسج ٹائپس (`gz.msgs.Image`, `gz.msgs.LaserScan`) میں سینسر ڈیٹا پبلش کرتا ہے۔

**ROS 2** نوڈز (nodes) اور ٹاپکس (topics) کے لحاظ سے سوچتا ہے۔ یہ ROS میسج ٹائپس (`sensor_msgs/msg/Image`, `sensor_msgs/msg/LaserScan`) میں ڈیٹا پبلش کرتا ہے۔

ROS 2 ایپلیکیشن کو Gazebo میں روبوٹ کو کنٹرول کرنے کے لیے، کسی چیز کو ان کے درمیان ترجمہ کرنا ضروری ہے۔ وہ چیز `ros_gz_bridge` ہے—ایک ٹول جو دو طرفہ طور پر ROS 2 ٹاپکس کو Gazebo ٹاپکس سے میپ کرتا ہے، ضرورت کے مطابق میسجز کو تبدیل کرتا ہے۔

برج کے بغیر: آپ کا ROS 2 نوڈ `/cmd_vel` پر `geometry_msgs/msg/Twist` پبلش کرتا ہے، لیکن Gazebo کو نہیں معلوم کہ یہ میسج کیا ہے۔ آپ کا روبوٹ حرکت نہیں کرتا۔

برج کے ساتھ: برج `/cmd_vel (Twist)` کو Gazebo کے `/cmd_vel (gz.msgs.Twist)` میں ترجمہ کرتا ہے، اور آپ کا روبوٹ جواب دیتا ہے۔

---

## برج آرکیٹیکچر کو سمجھنا (Understanding Bridge Architecture)

ros_gz_bridge کو ٹیلی فون پر گفتگو میں مترجم کے طور پر سوچیں:

```
ROS 2 Node (English)
    ↓
    "Set velocity to 1.0 m/s forward"
    ↓
ros_gz_bridge (Translator)
    ↓
    [ارادے کو سمجھتا ہے، Gazebo فارمیٹ میں تبدیل کرتا ہے]
    ↓
Gazebo (Gazebo language)
    "Apply velocity vector [1.0, 0, 0]"
    ↓
Physics Simulation
    Robot moves
```

برج تین آپریشنز سنبھالتا ہے:

**1. ROS 2 → Gazebo (ROS_TO_GZ)**
- ROS نوڈ ٹاپک `/cmd_vel` پر میسج پبلش کرتا ہے
- برج وصول کرتا ہے، Gazebo فارمیٹ میں تبدیل کرتا ہے
- برج Gazebo ٹاپک `/cmd_vel` پر لکھتا ہے
- Gazebo اینٹیٹی کمانڈ پر عملدرآمد کرتی ہے

**2. Gazebo → ROS 2 (GZ_TO_ROS)**
- Gazebo اندرونی ٹاپک پر سینسر ڈیٹا پبلش کرتا ہے (مثلاً کیمرہ امیج)
- برج وصول کرتا ہے، ROS میسج فارمیٹ میں تبدیل کرتا ہے
- برج ROS 2 ٹاپک (مثلاً `/camera/image_raw`) پر پبلش کرتا ہے
- ROS 2 نوڈز سبسکرائب کرتے ہیں اور پروسیس کرتے ہیں

**3. دو طرفہ (BIDIRECTIONAL)**
- ایک ہی ٹاپک، میسجز دونوں سمتوں میں بہتے ہیں
- کم عام، بنیادی طور پر ہم آہنگ حالت (synchronized state) کے لیے

---

## برج سینٹیکس: تین حصے (Bridge Syntax: The Three Parts)

برج کنفیگریشن کے تین اہم اجزاء ہوتے ہیں:

```
/topic_name @ ROS_MESSAGE_TYPE @ GAZEBO_MESSAGE_TYPE
```

**مثال**:
```
/cmd_vel @ geometry_msgs/msg/Twist @ gz.msgs.Twist
```

اس کی تفصیل:

### حصہ 1: ٹاپک کا نام (Part 1: Topic Name)
```
/cmd_vel
```
وہ ٹاپک جو دونوں اطراف استعمال کرتے ہیں (یا اگر مختلف ہو تو Gazebo ٹاپک)۔ عملی طور پر، یہ عام طور پر ایک ہی ہوتا ہے۔

### حصہ 2: ROS میسج کی قسم (Part 2: ROS Message Type)
```
geometry_msgs/msg/Twist
```
وہ میسج کی قسم جو ROS 2 استعمال کرتا ہے۔ مکمل کوالیفائیڈ نام: `package/msg/MessageName`۔

**عام ROS ٹائپس**:
- `geometry_msgs/msg/Twist` — ویلوسٹی کمانڈز (لینیئر + اینگولر)
- `sensor_msgs/msg/Image` — کیمرہ امیجز
- `sensor_msgs/msg/LaserScan` — LIDAR ڈیٹا (2D)
- `sensor_msgs/msg/PointCloud2` — پوائنٹ کلاؤڈز (3D)
- `std_msgs/msg/Float64` — سکیلر عددی اقدار

### حصہ 3: Gazebo میسج کی قسم (Part 3: Gazebo Message Type)
```
gz.msgs.Twist
```
وہ میسج کی قسم جو Gazebo استعمال کرتا ہے۔ نیم اسپیس ہمیشہ `gz.msgs` ہوتا ہے۔

**عام Gazebo ٹائپس**:
- `gz.msgs.Twist` — ویلوسٹی کمانڈز
- `gz.msgs.Image` — کیمرہ امیجز
- `gz.msgs.LaserScan` — LIDAR ڈیٹا
- `gz.msgs.PointCloudPacked` — پوائنٹ کلاؤڈز (پیک شدہ بائنری)
- `gz.msgs.Double` — سکیلر اقدار
- `gz.msgs.Boolean` — سچ/جھوٹ اقدار

---

## کمانڈ لائن برج: فوری جانچ (Command-Line Bridge: Quick Testing)

برج کو جانچنے کا سب سے آسان طریقہ کمانڈ لائن کے ذریعے ہے:

```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

یہ `/cmd_vel` ٹاپک کے لیے ایک واحد برج لانچ کرتا ہے، جو ROS Twist اور Gazebo Twist کے درمیان تبدیل کرتا ہے۔

**مزید ٹاپکس شامل کرنے کے لیے**، الگ الگ ٹرمینلز میں اضافی برج کمانڈز چلائیں:

```bash
# ٹرمینل 1: ویلوسٹی کمانڈز
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# ٹرمینل 2: کیمرہ امیج
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image

# ٹرمینل 3: LIDAR اسکین
ros2 run ros_gz_bridge parameter_bridge /lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**برج کی جانچ**:

ایک ٹرمینل میں، ڈیٹا کے بہاؤ کی تصدیق کے لیے ٹاپک کو ایکو (echo) کریں:

```bash
ros2 topic echo /cmd_vel
```

دوسرے ٹرمینل میں، ایک ٹیسٹ میسج پبلش کریں:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.5}, "angular": {"z": 0.0}}'
```

اگر روبوٹ حرکت کرتا ہے، تو برج کام کر رہا ہے۔

---

## YAML کنفیگریشن: پروڈکشن سیٹ اپ (YAML Configuration: Production Setup)

کمانڈ لائن جانچ کے لیے کام کرتی ہے۔ حقیقی ایپلی کیشنز کے لیے، تمام برج کو ایک ساتھ بیان کرنے کے لیے YAML کنفیگریشن استعمال کریں:

**فائل**: `bridge.yaml`
```yaml
# سنگل ٹاپک برج (ROS سے Gazebo)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

# سنگل ٹاپک برج (Gazebo سے ROS)
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# دو طرفہ برج
- ros_topic_name: "/odom"
  gz_topic_name: "/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: BIDIRECTIONAL
```

**YAML کے ساتھ برج لانچ کریں**:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

**YAML کے فوائد**:
- تمام برج ایک جگہ پر
- ورژن کنٹرول کے لیے دوستانہ (git میں کمٹ کریں)
- متعدد روبوٹس میں دوبارہ قابل استعمال
- میسج میپنگ کی واضح دستاویزات

---

## عام میسج ٹائپ میپنگز (Common Message Type Mappings)

یہ سب سے زیادہ کارآمد ROS 2 ↔ Gazebo میسج میپنگز ہیں:

### ویلوسٹی اور حرکت (Velocity and Motion)
```
ROS 2                           Gazebo
geometry_msgs/msg/Twist      ↔ gz.msgs.Twist
geometry_msgs/msg/Pose       ↔ gz.msgs.Pose
geometry_msgs/msg/Transform  ↔ gz.msgs.Transform
```

**استعمال کریں**: روبوٹ ویلوسٹی کمانڈز، پوزیشن کمانڈز، کوآرڈینیٹ ٹرانسفارمز کے لیے

### سینسرز (Sensors)
```
ROS 2                           Gazebo
sensor_msgs/msg/Image        ↔ gz.msgs.Image
sensor_msgs/msg/LaserScan    ↔ gz.msgs.LaserScan
sensor_msgs/msg/PointCloud2  ↔ gz.msgs.PointCloudPacked
sensor_msgs/msg/Imu          ↔ gz.msgs.IMU
```

**استعمال کریں**: کیمرہ آؤٹ پٹ، LIDAR اسکینز، 3D پوائنٹ کلاؤڈز، انرشیل ڈیٹا کے لیے

### سادہ اقدار (Simple Values)
```
ROS 2                         Gazebo
std_msgs/msg/Float64      ↔ gz.msgs.Double
std_msgs/msg/Int32        ↔ gz.msgs.Int32
std_msgs/msg/Bool         ↔ gz.msgs.Boolean
std_msgs/msg/String       ↔ gz.msgs.StringMsg
```

**استعمال کریں**: سکیلر سینسر ریڈنگز، کنفیگریشن اقدار، سادہ سگنلز کے لیے

---

## برج کنیکٹیویٹی کی تصدیق (Verifying Bridge Connectivity)

ایک بار جب آپ کا برج چل رہا ہو، تو تصدیق کریں کہ یہ واقعی منسلک ہے:

**مرحلہ 1: ROS 2 ٹاپکس چیک کریں**
```bash
ros2 topic list
```
آپ کو اپنے برج ٹاپکس کی فہرست نظر آنی چاہیے۔

**مرحلہ 2: Gazebo ٹاپکس چیک کریں**
```bash
gz topic --list
```
آپ کو Gazebo کے اندرونی ٹاپکس کی فہرست نظر آنی چاہیے۔

**مرحلہ 3: ٹاپک ڈیٹا ایکو کریں**
```bash
# چیک کریں کہ ڈیٹا بہہ رہا ہے یا نہیں
ros2 topic echo /cmd_vel
```
اگر آپ کو میسجز نظر آتے ہیں، تو برج پبلش کر رہا ہے۔

**مرحلہ 4: پبلش شدہ میسج کے ساتھ ٹیسٹ کریں**
```bash
# ایک ٹیسٹ کمانڈ بھیجیں
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 1.0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}'
```
سمولیٹڈ روبوٹ کو دیکھیں۔ اگر یہ حرکت کرتا ہے، تو برج اینڈ ٹو اینڈ کام کر رہا ہے۔

---

## برج کے مسائل کو ڈیبگ کرنا (Debugging Bridge Problems)

**مسئلہ 1: برج شروع نہیں ہو رہا**
```
[ERROR] ros_gz_bridge: Invalid bridge syntax: /cmd_vel@...
```
چیک کریں:
- سینٹیکس `/topic@ROS_TYPE@GZ_TYPE` ہے
- کوئی اضافی خالی جگہ نہیں
- ROS ٹائپ میں پیکیج شامل ہے (`geometry_msgs/msg/Twist`، صرف `Twist` نہیں)
- Gazebo ٹائپ `gz.msgs` سے شروع ہوتا ہے

**مسئلہ 2: ڈیٹا نہیں بہہ رہا (برج چل رہا ہے لیکن کوئی میسج نہیں)**
```bash
# چیک کریں کہ برج سن رہا ہے یا نہیں
ros2 topic info /cmd_vel
# "1 publisher, 0 subscribers" یا اس طرح کا کچھ دکھانا چاہیے

# دستی طور پر ایک ٹیسٹ میسج پبلش کریں
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 1.0}}'
```
اگر دستی پبلشنگ کام کرتی ہے لیکن خود بخود کچھ نہیں ہوتا:
- چیک کریں کہ آپ کا ROS 2 نوڈ واقعی پبلش کر رہا ہے (استعمال کریں `ros2 topic echo`)
- میسج فارمیٹ سے مماثلت کی تصدیق کریں (غلط فیلڈز = میسج مسترد)

**مسئلہ 3: ٹائپ کی بے میل (mismatch) کے غلطیاں**
```
[WARN] Cannot find ROS type: geometry_msgs/msg/Twist
```
ٹائپ موجود نہیں ہے یا انسٹال نہیں ہے۔ میسج پیکیج انسٹال کریں:
```bash
sudo apt install ros-humble-geometry-msgs
```

---

## مشق: اپنا پہلا برج کنفیگر کریں (Exercise: Configure Your First Bridge)

**ہدف**: Gazebo میں ایک سمولیٹڈ روبوٹ پر ROS 2 سے ویلوسٹی کمانڈز کو برج کریں۔

**سیٹ اپ**:
1. ایک روبوٹ ماڈل کے ساتھ Gazebo لانچ کریں (باب 10/11 سے)
2. ایک ٹرمینل میں، برج شروع کریں:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
   ```
3. دوسرے ٹرمینل میں، ایک ویلوسٹی کمانڈ پبلش کریں:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.5, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}'
   ```

**کامیابی**: روبوٹ Gazebo میں آگے بڑھتا ہے۔

**تصدیق کریں**:
- `ros2 topic list` میں `/cmd_vel` شامل ہے
- `ros2 topic echo /cmd_vel` آپ کے ویلوسٹی میسجز دکھاتا ہے
- Gazebo میں روبوٹ کمانڈز پر ردعمل ظاہر کرتا ہے

---

## AI کے ساتھ کوشش کریں (Try With AI)

**سیٹ اپ**: ChatGPT یا Claude کھولیں اور اپنے مخصوص روبوٹ کے لیے ros_gz_bridge کنفیگریشن کے بارے میں پوچھیں۔

**پرامپٹ سیٹ**:

**پرامپٹ 1** (میسج ٹائپس کو سمجھنا):
```
I'm working with ros_gz_bridge and my robot has these sensors:
- A forward-facing camera
- A 2D LIDAR scanner
- An IMU sensor

Show me the YAML configuration for bridging these three sensors
from Gazebo to ROS 2. Include topic names and message types.
```

**پرامپٹ 2** (مسائل کا حل):
```
My ros_gz_bridge is running but ros2 topic echo /cmd_vel shows no messages
even though my node is publishing. What could be wrong? Walk me through
the debugging checklist.
```

**پرامپٹ 3** (ایڈوانسڈ کنفیگریشن):
```
I want to bridge the robot's odometry (position estimate) bidirectionally
between Gazebo and ROS 2. What message type should I use, and should
the direction be BIDIRECTIONAL or ROS_TO_GZ? Explain the tradeoff.
```

**متوقع نتائج**:
- AI کام کرنے والی YAML کنفیگریشن فراہم کرتا ہے جسے آپ اپنا سکتے ہیں
- AI ڈیبگنگ کے ایسے مراحل تجویز کرتا ہے جن پر آپ نے غور نہیں کیا ہوگا
- AI وضاحت کرتا ہے کہ کون سی میسج ٹائپس دوسروں کے مقابلے میں بہتر طریقے سے مماثل ہیں۔

**حفاظتی نوٹ**: تمام برج کنفیگریشن کو پہلے سمولیشن میں ٹیسٹ کریں۔ غلط کنفیگر شدہ برج تعینات ہونے پر غیر متوقع حرکت کا حکم دے سکتا ہے۔

---

## اگلے اقدامات (Next Steps)

آپ نے برج کو کنفیگر کر لیا ہے۔ اگلے سبق میں، آپ Gazebo دنیاؤں میں روبوٹس کو متحرک طور پر اسپان (spawn) کرنے کے لیے اس برج کا استعمال کریں گے، جس سے ماڈیولر ٹیسٹنگ اور ملٹی روبوٹ منظرنامے ممکن ہو سکیں گے۔

**اس سبق سے کیا سامنے آیا**: ROS 2 اور Gazebo کے درمیان بات چیت کا طریقہ کار سمجھنا چیزوں کے خراب ہونے پر ڈیبگ کرنے کے قابل بناتا ہے۔ برج تمام ROS 2-Gazebo انٹیگریشن کام کی بنیاد ہے۔

---

## کلیدی تصورات کا چیک پوائنٹ (Key Concepts Checkpoint)

آگے بڑھنے سے پہلے، اس بات کی تصدیق کریں کہ آپ یہ سمجھتے ہیں:

- **برج کا مقصد**: ROS 2 اور Gazebo میسج فارمیٹس کے درمیان ترجمہ کرنا
- **برج کا سینٹیکس**: `/topic@ROS_TYPE@GZ_TYPE` سمت کے ساتھ
- **کنفیگریشن کے طریقے**: جانچ کے لیے کمانڈ لائن، پروڈکشن کے لیے YAML
- **میسج میپنگز**: یہ جاننا کہ کون سی ROS ٹائپ کس Gazebo ٹائپ سے مطابقت رکھتی ہے
- **تصدیق**: برج کی فعالیت کی تصدیق کے لیے `ros2 topic list/echo` کا استعمال

اگر یہ واضح ہیں، تو آپ سبق 12.2 کے لیے تیار ہیں۔