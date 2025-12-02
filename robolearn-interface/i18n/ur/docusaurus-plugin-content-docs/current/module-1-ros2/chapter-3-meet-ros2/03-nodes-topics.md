---
id: lesson-3-3-nodes-topics
title: 'Lesson 3.3: Nodes and Topics (CLI Exploration)'
sidebar_position: 3
sidebar_label: 3.3 Nodes & Topics
description: >-
  Explore ROS 2 nodes and topics using CLI tools to understand
  publisher/subscriber communication.
duration_minutes: 60
proficiency_level: A2
layer: L1
hardware_tier: 1
learning_objectives:
  - List running nodes using ros2 node list
  - List active topics using ros2 topic list -t
  - Echo topic data in real-time using ros2 topic echo
  - Understand publisher/subscriber pattern through observation
skills:
  - ros2-fundamentals
  - ros2-cli
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# نوڈز اور ٹاپکس (CLI ایکسپلوریشن)

لیسن 3.2 میں، آپ نے دیکھا کہ ٹرٹل سم (turtlesim) اور ٹیلی اوپ (teleop) ایک ساتھ کام کر رہے ہیں۔ اب آپ کمانڈ لائن ٹولز کا استعمال کرتے ہوئے **نظام کو منظم طریقے سے دریافت** کریں گے۔

اس لیسن کے اختتام تک، آپ کو ان باتوں کی درست سمجھ آ جائے گی:
- کون سے **نوڈز** چل رہے ہیں
- وہ بات چیت کے لیے کون سے **ٹاپکس** استعمال کرتے ہیں
- ان ٹاپکس کے ذریعے کون سا **ڈیٹا** بہہ رہا ہے

**مدت**: 60 منٹ | **ہارڈ ویئر ٹائر**: ٹائر 1 | **صرف CLI ایکسپلوریشن—ابھی کوئی کوڈنگ نہیں**

---

## ضروریات (Prerequisites)

شروع کرنے سے پہلے، لیسن 3.2 سے ٹرٹل سم کو چلتا ہوا رکھیں:

**ٹرمینل 1**:
```bash
ros2 run turtlesim turtlesim_node
```

**ٹرمینل 2**:
```bash
ros2 run turtlesim turtle_teleop_key
```

جب آپ ٹرمینل 3 میں ایکسپلوریشن کر رہے ہوں تو دونوں کو چلتا رکھیں۔

---

## بنیادی تصورات (Core Concepts)

ایکسپلوریشن سے پہلے، سمجھیں کہ آپ کیا ڈھونڈ رہے ہیں:

### نوڈ کیا ہے؟ (What is a Node?)

ایک **نوڈ** ROS لاجک چلانے والا ایک آزاد عمل (independent process) ہے۔ اسے ایک پروگرام سمجھیں۔

مثالیں:
- `turtlesim_node`: ایک پروگرام جو ایک کچوے کی نقل (simulate) کرتا ہے
- `turtle_teleop_key`: ایک پروگرام جو کی بورڈ ان پٹ پڑھتا ہے
- `rqt_graph`: ایک پروگرام جو نظام کا بصری نقشہ (visualize) بناتا ہے

ہر نوڈ:
- کا ایک نام ہوتا ہے (مثلاً `/turtlesim`, `/teleop_turtle`)
- ٹاپکس پر پبلش کرتا ہے (ڈیٹا بھیجتا ہے)
- ٹاپکس کو سبسکرائب کرتا ہے (ڈیٹا وصول کرتا ہے)
- سروسز پیش کر سکتا ہے (درخواست/جواب، جو لیسن 3.4 میں شامل ہوں گی)

---

### ٹاپک کیا ہے؟ (What is a Topic?)

ایک **ٹاپک** پیغامات کے لیے ایک نامزد چینل ہے۔ اسے پب/سب کمیونیکیشن کے لیے ایک "موضوع" سمجھیں۔

مثالیں:
- `/turtle1/cmd_vel`: کچوے کو حرکت دینے کے احکامات
- `/turtle1/odometry`: کچوے سے پوزیشن/رفتار کا فیڈ بیک
- `/turtle1/color_sensor`: (فرضی) سینسر سے رنگ کا ڈیٹا

ہر ٹاپک:
- کا نام `/` سے شروع ہوتا ہے (مثلاً `/turtle1/cmd_vel`)
- کا ایک میسج ٹائپ ہوتا ہے (مثلاً رفتار کے لیے `geometry_msgs/Twist`)
- کے متعدد پبلشرز ہو سکتے ہیں (عام طور پر صرف 1)
- کے متعدد سبسکرائبرز ہو سکتے ہیں (عام طور پر کئی)

---

### پبلشر بمقابلہ سبسکرائبر (Publisher vs Subscriber)

جب کوئی نوڈ کسی ٹاپک پر پبلش کرتا ہے، تو وہ پیغامات بھیجتا ہے۔
جب کوئی نوڈ کسی ٹاپک کو سبسکرائب کرتا ہے، تو وہ پیغامات وصول کرتا ہے۔

**مثال**:
- `turtle_teleop_key` کی بورڈ ان پٹ کو `/turtle1/cmd_vel` پر **پبلش** کرتا ہے
- `turtlesim_node` `/turtle1/cmd_vel` کو **سبسکرائب** کرتا ہے اور پیغامات پر عمل کرتا ہے

دونوں نوڈز براہ راست تعامل نہیں کرتے—ROS 2 مڈل ویئر پیغام کی ترسیل سنبھالتا ہے۔

---

## مرحلہ 1: تمام چلنے والے نوڈز کی فہرست بنائیں

**ٹرمینل 3** کھولیں اور چلائیں:

```bash
ros2 node list
```

**متوقع آؤٹ پٹ**:
```
/teleop_turtle
/turtlesim
```

**یہ کیا ظاہر کرتا ہے**:
- دو نوڈز چل رہے ہیں
- `/teleop_turtle`: کی بورڈ ٹیلی اوپ کا عمل (process)
- `/turtlesim`: سمولیٹر کا عمل

**نام `/` سے شروع ہوتے ہیں** — یہ نوڈ کے ناموں کے لیے ROS 2 کا رواج (convention) ہے۔

---

## مرحلہ 2: ایک نوڈ کا معائنہ کریں

`/turtlesim` نوڈ کے بارے میں تفصیلی معلومات حاصل کریں:

```bash
ros2 node info /turtlesim
```

**متوقع آؤٹ پٹ**:
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /turtle1/color_sensor: std_msgs/msg/UInt32
    /turtle1/odometry: nav_msgs/msg/Odometry
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Reset
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:
    (none)
```

**یہ آپ کو کیا بتاتا ہے**:

- **سبسکرائبرز**: `/turtlesim` `/turtle1/cmd_vel` (رفتار کے احکامات) سنتا ہے
- **پبلشرز**: `/turtlesim` اودومیٹری (پوزیشن) اور رنگ سینسر ڈیٹا بھیجتا ہے
- **سروس سرورز**: `/turtlesim` `/spawn`, `/kill`, `/reset` جیسی سروسز پیش کرتا ہے (جو لیسن 3.4 میں شامل ہوں گی)

---

## مرحلہ 3: تمام ٹاپکس کی فہرست بنائیں

نظام میں تمام ٹاپکس دیکھیں:

```bash
ros2 topic list
```

**متوقع آؤٹ پٹ**:
```
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/odometry
```

**آپ کیا دیکھتے ہیں**:
- `/rosout` اور `/rosout_agg`: نظام کی لاگنگ (فی الحال نظر انداز کریں)
- `/turtle1/cmd_vel`: رفتار کے احکامات (کی بورڈ ٹیلی اوپ → ٹرٹل سم)
- `/turtle1/color_sensor`: سینسر کا ڈیٹا (فرضی رنگ پڑھنا)
- `/turtle1/odometry`: ٹرٹل سم سے پوزیشن/رفتار کا فیڈ بیک

---

## مرحلہ 4: ٹاپک کی تفصیلات حاصل کریں

پیغام کی اقسام کے ساتھ ٹاپکس کی فہرست بنائیں:

```bash
ros2 topic list -t
```

**متوقع آؤٹ پٹ**:
```
/rosout [rcl_interfaces/msg/Log]
/rosout_agg [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [std_msgs/msg/UInt32]
/turtle1/odometry [nav_msgs/msg/Odometry]
```

**`-t` کیا دکھاتا ہے**:
- ہر ٹاپک کے لیے میسج ٹائپ
- `geometry_msgs/msg/Twist`: رفتار کے احکامات (لکیری + اینگولر رفتار)
- `nav_msgs/msg/Odometry`: پوزیشن کا ڈیٹا (x, y, تھیٹا، رفتاریں)
- `std_msgs/msg/UInt32`: ایک ہی عدد (رنگ کی قیمت)

---

## مرحلہ 5: ایک ٹاپک کو ایکو کریں (ریئل ٹائم ڈیٹا دیکھیں)

`/turtle1/cmd_vel` سے بہنے والے اصل ڈیٹا کو دیکھیں:

```bash
ros2 topic echo /turtle1/cmd_vel
```

اب ٹرمینل 2 پر جائیں (جہاں ٹیلی اوپ چل رہا ہے) اور **تیر والے بٹن (arrow keys) دبائیں**۔

**ٹرمینل 3 میں، آپ کو ریئل ٹائم اپ ڈیٹس نظر آئیں گی**:

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

**آپ کیا دیکھتے ہیں**:
- ہر `---` سیپریٹر ایک نیا پیغام ہے
- `linear.x = 2.0`: کچوا 2 میٹر فی سیکنڈ کی رفتار سے آگے بڑھ رہا ہے
- `angular.z`: گردش (0 کا مطلب کوئی گردش نہیں)
- جب آپ ← یا → دبائیں گے، تو `angular.z` تبدیل ہو جائے گا

**کیا ہو رہا ہے**:
- آپ ٹرمینل 2 میں ایک بٹن دباتے ہیں
- ٹیلی اوپ `/turtle1/cmd_vel` پر ایک ٹویسٹ میسج پبلش کرتا ہے
- آپ اسے یہاں ٹرمینل 3 میں دیکھتے ہیں (آپ سبسکرائب کر رہے ہیں)
- دریں اثنا، ٹرٹل سم بھی سبسکرائب کرتا ہے اور کچوے کی پوزیشن کو اپ ڈیٹ کرتا ہے

**ایکونگ روکنے کے لیے Ctrl+C دبائیں۔**

---

## مرحلہ 6: اودومیٹری کو ایکو کریں (پوزیشن فیڈ بیک)

دیکھیں کہ کچوا کہاں ہے:

```bash
ros2 topic echo /turtle1/odometry
```

ٹرمینل 2 پر واپس جائیں اور کچوے کو ادھر ادھر حرکت دیں۔

**ٹرمینل 3 میں، آپ کو پوزیشن اپ ڈیٹس نظر آئیں گی**:

```
header:
  stamp:
    sec: 123
    nanosec: 456789012
  frame_id: world
child_frame_id: turtle1
pose:
  pose:
    position:
      x: 5.544444
      y: 5.544444
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071
      w: 0.7071
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
---
```

**اہم ڈیٹا**:
- `position.x, position.y`: کچوے کا موجودہ مقام
- `orientation` (کواٹرنیئن): کچوے کی گردش
- `twist.linear, twist.angular`: موجودہ رفتار (عام طور پر 0 جب کچوا رک جاتا ہے)

**کیا ہو رہا ہے**:
- ٹرٹل سم کچوے کی حالت (پوزیشن، سمت) برقرار رکھتا ہے
- ہر اپ ڈیٹ سائیکل میں، یہ اس حالت کو اودومیٹری میسج کے طور پر پبلش کرتا ہے
- آپ اس ڈیٹا کو وصول کر رہے ہیں اور دیکھ رہے ہیں

**روکنے کے لیے Ctrl+C دبائیں۔**

---

## سب کو ایک ساتھ لانا: پیغام کا بہاؤ (Message Flow)

اب مکمل بہاؤ کا تصور کریں:

```
ٹرمینل 2 (teleop_turtle)           ROS 2 مڈل ویئر              ٹرمینل 1 (turtlesim_node)
┌──────────────────────┐             ┌──────────────────┐          ┌───────────────────┐
│  کی بورڈ ان پٹ پڑھتا ہے      │             │  /turtle1/cmd_vel│          │  cmd_vel پڑھتا ہے    │
│                      │───پبلش کرتا ہے→│   [Twist msg]    │─سبسکرائب│  پوزیشن اپ ڈیٹ کرتا ہے │
│                      │             │                  │          │  /turtle1/ پر اودومیٹری پبلش کرتا ہے
└──────────────────────┘             │ /turtle1/odometry│          │  odometry          │
                                     │   [Odometry msg] │←سبسکرائب│                   │
                                     │                  │          │ کینوس نئے کچوے کی پوزیشن کے ساتھ اپ ڈیٹ ہوتا ہے
                                     └──────────────────┘          └───────────────────┘
آپ کا کی بورڈ
(↑ ↓ ← →)
```

**اہم بصیرت**: دونوں نوڈز مکمل طور پر الگ تھلگ (decoupled) ہیں۔ ٹیلی اوپ کو ٹرٹل سم کے نفاذ (implementation) کے بارے میں معلوم نہیں ہے۔ ٹرٹل سم کو کی بورڈ ان پٹ کے بارے میں معلوم نہیں ہے۔ وہ صرف ٹاپکس کے بارے میں جانتے ہیں۔ ROS 2 مڈل ویئر تمام پیغام کی روٹنگ سنبھالتا ہے۔

---

## عملی کام: معلومات جمع کرنا

اب جب آپ ٹولز جان گئے ہیں، تو اپنے نظام کے بارے میں معلومات جمع کریں:

### کام 1: `/turtle1/cmd_vel` کے کتنے پبلشرز ہیں؟

```bash
ros2 topic info /turtle1/cmd_vel
```

**متوقع آؤٹ پٹ**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

**جواب**: 1 پبلشر (teleop_turtle)

---

### کام 2: `/turtle1/odometry` کے کتنے سبسکرائبرز ہیں؟

```bash
ros2 topic info /turtle1/odometry
```

**متوقع آؤٹ پٹ**:
```
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
```

**جواب**: 0 سبسکرائبرز اس وقت (آپ نے اسے ایکو کیا تھا، لیکن ایکو عارضی ہوتا ہے اور مستقل سبسکرپشن کے طور پر شمار نہیں ہوتا)

---

### کام 3: ٹویسٹ میسج کی ساخت کیا ہے؟

```bash
ros2 interface show geometry_msgs/msg/Twist
```

**آؤٹ پٹ**:
```
# This expresses velocity in free space with only linear and angular parts.
Vector3  linear
  float64 x
  float64 y
  float64 z
Vector3  angular
  float64 x
  float64 y
  float64 z
```

**یہ کیا دکھاتا ہے**: ٹویسٹ میں 6 اجزاء ہیں—3 لکیری (x,y,z) اور 3 اینگولر (x,y,z)۔ ٹرٹل سم صرف `linear.x` اور `angular.z` استعمال کرتا ہے۔

---

### کام 4: `teleop_turtle` کن ٹاپکس پر پبلش کرتا ہے؟

```bash
ros2 node info /teleop_turtle
```

**متوقع آؤٹ پٹ**:
```
/teleop_turtle
  Subscribers:
    (none)
  Publishers:
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    (none)
  Service Clients:
    (none)
```

**جواب**: `/turtle1/cmd_vel` اور `/rosout` پر پبلش کرتا ہے

---

## یہ کیوں اہم ہے

آپ نے بغیر کوئی کوڈ لکھے، **منظم طریقے سے** ROS 2 نظام کی کھوج کی ہے:

1. **نوڈز دریافت کے قابل ہیں**: `ros2 node list`
2. **ٹاپکس قابلِ استفسار ہیں**: `ros2 topic list -t`, `ros2 topic info`
3. **ڈیٹا قابلِ مشاہدہ ہے**: `ros2 topic echo` ریئل ٹائم پیغامات دکھاتا ہے
4. **تعلقات واضح ہیں**: نوڈ کی معلومات دکھاتی ہے کہ کون کیا پبلش/سبسکرائب کر رہا ہے

یہ طاقتور ہے کیونکہ:
- **ڈیبگنگ**: آپ کوڈ کو چھوئے بغیر چلتے ہوئے نظام کا معائنہ کر سکتے ہیں
- **فہم**: نئے آنے والے نظام کے کام کرنے کا طریقہ ایکسپلوریشن کے ذریعے سیکھ سکتے ہیں
- **ماڈیولرٹی**: آپ نوڈز کو تبدیل کر سکتے ہیں (مثلاً ٹیلی اوپ کو ماؤس کنٹرولر سے بدلنا) بغیر ٹرٹل سم کو تبدیل کیے

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: ٹرٹل سم اور ٹیلی اوپ کو چلتا رکھیں. اپنا AI ٹول کھولیں۔

**پرامپٹ 1** (ڈی کپلنگ کو سمجھنا):

اپنے AI سے پوچھیں:

```
In the turtlesim system, /teleop_turtle publishes to /turtle1/cmd_vel,
and /turtlesim subscribes to the same topic.

Why is this better than teleop calling a function directly on turtlesim?
What problems would arise if teleop directly called turtlesim functions?
```

**متوقع بصیرت**: ڈھیلا جوڑ (Loose coupling)، توسیع پذیری (extensibility)، اور جانچ کی صلاحیت (testability) اہم فوائد ہیں۔ یہ ڈیزائن فلسفہ آپ کو باب 4 کے لیے تیار کرتا ہے جب آپ اپنے نوڈز لکھیں گے۔

---

**پرامپٹ 2** (ٹاپک ڈیزائن):

اپنے AI سے پوچھیں:

```
Turtlesim publishes odometry on /turtle1/odometry.
Right now, nothing subscribes to it.

If I wanted to log all turtle positions to a file, how would I do that?
Would I need to modify teleop or turtlesim? How would a logging node get the data?
```

**آپ کیا سیکھتے ہیں**: اودومیٹری کو سبسکرائب کرنے والا ایک نیا نوڈ لکھنا موجودہ نوڈز کو چھوئے بغیر آزاد ہے۔ AI وضاحت کرتا ہے کہ آپ ٹیلی اوپ/ٹرٹل سم کوڈ کو چھوئے بغیر ایک سبسکرائبر لکھیں گے۔

---

**پرامپٹ 3** (میسج کی اقسام):

اپنے AI سے پوچھیں:

```
I echoed /turtle1/cmd_vel and saw a Twist message with linear.x and angular.z.
What do the 6 fields of a Twist message represent in a real robot?
When would you use linear.y or linear.z in 3D robots?
```

**آپ کیا سیکھتے ہیں**: ٹویسٹ 6 DOF (6-ڈگری-آف-فریڈم) حرکت کے لیے ڈیزائن کیا گیا ہے۔ ٹرٹل سم صرف 2 استعمال کرتا ہے (لکیری آگے، اینگولر گردش)۔ حقیقی روبوٹس زیادہ فیلڈز استعمال کرتے ہیں۔

---

## اگلے لیسن سے پہلے چیک پوائنٹ

لیسن 3.4 (سروسز) سے پہلے، تصدیق کریں کہ آپ یہ کر سکتے ہیں:

- [ ] `ros2 node list` چلائیں اور `/teleop_turtle` اور `/turtlesim` دیکھیں
- [ ] `ros2 node info /turtlesim` چلائیں اور آؤٹ پٹ کو سمجھیں
- [ ] `ros2 topic list -t` چلائیں اور تمام فعال ٹاپکس دیکھیں
- [ ] `ros2 topic echo /turtle1/cmd_vel` چلائیں اور کچوے کو حرکت دیتے وقت ڈیٹا کا بہاؤ دیکھیں
- [ ] یہ سمجھیں کہ نوڈز فنکشن کالز کے بجائے ٹاپکس کے ذریعے بات چیت کرتے ہیں

اگر تمام چیک باکس پاس ہو جاتے ہیں، تو آپ لیسن 3.4 کے لیے تیار ہیں، جہاں آپ **سروسز**—درخواست/جواب کے تعاملات کے لیے ایک مختلف مواصلاتی پیٹرن—کی کھوج کریں گے۔

---

**اگلا لیسن**: [→ لیسن 3.4: سروسز اور پیرامیٹرز (CLI ایکسپلوریشن)](./04-services-parameters.md)