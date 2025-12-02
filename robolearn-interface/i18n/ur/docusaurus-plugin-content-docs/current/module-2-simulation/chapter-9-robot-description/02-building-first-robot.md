---
id: lesson-9-2-building-first-robot
title: 'Lesson 9.2: Building Your First Robot'
sidebar_position: 2
sidebar_label: 9.2 Building First Robot
description: Create a simple two-wheeled mobile robot from scratch using URDF
chapter: 9
lesson: 2
duration_minutes: 75
proficiency_level: A2
layer: L1
cognitive_load:
  new_concepts: 7
learning_objectives:
  - Create a complete URDF file for a two-wheeled robot
  - 'Define chassis, wheels, and caster link properly'
  - Configure continuous joints for wheel rotation
  - Set correct coordinate frame origins and rotations
  - Validate URDF syntax
  - Load and visualize robot in Gazebo
  - Debug common URDF errors
skills:
  - urdf-basics
  - robot-modeling
hardware_tier: 1
tier_1_path: The Construct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 9.2: اپنا پہلا روبوٹ بنانا

اب آپ URDF کے تصورات (concepts) سمجھ چکے ہیں۔ وقت ہے کہ آپ اپنی پہلی مکمل روبوٹ تفصیل (description) لکھیں۔

آپ ایک **ڈیفرنشل-ڈرائیو روبوٹ** بنائیں گے—جو سب سے سادہ موبائل پلیٹ فارم ہے۔ دو پاورڈ پہیے حرکت کو کنٹرول کرتے ہیں: سیدھا جانے کے لیے دونوں کو آگے گھمائیں، بائیں مڑنے کے لیے بائیں پہیے کو تیز گھمائیں۔ استحکام (stability) کے لیے ایک کاسٹر وہیل شامل کریں، اور آپ کے پاس ایک کام کرنے والا موبائل بیس تیار ہے۔

یہ عملی کام ہے۔ آپ شروع سے URDF لکھیں گے، اسے Gazebo میں لوڈ کریں گے، اپنی اسکرین پر اپنا روبوٹ دیکھیں گے، اور جب کچھ غلط ہو تو اسے درست کریں گے۔ آخر تک، آپ ایک حقیقی، فعال روبوٹ تفصیل لکھ چکے ہوں گے۔

---

## روبوٹ کا ڈیزائن

آپ کا روبوٹ:
- **چیسس (Chassis)**: مرکزی جسم کی نمائندگی کرنے والا ایک مستطیل ڈبہ (30 سینٹی میٹر × 20 سینٹی میٹر × 10 سینٹی میٹر)
- **بایاں پہیا (Left wheel)**: بائیں جانب ایک سلنڈر (5 سینٹی میٹر رداس، 5 سینٹی میٹر چوڑا)
- **دائیں پہیا (Right wheel)**: دائیں جانب ایک سلنڈر (5 سینٹی میٹر رداس، 5 سینٹی میٹر چوڑا)
- **کاسٹر پہیا (Caster wheel)**: توازن کے لیے پیچھے ایک چھوٹا گولا (2.5 سینٹی میٹر رداس)

یہ ایک کلاسک ڈیفرنشل-ڈرائیو ڈیزائن ہے جو TurtleBot سے لے کر کسٹم بلڈز تک روبوٹس میں استعمال ہوتا ہے۔

---

## مرحلہ 1: URDF فائل بنائیں

The Construct کلاؤڈ ماحول میں، ایک نئی فائل بنائیں:

**فائل کا نام:** `my_first_robot.urdf`

**مقام:** آپ کا ROS 2 ورک اسپیس، غالباً `src/my_package/urdf/` میں

بنیادی ڈھانچے سے آغاز کریں:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- ہم یہاں مواد شامل کریں گے -->

</robot>
```

**ٹائپ کرنے کے بعد متوقع آؤٹ پٹ:**
```
File created successfully: ~/catkin_ws/src/my_package/urdf/my_first_robot.urdf
```

---

## مرحلہ 2: بیس لنک (چیسس) شامل کریں

چیسس ایک سخت ڈبہ ہے۔ اسے `<robot>` ٹیگز کے اندر شامل کریں:

```xml
  <!-- چیسس: روبوٹ کا مرکزی جسم -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
  </link>
```

**یہ کیا کرتا ہے:**
- `base_link` نام کا ایک لنک متعین کرتا ہے
- بصری (Visual): ایک نیلا ڈبہ 30 سینٹی میٹر لمبا، 20 سینٹی میٹر چوڑا، 10 سینٹی میٹر اونچا
- ٹکراؤ (Collision): ایک ہی ڈبے کی شکل (فزکس انجن اسے اسی طرح سمجھتا ہے)

**لوڈ ہونے پر متوقع آؤٹ پٹ:**
آپ کو Gazebo میں ایک نیلا مستطیل ڈبہ نظر آئے گا۔

---

## مرحلہ 3: بایاں پہیا شامل کریں

```xml
  <!-- بایاں پہیا -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>
```

**یہ کیا کرتا ہے:**
- ایک سیاہ پہیا (سلنڈر) متعین کرتا ہے
- 5 سینٹی میٹر رداس، 5 سینٹی میٹر چوڑا (لمبائی)

**لوڈ ہونے پر متوقع آؤٹ پٹ:**
آپ کو ایک سیاہ سلنڈر نظر آئے گا، لیکن جب تک ہم اسے جوڑ (joint) سے نہیں جوڑتے یہ ہوا میں تیرتا رہے گا۔

---

## مرحلہ 4: دائیں پہیا شامل کریں

```xml
  <!-- دائیں پہیا -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>
```

بائیں پہیے جیسا ہی ہے۔

---

## مرحلہ 5: کاسٹر پہیا شامل کریں

```xml
  <!-- توازن کے لیے کاسٹر پہیا -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
  </link>
```

**یہ کیا کرتا ہے:**
- کاسٹر کے لیے ایک چھوٹا سرمئی گولا (2.5 سینٹی میٹر رداس)
- کوئی موٹر نہیں—یہ صرف آزادانہ طور پر گھومتا ہے

---

## مرحلہ 6: بائیں پہیے کو جوڑ (Joint) سے جوڑیں

اب بائیں پہیے کو چیسس سے جوڑیں۔ تمام لنک تفصیلات کے بعد اسے شامل کریں:

```xml
  <!-- بایاں پہیا جوڑ -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
```

**یہ کیا کرتا ہے:**
- جوڑ کا نام: `left_wheel_joint`
- قسم: `continuous` (ہمیشہ گھوم سکتا ہے—ایک موٹر)
- والدین (Parent): `base_link` (چیسس)
- بچہ (Child): `left_wheel` (جسے ہم جوڑ رہے ہیں)
- اصل (Origin): چیسس کے مرکز کے حوالے سے (x=0, y=0.15, z=-0.05) پر پوزیشن
  - Y=0.15 اسے 15 سینٹی میٹر بائیں طرف رکھتا ہے
  - Z=-0.05 اسے چیسس کے نچلے حصے میں رکھتا ہے
- `rpy="1.5708 0 0"`: پہیے کو 90 ڈگری (1.5708 ریڈین = 90°) گھماتا ہے تاکہ یہ سائیڈ ویز ہو
- محور (Axis): `0 0 1` کا مطلب ہے کہ یہ Z محور کے گرد گھومتا ہے

**لوڈ ہونے پر متوقع آؤٹ پٹ:**
بایاں پہیا چیسس کی بائیں جانب منسلک نظر آئے گا، جو آگے کی طرف گھومنے کے لیے صحیح پوزیشن میں ہے۔

---

## مرحلہ 7: دائیں پہیے کو جوڑ سے جوڑیں

```xml
  <!-- دائیں پہیا جوڑ -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
```

**بائیں پہیے سے فرق:**
- Y=-0.15 اسے 15 سینٹی میٹر دائیں طرف (دوسری طرف) رکھتا ہے

---

## مرحلہ 8: کاسٹر پہیے کو جوڑ سے جوڑیں

```xml
  <!-- کاسٹر پہیا جوڑ (گرنے سے روکنے کے لیے مقرر) -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.1 0 -0.075" rpy="0 0 0"/>
  </joint>
```

**یہ کیا کرتا ہے:**
- قسم: `fixed` (حرکت نہیں کر سکتا، سختی سے منسلک)
- پوزیشن: x=-0.1 (پیچھے کی طرف)، z=-0.075 (نیچے کی طرف)
- چونکہ یہ گھومتا نہیں ہے اس لیے `<axis>` عنصر کی ضرورت نہیں ہے۔

---

## مکمل URDF فائل

یہ پوری فائل ہے جس میں تمام حصے اکٹھے ہیں:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- چیسس: مرکزی جسم -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- بایاں پہیا -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- دائیں پہیا -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- کاسٹر پہیا -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
  </link>

  <!-- بایاں پہیا جوڑ -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- دائیں پہیا جوڑ -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- کاسٹر پہیا جوڑ -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.1 0 -0.075" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## مرحلہ 9: توثیق (Validate) کریں اور Gazebo میں لوڈ کریں

اپنی فائل محفوظ کریں۔ پھر سنٹیکس کی غلطیوں کے لیے چیک کریں:

```bash
check_urdf my_first_robot.urdf
```

**درست ہونے پر متوقع آؤٹ پٹ:**
```
robot name is: my_robot
---------- Link my_robot ----------
parent
child(1): base_link
---------- Link base_link ----------
parent
child(3): left_wheel, right_wheel, caster_wheel
---------- Link left_wheel ----------
parent: base_link
---------- Link right_wheel ----------
parent: base_link
---------- Link caster_wheel ----------
parent: base_link
```

اس کا مطلب ہے کہ آپ کا URDF سنٹیکس کے لحاظ سے درست ہے اور ڈھانچہ (structure) سمجھ میں آتا ہے۔

**اگر آپ کو غلطیاں نظر آئیں:**
- چیک کریں کہ XML ٹیگ ٹھیک سے بند ہیں
- تصدیق کریں کہ تمام `<link>` اور `<joint>` کے نام منفرد (unique) ہیں
- یقینی بنائیں کہ `<parent>` اور `<child>` کے حوالہ جات موجود ہیں

پھر Gazebo میں لوڈ کریں:

```bash
gazebo --verbose -u -g libgazebo_ros_init.so my_first_robot.urdf
```

**متوقع بصری آؤٹ پٹ:**
آپ دیکھیں گے:
- مرکز میں ایک نیلا مستطیل چیسس
- اطراف میں دو سیاہ پہیے
- پیچھے ایک چھوٹا سرمئی گولا

روبوٹ زمین پر بیٹھا ہے، سمولیشن کے لیے تیار ہے۔

---

## عام غلطیاں اور انہیں کیسے ٹھیک کریں

| غلطی | علامت (Symptom) | حل (Fix) |
|---------|---------|-----|
| مسلسل جوڑ میں `<axis>` غائب ہونا | جوڑ گھومتا ہے لیکن کچھ بھی گھومتا نہیں | `<axis xyz="0 0 1"/>` شامل کریں |
| `rpy` میں غلط گردش کی قدریں | پہیے پیچھے کی طرف اشارہ کر رہے ہیں | سائیڈ ویز سلنڈر کے لیے `rpy="1.5708 0 0"` استعمال کریں |
| منفی Z قدریں غلط ہونا | پہیے زمین سے اوپر تیر رہے ہیں | پہیے کی اونچائی پر رکھنے کے لیے `z=-0.05` استعمال کریں |
| `<link>` بند کرنے کا ٹیگ غائب ہونا | XML پارس کی غلطی | ہر `<link>` کو `</link>` کے ساتھ بند ہونا چاہیے |
| جوڑ کا والدین/بچہ موجود نہیں | لنک غائب ہونے کی غلطی | تمام لنک کے ناموں کی ہجے (spelling) درست چیک کریں |

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ:** اب آپ کے پاس ایک کام کرنے والا URDF ہے۔ اسے بہتر بنانے کے لیے AI کے ساتھ تبدیلی (iterate) کرتے ہیں۔

**پرامپٹ سیٹ:**

```
Prompt 1: "Here's my URDF for a two-wheeled robot.
[paste your complete URDF file]
What could go wrong when I use this in Gazebo?
What improvements would you suggest?"

Prompt 2: "How would I modify this URDF to add a camera
mounted on the front of the chassis? Show me the XML
I need to add."

Prompt 3: "I want to add a fourth wheel (another caster at the front).
Show me how to define the link and joint for it."
```

**متوقع نتائج:**
- AI ممکنہ مسائل کی نشاندہی کرے گا (مثلاً سبق 9.3 سے فزکس پراپرٹیز غائب ہونا)
- کیمرے کے لیے کوڈ میں ایک نیا لنک اور فکسڈ جوائنٹ شامل ہونا چاہیے
- چوتھا کاسٹر پہلے کاسٹر جیسا ہی پیٹرن فالو کرے گا

**حفاظتی نوٹ:** اس مرحلے پر، URDF صرف جیومیٹری ہے—آپ کا روبوٹ ابھی تک حرکت نہیں کرتا۔ باب 12 میں، آپ اسے ROS 2 کنٹرولرز سے جوڑیں گے جو دراصل حرکت کا حکم دیتے ہیں۔ حفاظتی غور و خوض تب آتے ہیں جب موٹریں شامل ہوتی ہیں۔

**اختیاری توسیع (Optional Stretch):** پوچھیں: "میں Gazebo میں اپنے روبوٹ میں کلر میٹیریلز کیسے شامل کروں؟" یا " `<visual>` اور `<collision>` میں کیا فرق ہے اور مجھے دونوں کی ضرورت کیوں ہے؟"