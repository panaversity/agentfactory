---
id: lesson-9-1-understanding-urdf
title: 'Lesson 9.1: Understanding URDF'
sidebar_position: 1
sidebar_label: 9.1 Understanding URDF
description: >-
  Introduction to Unified Robot Description Format (URDF) for describing robot
  structure
chapter: 9
lesson: 1
duration_minutes: 60
proficiency_level: A2
layer: L1
cognitive_load:
  new_concepts: 6
learning_objectives:
  - Explain what URDF is and why it is needed in robotics
  - Identify the basic XML structure of a URDF file
  - Distinguish between links and joints in robot descriptions
  - Recognize visual and collision geometry definitions
  - Understand coordinate frames and transformations
  - Validate URDF syntax using basic tools
skills:
  - urdf-basics
hardware_tier: 1
tier_1_path: The Construct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 9.1: URDF کو سمجھنا

ایک روبوٹ صرف کوڈ نہیں ہوتا—یہ ایک جسمانی ڈھانچہ ہوتا ہے۔ اس سے پہلے کہ Gazebo آپ کے روبوٹ کی سمولیشن (simulation) کر سکے، اسے ایک خاکہ (blueprint) درکار ہوتا ہے۔ وہ خاکہ **URDF** میں لکھا جاتا ہے: Unified Robot Description Format۔

URDF ایک XML زبان ہے۔ اسے دیکھ کر گھبرائیں نہیں۔ XML صرف ٹیگز کے ساتھ ایک منظم متن (structured text) ہے۔ URDF کو روبوٹ کے لیے ایک ترکیب (recipe) سمجھیں: آپ اجزاء (links—سخت حصے) کی فہرست بناتے ہیں، وضاحت کرتے ہیں کہ انہیں کیسے جوڑنا ہے (joints—کنکشنز)، اور ان کی ظاہری شکل (geometry) بتاتے ہیں۔ سمولیٹر اس ترکیب کو پڑھتا ہے اور آپ کا روبوٹ تیار کر دیتا ہے۔

اس سبق میں، آپ سیکھیں گے کہ URDF کیا ہے، یہ کیوں اہم ہے، اور URDF فائلوں کو کیسے پڑھا جاتا ہے۔ آپ ابھی پیچیدہ URDF نہیں لکھیں گے—آپ تصورات کو سمجھیں گے تاکہ جب آپ انہیں سبق 9.2 میں لکھیں تو وہ بامعنی لگیں۔

---

## ہمیں URDF کی ضرورت کیوں ہے؟

تصور کریں کہ آپ حقیقی دنیا میں ایک روبوٹ بنا رہے ہیں۔ آپ یہ بتائیں گے:
- کون سے پرزے موجود ہیں: "میرے پاس ایک چیسس، دو پہیے، اور ایک کیمرہ ہے"
- وہ کیسے جڑتے ہیں: "ہر پہیہ چیسس سے ایک گھومنے والے جوڑ (revolving joint) کے ذریعے جڑا ہوا ہے"
- وہ کیسے نظر آتے ہیں: "چیسس ایک مستطیل ڈبہ ہے؛ پہیے سلنڈر ہیں"
- ان کا وزن کتنا ہے: "چیسس کا وزن 5 کلوگرام ہے؛ ہر پہیے کا وزن 0.5 کلوگرام ہے"

URDF اس تمام معلومات کو ایک منظم، مشین کے پڑھنے کے قابل فارمیٹ میں محفوظ کرتا ہے۔ سمولیٹر URDF فائل کو پڑھتا ہے اور آپ کے روبوٹ کے ڈھانچے کو فوری طور پر سمجھ جاتا ہے۔

**URDF کے بغیر:** آپ اپنے روبوٹ کو سادہ اردو میں بیان کریں گے، اور سمولیٹر اسے نہیں سمجھے گا۔

**URDF کے ساتھ:** سمولیٹر خود بخود آپ کا روبوٹ بناتا ہے، حساب لگاتا ہے کہ وہ کہاں حرکت کر سکتا ہے، فزکس کی قوتوں کو درست طریقے سے لاگو کرتا ہے، اور اسے اسکرین پر دکھاتا ہے۔

---

## بنیادی تصور 1: لنکس (سخت اجسام)

ایک **link** ایک سخت جسم (rigid body) ہوتا ہے—آپ کے روبوٹ کا وہ حصہ جو جھکتا یا تبدیل نہیں ہوتا۔ مثالیں:
- چیسس (مرکزی جسم)
- پہیہ
- بازو کا حصہ (Arm segment)
- کیمرہ ماؤنٹ

URDF میں، ایک لنک میں یہ چیزیں ہوتی ہیں:
- ایک **نام** (جس سے آپ اسے اپنے کوڈ میں پکارتے ہیں)
- **بصری جیومیٹری (Visual geometry)** (یہ کیسا لگتا ہے: شکل، سائز، رنگ)
- **ٹکراؤ کی جیومیٹری (Collision geometry)** (فزکس انجن اسے کس شکل سمجھتا ہے)
- **جڑتا خصوصیات (Inertial properties)** (ماس، وزن کیسے تقسیم ہوتا ہے)

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
  </inertial>
</link>
```

**اہم نکتہ:** ہر روبوٹ میں کم از کم ایک لنک ہوتا ہے جسے `base_link` کہا جاتا ہے۔ یہ حوالہ فریم (reference frame) ہے—وہ اصل نقطہ جہاں سے پیمائشیں شروع ہوتی ہیں۔

---

## بنیادی تصور 2: جوڑ (کنکشنز)

ایک **joint** دو لنکس کو جوڑتا ہے۔ یہ بتاتا ہے کہ ایک لنک دوسرے کے مقابلے میں کیسے حرکت کر سکتا ہے۔

جوڑوں کی اقسام:
- **revolute**: ایک محور (axis) کے گرد گھومتا ہے (جیسے پہیہ گھومتا ہے)
- **continuous**: لامحدود گردش (unlimited rotation) (جیسے گھومنے والا موٹر)
- **prismatic**: ایک محور کے ساتھ سلائیڈ کرتا ہے (جیسے لکیری ایکچویٹر)
- **fixed**: حرکت نہیں کر سکتا (سختی سے جڑا ہوا)

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.15 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

یہ جوڑ بتاتا ہے:
- یہ `base_link` (والد/parent) کو `left_wheel` (بچہ/child) سے جوڑتا ہے
- بائیں پہیے کی پوزیشن چیسس سے 0.15 میٹر بائیں طرف ہے
- یہ Y محور کے گرد مسلسل گھوم سکتا ہے

**اہم نکتہ:** جوڑ سمت دار (directional) ہوتے ہیں۔ پیرنٹ لنک حوالہ ہوتا ہے؛ چائلڈ لنک اس کے نسبت حرکت کرتا ہے۔

---

## بنیادی تصور 3: کوآرڈینیٹ فریمز

ہر لنک کا ایک **کوآرڈینیٹ فریم** ہوتا ہے—ایک اصل نقطہ (0, 0, 0) اور تین محور (X, Y, Z)۔

```
      Z (اوپر)
      ^
      |
      o----- X (آگے)
     /
    Y (بائیں)
```

جب آپ کسی جوڑ کا `origin` بتاتے ہیں، تو آپ کہہ رہے ہوتے ہیں:
- `xyz="0 0.15 0"`: چائلڈ لنک کا اصل نقطہ Y محور کے ساتھ 0.15 میٹر دور ہے
- `rpy="0 0 0"`: کوئی گردش نہیں (Pitch=0, Roll=0, Yaw=0)

یہ نظام آپ کو پیچیدہ روبوٹس کو بغیر کسی ابہام کے بیان کرنے دیتا ہے۔

---

## بنیادی تصور 4: بصری بمقابلہ ٹکراؤ کی جیومیٹری

**بصری جیومیٹری (Visual geometry)** وہ ہے جو انسان دیکھتے ہیں۔ یہ خوبصورت نظر آتی ہے۔

**ٹکراؤ کی جیومیٹری (Collision geometry)** وہ ہے جسے فزکس انجن استعمال کرتا ہے۔ تیز سمولیشن کے لیے اسے آسان بنایا جا سکتا ہے۔

مثال: ایک روبوٹ آرم میں ایک تفصیلی 3D ماڈل (بصری) ہو سکتا ہے لیکن ٹکراؤ کے لیے سادہ ڈبوں (boxes) کا استعمال کیا جا سکتا ہے (تیز فزکس حسابات کے لیے)۔

```xml
<link name="arm_link">
  <visual>
    <mesh filename="arm_detailed.stl"/>
  </visual>
  <collision>
    <box size="0.1 0.05 0.3"/>
  </collision>
</link>
```

---

## ایک سادہ URDF فائل پڑھنا

آئیے ایک مکمل کم سے کم روبوٹ پر نظر ڈالتے ہیں:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- بیس لنک: چیسس -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- بائیں پہیا -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
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
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- بائیں پہیا جوڑ -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- دائیں پہیا جوڑ -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

**یہ URDF کیا بیان کرتا ہے:**
- تین لنکس والا روبوٹ: بیس (چیسس) اور دو پہیے
- بائیں پہیا چیسس سے 0.15 میٹر بائیں طرف (+Y سمت) رکھا گیا ہے
- دائیں پہیا چیسس سے 0.15 میٹر دائیں طرف (-Y سمت) رکھا گیا ہے
- دونوں پہیے مسلسل گھوم سکتے ہیں (موٹریں)
- `rpy="1.5708 0 0"` پہیے کو 90 ڈگری گھماتا ہے تاکہ وہ صحیح سمت میں نظر آئیں

**جب آپ اسے Gazebo میں لوڈ کرتے ہیں:**
```
Expected output:
[INFO] Loaded robot with 3 links and 2 joints
[INFO] robot_name: simple_robot
[INFO] Links: base_link, left_wheel, right_wheel
[INFO] Joints: left_wheel_joint, right_wheel_joint
```

---

## XML ٹیگز کو سمجھنا

URDF XML ہے، جس کا مطلب ہے کہ ہر چیز ٹیگز میں لپٹی ہوئی ہے: `<tag>content</tag>`

بنیادی ڈھانچہ:
- `<robot>`: روٹ ایلیمنٹ، ہر چیز کو لپیٹتا ہے
- `<link>`: ایک سخت جسم کی وضاحت کرتا ہے
- `<joint>`: دو لنکس کو جوڑتا ہے
- `<geometry>`: شکل کی وضاحت کرتا ہے
- `<visual>`: یہ کیسا لگتا ہے
- `<collision>`: فزکس اسے کیسے سمجھتا ہے

ایٹریبیوٹس (Attributes) ٹیگز کے اندر کی خصوصیات ہیں:
```xml
<joint name="my_joint" type="continuous">
```
یہاں، `name` اور `type` ایٹریبیوٹس ہیں۔

**اہم پیٹرن:**
- ہر ایٹریبیوٹ کوٹیشن مارکس میں جاتا ہے: `name="value"`
- ٹیگز کو مناسب طریقے سے بند ہونا چاہیے: `<tag>content</tag>` یا `<tag/>`
- انڈنٹیشن انسانوں کو پڑھنے میں مدد دیتی ہے لیکن کمپیوٹر کے لیے اس کی کوئی اہمیت نہیں ہوتی

---

## اپنی سمجھ کی جانچ کریں

URDF لکھنے کی طرف بڑھنے سے پہلے، یقینی بنائیں کہ آپ جواب دے سکتے ہیں:

1. **لنک کیا ہے؟** (ایک سخت جسم—آپ کے روبوٹ کا ایک حصہ)
2. **جوڑ کیا ہے؟** (دو لنکس کے درمیان ایک کنکشن، جو بتاتا ہے کہ وہ کیسے حرکت کرتے ہیں)
3. **بصری اور ٹکراؤ کی جیومیٹری کو الگ کیوں کیا جاتا ہے؟** (بصری خوبصورت دکھنے کے لیے؛ ٹکراؤ تیز حسابات کے لیے)
4. **`base_link` کا کیا مطلب ہے؟** (حوالہ فریم، روبوٹ کا اصل نقطہ)
5. **اگر جوڑ میں `<axis>` ایلیمنٹ شامل کرنا بھول جائیں تو کیا ہوگا؟** (سمولیٹر کو پتہ نہیں چلے گا کہ کس سمت میں گھومنا ہے)

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ:** اپنا پسندیدہ AI ٹول (ChatGPT, Claude, وغیرہ) کھولیں اور URDF کے بارے میں گفتگو کریں۔

**پرامپٹ سیٹ:**

```
Prompt 1: "URDF فائل کیا ہے، یہ سمجھاتے ہوئے جیسے آپ کسی ابتدائی طالب علم کو سکھا رہے ہوں۔
مجھے سمجھانے کے لیے ایک مثال (جیسے کھانا پکانا، تعمیر کرنا، وغیرہ) استعمال کریں۔"

Prompt 2: "ایک سادہ پہیے والے روبوٹ کے لیے URDF فائل دکھائیں۔
پھر ہر حصے کی وضاحت کریں—ہر ٹیگ کیا کرتا ہے؟"

Prompt 3: "اگر میرے روبوٹ میں 2 کے بجائے 4 پہیے ہوں،
تو URDF میں کیا تبدیلی آئے گی؟ کوڈ دکھائیں۔"
```

**متوقع نتائج:**
- AI کو URDF کو خاکہ یا ترکیب کے طور پر سمجھانا چاہیے
- کوڈ کی مثال میں پہیوں کے لیے لنکس اور ہر پہیے کے لیے جوڑ شامل ہونا چاہیے
- آپ کو مختلف پہیوں کی تعداد کے لیے URDF میں ترمیم کرنے کا طریقہ سمجھ آنا چاہیے

**حفاظتی نوٹ:** URDF صرف ڈیٹا ہے—یہ ڈھانچے کی وضاحت کرتا ہے لیکن ابھی تک کسی چیز کو کنٹرول نہیں کرتا۔ صرف URDF سے آپ کچھ بھی خراب نہیں کر سکتے۔ جب ہم فزکس (سبق 9.3) اور موٹریں (باب 12) شامل کریں گے، تب حفاظت اہم ہو جائے گی۔

**اختیاری توسیع:** اپنے AI ٹول سے پوچھیں: "URDF اور SDF میں کیا فرق ہے؟ آپ کس وقت کس کا استعمال کریں گے؟" (آپ Gazebo میں دنیا بناتے وقت SDF کا سامنا کریں گے۔)