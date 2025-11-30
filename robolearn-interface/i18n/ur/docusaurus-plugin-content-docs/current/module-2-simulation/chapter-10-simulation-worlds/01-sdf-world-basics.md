---
id: lesson-10-1-sdf-world-basics
title: 'Lesson 10.1: SDF World Basics'
sidebar_position: 1
sidebar_label: 10.1 SDF World Basics
description: >-
  Understanding Simulation Description Format (SDF) for creating robot
  environments with ground planes, lighting, and physics configuration.
duration_minutes: 60
proficiency_level: A2
layer: L1
hardware_tier: 1
learning_objectives:
  - Explain the difference between URDF and SDF formats
  - Describe the structure of SDF world files
  - Create a basic SDF world with ground plane and lighting
  - Configure world physics properties including gravity and step size
skills:
  - sdf-world-creation
  - gazebo-configuration
cognitive_load:
  new_concepts: 6
tier_1_path: The Construct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 10.1: SDF ورلڈ کی بنیادی باتیں

آپ اب URDF کو سمجھ چکے ہیں—یہ بتانا کہ روبوٹ کا ڈھانچہ، جوڑ (joints)، اور فزکس (physics) کیسے بیان کیا جاتا ہے۔ لیکن ایک روبوٹ ایک ماحول (environment) میں موجود ہوتا ہے۔ اس سے پہلے کہ آپ کا روبوٹ حرکت کر سکے، اسے حرکت کرنے کے لیے ایک دنیا کی ضرورت ہے۔

یہ سبق **SDF** (Simulation Description Format) کا تعارف کراتا ہے، جو وہ زبان ہے جو Gazebo مکمل سمولیشن ماحول کو بیان کرنے کے لیے استعمال کرتا ہے۔ جہاں URDF ایک واحد روبوٹ کو بیان کرتا ہے، وہیں SDF پورے اسٹیج کو بیان کرتا ہے: زمین، آسمان، روشنی، فزکس کے اصول، اور دنیا میں موجود تمام اشیاء۔

## URDF بمقابلہ SDF کو سمجھنا

آپ پہلے بھی URDF دیکھ چکے ہیں (باب 9)۔ آئیے فرق واضح کرتے ہیں:

**URDF (Unified Robot Description Format)**
- صرف **ایک روبوٹ** کو بیان کرتا ہے
- لنکس (سخت اجسام) اور جوڑوں (کنکشنز) کی وضاحت کرتا ہے
- بصری جیومیٹری (visual geometry) اور ٹکراؤ کی شکلیں (collision shapes) شامل کرتا ہے
- فزکس کی خصوصیات (جیسے ماس، جڑتا، رگڑ) کو انکوڈ کرتا ہے
- **حد**: ماحول یا ایک سے زیادہ روبوٹس کو بیان نہیں کر سکتا
- **استعمال**: "یہ روبوٹ کیسا لگتا ہے اور یہ کیسے حرکت کرتا ہے؟"

**SDF (Simulation Description Format)**
- **مکمل دنیاؤں** کو بیان کرتا ہے (روبوٹ + ماحول + اشیاء + فزکس + روشنی)
- ایک سے زیادہ روبوٹس اور ماحولیاتی اشیاء شامل کر سکتا ہے
- دنیا کی سطح کی خصوصیات کی وضاحت کرتا ہے: کشش ثقل کی سمت، فزکس سٹیپ کا سائز، محیطی روشنی
- سورج/آسمان/روشنی کی ترتیب شامل کرتا ہے
- فزکس انجن کے انتخاب اور ٹیوننگ کی حمایت کرتا ہے
- **استعمال**: "اس روبوٹ کا پورا آپریٹنگ ماحول کیسا لگتا ہے؟"

**عملی تعلق**: آپ کے URDF میں بیان کردہ روبوٹ کو SDF میں بیان کردہ دنیا کے **اندر** رکھا جاتا ہے۔ دنیا (SDF) میں شامل کرنے سے پہلے روبوٹ کا خاکہ (URDF) موجود ہونا ضروری ہے۔

## SDF ورلڈ فائل کی ساخت

ایک SDF ورلڈ فائل XML ہوتی ہے۔ یہاں بنیادی ڈھانچہ دیا گیا ہے:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default_world">

    <!-- Physics configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane (floor) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

آئیے ہر حصے کی تفصیل سے وضاحت کریں:

### سیکشن 1: فزکس کنفیگریشن

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**یہ کیا کرتا ہے:**
- **type**: استعمال ہونے والا فزکس انجن ("dart"، "ode"، یا "bullet")
- **max_step_size**: ہر سمولیشن سٹیپ کتنا بڑا ہے (سیکنڈز میں)۔ چھوٹا = زیادہ درست لیکن سست۔ 0.001 سیکنڈ عام ہے۔
- **real_time_factor**: حقیقی وقت (wall-clock time) کے مقابلے میں رفتار۔ 1.0 کا مطلب ہے "حقیقی رفتار سے چلائیں"۔ 2.0 کا مطلب ہے "دو گنا تیز چلائیں"

### سیکشن 2: روشنی (Lighting)

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <direction>-0.5 0.5 -1</direction>
</light>
```

**یہ کیا کرتا ہے:**
- **type**: "directional" (سورج کی طرح)، "point" (لیمپ کی طرح)، یا "spot" (ٹارچ کی طرح)
- **pose**: روشنی کے ماخذ کی پوزیشن (x, y, z, رول، پچ، یاو)
- **direction**: جس سمت میں روشنی پڑ رہی ہے (ایک سمت ویکٹر کے طور پر)
- **cast_shadows**: کیا روشنی سائے بناتی ہے (زیادہ حقیقت پسندانہ لیکن سست)

### سیکشن 3: گراؤنڈ پلین (فرش)

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane>...</plane></geometry>
    </collision>
    <visual name="visual">
      <geometry><plane>...</plane></geometry>
    </visual>
  </link>
</model>
```

**یہ کیا کرتا ہے:**
- **model**: دنیا میں ایک نامزد کردہ شے (ground_plane فرش ہے)
- **static**: "true" کا مطلب ہے کہ یہ کبھی حرکت نہیں کرے گا (زمین اپنی جگہ رہے گی)
- **collision**: فزکس کی شکل (plane ایک لامحدود چپٹی سطح ہے)
- **visual**: کیمرے کو یہ کیسا نظر آتا ہے (ظاہری شکل)

## اپنی پہلی دنیا بنانا

آئیے ایک کم سے کم لیکن مکمل ورلڈ فائل بناتے ہیں۔ یہ سب سے سادہ درست SDF دنیا ہے:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">

    <!-- Physics: DART engine, 1ms steps, real-time speed -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Gravity: 9.81 m/s^2 downward (standard Earth) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Sky appearance -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun-like directional light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane (infinite flat floor) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**نتیجہ**: اگر آپ اسے `empty_world.sdf` کے طور پر محفوظ کرتے ہیں اور اسے Gazebo میں کھولتے ہیں، تو آپ دیکھیں گے:
- افق تک پھیلا ہوا ایک چپٹا سرمئی فرش
- ایک سمت والی روشنی (سورج) سے تیز روشنی
- اس دنیا میں آپ جو بھی اشیاء رکھیں گے وہ زمین جیسی کشش ثقل (نیچے کی طرف 9.81 m/s) کا تجربہ کریں گی

## کشش ثقل (Gravity) کو ترتیب دینا

کشش ثقل کو 3D ویکٹر (x, y, z) کے طور پر بیان کیا جاتا ہے۔ ہر جزو m/s² میں اس سمت میں ایکسلریشن ہے۔

**مثالیں:**

معیاری زمینی کشش ثقل (نیچے کی طرف):
```xml
<gravity>0 0 -9.81</gravity>
```

کم شدہ کشش ثقل (مریخ جیسی، زمین کا تقریباً 1/3):
```xml
<gravity>0 0 -3.71</gravity>
```

کشش ثقل نہیں (خلا):
```xml
<gravity>0 0 0</gravity>
```

غیر معمولی سمت میں کشش ثقل (خصوصی منظرناموں کے لیے):
```xml
<gravity>-9.81 0 0</gravity>  <!-- کشش ثقل منفی X کی طرف کھینچتی ہے -->
```

کشش ثقل کیوں اہم ہے؟ اگر آپ کا روبوٹ زمینی کشش ثقل کے لیے ڈیزائن کیا گیا ہے لیکن مریخ کی کشش ثقل کے ساتھ سمولیٹ کیا جاتا ہے، تو یہ مختلف طریقے سے برتاؤ کرے گا—بھاری قدم، مختلف توازن کی حرکیات۔ درست سمولیشن ٹو ریل ٹرانسفر کے لیے، جب تک کہ آپ کے پاس کوئی خاص وجہ نہ ہو، زمینی کشش ثقل استعمال کریں۔

## فزکس سٹیپ سائز اور رئیل ٹائم فیکٹر

دو اہم پیرامیٹرز سمولیشن کی درستگی اور رفتار کو کنٹرول کرتے ہیں:

**max_step_size**: ہر ٹائم سٹیپ کتنا بڑا ہے (سیکنڈز میں)
- 0.001s (1 ملی سیکنڈ) = درست لیکن سست
- 0.01s (10 ملی سیکنڈ) = زیادہ تر روبوٹکس کے لیے معیاری
- 0.1s (100 ملی سیکنڈ) = تیز لیکن شاید غیر درست

**real_time_factor**: حقیقی وقت کے مقابلے میں سمولیشن کی رفتار
- 1.0 = "حقیقی وقت" (1 سیکنڈ سمولیشن = 1 سیکنڈ حقیقی وقت)
- 2.0 = "2x رفتار" (1 سیکنڈ سمولیشن = 0.5 سیکنڈ حقیقی وقت)
- 0.5 = "سست حرکت" (1 سیکنڈ سمولیشن = 2 سیکنڈ حقیقی وقت)

اگر آپ کے روبوٹ کی فزکس غلط لگتی ہے (اشیاء عجیب طرح سے اچھلتی ہیں، روبوٹ غیر متوقع طور پر گر جاتے ہیں)، تو سٹیپ سائز اکثر قصوروار ہوتا ہے۔ چھوٹے سٹیپ سائز زیادہ درست ہوتے ہیں لیکن سست ہوتے ہیں۔ 0.001s سے شروع کریں اور ضرورت کے مطابق ایڈجسٹ کریں۔

## روشنی اور بصری معیار

SDF دنیاؤں میں روشنی کی کئی اقسام کی حمایت ہوتی ہے:

**سمت والی روشنی (Directional Light)** (سورج کی طرح)
```xml
<light type="directional" name="sun">
  <direction>-0.5 0.5 -1</direction>
  <diffuse>0.8 0.8 0.8 1</diffuse>
</light>
```

**نقطہ روشنی (Point Light)** (لیمپ یا بلب کی طرح)
```xml
<light type="point" name="lamp">
  <pose>0 0 2 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <attenuation>
    <range>5</range>
    <constant>1</constant>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

**اسپاٹ لائٹ (Spot Light)** (ٹارچ یا اسٹیج لائٹ کی طرح)
```xml
<light type="spot" name="spotlight">
  <pose>0 0 5 0 0 0</pose>
  <direction>0 0 -1</direction>
  <spot>
    <inner_angle>0.1</inner_angle>
    <outer_angle>1</outer_angle>
    <falloff>1</falloff>
  </spot>
</light>
```

رنگ کے اقدار RGBA (سرخ، سبز، نیلا، الفا) ہیں جو 0 سے 1 تک ہوتے ہیں:
- (1, 0, 0, 1) = سرخ
- (0, 1, 0, 1) = سبز
- (0, 0, 1, 1) = نیلا
- (1, 1, 1, 1) = سفید
- (0, 0, 0, 1) = سیاہ

## سب کو ایک ساتھ لانا: ایک مکمل کم سے کم دنیا

یہ ایک ورلڈ فائل ہے جسے آپ فوری طور پر کاپی اور استعمال کر سکتے ہیں:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_first_world">

    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**اگلا قدم:**
1. اس کوڈ کو کاپی کریں۔
2. اسے `my_first_world.sdf` کے نام سے محفوظ کریں۔
3. اسے Gazebo میں کھولیں: `gazebo my_first_world.sdf`
4. آپ کو روشنی اور سائے کے ساتھ ایک سرمئی فرش نظر آنا چاہیے

## اہم باتیں (Key Takeaways)

**SDF بمقابلہ URDF**: URDF روبوٹس کو بیان کرتا ہے، SDF ان دنیاؤں کو بیان کرتا ہے جن میں روبوٹس ہوتے ہیں۔

**ورلڈ کی ساخت**: ہر SDF فائل میں فزکس کنفیگریشن، روشنی، اور ایک گراؤنڈ پلین کی ضرورت ہوتی ہے۔

**فزکس پیرامیٹرز اہم ہیں**: سٹیپ سائز اور رئیل ٹائم فیکٹر درستگی اور رفتار کو کنٹرول کرتے ہیں۔ اپنی ضروریات کے مطابق انہیں سیٹ کریں۔

**کشش ثقل قابل ترتیب ہے**: حقیقت پسندانہ رویے کے لیے زمینی کشش ثقل (0, 0, -9.81) استعمال کریں۔ صرف ارادتاً اسے تبدیل کریں۔

**روشنی سمولیشن پر اثر انداز ہوتی ہے**: زیادہ لائٹس = زیادہ حقیقت پسندانہ لیکن سست۔ سادہ آغاز کریں (ایک سمت والی روشنی) اور ضرورت کے مطابق پیچیدگی شامل کریں۔

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: اپنا پسندیدہ AI ٹول (ChatGPT، Claude، یا اسی طرح کا) کھولیں اور پیسٹ کرنے کے لیے تیار ایک ورلڈ فائل (اوپر دیا گیا کوڈ) رکھیں۔

**پرامپٹ سیٹ**:

**پرامپٹ 1** (سمجھنا):
```
میں Gazebo دنیاؤں (SDF فارمیٹ) کے بارے میں سیکھ رہا ہوں۔ یہاں میری بنیادی ورلڈ فائل ہے:
[اوپر دی گئی empty_world.sdf پیسٹ کریں]

ہر بڑے حصے کا 2-3 جملوں میں وضاحت کریں کہ وہ کیا کرتا ہے۔
```

**متوقع نتیجہ**: AI سادہ زبان میں فزکس، روشنی، اور گراؤنڈ پلین کی وضاحت کرتا ہے۔

**پرامپٹ 2** (ترمیم):
```
اس دنیا میں میں مندرجہ ذیل تبدیلیاں کیسے کروں گا:
1. آدھی کشش ثقل (مریخ جیسی)
2. تیز فزکس سٹیپ سائز (تیز جانچ کے لیے)
3. مدھم روشنی (سورج کی کم چمک)

SDF فائل کے ترمیم شدہ حصے مجھے دکھائیں۔
```

**متوقع نتیجہ**: AI ترمیم شدہ XML حصے فراہم کرتا ہے جس میں مختصر وضاحتیں شامل ہیں۔

**پرامپٹ 3** (خرابیوں کا سراغ لگانا):
```
اگر میں نے یہ دنیا Gazebo میں کھولی اور اشیاء فرش سے گرتی رہیں، تو SDF فائل میں تین ممکنہ وجوہات کیا ہیں؟
ہر وجہ کے لیے، مجھے کیا چیک کرنا یا تبدیل کرنا چاہیے؟
```

**متوقع نتیجہ**: AI ممکنہ مسائل (غلط فزکس انجن، خراب ٹکراؤ جیومیٹری، غلط کشش ثقل) کی نشاندہی کرتا ہے اور اصلاحات تجویز کرتا ہے۔