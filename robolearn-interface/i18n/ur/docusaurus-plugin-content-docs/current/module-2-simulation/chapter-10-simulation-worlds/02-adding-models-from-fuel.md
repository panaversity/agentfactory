---
id: lesson-10-2-adding-models-from-fuel
title: 'Lesson 10.2: Adding Models from Fuel'
sidebar_position: 2
sidebar_label: 10.2 Models from Fuel
description: >-
  Using Gazebo Fuel to populate worlds with pre-built 3D models, positioning,
  and scaling.
duration_minutes: 60
proficiency_level: A2
layer: L1
hardware_tier: 1
learning_objectives:
  - Navigate and search the Gazebo Fuel model repository
  - Include Fuel models in SDF world files using proper URI syntax
  - Position and scale imported models correctly in simulation space
  - Compose complete worlds by combining multiple Fuel models
skills:
  - gazebo-fuel
  - world-composition
cognitive_load:
  new_concepts: 5
tier_1_path: The Construct cloud environment + web browser
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 10.2: فیول سے ماڈلز شامل کرنا

ہر 3D آبجیکٹ (جیسے جیومیٹری، فزکس، ظاہری شکل) کو شروع سے بنانا ایک کمرے کے لیے بھی کئی دن لے سکتا ہے۔ اس کے بجائے، **Gazebo Fuel** (https://app.gazebosim.org/fuel) ہزاروں پہلے سے بنے ہوئے 3D ماڈلز کا ایک آن لائن ذخیرہ ہے جو روبوٹکس کمیونٹی نے فراہم کیے ہیں۔

Gazebo Fuel میں میزیں، کرسیاں، عمارتیں، پودے، ٹریفک کونز، روبوٹس، اور بہت کچھ شامل ہے—یہ سب استعمال کرنے کے لیے مفت ہیں۔ یہ سبق آپ کو ان ماڈلز کو تلاش کرنے، شامل کرنے اور اپنے SDF دنیاؤں میں ان کی پوزیشن مقرر کرنے کا طریقہ سکھاتا ہے۔

## Gazebo Fuel کیا ہے؟

Gazebo Fuel 3D ماڈلز کا ایک عوامی ذخیرہ ہے جو روبوٹکس سمولیشن کے لیے بہتر (optimized) بنائے گئے ہیں۔ ماڈلز کو کیٹیگری، معیار، اور لائسنسنگ کے لحاظ سے منظم کیا گیا ہے۔ زیادہ تر ماڈلز مفت اور اوپن سورس ہیں۔

**اہم خصوصیات:**
- نام، کیٹیگری، یا ٹیگ کے ذریعے تلاش کے قابل
- ماڈلز میں فزکس پراپرٹیز، ٹکراؤ کی شکلیں (collision shapes)، اور بصری میشز (visual meshes) شامل ہیں
- ہر ماڈل کا SDF فائلوں میں شامل کرنے کے لیے ایک منفرد URI (ویب ایڈریس) ہوتا ہے
- کمیونٹی سے تعاون یافتہ: دنیا بھر کے روبوٹک ماہرین کے ہزاروں ماڈلز
- معیار مختلف ہوتا ہے: کچھ پروڈکشن-گریڈ کے ہوتے ہیں، دوسرے تعلیمی ہوتے ہیں

**رسائی**: https://app.gazebosim.org/fuel

## Gazebo Fuel میں ماڈلز تلاش کرنا

https://app.gazebosim.org/fuel/models پر جائیں۔ آپ کو دستیاب ماڈلز کی ایک گرڈ نظر آئے گی۔

**کسی مخصوص ماڈل کو تلاش کرنے کے لیے:**
1. سرچ باکس (اوپر دائیں کونے میں) استعمال کریں
2. ماڈل کا نام ٹائپ کریں (مثلاً، "table"، "chair"، "lamp")
3. تفصیلات دیکھنے کے لیے کسی نتیجے پر کلک کریں

**مثال تلاش کے نتائج:**

"table" تلاش کریں:
- OpenRobotics کا "Table"
- user123 کا "Standing Desk"
- OpenRobotics کا "Picnic Table"
- user456 کا "Conference Table"

ہر ایک میں ایک تھمب نیل (پیش منظر کی تصویر)، تخلیق کار کا نام، اور ڈاؤن لوڈ کی تعداد ہوتی ہے۔

**کسی مخصوص ماڈل کا URI تلاش کرنے کے لیے:**
1. ماڈل پر کلک کریں
2. "URI" فیلڈ یا کاپی بٹن تلاش کریں
3. URI کی شکل یہ ہے: `https://fuel.gazebosim.org/1.0/[user]/models/[model-name]`

**مثال URIs:**

معیاری میز:
```
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master
```

آفس کرسی:
```
https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master
```

## SDF میں فیول ماڈلز شامل کرنا

ایک بار جب آپ کے پاس ماڈل کا URI ہو جائے، تو اسے اپنی SDF دنیا میں شامل کرنا آسان ہے۔ `<include>` ٹیگ استعمال کریں:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>0 0 0 0 0 0</pose>
  <name>my_table</name>
</include>
```

**include ٹیگ کی وضاحت:**

- **uri**: فیول ماڈل کا مکمل ویب ایڈریس
- **pose**: پوزیشن اور گردش (x y z رول پچ یا yaw)
- **name**: اس مثال کے لیے ایک منفرد نام (آپ مختلف ناموں کے ساتھ ایک ہی ماڈل کو کئی بار شامل کر سکتے ہیں)

### Pose: پوزیشن اور گردش

pose ٹیگ 6 اقدار استعمال کرتا ہے: **x y z رول پچ یا yaw**

**پوزیشن (پہلی 3 اقدار: x y z)**
- x: آگے/پیچھے (میٹر میں)
- y: بائیں/دائیں (میٹر میں)
- z: اوپر/نیچے (میٹر میں)

مثال: `<pose>1 2 0.8 0 0 0</pose>` ماڈل کو 1m آگے، 2m دائیں، 0.8m اوپر رکھتا ہے، بغیر کسی گردش کے۔

**گردش (آخری 3 اقدار: رول پچ یا yaw ریڈینز میں)**
- رول: x-axis کے گرد گردش (0 سے 2π ≈ 0 سے 6.28)
- پچ: y-axis کے گرد گردش
- یاو (Yaw): z-axis کے گرد گردش (عمودی محور کے گرد گردش)

ابھی کے لیے، گردش کو 0 پر رکھیں۔ گردش کو ریڈینز میں ماپا جاتا ہے:
- 0 ریڈین = کوئی گردش نہیں
- π/2 ریڈین ≈ 1.57 ≈ 90 ڈگری
- π ریڈین ≈ 3.14 ≈ 180 ڈگری
- 2π ریڈین ≈ 6.28 ≈ 360 ڈگری

گردش کے ساتھ مثال:
```xml
<pose>2 0 0.8 0 0 1.57</pose>
```
یہ ماڈل کو 2m آگے، 0.8m اوپر رکھتا ہے، اور اسے عمودی محور (yaw) کے گرد 90 ڈگری گھماتا ہے۔

## مثال: ایک سادہ کمرہ بنانا

آئیے فیول ماڈلز کا استعمال کرتے ہوئے میز اور دو کرسیوں والا ایک کمرہ بناتے ہیں۔ یہ مکمل SDF ہے:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="furnished_room">

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
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane -->
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

    <!-- Table from Gazebo Fuel -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>0 0 0 0 0 0</pose>
      <name>dining_table</name>
    </include>

    <!-- Chair 1 (left side of table) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>-0.5 0 0 0 0 0</pose>
      <name>chair_left</name>
    </include>

    <!-- Chair 2 (right side of table) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>0.5 0 0 0 0 3.14</pose>
      <name>chair_right</name>
    </include>

  </world>
</sdf>
```

**یہ کیا بناتا ہے:**
- اصل مقام (0, 0, 0) پر ایک میز
- میز کے بائیں طرف 0.5m پر ایک کرسی
- میز کے دائیں طرف 0.5m پر ایک کرسی، جسے 180 ڈگری گھمایا گیا ہے (تاکہ شخص دوسری طرف منہ کرے)

**آؤٹ پٹ**: Gazebo میں، آپ کو ایک فرنیچر والا منظر نظر آئے گا: ایک میز جس کے سامنے دو کرسیاں ہیں۔

## ماڈلز کو اسکیل کرنا (بڑا یا چھوٹا کرنا)

کچھ ماڈلز آپ کی دنیا کے لیے بہت بڑے یا بہت چھوٹے ہو سکتے ہیں۔ آپ `<scale>` ٹیگ استعمال کر کے انہیں اسکیل کر سکتے ہیں:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>0 0 0 0 0 0</pose>
  <name>small_table</name>
  <scale>0.5 0.5 0.5</scale>
</include>
```

اسکیل کی اقدار ماڈل کے سائز کو ضرب دیتی ہیں:
- `1.0` = اصل سائز
- `0.5` = آدھا سائز
- `2.0` = دوگنا سائز

آپ غیر یکساں طور پر بھی اسکیل کر سکتے ہیں (مختلف محوروں پر مختلف اسکیل):
```xml
<scale>1.0 1.0 2.0</scale>
```
یہ آبجیکٹ کو اونچائی میں دوگنا کر دیتا ہے جبکہ چوڑائی اور گہرائی کو وہی رکھتا ہے۔

**اسکیل کب کریں:**
- ماڈل آپ کے ماحول کے لیے بہت بڑا ہے؟ اسے چھوٹا کریں (0.5 سے 0.8)
- ماڈل بہت چھوٹا ہے؟ اسے بڑا کریں (1.5 سے 2.0)
- Gazebo Fuel میں زیادہ تر ماڈلز پہلے سے ہی انسانی ماحول کے لیے سائز کے ہوتے ہیں، لہذا اسکیلنگ اکثر ضروری نہیں ہوتی

## ماڈل URIs کو سمجھنا

ہر فیول ماڈل کا ایک منفرد URI ہوتا ہے۔ آئیے URI کی ساخت کو توڑتے ہیں:

```
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master
```

- `https://fuel.gazebosim.org/1.0/` - بنیادی Gazebo Fuel ڈومین
- `openrobotics` - ماڈل کا مالک/تخلیق کار
- `models` - اثاثہ کی قسم (مکمل دنیاؤں کے لیے "worlds" بھی ہو سکتا ہے)
- `table` - ماڈل کا نام
- `tip/master` - ورژن (عام طور پر تازہ ترین کے لیے "tip/master")

**متبادل فارمیٹ** (/1.0/ سابقہ کے بغیر):
```
model://openrobotics/table
```
یہ شارٹ ہینڈ ہے۔ SDF فائلوں میں دونوں فارمیٹس کام کرتے ہیں۔

## فیول ماڈلز کے لیے عملی نکات

### مؤثر طریقے سے ماڈلز تلاش کرنا

**کیٹیگری کے لحاظ سے**: https://app.gazebosim.org/fuel پر جائیں اور قسم کے لحاظ سے فلٹر کریں:
- فرنیچر (میزیں، کرسیاں، ڈیسک)
- پودے (درخت، گھاس، پھول)
- عمارتیں (دیواریں، دروازے، کھڑکیاں)
- گاڑیاں (کاریں، ٹرک)
- اشیاء (کونز، بکس، گیندیں)

**شہرت کے لحاظ سے**: زیادہ ڈاؤن لوڈ کی تعداد والے ماڈلز دیکھیں۔ مقبول ماڈلز عام طور پر اچھی طرح سے بنائے گئے اور جانچے ہوئے ہوتے ہیں۔

**تخلیق کار کے لحاظ سے**: OpenRobotics Gazebo ٹیم کی طرف سے سرکاری ہے۔ ان کے ماڈلز قابل اعتماد ہیں۔

### ماڈل کے معیار کی جانچ کرنا

ماڈل شامل کرنے سے پہلے:
1. اس کی پیش منظر کی تصویر ڈاؤن لوڈ کریں (دیکھیں کہ کیا یہ ٹھیک لگ رہا ہے)
2. تفصیل دیکھیں (دیکھیں کہ کیا اس میں فزکس پراپرٹیز ترتیب دی گئی ہیں)
3. اگر دستیاب ہو تو تبصرے/جائزے چیک کریں (صارفین مسائل کی اطلاع دے سکتے ہیں)
- پیچیدہ منظرناموں میں استعمال کرنے سے پہلے اسے ایک سادہ دنیا میں جانچیں

### بڑے ماڈلز سے نمٹنا

کچھ فیول ماڈلز (خاص طور پر تفصیلی عمارتیں یا پودے) میں پیچیدہ میشز ہوتے ہیں۔ بڑے ماڈلز سمولیشن کو سست کر سکتے ہیں۔ اگر کارکردگی کا مسئلہ ہے:
1. آسان ماڈلز استعمال کریں (متغیرات کے لیے "simple"، "lowpoly"، یا "low-poly" تلاش کریں)
2. بھاری ماڈلز سے سائے ہٹا دیں
3. فزکس کی پیچیدگی کو کم کریں (جہاں ممکن ہو "static" ماڈلز استعمال کریں)

## ایک مکمل دنیا بنانا

آئیے سب کچھ یکجا کریں: گراؤنڈ پلین، فزکس، لائٹنگ، اور متعدد فیول ماڈلز۔

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="busy_room">

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

    <!-- Work area -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>0 0 0 0 0 0</pose>
      <name>work_table</name>
    </include>

    <!-- Desk chair -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>0 -0.5 0 0 0 0</pose>
      <name>desk_chair</name>
    </include>

    <!-- Decorative plant in corner -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/potted_plant/tip/master</uri>
      <pose>2 2 0 0 0 0</pose>
      <name>plant_corner</name>
    </include>

  </world>
</sdf>
```

**یہاں کیا ہو رہا ہے:**
1. گراؤنڈ اور لائٹنگ (بنیاد)
2. مرکز میں کام کی میز
3. میز کے جنوب میں رکھی ہوئی کرسی
4. شمال مشرقی کونے میں پودا

اب آپ کے پاس ایک فرنیچر والا ماحول ہے جو روبوٹ کے نیویگیٹ کرنے کے لیے تیار ہے۔

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: ایک ٹیکسٹ ایڈیٹر کھلا رکھیں اور SDF include سٹیٹمنٹس لکھنے کے لیے تیار رہیں۔ اپنی پسند کے AI ٹول تک رسائی حاصل کریں۔

**پرامپٹ 1** (دریافت):
```
میں ایک Gazebo دنیا بنانا چاہتا ہوں جس میں یہ چیزیں ہوں:
1. کھانے کا علاقہ (میز + کرسیاں)
2. ایک پودے کے ساتھ ایک کونہ
3. ایک سٹوریج شیلف یونٹ

مجھے کن فیول ماڈلز کو تلاش کرنا چاہیے؟
ہر عنصر کے لیے، ایک ماڈل کا نام تجویز کریں اور بتائیں کہ آپ کون سی pose اقدار استعمال کر سکتے ہیں۔
```

**متوقع نتیجہ**: AI متعلقہ فیول ماڈلز تجویز کرتا ہے اور پوزیشننگ کے بارے میں تخمینی رہنمائی دیتا ہے۔

**پرامپٹ 2** (Syntax):
```
میں اپنے SDF دنیا میں یہ فیول ماڈل شامل کرنا چاہتا ہوں:
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master

اس میز کو رکھنے کے لیے XML include سٹیٹمنٹ لکھیں:
- پوزیشن (2, 3, 0) پر
- بغیر کسی گردش کے
- نام "conference_table" کے ساتھ
```

**متوقع نتیجہ**: AI صحیح XML include بلاک لکھتا ہے۔

**پرامپٹ 3** (Debugging):
```
میں نے اپنے SDF دنیا میں یہ ماڈل شامل کیا:
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>1 2 0 0 0 0</pose>
  <name>table1</name>
</include>

جب میں اسے Gazebo میں کھولتا ہوں، تو میز زمین پر رکھنے کے بجائے ہوا میں تیر رہی ہوتی ہے۔
کیا غلط ہو سکتا ہے، اور مجھے pose اقدار کو کیسے ٹھیک کرنا چاہیے؟
```

**متوقع نتیجہ**: AI وضاحت کرتا ہے کہ ماڈل کا اصل مقام (origin) اس کی بنیاد سے اوپر ہو سکتا ہے، اور منفی Z قدر آزمانے یا ماڈل کی دستاویزات چیک کرنے کا مشورہ دیتا ہے۔