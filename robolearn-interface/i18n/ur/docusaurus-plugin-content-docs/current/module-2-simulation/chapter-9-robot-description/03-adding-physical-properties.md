---
id: lesson-9-3-adding-physical-properties
title: 'Lesson 9.3: Adding Physical Properties'
sidebar_position: 3
sidebar_label: 9.3 Physical Properties
description: 'Adding mass, inertia, and collision geometry for realistic physics simulation'
chapter: 9
lesson: 3
duration_minutes: 60
proficiency_level: A2
layer: L1
cognitive_load:
  new_concepts: 6
learning_objectives:
  - Add mass and inertia properties to robot links
  - Calculate inertia tensors for common shapes
  - Configure collision geometry separate from visual geometry
  - Understand why physics properties matter for simulation
  - Add material properties for friction and restitution
  - Debug physics issues in Gazebo
skills:
  - urdf-basics
  - physics-properties
hardware_tier: 1
tier_1_path: The Construct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 9.3: طبعی خصوصیات شامل کرنا (Adding Physical Properties)

آپ نے ایک ایسا روبوٹ بنایا ہے جس میں جیومیٹری (shape) اور ظاہری شکل موجود ہے۔ لیکن طبعی خصوصیات (physics properties) کے بغیر، آپ کا روبوٹ حقیقی چیز کی طرح برتاؤ نہیں کرے گا۔ یہ ہوا میں تیرے گا۔ یہ گرے گا نہیں۔ کشش ثقل (Gravity) کا کوئی اثر نہیں ہوگا۔ ٹکراؤ صرف جیومیٹری کے لحاظ سے ہوں گے، طبعی لحاظ سے نہیں۔

اس سبق میں، آپ وہ اعداد شامل کریں گے جو سمولیشن کو حقیقت کے قریب لاتے ہیں: **ماس** (کتنا بھاری)، **جڑیت** (Inertia - وزن کیسے تقسیم ہوا ہے)، اور **گِریڑ** (Friction - سطحیں کیسے تعامل کرتی ہیں)۔ یہ خصوصیات آپ کے URDF کو صرف ایک بصری ماڈل سے ایک طبعی طور پر درست سمولیشن میں تبدیل کر دیتی ہیں۔

---

## طبعی خصوصیات کیوں اہم ہیں؟

تصور کریں کہ Gazebo میں ایک روبوٹ ہے جس کا کوئی ماس نہیں ہے۔ اگر آپ کوئی قوت (force) لگاتے ہیں، تو کیا ہوگا؟ **کچھ نہیں**۔ طبعی انجن یہ فرض کرتے ہیں کہ بے وزن اشیاء کی جڑیت لامحدود ہوتی ہے—وہ تیز نہیں ہوتیں۔ آپ کے پہیے گھومتے ہیں لیکن چیسس کو حرکت نہیں دیتے۔ جب روبوٹ کو سہارا نہ ہو تو وہ گرتا نہیں۔

ماس کے ساتھ، طبعی انجن کشش ثقل کا اطلاق کرتا ہے۔ درست جڑیت کے بغیر، گردش (rotation) غلط ہوتی ہے—روبوٹ غیر متوقع طور پر پلٹ جاتا ہے یا بہت آسانی سے گھوم جاتا ہے۔

**مثال: الٹ جانا (Tipping Over)**
- جڑیت کے بغیر روبوٹ: تھوڑا سا بھی جھکتا ہے اور فوراً الٹ جاتا ہے
- درست جڑیت والا روبوٹ: اپنے پہیوں پر مستحکم رہتا ہے، صرف حقیقی زاویوں پر جھکتا ہے

---

## بنیادی تصور 1: ماس (Mass)

**ماس** یہ ہے کہ ایک لنک میں کتنا "مادہ" موجود ہے۔ یہ کلوگرام میں ماپا جاتا ہے۔

```xml
<inertial>
  <mass value="5.0"/>
</inertial>
```

یہ کہتا ہے کہ لنک کا وزن 5 کلوگرام ہے۔

**روبوٹس کے لیے اصول:**
- چھوٹا پہیہ: 0.2 - 0.5 کلوگرام
- درمیانی چیسس: 2 - 5 کلوگرام
- بڑا روبوٹ: 10 - 50 کلوگرام

---

## بنیادی تصور 2: جڑیت (Inertia)

**جڑیت** بیان کرتا ہے کہ ماس کیسے تقسیم ہوا ہے۔ یہ متاثر کرتا ہے کہ کسی چیز کو گھمانا کتنا مشکل ہے۔

ایک بھاری گیند اور ایک بھاری سلاخ کا وزن ایک جیسا ہو سکتا ہے، لیکن سلاخ کو گھمانا زیادہ مشکل ہوتا ہے کیونکہ ماس مرکز سے دور ہوتا ہے۔

جڑیت کو ایک **ٹینسور** (3×3 میٹرکس) کے طور پر بیان کیا جاتا ہے۔ سادگی کے لیے، ہم صرف ترچھی اقدار (diagonal values) استعمال کرتے ہیں:

```xml
<inertia ixx="0.01" ixy="0" ixz="0"
         iyy="0.02" iyz="0"
         izz="0.03"/>
```

یہ اقدار ہیں:
- **ixx, iyy, izz**: ہر محور کے گرد گردش کی مزاحمت
- **ixy, ixz, iyz**: کراس ٹرمز (عام طور پر متوازی اشیاء کے لیے 0 ہوتے ہیں)

### عام اشکال کے فارمولے

**باکس (Box)** کے لیے (لمبائی L، چوڑائی W، اونچائی H، ماس M):
```
ixx = (1/12) * M * (W² + H²)
iyy = (1/12) * M * (L² + H²)
izz = (1/12) * M * (L² + W²)
```

**سلنڈر (Cylinder)** کے لیے (نصف قطر R، لمبائی L، ماس M):
```
ixx = (1/12) * M * (3*R² + L²)
iyy = (1/12) * M * (3*R² + L²)
izz = (1/2) * M * R²
```

**کروی (Sphere)** کے لیے (نصف قطر R، ماس M):
```
ixx = iyy = izz = (2/5) * M * R²
```

---

## اپنے روبوٹ کے لیے جڑیت کا حساب لگانا

سبق 9.2 سے، آپ کے روبوٹ میں یہ ہیں:

### چیسس (باکس: 0.3m × 0.2m × 0.1m، ماس 5 کلوگرام)

```
ixx = (1/12) * 5 * (0.2² + 0.1²) = (1/12) * 5 * 0.05 = 0.0208
iyy = (1/12) * 5 * (0.3² + 0.1²) = (1/12) * 5 * 0.1 = 0.0417
izz = (1/12) * 5 * (0.3² + 0.2²) = (1/12) * 5 * 0.13 = 0.0542
```

### ہر پہیہ (سلنڈر: 0.05m نصف قطر، 0.05m لمبائی، ماس 0.3 کلوگرام)

```
ixx = (1/12) * 0.3 * (3*0.05² + 0.05²) = 0.000625
iyy = (1/12) * 0.3 * (3*0.05² + 0.05²) = 0.000625
izz = (1/2) * 0.3 * 0.05² = 0.000375
```

### کاسٹر وہیل (کروی: 0.025m نصف قطر، ماس 0.1 کلوگرام)

```
ixx = iyy = izz = (2/5) * 0.1 * 0.025² = 0.000025
```

---

## مرحلہ 1: بیس لنک میں ماس اور جڑیت شامل کریں

سبق 9.2 سے اپنے `base_link` کو جڑیت کی خصوصیات شامل کرنے کے لیے تبدیل کریں:

```xml
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
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0208" ixy="0" ixz="0"
               iyy="0.0417" iyz="0"
               izz="0.0542"/>
    </inertial>
  </link>
```

**یہ کیا کرتا ہے:**
- 5 کلوگرام کا ماس کشش ثقل کے ساتھ روبوٹ کو نیچے کھینچتا ہے
- جڑیت کی اقدار گردش کو حقیقت کے قریب بناتی ہیں

---

## مرحلہ 2: بائیں اور دائیں پہیوں میں خصوصیات شامل کریں

```xml
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
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.000625" ixy="0" ixz="0"
               iyy="0.000625" iyz="0"
               izz="0.000375"/>
    </inertial>
  </link>

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
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.000625" ixy="0" ixz="0"
               iyy="0.000625" iyz="0"
               izz="0.000375"/>
    </inertial>
  </link>
```

---

## مرحلہ 3: کاسٹر وہیل میں خصوصیات شامل کریں

```xml
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
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000025" ixy="0" ixz="0"
               iyy="0.000025" iyz="0"
               izz="0.000025"/>
    </inertial>
  </link>
```

---

## مرحلہ 4: گِریڑ شامل کرنا (Gazebo Extension)

معیاری URDF گِریڑ کی وضاحت نہیں کرتا۔ Gazebo ایک مخصوص توسیع (custom extension) استعمال کرتا ہے:

اسے ہر پہیے کے لنک میں، `<collision>` عنصر کے اندر شامل کریں:

```xml
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
```

**یہ کیا کرتا ہے:**
- `<mu>0.8</mu>`: گِریڑ کا گتانک (coefficient) (0 = پھسلتی ہوئی برف، 1.0 = کھردرا ربڑ)
- 0.8 گِریڑ والے پہیے زمین پر حقیقت پسندانہ طور پر پکڑ بناتے ہیں

---

## پہلے اور بعد کا موازنہ

### پہلے (کوئی طبعی خصوصیات نہیں)

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
</link>
```

**Gazebo میں لوڈ ہونے پر:**
- روبوٹ خلا میں تیرتا ہے
- کشش ثقل کا کوئی اثر نہیں ہوتا
- پہیے نہیں گھومتے (حرکت کی مزاحمت کے لیے کوئی جڑیت نہیں)
- ٹکراؤ کا پتہ تو چلتا ہے لیکن حقیقت پسندانہ نہیں

### بعد میں (طبعی خصوصیات کے ساتھ)

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
    <inertia ixx="0.0208" ixy="0" ixz="0"
             iyy="0.0417" iyz="0"
             izz="0.0542"/>
  </inertial>
</link>
```

**Gazebo میں لوڈ ہونے پر:**
- روبوٹ زمین پر گرتا ہے (کشش ثقل کھینچتی ہے)
- پہیے وزن سنبھالتے ہیں
- جب پہیوں کو طاقت دی جاتی ہے، تو روبوٹ حقیقت پسندانہ طور پر تیز ہوتا/رفتار کم کرتا ہے (جڑیت کی وجہ سے)
- ٹکراؤ سے حقیقت پسندانہ قوتیں پیدا ہوتی ہیں

---

## Gazebo میں طبعیات کا تجربہ کرنا

اپنا اپ ڈیٹ شدہ URDF لوڈ کریں:

```bash
gazebo my_first_robot.urdf
```

**متوقع بصری نتیجہ:**
1. روبوٹ زمین پر گرتا ہے (کشش ثقل کا اثر)
2. روبوٹ تین پہیوں پر مستحکم بیٹھتا ہے (توازن)
3. اگر آپ پہیے کے موٹر کو چلاتے ہیں (کمانڈز کا استعمال کرتے ہوئے)، تو روبوٹ کو رول کرنا چاہیے

**عام طبعی مسائل:**

| مسئلہ | وجہ | حل |
|-------|-------|-----|
| روبوٹ الٹا پلٹ جاتا ہے | جڑیت کی اقدار غلط ہیں یا بہت ہلکی ہیں | جڑیت کا دوبارہ حساب لگائیں، ماس بڑھائیں |
| پہیے بہت زیادہ پھسلتے ہیں | گِریڑ بہت کم ہے | `<mu>` کو 0.8-1.0 تک بڑھائیں |
| روبوٹ زمین میں دھنس جاتا ہے | ٹکراؤ کی جیومیٹری بہت چھوٹی ہے | `<box>` یا `<cylinder>` کا سائز بڑھائیں |
| پہیے طاقت دینے پر نہیں گھومتے | پہیوں پر `<inertial>` نہیں ہے | ہر پہیے میں `<inertial>` شامل کریں |

---

## مشق: اپنے روبوٹ میں طبعیات شامل کریں

سبق 9.2 سے اپنا URDF لیں اور:

1. تمام تین لنکس (base_link، left_wheel، right_wheel، caster_wheel) میں جڑیت کی خصوصیات شامل کریں۔
2. درست اقدار کا حساب لگانے کے لیے فراہم کردہ فارمولوں کا استعمال کریں۔
3. اسے Gazebo میں لوڈ کریں اور تصدیق کریں کہ روبوٹ زمین پر گرتا ہے۔
4. استحکام کا تجربہ کریں: روبوٹ کو چپٹا بیٹھنا چاہیے، الٹنا نہیں چاہیے۔

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ:** اب آپ کے پاس طبعیات والا روبوٹ ہے۔ آئیے اس کی توثیق اور بہتری کے لیے AI کا استعمال کریں۔

**پرامپٹ سیٹ:**

```
Prompt 1: "I calculated inertia for my robot using the formulas.
Here are my values: [paste your inertial values]
Do these seem reasonable for a small mobile robot?
Are there any that look wrong?"

Prompt 2: "What would happen if I doubled the mass of my wheels
but kept the inertia the same? Would the robot behave differently?"

Prompt 3: "Explain friction in simulation and how it affects robot motion.
What friction value should I use for wheels on carpet vs. hard floor?"
```

**متوقع نتائج:**
- AI کو حسابات کی توثیق کرنی چاہیے اور غیر حقیقی اقدار کو جھنڈا کرنا چاہیے
- آپ کو سمجھ آنا چاہیے کہ صرف ماس تبدیل کرنے سے طبعیات کیوں ٹوٹ جاتی ہے
- آپ کو سیکھنا چاہیے کہ گِریڑ سطح کے لحاظ سے مختلف ہوتی ہے

**حفاظتی نوٹ:** طبعی خصوصیات درست سمولیشن کے لیے بہت اہم ہیں۔ غلط جڑیت یہ مسائل پیدا کر سکتی ہے:
- روبوٹ غیر متوقع طور پر پلٹ جاتا ہے (اگر سمولیشن پر انحصار کیا جائے تو حفاظتی خطرہ)
- پہیے گھومتے ہیں لیکن پکڑ نہیں بناتے (نقل و حرکت کا ٹوٹا ہوا سمولیشن)
- جھٹکے دار، غیر حقیقی حرکت (غلط کنٹرول ٹیوننگ)

ہمیشہ طبعیات کو حقیقی دنیا کے روبوٹ کے برتاؤ کے مقابلے میں جانچیں۔

**اختیاری توسیع:** اپنے AI ٹول سے پوچھیں: "How do I add damping to my robot to make movement less jerky?" یا "What's the difference between `<friction>` and `<damping>`?"