---
id: lesson-3-1-setup-environment
title: 'Lesson 3.1: Setting Up Your ROS 2 Environment'
sidebar_position: 1
sidebar_label: 3.1 Environment Setup
description: >-
  Install ROS 2 Humble locally or access cloud ROS 2 environment with workspace
  structure verification.
duration_minutes: 60
proficiency_level: A2
layer: L1
hardware_tier: 1
learning_objectives:
  - Install ROS 2 Humble OR access cloud ROS 2 environment
  - Understand workspace structure and package paths
  - Verify environment with diagnostic commands
skills:
  - ros2-fundamentals
tier_1_path: Cloud ROS 2 (TheConstruct) or local Ubuntu/WSL installation
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# اپنا ROS 2 ماحول (Environment) سیٹ اپ کرنا

آپ نے سیکھ لیا کہ ROS 2 کیوں اہم ہے۔ اب وقت ہے کہ اسے اپنی مشین (یا کلاؤڈ میں) چلایا جائے۔

یہ سبق دو راستے فراہم کرتا ہے: **مقامی تنصیب (Local Installation)** (اگر آپ کے پاس Ubuntu یا WSL ہے) یا **کلاؤڈ ROS 2 (Cloud ROS 2)** (اگر آپ براؤزر پر مبنی رسائی کو ترجیح دیتے ہیں)۔ دونوں راستے ایک ہی منزل پر لے جاتے ہیں—ایک کام کرنے والا ROS 2 Humble ماحول جہاں آپ کمانڈز چلا سکتے ہیں اور ایپلیکیشنز لانچ کر سکتے ہیں۔

**مدت**: 60 منٹ | **ہارڈ ویئر ٹائر**: ٹائر 1 (کلاؤڈ یا مقامی) | **ابھی تک کوئی کوڈنگ نہیں—صرف سیٹ اپ اور تصدیق**

---

## اپنے دو راستوں کو سمجھنا

انتخاب کرنے سے پہلے، سمجھیں کہ ہر راستہ کیا پیش کرتا ہے:

### راستہ A: کلاؤڈ ROS 2 (سب سے آسان، تجویز کردہ)

**کے لیے بہترین**: وہ طلباء جن کے پاس Ubuntu نہیں ہے، کم سے کم تنصیب کی پریشانی، ضمانت شدہ مطابقت (compatibility)

**آپ کو کیا ملے گا**:
- براؤزر پر مبنی ROS 2 ٹرمینل (کوئی تنصیب نہیں)
- پہلے سے ترتیب شدہ ROS 2 Humble ماحول
- GUI ایپلیکیشنز کے لیے VNC ریموٹ ڈیسک ٹاپ
- کوئی انحصار کا تصادم (dependency conflicts) نہیں

**وقت کی سرمایہ کاری**: 5 منٹ (سائن اپ + لانچ)

**کہاں**: The Construct (TheConstruct.org) — مفت ٹیر میں کلاؤڈ ROS 2 شامل ہے

---

### راستہ B: مقامی تنصیب (Fastest Execution, More Setup)

**کے لیے بہترین**: وہ طلباء جن کے پاس Ubuntu Linux یا WSL ہے، جو مقامی عمل درآمد کی رفتار چاہتے ہیں

**آپ کو کیا ملے گا**:
- آپ کی مشین پر مقامی ROS 2
- کلاؤڈ سے تیز کمانڈ پر عمل درآمد
- تنصیب پر مکمل کنٹرول
- سیٹ اپ کے بعد آف لائن کام کرتا ہے

**وقت کی سرمایہ کاری**: 15-20 منٹ (تنصیب + تصدیق)

**کہاں**: آپ کی Ubuntu 22.04 مشین (یا Windows پر WSL)

---

## اپنا راستہ منتخب کریں

**فیصلہ گائیڈ**:

| سوال | راستہ A (کلاؤڈ) | راستہ B (مقامی) |
|---|---|---|
| کیا آپ کے پاس Ubuntu 22.04 ہے؟ | کوئی مسئلہ نہیں! | اسے استعمال کریں ✅ |
| کیا آپ تیز ترین عمل درآمد چاہتے ہیں؟ | اسے چھوڑ دیں | اسے آزمائیں ✅ |
| تنصیب سے بچنا چاہتے ہیں؟ | بہترین ✅ | اسے چھوڑ دیں |
| محدود ڈسک کی جگہ ہے؟ | ہاں ✅ | ~5GB درکار ہے |
| اکثر آف لائن کام کرتے ہیں؟ | نہیں | ہاں ✅ |

**زیادہ تر طلباء پہلے راستہ A کا انتخاب کرتے ہیں، پھر رفتار کے لیے راستہ B پر جاتے ہیں۔**

---

## راستہ A: کلاؤڈ ROS 2 (TheConstruct)

### مرحلہ 1: ایک مفت اکاؤنٹ بنائیں

1. **TheConstruct.org** پر جائیں
2. **Sign Up** پر کلک کریں
3. ای میل/پاس ورڈ کے ساتھ اکاؤنٹ بنائیں
4. ای میل کی تصدیق کریں (اسپیم فولڈر چیک کریں)

**تخمینہ وقت**: 3 منٹ

---

### مرحلہ 2: ایک ROS 2 ماحول لانچ کریں

1. اپنے The Construct اکاؤنٹ میں لاگ ان کریں
2. **MY COURSES** → **Create New Rosject** پر کلک کریں
3. ترتیب دیں (Configure):
   - **Name**: "ROS 2 Learning" (یا آپ کی پسند کا)
   - **ROS Distribution**: **ROS 2 Humble** منتخب کریں
   - **Type**: **ROS Desktop** منتخب کریں (اس میں ویژولائزیشن ٹولز شامل ہیں)
4. **Create Rosject** پر کلک کریں

**انتظار کریں**: سسٹم آپ کا ماحول لانچ کرتا ہے (~30 سیکنڈ)

---

### مرحلہ 3: اپنے ٹرمینل تک رسائی حاصل کریں

ایک بار لانچ ہونے کے بعد، آپ کو ایک ویب انٹرفیس نظر آئے گا جس میں یہ شامل ہوں گے:
- **بائیں سائڈبار**: فائل براؤزر
- **مرکز**: ٹیکسٹ ایڈیٹر (ابھی ضرورت نہیں)
- **دائیں پینل**: **Open Shell** بٹن

ٹرمینل کھولنے کے لیے **Open Shell** پر کلک کریں۔

اب آپ کے پاس اپنے براؤزر میں ایک کام کرنے والا ROS 2 Humble ماحول ہے۔

**تصدیق کریں کہ یہ کام کرتا ہے**:

```bash
echo $ROS_DISTRO
```

**متوقع آؤٹ پٹ**:
```
humble
```

اگر آپ کو `humble` نظر آتا ہے، تو آپ سیٹ اپ کے ساتھ ہو گئے ہیں! نیچے "ماحول کی تصدیق" پر جائیں۔

---

## راستہ B: مقامی تنصیب (Ubuntu/WSL)

### پیشگی تقاضے (Prerequisites)

- **Ubuntu 22.04 LTS** (مقامی یا Windows پر WSL)
- **Sudo رسائی** (`sudo` کمانڈ چلانے کی صلاحیت)
- ROS 2 + انحصار کے لیے **~5GB ڈسک کی جگہ**
- **15-20 منٹ** صبر

---

### مرحلہ 1: ROS 2 ریپوزٹری شامل کریں

ایک ٹرمینل کھولیں اور چلائیں:

```bash
sudo apt update
sudo apt install curl gnupg lsb-release

sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros2.org/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

یہ آپ کے پیکیج مینیجر میں سرکاری ROS 2 ریپوزٹری شامل کرتا ہے۔

**اگر غلطیاں (errors) آتی ہیں**: `sudo apt update` چلائیں اور اوپر دی گئی کمانڈز کو دوبارہ آزمائیں۔

---

### مرحلہ 2: ROS 2 Humble انسٹال کریں

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

**کیا ہوتا ہے**:
- ٹرمینل پیشرفت دکھاتا ہے (اس میں 5-10 منٹ لگ سکتے ہیں)
- آپ کا پاس ورڈ پوچھتا ہے (اسے درج کریں)
- ROS 2، پائیتھن لائبریریاں، بلڈ ٹولز انسٹال کرتا ہے

**مکمل ہونے پر**: آپ کو بغیر کسی غلطی کے ایک تازہ پرامپٹ نظر آتا ہے۔

---

### مرحلہ 3: سیٹ اپ اسکرپٹ کو سورس کریں (Source)

ROS 2 کو اپنے شیل ماحول میں شامل کریں:

```bash
source /opt/ros/humble/setup.bash
```

**یہ کیا کرتا ہے**:
- PATH میں ROS 2 ایگزیکیوٹیبلز شامل کرتا ہے
- `ROS_DISTRO` انوائرنمنٹ ویری ایبل سیٹ کرتا ہے
- ROS 2 پیکیجز کے لیے پائیتھن راستے (paths) کو ترتیب دیتا ہے

**اسے خودکار بنائیں** (تجویز کردہ):

ہر بار جب آپ ٹرمینل کھولیں تو اسے چلانے کے لیے سورس کمانڈ کو اپنی شیل اسٹارٹ اپ فائل میں شامل کریں:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

اب ہر نیا ٹرمینل خود بخود ROS 2 لوڈ کر دے گا۔

---

## ماحول کی تصدیق (Environment Verification) (دونوں راستوں کے لیے)

اس بات سے قطع نظر کہ آپ نے کون سا راستہ منتخب کیا، تصدیق کریں کہ آپ کی تنصیب کام کر رہی ہے:

### چیک 1: ROS ڈسٹرو

```bash
echo $ROS_DISTRO
```

**متوقع آؤٹ پٹ**: `humble`

**اگر خالی ہے یا غلطی ہے**: آپ نے سورسنگ مرحلہ چھوڑ دیا ہے۔ چلائیں:
```bash
source /opt/ros/humble/setup.bash  # کلاؤڈ: اسے چھوڑ دیں، پہلے ہی سورس ہو چکا ہے
source ~/.bashrc                     # مقامی: سیٹ اپ کی تصدیق کریں
```

---

### چیک 2: ROS 2 کمانڈز دستیاب ہیں

```bash
ros2 --version
```

**متوقع آؤٹ پٹ**:
```
ROS 2 humble (release date: 2023-05-23)
```

---

### چیک 3: ROS پیکیجز تلاش کریں

```bash
ros2 pkg list | head -10
```

یہ انسٹال شدہ پہلے 10 ROS 2 پیکیجز دکھاتا ہے۔ آپ کو کچھ ایسا نظر آنا چاہیے:

```
ament_cmake
ament_cmake_auto
ament_cmake_gmock
ament_cmake_google_benchmark
...
```

**اگر کوئی آؤٹ پٹ نہیں ہے**: سورسنگ کام نہیں کی۔ اپنا ٹرمینل دوبارہ شروع کریں اور چیک 1 کو دوبارہ آزمائیں۔

---

### چیک 4: ROS 2 کے لیے پائیتھن پاتھ

```bash
python3 -c "import rclpy; print('✓ rclpy installed')"
```

**متوقع آؤٹ پٹ**: `✓ rclpy installed`

**اگر ModuleNotFoundError ہے**: ROS 2 سورس نہیں ہوا ہے۔ سورسنگ کو دوبارہ آزمائیں اور ٹرمینل دوبارہ شروع کریں۔

---

## خرابیوں کا سراغ لگانا (Troubleshooting)

### "Command ros2 not found"

**وجہ**: ROS 2 سورس نہیں ہوا یا تنصیب نامکمل ہے

**حل**:
1. `source /opt/ros/humble/setup.bash` چلائیں
2. ٹرمینل دوبارہ شروع کریں
3. دوبارہ کوشش کریں

مستقل حل کے لیے: اسے `~/.bashrc` میں شامل کریں (اگر آپ نے مرحلہ B3 کی پیروی کی تو یہ پہلے ہی ہو چکا ہے)

---

### "ROS_DISTRO is empty"

**وجہ**: سورسنگ ناکام ہو گئی

**حل**:
1. فائل کی موجودگی کی تصدیق کریں: `ls /opt/ros/humble/setup.bash` (مقامی) یا The Construct ٹرمینل چیک کریں
2. سورس کمانڈ چلائیں: `source /opt/ros/humble/setup.bash`
3. تصدیق کریں: `echo $ROS_DISTRO`

---

### تنصیب ناکام ہو گئی (مقامی راستہ)

**اگر `apt install` نے غلطیاں دیں**:

1. Ubuntu ورژن چیک کریں: `lsb_release -cs` (22.04 کے لیے `jammy` ہونا چاہیے)
2. ریپوزٹری دوبارہ شامل کریں (شاید ٹائپو ہو)
3. دوبارہ کوشش کریں: `sudo apt install ros-humble-desktop`

**اگر اب بھی ناکام ہو رہا ہے**: اس کے بجائے کلاؤڈ راستہ استعمال کریں (تنصیب کی ضرورت نہیں)

---

### "Permission denied" غلطیاں

**وجہ**: sudo کی کمی

**حل**: کمانڈز کے آگے `sudo` لگائیں:
```bash
sudo apt update
sudo apt install ...
```

---

## اپنے ماحول کو سمجھنا

اب جب ROS 2 چل رہا ہے، سمجھیں کہ آپ نے ابھی کیا سیٹ اپ کیا ہے:

### ورک اسپیس ڈھانچہ (مقامی تنصیب)

جب آپ ROS 2 مقامی طور پر انسٹال کرتے ہیں، تو یہ بناتا ہے:

```
/opt/ros/humble/           # ROS 2 تنصیب کا ڈائریکٹری
├── bin/                   # ایگزیکیوٹیبلز (ros2 کمانڈ، وغیرہ)
├── lib/                   # لائبریریاں
├── share/                 # پیکیجز اور ڈیٹا فائلیں
└── setup.bash             # ماحول سیٹ اپ کرنے والا اسکرپٹ
```

**جب آپ `source setup.bash` چلاتے ہیں**, تو اسکرپٹ:
1. PATH میں `/opt/ros/humble/bin/` شامل کرتا ہے (تاکہ `ros2` کمانڈ مل جائے)
2. `ROS_DISTRO=humble` سیٹ کرتا ہے (ROS 2 کو بتاتا ہے کہ کون سا ورژن استعمال کرنا ہے)
3. `PYTHONPATH` کو ترتیب دیتا ہے (تاکہ پائیتھن `rclpy` لائبریری تلاش کر سکے)

---

### پیکیج کی دریافت (کلاؤڈ اور مقامی)

جب آپ `ros2 pkg list` چلاتے ہیں، تو ROS 2 ان ڈائریکٹریز میں پیکیجز تلاش کرتا ہے جو `ROS_PACKAGE_PATH` کے ذریعے بتائی گئی ہیں۔

**ایک صاف تنصیب کے لیے**, یہ سب `/opt/ros/humble/share/` میں ہیں۔

بعد میں (باب 4-7)، آپ ایک **ورک اسپیس** (`~/ros2_ws/`) میں اپنے پیکیجز بنائیں گے، اور ایک بار سورس ہونے کے بعد وہ خود بخود دریافت ہو جائیں گے۔

---

## اگلے اقدامات

اب آپ کے پاس ایک کام کرنے والا ROS 2 Humble ماحول ہے۔ اگلے سبق میں، آپ یہ کریں گے:

1. **turtlesim** لانچ کریں—ایک سادہ 2D روبوٹ سمیلیٹر
2. اسے کی بورڈ ان پٹ سے کنٹرول کریں
3. ROS 2 کے پبلش/سبسکرائب سسٹم کو عمل میں دیکھیں

لیکن اس سے پہلے، آئیے ایک مختصر چیک پوائنٹ کے ساتھ اپنی سمجھ کو مضبوط کریں۔

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: اپنا پسندیدہ AI ٹول (ChatGPT، Claude، یا اسی طرح کا) اور اپنا ROS 2 ٹرمینل ساتھ ساتھ کھولیں۔

**پرامپٹ 1** (PATH کو سمجھنا):

اپنے AI سے پوچھیں:

```
I just ran 'source /opt/ros/humble/setup.bash'.
What does this command actually do to my shell?
Explain what PATH environment variable is and why ROS 2 needs to modify it.
```

**متوقع بصیرت**: PATH آپ کے شیل کو بتاتا ہے کہ ایگزیکیوٹیبلز کہاں تلاش کرنے ہیں۔ اسے سمجھنا بعد میں "command not found" غلطیوں کو ٹھیک کرنے میں مدد کرتا ہے۔

---

**پرامپٹ 2** (تصدیق چیک):

اپنے AI سے پوچھیں:

```
I'm verifying my ROS 2 installation. I ran these checks:
- echo $ROS_DISTRO → outputs "humble" ✓
- ros2 --version → outputs "ROS 2 humble (release date: 2023-05-23)" ✓
- ros2 pkg list | head → shows packages ✓
- python3 -c "import rclpy" → outputs nothing (success) ✓

Is my installation complete and correct? What might still be missing?
```

**متوقع بصیرت**: AI تجویز کر سکتا ہے کہ `ROS_DOMAIN_ID` یا `RMW_IMPLEMENTATION` (ایڈوانس سیٹنگز جن کی آپ کو ابھی ضرورت نہیں ہے) کی تصدیق کریں۔ تنصیب کی بنیادی باتوں سے آگے بھی گہرائی ہو سکتی ہے۔

---

**پرامپٹ 3** (ناکامی سے بحالی):

اپنے AI سے یہ منظر نامہ پوچھیں:

```
I get the error: "ros2: command not found" even after running "source /opt/ros/humble/setup.bash".
What are 3 things I should check, and how would I diagnose each one?
```

**آپ کیا سیکھتے ہیں**: منظم خرابیوں کا سراغ لگانا۔ AI آپ کو سکھاتا ہے کہ غلطیوں کے متعدد ممکنہ اسباب ہو سکتے ہیں (غلط راستہ، غلط شیل میں سورسنگ، نامکمل تنصیب)، اور آپ انہیں ایک ایک کرکے مسترد کر سکتے ہیں۔

---

**اختیاری توسیع**:

اگر آپ اپنی مخصوص تنصیب کے بارے میں متجسس ہیں، تو پوچھیں:

```
Show me what happens when I run 'source /opt/ros/humble/setup.bash' step-by-step.
What environment variables change? Which files get added to PATH?
```

یہ ROS 2 تنصیب کے کام کرنے کے طریقے کے بارے میں آپ کے ذہنی ماڈل کو گہرا کرتا ہے۔

---

## چیک پوائنٹ

اگلے سبق 3.2 پر جانے سے پہلے، تصدیق کریں کہ آپ یہ کر سکتے ہیں:

- [ ] `echo $ROS_DISTRO` چلائیں اور `humble` دیکھیں
- [ ] `ros2 --version` چلائیں اور ورژن آؤٹ پٹ دیکھیں
- [ ] `ros2 pkg list | head -5` چلائیں اور پیکیجز دیکھیں
- [ ] (اگر مقامی راستہ) سمجھیں کہ ROS 2 کہاں انسٹال ہے (`/opt/ros/humble/`)
- [ ] (اگر کلاؤڈ راستہ) جانیں کہ اپنے The Construct ٹرمینل تک کیسے رسائی حاصل کریں۔

اگر تمام خانے چیک ہو گئے ہیں، تو آپ سبق 3.2 میں turtlesim لانچ کرنے کے لیے تیار ہیں۔

اگر کوئی ناکام ہو جاتا ہے، تو خرابیوں کا سراغ لگانے والے سیکشن کو دوبارہ پڑھیں یا مدد کے لیے اپنے AI ٹول سے پوچھیں۔

**اگلا سبق**: [→ سبق 3.2: ایکشن میں ٹرٹلسم](./02-turtlesim-action.md)