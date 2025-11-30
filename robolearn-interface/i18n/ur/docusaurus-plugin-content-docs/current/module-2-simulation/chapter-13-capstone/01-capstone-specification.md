---
id: lesson-13-1-capstone-specification
title: 'Lesson 13.1: Capstone Specification'
sidebar_position: 1
sidebar_label: 13.1 Capstone Specification
description: Write clear specifications for simulation projects before building them
duration_minutes: 60
proficiency_level: B1
layer: L4
hardware_tier: 1
learning_objectives:
  - Write clear intent statements that define the problem you're solving
  - 'List robot, world, sensor, and behavior requirements systematically'
  - Define measurable success criteria that you can validate later
  - Identify constraints that shape your design decisions
  - Evaluate a vague specification and identify missing information
skills:
  - specification-writing
cognitive_load:
  new_concepts: 6
tier_1_path: TheConstruct cloud environment
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 13.1: کیپ اسٹون تفصیلات (Capstone Specification)

AI-نیٹو ڈویلپمنٹ میں، **تفصیلات (specifications) ہی نیا سینٹیکس (syntax) ہیں۔** آپ سیدھے کوڈ یا URDF فائلوں میں نہیں کودتے۔ آپ پہلے اپنا ارادہ (intent) لکھتے ہیں۔ ایک واضح تفصیل یہ سوال کا جواب دیتی ہے: "میں کون سا مسئلہ حل کر رہا ہوں، اور مجھے کیسے پتہ چلے گا کہ میں نے اسے حل کر لیا ہے؟"

یہ سبق وہ مہارت سکھاتا ہے جو آپ اپنے کیریئر میں استعمال کریں گے: ایسی تفصیلات لکھنا جو ڈیزائن کی رہنمائی کریں اور نتائج کی توثیق (validate) کریں۔

---

## تفصیلات-پہلے کا اصول (The Specification-First Principle)

تفصیلات کو اپنے اور اپنے مستقبل کے آپ (implementer) کے درمیان ایک معاہدے کی طرح سمجھیں:

**آپ کا مستقبل کا آپ پوچھتا ہے**: "میں نے بالکل کیا بنانے پر اتفاق کیا تھا؟"
**تفصیلات جواب دیتی ہے**: "ایک ایسا روبوٹ بنائیں جو گودام میں نیویگیٹ کرے، رکاوٹوں سے بچ جائے، اور 5 منٹ کے اندر ایک مقررہ مقام پر پہنچ جائے۔ کامیابی کا مطلب ہے کہ روبوٹ 10 میں سے 8 بار ہدف تک پہنچ جائے۔"

اس وضاحت کے بغیر، عمل درآمد (implementation) بھٹک جاتا ہے۔ آپ ایسی خصوصیات بناتے ہیں جو مسئلہ حل نہیں کرتیں۔ آپ ایسی بہتریوں کو نافذ کرتے ہیں جن کی کوئی اہمیت نہیں ہے۔ آپ ایسی چیزوں کی توثیق کرتے ہیں جو اصل ہدف نہیں تھیں۔

**تفصیلات کی وضاحت کے ساتھ**:
- آپ کو بالکل معلوم ہوتا ہے کہ کیا بنانا ہے
- آپ تصدیق کر سکتے ہیں کہ کب کام مکمل ہوا
- آپ دوسروں کو ڈیزائن کے انتخاب کی وضاحت کر سکتے ہیں

---

## تفصیلات کا ڈھانچہ: پانچ سوالات

ایک مکمل تفصیل ان پانچ سوالات کا جواب دیتی ہے:

### سوال 1: ارادہ کیا ہے؟ (What is the intent?)

ایک یا دو جملے جو اس مسئلے کی وضاحت کرتے ہیں جسے آپ حل کر رہے ہیں۔ یہ نہیں کہ آپ اسے *کیسے* حل کریں گے، بلکہ یہ کہ آپ *کیا* حل کر رہے ہیں۔

**مثال (ڈیلیوری روبوٹ)**:
> "ایک ایسا روبوٹ بنائیں جو گودام کے راہداریوں میں نیویگیٹ کرتے ہوئے اور رکاوٹوں سے بچتے ہوئے، خود مختار طور پر پارسل پہنچائے، اور مقررہ ڈراپ آف پوائنٹس پر پہنچائے۔"

**مثال (معائنہ روبوٹ)**:
> "ایک ایسا روبوٹ بنائیں جو آن بورڈ کیمرے کا استعمال کرتے ہوئے گودام کے بنیادی ڈھانچے (شیلف، بیم، چھت) کا معائنہ کرے، اور بعد میں تجزیہ کے لیے تصاویر اکٹھی کرے۔"

**کیا شامل نہیں کرنا ہے**:
- ❌ نفاذ کی تفصیلات ("ROS 2 اور Gazebo استعمال کرتا ہے")
- ❌ ٹیکنالوجی کے انتخاب ("ڈیپ لرننگ" یا "URDF ماڈلز")
- ❌ عمل کی تفصیلات ("100 بار تکرار کرتا ہے")

**کیا شامل کرنا ہے**:
- ✅ ہدف (پہنچانا، معائنہ کرنا، نیویگیٹ کرنا، نقشہ بنانا)
- ✅ ماحول (گودام، دفتر، باہر کا علاقہ)
- ✅ حدود (رکاوٹیں، تنگ جگہیں، وقت کی حدیں)

---

### سوال 2: روبوٹ کی ضروریات کیا ہیں؟ (What are the robot requirements?)

آپ کے روبوٹ کے لیے درکار جسمانی ڈھانچے کی فہرست بنائیں:
- **لنکس (Links)** (چیسس، پہیے، بازو، پکڑنے والا، سینسر)
- **جوڑ (Joints)** (پہیے کے موٹر، بازو کے جوڑ، پکڑنے والے کی حرکت)
- **سینسرز (Sensors)** (کیمرہ، LIDAR، IMU، بمپر)
- **ایکچویٹرز (Actuators)** (کیا حرکت کرتا ہے: پہیے، بازو، پکڑنے والا)

**مثال (ڈیلیوری روبوٹ)**:
```
Robot Requirements:
- Chassis: 50cm x 30cm x 40cm footprint (fits warehouse corridors)
- Wheels: 2 differential drive wheels (enable turning)
- Bumpers: Front and rear contact sensors (obstacle detection)
- Camera: Forward-facing for navigation
- LIDAR: 2D scanner for obstacle mapping
```

**مثال (معائنہ روبوٹ)**:
```
Robot Requirements:
- Arm: 5-joint robotic arm (reach 1.5m height for shelf inspection)
- Camera: High-resolution mounted on arm end-effector
- IMU: Track arm vibration and acceleration
- Base: Wheeled platform for mobility
```

---

### سوال 3: دنیا کی ضروریات کیا ہیں؟ (What are the world requirements?)

اس ماحول کی وضاحت کریں جہاں آپ کا روبوٹ کام کرتا ہے:
- **منظر (Layout)** (گودام، دفتری عمارت، باہر کا راستہ)
- **رکاوٹیں (Obstacles)** (شیلف، دیواریں، زمین پر اشیاء)
- **سطح کی خصوصیات (Surface properties)** (کنکریٹ، قالین، کھردرا علاقہ)
- **روشنی (Lighting)** (اندرونی، بیرونی، متغیر سائے)
- **ابتدائی حالات (Initial conditions)** (شروع کی پوزیشن، ہدف کا مقام)

**مثال (ڈیلیوری روبوٹ)**:
```
World Requirements:
- Warehouse floor: 30m x 20m rectangular space
- Shelves: 4 rows of 10m shelves creating corridors 2m wide
- Obstacles: Cardboard boxes randomly placed on floor (10-20 per simulation)
- Physics: Gravity 9.81 m/s², friction coefficient 0.5
- Starting position: Dock at (0, 0)
- Goal positions: 5 drop-off points at (30, 5), (30, 10), (30, 15), (30, 20), (30, 25)
```

---

### سوال 4: رویے کی ضروریات کیا ہیں؟ (What are the behavior requirements?)

بیان کریں کہ روبوٹ *کیا کرتا ہے*:
- ** ادراک (Perception)** (یہ کیا محسوس کرتا ہے اور سمجھتا ہے؟)
- **فیصلہ سازی (Decision-making)** (کون سا منطق اس کے اعمال کا تعین کرتا ہے؟)
- **نفاذ (Execution)** (یہ کون سے احکامات بھیجتا ہے؟)

**مثال (ڈیلیوری روبوٹ)**:
```
Behavior Requirements:
- Perception: Use LIDAR to detect obstacles within 2m radius
- Decision: If obstacle closer than 0.5m, stop; otherwise move forward
- Navigation: Follow path using odometry (dead reckoning) to reach goal
- Execution: Publish velocity commands to drive toward goal waypoints
```

---

### سوال 5: کامیابی کے معیار کیا ہیں؟ (What are the success criteria?)

قابل پیمائش نتائج کی وضاحت کریں جن کی آپ بعد میں توثیق کریں گے:
- **قابل جانچ ہونا چاہیے** (کیا آپ اسے سمولیشن میں ماپ سکتے ہیں؟)
- **قابل مقدار ہونا چاہیے** (فیصد، وقت، گنتی، فاصلہ)
- **مسئلے سے متعلق ہونا چاہیے** (نفاذ کی تفصیلات نہیں)

**مثال (ڈیلیوری روبوٹ)**:
```
Success Criteria:
1. Robot reaches goal location within 5 minutes (time limit)
2. Robot successfully avoids all obstacles (never collides)
3. Robot reaches goal 8 out of 10 times (reliability)
4. Robot path is reasonably efficient (not wandering randomly)
```

**مثال (معائنہ روبوٹ)**:
```
Success Criteria:
1. Arm reaches all shelf heights (0.5m to 2.0m)
2. Camera captures clear images from minimum 0.5m distance
3. Inspection completes in under 10 minutes
4. Robot detects and reports placement of 5 marked targets
```

---

## مثال کی تفصیلات: ڈیلیوری روبوٹ

یہ ایک مکمل تفصیل ہے جسے آپ سبق 13.2 میں نافذ کریں گے:

```
CAPSTONE SPECIFICATION: Autonomous Warehouse Delivery Robot

INTENT
------
Build a robot that autonomously delivers packages from a central dock
to designated drop-off points in a warehouse, navigating corridors while
avoiding obstacles.

ROBOT REQUIREMENTS
------------------
- Chassis: 50cm x 30cm x 40cm (fits 2m-wide warehouse corridors)
- Wheels: 2 differential drive wheels (rear) + caster (front)
- Motors: Wheel velocity control (0 to 1.0 m/s)
- Sensors:
  * LIDAR (2D, 10m range, 270° field of view)
  * Camera (front-facing, for navigation landmarks)
  * IMU (track acceleration, orientation)
  * Bumpers (front and rear contact switches)
- Payload: Gripper to hold packages (simplified as model attachment)

WORLD REQUIREMENTS
------------------
- Environment: Warehouse 30m x 20m
- Obstacles:
  * Shelves: 4 rows x 10m long, 1.5m high, 2m spacing creates corridors
  * Floor objects: 15-20 cardboard boxes randomly distributed
  * Walls: Perimeter enclosure
- Physics: Standard gravity (9.81 m/s²), friction 0.5, soft contacts
- Lighting: Uniform indoor lighting
- Initial State:
  * Robot starts at dock (position 0, 0)
  * Goal locations: (30, 5), (30, 10), (30, 15), (30, 20), (30, 25)
  * Boxes placed randomly in corridors

SENSOR REQUIREMENTS
-------------------
- LIDAR: Publish obstacle distance to /scan topic (sensor_msgs/LaserScan)
- Camera: Publish images to /camera/image_raw (sensor_msgs/Image)
- IMU: Publish acceleration/orientation to /imu/data (sensor_msgs/Imu)
- Bumper: Publish contact events to /bumper (sensor_msgs/BumperEvent)

BEHAVIOR REQUIREMENTS
---------------------
- Perception: Use LIDAR to detect obstacles within 3m radius
- Navigation:
  * From dock, navigate to first goal using odometry
  * Detect obstacles; adjust course to avoid
  * Reach goal location (within 0.5m tolerance)
  * Return to dock for next delivery
- Safety: Never collide with obstacles (bumper reading = failure)
- Communication: ROS 2 node subscribes to /goal and publishes to /cmd_vel

CONSTRAINTS
-----------
- Only ROS 2 Humble available (no custom packages)
- Simulation limited to 10 minute run time per trial
- No ML/neural networks (behavior is rule-based)
- Gazebo Harmonic physics simulator

SUCCESS CRITERIA
----------------
1. Deliverability: Robot reaches all 5 goal locations (100% success rate in single run)
2. Safety: No collisions with obstacles (bumper never triggered)
3. Efficiency: Complete all 5 deliveries in under 5 minutes
4. Reliability: Repeat simulation 3 times; succeed in 3/3 runs
5. ROS 2 Integration: All sensor topics publish data; cmd_vel commands work
```

یہ تفصیلات اتنی مخصوص ہیں کہ آپ کو بالکل معلوم ہے کہ کیا نافذ کرنا ہے، پھر بھی اتنی لچکدار ہیں کہ آپ ڈیزائن کے فیصلے کر سکتے ہیں (مثلاً LIDAR کی درست ٹیوننگ، رکاوٹ سے بچنے کا الگورتھم، وغیرہ)۔

---

## ایک اچھی تفصیلات کیا بناتی ہے؟

**اچھی تفصیلات** (واضح، جانچنے کے قابل):
```
Success Criteria:
- Robot reaches goal location within 5 minutes
- Robot avoids 100% of obstacles (zero collisions)
- Robot completes 5 deliveries in single run
- Behavior succeeds in 3/3 trial runs
```

قابل پیمائش؟ ہاں (وقت، ٹکراؤ کی تعداد، کامیابی کی شرح)
قابل جانچ؟ ہاں (سمولیشن میں ہر معیار کی توثیق کی جا سکتی ہے)
مسئلے سے متعلق؟ ہاں (تمام معیار ڈیلیوری کے کام کے لیے اہم ہیں)

**خراب تفصیلات** (مبہم، ناقابل جانچ):
```
Success Criteria:
- Robot works well
- Behavior is good
- Simulation runs
```

قابل پیمائش؟ نہیں ( "اچھی طرح کام کرتا ہے" کا کیا مطلب ہے؟)
قابل جانچ؟ نہیں ("اچھا" کا کیا مطلب ہے میٹر یا سیکنڈ میں؟)
مسئلے سے متعلق؟ نہیں (یہ ڈیلیوری کی صلاحیت کو نہیں ماپتے)

---

## مشق: اپنی کیپ اسٹون تفصیلات لکھیں

**آپ کا کام**: اپنے سمولیشن پروجیکٹ کے انتخاب کے لیے ایک مکمل کیپ اسٹون تفصیلات لکھیں۔

**اختیارات** (ایک کا انتخاب کریں، یا اپنا ایجاد کریں):

**آپشن 1: ڈیلیوری روبوٹ** (اوپر دی گئی مثال پر عمل کریں)
- گودام کے منظر یا اہداف میں ترمیم کریں
- سینسر کی ترتیب تبدیل کریں
- حدود شامل کریں (وقت کی حدیں، راستے کی پابندیاں)

**آپشن 2: معائنہ روبوٹ**
- روبوٹ کا بازو مختلف اونچائیوں پر اشیاء کا معائنہ کرتا ہے
- کیمرہ تجزیہ کے لیے تصاویر لیتا ہے
- کامیابی: بازو تمام ہدف کی اونچائیوں تک پہنچتا ہے، 10 سے زیادہ تصاویر لیتا ہے

**آپشن 3: تلاش روبوٹ (Exploration Robot)**
- روبوٹ نامعلوم ماحول میں نیویگیٹ کرتا ہے اور نقشہ بناتا ہے
- دیواروں اور رکاوٹوں کا پتہ لگانے کے لیے LIDAR استعمال کرتا ہے
- کامیابی: نقشہ 80% ماحول کا احاطہ کرتا ہے، کوئی ٹکراؤ نہیں

**آپشن 4: تلاش اور بچاؤ روبوٹ (Search and Rescue Robot)**
- روبوٹ ملبے جیسے ماحول میں اشیاء کو تلاش کرتا اور "بچاتا" ہے
- رنگین اہداف کا پتہ لگانے کے لیے کیمرہ استعمال کرتا ہے
- کامیابی: 10 منٹ سے کم وقت میں 5 ہدف اشیاء تلاش کرتا ہے

**آپ کی تفصیلات میں شامل ہونا چاہیے**:
- ✅ ارادے کا بیان (1-2 جملے)
- ✅ روبوٹ کی ضروریات (جسمانی ڈھانچہ، سینسر، ایکچویٹرز)
- ✅ دنیا کی ضروریات (ماحول، رکاوٹیں، ابتدائی حالات)
- ✅ سینسر کی ضروریات (کون سے سینسر، کون سے ٹاپکس)
- ✅ رویے کی ضروریات (روبوٹ کیا کرتا ہے)
- ✅ کامیابی کے معیار (5 قابل پیمائش نتائج)
- ✅ حدود (آپ کیا *نہیں* کر رہے ہیں)

**جائزہ**:
- کیا آپ کا ارادہ واضح ہے؟ (کیا کوئی 30 سیکنڈ میں ہدف سمجھ سکتا ہے؟)
- کیا ضروریات مخصوص ہیں؟ (کیا آپ تفصیلات سے انہیں نافذ کر سکتے ہیں؟)
- کیا کامیابی کے معیار قابل پیمائش ہیں؟ (کیا آپ انہیں سمولیشن میں جانچ سکتے ہیں؟)
- کیا پروجیکٹ معقول ہے؟ (کیا آپ اسے 90 منٹ میں نافذ کر سکتے ہیں؟ سبق 13.2 دیکھیں)

---

## AI کے ساتھ کوشش کریں

**سیٹ اپ**: ChatGPT یا Claude کھولیں اور اپنی کیپ اسٹون تفصیلات کو بہتر بنائیں۔

**پرامپٹ سیٹ**:

**پرامپٹ 1** (اپنی تفصیلات کی توثیق):
```
I've written a capstone specification for a robot simulation project:

[Paste your specification here]

Review this specification and identify:
1. Ambiguous requirements (what's unclear?)
2. Missing information (what's not specified?)
3. Unrealistic success criteria (what's too hard?)
4. How to make it more specific and measurable
```

**پرامپٹ 2** (اچھی تفصیلات سے سیکھنا):
```
Show me an example of a well-written specification for a robotic arm
task (e.g., bin picking, object placement). Include:
- Clear intent
- Robot and world requirements
- 5 measurable success criteria
- Explain what makes each criterion testable
```

**پرامپٹ 3** (اپنی تفصیلات کو بہتر بنانا):
```
I'm building a [YOUR_ROBOT_TYPE] in Gazebo. Here's my current spec:

[Paste your specification]

What should I change or clarify to make this easier to implement?
Which success criteria will be hardest to achieve? What's a realistic time frame?
```

**متوقع نتائج**:
- AI آپ کی تفصیلات میں مبہم پن کی نشاندہی کرتا ہے
- AI اچھی طرح لکھی گئی تفصیلات کی مثالیں فراہم کرتا ہے
- AI آپ کی سمولیشن کی صلاحیتوں کی بنیاد پر معقول حدود تجویز کرتا ہے

**حفاظتی نوٹ**: آپ کی تفصیلات آپ کے نفاذ کی رہنمائی کرتی ہیں۔ ایک مبہم تفصیلات غلط چیز بنانے میں وقت ضائع کرنے کا باعث بنتی ہیں۔ ایک واضح تفصیلات الجھن کے 10 گھنٹے بچاتی ہیں۔

---