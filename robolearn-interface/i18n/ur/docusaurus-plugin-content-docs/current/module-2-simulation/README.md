---
id: module-2-simulation
title: 'Module 2: Gazebo/Unity Simulation'
sidebar_position: 2
sidebar_label: 'Module 2: Simulation'
description: >-
  Learn robotics simulation using Gazebo Harmonic - from digital twins to ROS 2
  integration
---


# ماڈیول 2: Gazebo/Unity سمولیشن

**مدت**: ~4 ہفتے (22 اسباق) | **قابلیت**: A2 → B1 | **ہارڈویئر**: ٹائر 1 (کلاؤڈ)

ماڈیول 2 میں خوش آمدید! آپ نے ماڈیول 1 میں ROS 2 کی بنیادی باتیں سیکھ لی ہیں۔ اب آپ حقیقی ہارڈویئر پر تعینات کرنے سے پہلے روبوٹس کو سمولیٹ کرنا سیکھیں گے—یہ پیشہ ورانہ روبوٹکس ڈویلپمنٹ کے لیے انڈسٹری کا معیار ہے۔

## آپ کیا سیکھیں گے

یہ ماڈیول مکمل کرنے کے بعد، آپ یہ کرنے کے قابل ہو جائیں گے:

1. **وضاحت** کریں کہ پیشہ ورانہ روبوٹکس میں جسمانی تعینات سے پہلے سمولیشن کیوں ضروری ہے
2. **URDF** (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) کا استعمال کرتے ہوئے روبوٹ ماڈلز **بنائیں**
3. **SDF** کا استعمال کرتے ہوئے طبیعیات کے ساتھ سمولیشن دنیا **بنائیں**
4. سمولیٹڈ روبوٹس پر سینسرز (کیمرہ، LIDAR، IMU) **کنفیگر** کریں
5. **ros\_gz\_bridge** کا استعمال کرتے ہوئے Gazebo سمولیشنز کو ROS 2 سے **جوڑیں**
6. تمام مہارتوں کو مربوط کرنے والا ایک کیپ اسٹون پروجیکٹ **مکمل** کریں

## پیشگی ضروریات

- **ماڈیول 1**: ROS 2 فاؤنڈیشنز (تمام 7 ابواب)
- **بنیادی XML**: ٹیگز، ایٹریبیوٹس، نیسٹنگ کی سمجھ
- **کمانڈ لائن**: ٹرمینل آپریشنز سے واقفیت

## ہارڈویئر کی ضروریات

| ٹائر | سامان | ضروری؟ |
|------|-----------|-----------|
| **ٹائر 1** | براؤزر + کلاؤڈ | ہاں (تمام مواد یہاں کام کرتا ہے) |
| **ٹائر 2** | مقامی GPU | اختیاری (تیز تکرار کے لیے) |

**کلاؤڈ پاتھ**: تمام مواد [TheConstruct](https://www.theconstructsim.com/) کلاؤڈ ماحول کے ذریعے کام کرتا ہے۔ مقامی تنصیب کی ضرورت نہیں ہے۔

## باب کا جائزہ

### باب 8: سمولیٹ کیوں کریں؟
*3 اسباق | لیئر L1 | قابلیت A2*

سمولیشن-پہلے فلسفہ اور Gazebo کی ساخت کو سمجھیں۔

- [8.1 The Digital Twin Concept](./chapter-8-why-simulate/01-digital-twin-concept.md)
- [8.2 Simulation-First Development](./chapter-8-why-simulate/02-simulation-first.md)
- [8.3 Meet Gazebo Harmonic](./chapter-8-why-simulate/03-meet-gazebo.md)

### باب 9: روبوٹ ڈسکرپشن فارمیٹس
*4 اسباق | لیئر L1→L2 | قابلیت A2*

لنکس، جوائنٹس، اور فزیکل پراپرٹیز کے ساتھ URDF کا استعمال کرتے ہوئے روبوٹ ماڈلز بنائیں۔

- [9.1 Understanding URDF](./chapter-9-robot-description/01-understanding-urdf.md)
- [9.2 Building Your First Robot](./chapter-9-robot-description/02-building-first-robot.md)
- [9.3 Adding Physical Properties](./chapter-9-robot-description/03-adding-physical-properties.md)
- [9.4 URDF with AI](./chapter-9-robot-description/04-urdf-with-ai.md)

### باب 10: سمولیشن دنیا بنانا
*4 اسباق | لیئر L1→L2 | قابلیت A2→B1*

زمین، رکاوٹوں، اور طبیعیات کے ساتھ سمولیشن ماحول ڈیزائن کریں۔

- [10.1 SDF World Basics](./chapter-10-simulation-worlds/01-sdf-world-basics.md)
- [10.2 Adding Models from Fuel](./chapter-10-simulation-worlds/02-adding-models-from-fuel.md)
- [10.3 Physics Configuration](./chapter-10-simulation-worlds/03-physics-configuration.md)
- [10.4 World Building with AI](./chapter-10-simulation-worlds/04-world-building-with-ai.md)

### باب 11: سمولیشن میں سینسرز
*4 اسباق | لیئر L1→L2 | قابلیت B1*

اپنے روبوٹس میں کیمرے، LIDAR، اور IMU سینسرز شامل کریں۔

- [11.1 Camera Simulation](./chapter-11-sensors-simulation/01-camera-simulation.md)
- [11.2 LIDAR Simulation](./chapter-11-sensors-simulation/02-lidar-simulation.md)
- [11.3 IMU and Contact Sensors](./chapter-11-sensors-simulation/03-imu-contact-sensors.md)
- [11.4 Sensor Debugging and Visualization](./chapter-11-sensors-simulation/04-sensor-debugging-visualization.md)

### باب 12: ROS 2 + Gazebo انٹیگریشن
*4 اسباق | لیئر L2→L3 | قابلیت B1*

کلوزڈ لوپ کنٹرول کے لیے Gazebo کو ROS 2 سے جوڑیں۔

- [12.1 The ros_gz Bridge](./chapter-12-ros2-gazebo-integration/01-ros-gz-bridge.md)
- [12.2 Spawning Robots from ROS 2](./chapter-12-ros2-gazebo-integration/02-spawning-robots.md)
- [12.3 Closed-Loop Control](./chapter-12-ros2-gazebo-integration/03-closed-loop-control.md)
- [12.4 Creating ros_gz Skills](./chapter-12-ros2-gazebo-integration/04-creating-ros-gz-skills.md)

### باب 13: ماڈیول 2 کیپ اسٹون
*3 اسباق | لیئر L4 | قابلیت B1*

ایک مکمل سمولیشن پروجیکٹ کے ساتھ مربوط سیکھنے کا مظاہرہ کریں۔

- [13.1 Capstone Specification](./chapter-13-capstone/01-capstone-specification.md)
- [13.2 Building the Simulation](./chapter-13-capstone/02-building-simulation.md)
- [13.3 Testing, Validation, and Sim-to-Real Preview](./chapter-13-capstone/03-testing-validation-preview.md)

## سیکھنے کی پیشرفت

```
Chapter 8 (L1): سمولیٹ کیوں کرنا ہے یہ سمجھیں
    ↓
Chapter 9 (L1→L2): کیا سمولیٹ کرنا ہے (روبوٹس) بنائیں
    ↓
Chapter 10 (L1→L2): کہاں سمولیٹ کرنا ہے (دنیا) بنائیں
    ↓
Chapter 11 (L1→L2): روبوٹس کیسے محسوس کرتے ہیں (سینسرز) شامل کریں
    ↓
Chapter 12 (L2→L3): کنٹرول کے لیے ROS 2 سے جوڑیں
    ↓
Chapter 13 (L4): کیپ اسٹون میں سب کچھ مربوط کریں
```

## مہارتیں جو آپ بنائیں گے

یہ ماڈیول مکمل کرنے کے بعد، آپ کے پاس 4 دوبارہ استعمال کے قابل مہارتیں ہوں گی:

1. **urdf-robot-model**: مناسب طبیعیات کے ساتھ روبوٹ کی تفصیلات بنائیں
2. **gazebo-world-builder**: سمولیشن ماحول ڈیزائن کریں
3. **sensor-simulation**: کیمرے، LIDAR، IMU کنفیگر کریں
4. **ros2-gazebo-bridge**: Gazebo کو ROS 2 سے جوڑیں

## آگے کیا ہے

ماڈیول 2 مکمل کرنے کے بعد، آپ اس کے لیے تیار ہیں:

- **ماڈیول 3**: NVIDIA Isaac Sim — ڈومین رینڈمائزیشن کے ساتھ AI سے چلنے والی سمولیشن
- **ماڈیول 4**: VLA/Embodied AI — روبوٹ کنٹرول کے لیے ویژن-لینگویج-ایکشن ماڈلز

---

**شروع کرنے کے لیے تیار ہیں؟** [باب 8: سمولیٹ کیوں کریں؟](./chapter-8-why-simulate/README.md) سے شروع کریں