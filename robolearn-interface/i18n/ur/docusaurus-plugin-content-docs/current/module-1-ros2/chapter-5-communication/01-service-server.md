---
id: service-server
title: 'Lesson 5.1: Building Service Servers'
sidebar_label: 5.1 Service Servers
sidebar_position: 1
chapter: 5
lesson: 1
duration_minutes: 60
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 3
learning_objectives:
  - Understand request/response communication patterns in ROS 2
  - Create a service server node using rclpy
  - Handle synchronous service requests with callbacks
  - Compare services to topics based on use cases
  - Extend service implementations using AI feedback
skills:
  - ros2-service-pattern
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 5.1: سروس سرور بنانا

آپ نے سیکھا ہے کہ پبلشرز اور سبسکرائبرز ڈیٹا کی مسلسل سٹریم کیسے بھیجتے ہیں۔ لیکن کیا ہو اگر آپ کو ایک سنکرونس (synchronous) ریکویسٹ اور رسپانس کی ضرورت ہو؟ کیا ہو اگر کسی روبوٹ کو کوئی کمانڈ چلانی ہو اور یہ رپورٹ کرنا ہو کہ وہ کامیاب ہوئی یا نہیں؟

سروسز میں خوش آمدید۔

اسے فون کال کرنے جیسا سمجھیں۔ جب آپ کسی ٹاپک پر پبلش کرتے ہیں، تو یہ میگا فون پر براڈکاسٹ کرنے جیسا ہے—کوئی بھی سننے والا آپ کو سنتا ہے، لیکن کوئی جواب نہیں دیتا۔ جب آپ کسی سروس کو کال کرتے ہیں، تو یہ فون پر کسی دوست کو کال کرنے جیسا ہے—آپ ایک سوال پوچھتے ہیں، جواب کا انتظار کرتے ہیں، اور پھر آگے بڑھتے ہیں۔

اس سبق میں، آپ ایک سروس سرور بنائیں گے—ایک ایسا نوڈ جو ریکویسٹس وصول کرتا ہے، ان پر عمل کرتا ہے، اور جوابات واپس بھیجتا ہے۔ آخر تک، آپ سمجھ جائیں گے کہ ٹاپکس کے بجائے سروسز کا استعمال کب کرنا ہے، اور دوسرے نوڈز سے سنکرونس ریکویسٹس کو کیسے ہینڈل کرنا ہے۔

---

## ٹاپکس بمقابلہ سروسز: کس کا استعمال کب کریں

کوڈ میں کودنے سے پہلے، آئیے فرق واضح کریں۔ آپ ٹاپکس پہلے ہی جانتے ہیں۔ سروسز اس لیے مختلف ہیں کیونکہ وہ **سنکرونس** اور **ریکویسٹ/رسپانس** پر مبنی ہوتی ہیں۔

| پہلو | ٹاپکس | سروسز |
|--------|--------|----------|
| **پیٹرن** | پبلش/سبسکرائب | ریکویسٹ/رسپانس |
| **فلو** | فائر اینڈ فارگیٹ (بھیجو اور بھول جاؤ) | جواب کا انتظار کریں |
| **فریکوئنسی** | مسلسل (زیادہ شرح پر) | کبھی کبھار (طلب پر) |
| **استعمال کا کیس** | سینسر ڈیٹا، مسلسل سٹریمز | کمانڈز، استفسارات، کنفیگریشن |
| **مثال** | درجہ حرارت کا سینسر ہر 0.1 سیکنڈ میں پبلش کرتا ہے | "روبوٹ کو 1 میٹر آگے بڑھاؤ" کمانڈ |

**فیصلہ سازی کا اصول:**
- **ٹاپکس استعمال کریں** اگر وصول کنندہ کو جواب دینے کی ضرورت نہیں ہے
- **سروسز استعمال کریں** اگر بھیجنے والے کو اس بات کی تصدیق کی ضرورت ہے کہ کچھ ہوا ہے

حقیقی روبوٹکس مثالیں:
- **ٹاپک**: کیمرہ مسلسل تصاویر پبلش کرتا ہے
- **سروس**: روبوٹ "چیز اٹھاؤ" کمانڈ چلاتا ہے اور کامیابی/ناکامی کی تصدیق کرتا ہے

---

## سروس سرور پیٹرن

تمام ROS 2 سروس سرور ایک ہی ڈھانچے پر عمل کرتے ہیں۔ یہاں ایک مکمل مثال ہے:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        # وہ سروس بنائیں جو ریکویسٹس کو ہینڈل کرے گی
        self.srv = self.create_service(
            AddTwoInts,           # سروس کی قسم (ریکویسٹ/رسپانس فارمیٹ)
            'add_two_ints',       # سروس کا نام (کلائنٹس اسے کیسے کال کرتے ہیں)
            self.add_callback)    # ریکویسٹس کو ہینڈل کرنے کے لیے فنکشن

    def add_callback(self, request, response):
        """جب کوئی کلائنٹ ریکویسٹ بھیجتا ہے تو کال کیا جاتا ہے۔"""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = CalculatorService()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

آئیے دیکھیں کہ کیا ہو رہا ہے:

### کوڈ کو سمجھنا

**امپورٹس (Imports):**

```python
from example_interfaces.srv import AddTwoInts
```

یہ `AddTwoInts` سروس کی قسم کو امپورٹ کرتا ہے۔ یہ ایک بلٹ ان ROS 2 سروس ہے جس میں یہ چیزیں ہیں:
- **ریکویسٹ**: دو انٹیجرز `a` اور `b`
- **رسپانس**: ایک انٹیجر `sum`

**سروس بنانا:**

```python
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_callback)
```

یہ لائن ایک سروس سرور بناتی ہے:
- **پہلا دلیل** (`AddTwoInts`): سروس کی قسم (ریکویسٹ/رسپانس فارمیٹ کی وضاحت کرتا ہے)
- **دوسرا دلیل** (`'add_two_ints'`): سروس کا نام (کلائنٹس اس سروس کا حوالہ کیسے دیتے ہیں)
- **تیسرا دلیل** (`self.add_callback`): کال بیک فنکشن (جب کوئی ریکویسٹ آتی ہے تو چلتا ہے)

**کال بیک فنکشن:**

```python
def add_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(...)
    return response
```

جب کوئی کلائنٹ ریکویسٹ بھیجتا ہے:
1. `request` میں وہ ڈیٹا ہوتا ہے جو کلائنٹ نے بھیجا (یہاں: `a` اور `b`)
2. آپ اس پر عمل کرتے ہیں اور `response` کو پُر کرتے ہیں (یہاں: جمع کا نتیجہ)
3. `response` آبجیکٹ واپس کرتے ہیں
4. ROS 2 خود بخود اسے کلائنٹ کو واپس بھیج دیتا ہے

**ٹاپکس سے فرق:**

ٹاپکس کے ساتھ، آپ پبلش کرتے ہیں اور بھول جاتے ہیں۔ سروسز کے ساتھ، آپ رسپانس آبجیکٹ کا **انتظار** کرتے ہیں۔ جب تک آپ کا کال بیک واپس نہیں آتا، کلائنٹ بلاک رہتا ہے۔

---

## اپنا پہلا سروس سرور بنانا

آئیے ایک حقیقی مثال بناتے ہیں۔ ہم ایک ایسی سروس بنائیں گے جسے روبوٹ خود کو "فعال" (enable) یا "غیر فعال" (disable) کرنے کے لیے استعمال کر سکتا ہے—ایک سادہ کنٹرول کمانڈ۔

**مرحلہ 1: اپنے پیکیج پر جائیں**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
```

**مرحلہ 2: سروس سرور فائل بنائیں**

```bash
cat > enable_service.py << 'EOF'
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class RobotEnabler(Node):
    def __init__(self):
        super().__init__('robot_enabler')
        self.robot_enabled = False

        # سروس بنائیں: کلائنٹس 'enable_robot' کو True/False کے ساتھ کال کریں گے
        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_callback)

        self.get_logger().info('Robot enabler service ready')

    def enable_callback(self, request, response):
        """enable/disable ریکویسٹس کو ہینڈل کریں۔"""
        self.robot_enabled = request.data  # ریکویسٹ میں True یا False ہوتا ہے
        response.success = True

        if self.robot_enabled:
            response.message = 'Robot enabled'
            self.get_logger().info('Robot has been ENABLED')
        else:
            response.message = 'Robot disabled'
            self.get_logger().info('Robot has been DISABLED')

        return response

def main(args=None):
    rclpy.init(args=args)
    enabler = RobotEnabler()
    rclpy.spin(enabler)
    enabler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**مرحلہ 3: اسے setup.py میں شامل کریں**

`setup.py` میں ترمیم کریں (بیرونی `my_first_package` فولڈر میں) اور ایگزیکیوٹیبل شامل کریں:

```python
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
        'minimal_subscriber = my_first_package.minimal_subscriber:main',
        'enable_service = my_first_package.enable_service:main',  # یہ لائن شامل کریں
    ],
},
```

**مرحلہ 4: بلڈ اور سورس کریں**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## سروس چلانا

**ٹرمینل 1: سروس سرور شروع کریں**

```bash
ros2 run my_first_package enable_service
```

آپ کو یہ نظر آنا چاہیے:

```
[INFO] [rclpy]: Robot enabler service ready
```

سروس اب ریکویسٹس کا انتظار کر رہی ہے۔ یہ باہر نہیں نکلتی—یہ چلتی رہتی ہے۔

**ٹرمینل 2: سروس کو کال کریں**

اب آئیے اسے ریکویسٹ بھیجتے ہیں۔ ہم اسے فعال کرنے کے لیے کہیں گے:

```bash
ros2 service call /enable_robot std_srvs/SetBool "data: true"
```

آپ کو یہ نظر آنا چاہیے:

```
response:
  success: true
  message: 'Robot enabled'
```

اور ٹرمینل 1 میں:

```
[INFO] [robot_enabler]: Robot has been ENABLED
```

**ٹرمینل 2: اسے دوبارہ False کے ساتھ کال کریں**

```bash
ros2 service call /enable_robot std_srvs/SetBool "data: false"
```

رسپانس:

```
response:
  success: true
  message: 'Robot disabled'
```

اور ٹرمینل 1 میں:

```
[INFO] [robot_enabler]: Robot has been DISABLED
```

---

## یہاں کیا ہوا

ٹاپکس سے اہم فرق:

1. **سروس بنائی گئی** اور ریکویسٹس کا انتظار کر رہی ہے
2. **کلائنٹ نے کال کی** سروس کو ڈیٹا کے ساتھ (`data: true`)
3. **سرور نے** کال بیک میں ریکویسٹ **وصول کی**
4. **سرور نے** اس پر عمل کیا اور رسپانس کو پُر کیا
5. **کلائنٹ بلاک رہا** جب تک رسپانس نہیں آیا
6. **کلائنٹ نے** رسپانس پرنٹ کیا
7. **سرور چلتا رہا**، اگلی ریکویسٹ کے لیے تیار

ٹاپک پر پبلش کرنے کے برعکس، کلائنٹ جواب کا **انتظار کرتا ہے**۔ سروس **سنکرونس** ہے۔

---

## سروس کی اقسام کو سمجھنا

`SetBool` کیا ہے؟ یہ ایک بلٹ ان سروس کی قسم ہے جس میں یہ چیزیں ہیں:
- **ریکویسٹ فیلڈ**: `data` (بولین)
- **رسپانس فیلڈز**: `success` (بولین) اور `message` (سٹرنگ)

دیگر عام بلٹ ان سروسز:
- `AddTwoInts`: دو انٹیجرز کو جمع کرتا ہے
- `SetBool`: بولین کنٹرول (جیسے فعال/غیر فعال)
- `Trigger`: کوئی ریکویسٹ ڈیٹا نہیں، صرف "یہ کرو" (رسپانس: کامیابی + پیغام)

---

## اہم نکات

**سنکرونس بمقابلہ اسنکرونس:**
سروسز کلائنٹ کے نقطہ نظر سے سنکرونس ہیں—کلائنٹ انتظار کرتا ہے۔ سرور کال بیک کے اندر، تیز رہیں۔ بھاری کمپیوٹیشن دوسری ریکویسٹس کو بلاک کر دیتی ہے۔

**نامزد سروسز:**
سروس کا نام `ros2 service list` میں ظاہر ہوتا ہے۔ نام رکھنے کے اصول:
- انڈرسکور کے ساتھ چھوٹے حروف استعمال کریں
- `/service` یا `/command` جیسے عام ناموں سے گریز کریں
- `/robot_enable`, `/motor_status`, `/calculate_path` جیسے نام استعمال کریں

**رسپانس ڈھانچہ:**
واپس کرنے سے پہلے ہمیشہ پورے رسپانس آبجیکٹ کو پُر کریں۔ کلائنٹ توقع کرتا ہے کہ تمام فیلڈز پُر ہوں۔

---

## سروس کو بڑھانا

اب آئیے اسے مزید حقیقت پسندانہ بناتے ہیں۔ کیا ہو اگر ہم روبوٹ کو فعال کرنے پر اس کی بیٹری لیول رپورٹ کرنا چاہیں؟

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class RobotEnablerV2(Node):
    def __init__(self):
        super().__init__('robot_enabler_v2')
        self.robot_enabled = False
        self.battery_level = 85.0  # بیٹری کی نقل کریں

        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_callback)

        self.get_logger().info('Robot enabler v2 ready')

    def enable_callback(self, request, response):
        """بیٹری چیک کے ساتھ فعال/غیر فعال کریں۔"""
        if request.data and self.battery_level < 10.0:
            # اگر بیٹری بہت کم ہے تو فعال نہ کریں
            response.success = False
            response.message = f'Battery too low: {self.battery_level}%'
            self.get_logger().warn('Enable rejected: low battery')
            return response

        self.robot_enabled = request.data
        response.success = True

        if self.robot_enabled:
            response.message = f'Robot enabled. Battery: {self.battery_level}%'
            self.get_logger().info(f'Robot enabled (Battery: {self.battery_level}%)')
        else:
            response.message = 'Robot disabled'
            self.get_logger().info('Robot disabled')

        return response

def main(args=None):
    rclpy.init(args=args)
    enabler = RobotEnablerV2()
    rclpy.spin(enabler)
    enabler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

اب سروس فعال کرنے سے پہلے بیٹری چیک کرتی ہے۔ یہ بیٹری لیول کے بارے میں پیغام کے ساتھ جواب دیتی ہے۔ کلائنٹ اس پیغام کو پڑھ سکتا ہے اور فیصلہ کر سکتا ہے کہ کیا کرنا ہے۔

---

## AI کے ساتھ کوشش کریں

آپ کے پاس ایک کام کرنے والا سروس سرور ہے۔ اب آئیے اسے AI تعاون سے بہتر بنائیں۔

**اپنے AI سے پوچھیں:**

> "میرے پاس ایک ROS 2 سروس سرور ہے جو روبوٹ کو فعال/غیر فعال کرتا ہے اور بیٹری لیول چیک کرتا ہے۔ یہ کام کر رہا ہے لیکن میں اسے پروڈکشن کے قابل بنانا چاہتا ہوں۔ آپ کیا ایرر ہینڈلنگ اور لاگنگ بہتری تجویز کریں گے؟ مجھے مناسب شٹ ڈاؤن اور پیرامیٹر کنفیگریشن کے لیے مخصوص کوڈ دیں۔"

**متوقع نتیجہ:** AI تجویز دے گا:
- حفاظت کے لیے try/except بلاکس شامل کریں
- سروس کال ہونے بمقابلہ جواب دینے پر لاگ کریں
- بیٹری کی حد کے لیے پیرامیٹرز شامل کریں
- مناسب صفائی کے ساتھ مناسب شٹ ڈاؤن شامل کریں

**مشورہ کو چیلنج کریں:**

> "اچھے خیالات ہیں، لیکن بیٹری چیک بہت سخت لگ رہا ہے۔ کیا ہو اگر میں چاہوں کہ یہ قابلِ ترتیب ہو—صارف کو رن ٹائم پر 'minimum_battery' پیرامیٹر سیٹ کرنے کی اجازت دیں؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- `__init__` میں `declare_parameter()` استعمال کریں
- اسے کال بیک میں `get_parameter()` کے ساتھ پڑھیں
- `ros2 param set` کے ساتھ اسے تبدیل کرنے کا طریقہ دکھائیں

**مل کر تکرار کریں:**

> "بہترین۔ مجھے مکمل کوڈ دکھائیں جس میں قابلِ ترتیب بیٹری تھریشولڈ ہو اور ایک دوسری سروس ہو جسے 'get_status' کہا جاتا ہے جو صرف موجودہ بیٹری لیول واپس کرے بغیر فعال/غیر فعال کیے۔"

**متوقع نتیجہ:** AI فراہم کرے گا:
- پیرامیٹر ڈیکلریشن اور ریڈنگ کے ساتھ مکمل نفاذ
- دو الگ کال بیکس
- دونوں کو ٹیسٹ کرنے کے لیے مثال ros2 کمانڈز

یہ تھری رولز پیٹرن ہے: AI فن تعمیر تجویز کرتا ہے (ٹیچر)، آپ حدود کو بہتر بناتے ہیں (طالب علم)، مل کر آپ حل بناتے ہیں (ساتھی کارکن)۔

---

## مشقیں

1. **بنیادی سروس میں ترمیم کریں** تاکہ وہ ایک سٹرنگ نام قبول کرے اور ایک سلام کے ساتھ جواب دے
2. **اسی نوڈ میں دوسری سروس بنائیں** جسے `get_robot_status` کہا جاتا ہے جو کوئی اسٹیٹ تبدیل کیے بغیر صرف بیٹری لیول واپس کرے
3. **سروس کالز کو ٹریک کرنے کے لیے لاگنگ شامل کریں** کال بیک کے پہلے اور بعد میں
4. **تصدیق کرنے کے لیے ros2 service list کے ساتھ ٹیسٹ کریں** کہ دونوں سروسز دستیاب ہیں۔

---

## عکاسی (Reflection)

اگلے سبق پر جانے سے پہلے، ان پر غور کریں:

- ایک حقیقی روبوٹ ٹاپک کے مقابلے میں سروس کو کب ترجیح دے گا؟
- اگر سروس کال بیک کو مکمل ہونے میں 10 سیکنڈ لگتے ہیں تو کیا ہوتا ہے؟
- کلائنٹ روایتی فنکشن کالز کے مقابلے میں کس طرح "بلاک" ہوتا ہے؟

اگلا سبق آپ کو سکھائے گا کہ دوسرے نوڈ سے سروسز کو **کال** کیسے کریں—سرور نہیں بلکہ کلائنٹ کیسے بنیں۔