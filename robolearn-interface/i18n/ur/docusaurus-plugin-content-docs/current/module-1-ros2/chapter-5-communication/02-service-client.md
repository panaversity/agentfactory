---
id: service-client
title: 'Lesson 5.2: Service Clients and Async Patterns'
sidebar_label: 5.2 Service Clients
sidebar_position: 2
chapter: 5
lesson: 2
duration_minutes: 60
proficiency_level: B1
layer: L2
cognitive_load:
  new_concepts: 4
learning_objectives:
  - Create a service client node using rclpy
  - Understand async vs sync service calls
  - Implement wait_for_service() pattern for robustness
  - Handle service responses and failures gracefully
  - Improve client code through AI-guided iteration
skills:
  - ros2-service-pattern
hardware_tier: 1
tier_1_path: Cloud ROS 2 (TheConstruct)
generated_by: content-implementer v1.0.0
created: '2025-11-29'
version: 1.0.0
---


# سبق 5.2: سروس کلائنٹس اور غیر مطابقت پذیر پیٹرن (Async Patterns)

پچھلے سبق میں، آپ نے ایک سروس سرور بنایا—ایک ایسا نوڈ جو درخواستوں کا انتظار کرتا ہے۔ اب آپ اس کا دوسرا رخ بنائیں گے: ایک سروس کلائنٹ جو درخواستیں بھیجتا ہے اور جوابات کا انتظار کرتا ہے۔

سروس کلائنٹ فون کال کرنے جیسا ہے۔ آپ ڈائل کرتے ہیں (درخواست بھیجتے ہیں)، ہولڈ پر انتظار کرتے ہیں (جواب آنے تک بلاک رہتے ہیں)، پھر دوسرا شخص جواب دیتا ہے۔ آپ کو اس صورتحال کو سنبھالنا ہوگا جب کوئی موجود نہ ہو (سروس دستیاب نہیں ہے)، اور آپ کو یہ جاننے کی ضرورت ہے کہ جواب کے ساتھ کیا کرنا ہے۔

اس سبق میں، آپ **ہم آہنگ** (Synchronous - آسان، بلاکنگ) اور **غیر مطابقت پذیر** (Asynchronous - زیادہ پیچیدہ، نان بلاکنگ) دونوں پیٹرن سیکھیں گے۔ زیادہ تر حقیقی روبوٹس غیر مطابقت پذیر استعمال کرتے ہیں کیونکہ وہ دوسرے اہم کاموں کے چلتے رہنے کے دوران جواب کا انتظار کرتے ہوئے بلاک نہیں ہو سکتے۔

---

## ہم آہنگ بمقابلہ غیر مطابقت پذیر سروس کالز

### ہم آہنگ (Synchronous - سادہ)

```python
# سروس کو کال کریں، جواب آنے تک بلاک رہیں
response = node.call_service(request)
print(response.result)
# یہاں سے جاری رکھیں - جواب پہلے ہی آ چکا ہے
```

**فائدے:**
- سادہ کوڈ: "کال کریں، پھر جواب استعمال کریں"
- بہاؤ سمجھنے میں آسان

**نقصانات:**
- انتظار کرتے ہوئے پورے نوڈ کو بلاک کر دیتا ہے
- اگر سروس پھنس جائے تو پورا نوڈ منجمد ہو جاتا ہے

### غیر مطابقت پذیر (Asynchronous - حقیقی)

```python
# درخواست بھیجیں، بلاک نہ ہوں
future = node.call_service_async(request)
# یہاں دوسرے کام کرتے رہیں
# ...
# بعد میں، چیک کریں کہ جواب آیا ہے یا نہیں
if future.done():
    response = future.result()
    print(response.result)
```

**فائدے:**
- انتظار کے دوران نوڈ چلتا رہتا ہے
- ایک سے زیادہ درخواستوں کو سنبھال سکتا ہے
- حقیقی روبوٹس کی طرح زیادہ

**نقصانات:**
- کوڈ زیادہ پیچیدہ ہے
- یہ چیک کرنے کی ضرورت ہے کہ جواب کب آتا ہے

**حقیقی روبوٹکس کی وجہ:** ایک روبوٹ ایک سروس کا انتظار کرتے ہوئے منجمد نہیں ہو سکتا۔ اسے سینسر ڈیٹا شائع کرتے رہنے، کمانڈز سننے، ڈسپلے اپ ڈیٹ کرنے—یہ سب ایک ہی وقت میں کرنا ہوتا ہے۔

---

## ہم آہنگ پیٹرن (سادہ، بلاکنگ)

سروس کو کال کرنے کا سب سے آسان طریقہ یہ ہے:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        # ایک کلائنٹ بنائیں (سرور نہیں!)
        self.cli = self.create_client(SetBool, 'enable_robot')

    def call_enable_robot(self, enable: bool):
        """روبوٹ سرور کو فعال/غیر فعال کرنے کی درخواست بھیجیں۔"""
        # سروس کے دستیاب ہونے کا انتظار کریں
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # درخواست بنائیں
        request = SetBool.Request()
        request.data = enable

        # سروس کو کال کریں اور جواب آنے تک بلاک رہیں
        response = self.cli.call(request)

        # اب ہمارے پاس جواب ہے
        self.get_logger().info(f'Response: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    client = SimpleClient()

    # روبوٹ کو فعال کریں
    client.call_enable_robot(True)

    # بعد میں...
    client.call_enable_robot(False)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

آئیے ہر حصے کو سمجھیں:

**کلائنٹ بنانا:**

```python
self.cli = self.create_client(SetBool, 'enable_robot')
```

یہ ایک کلائنٹ بناتا ہے (سرور سے مختلف!):
- **پہلا دلیل** (`SetBool`): سروس کی قسم
- **دوسرا دلیل** (`'enable_robot'`): کال کرنے کے لیے سروس کا نام

**سروس کا انتظار کرنا:**

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting...')
```

**اہم:** کال کرنے سے پہلے ہمیشہ انتظار کریں۔ سروس ابھی چل نہیں رہی ہو سکتی:
- اگر سروس دستیاب ہے، تو فوری طور پر واپس آ جاتا ہے
- اگر نہیں، تو 1 سیکنڈ تک انتظار کرتا ہے
- اگر اب بھی دستیاب نہیں ہے، تو دوبارہ لوپ کرتا ہے اور دوبارہ انتظار کرتا ہے
- اگر آپ اسے چھوڑ دیتے ہیں اور سروس نہیں چل رہی ہے، تو کال ناکام ہو جاتی ہے

**درخواست بنانا:**

```python
request = SetBool.Request()
request.data = enable
```

ایک خالی درخواست آبجیکٹ بنائیں اور فیلڈز کو پُر کریں۔

**ہم آہنگ طریقے سے کال کرنا:**

```python
response = self.cli.call(request)
```

یہ پورے نوڈ کو **بلاک** کر دیتا ہے:
- سرور کو درخواست بھیجتا ہے
- جواب کا انتظار کرتا ہے
- اس لائن کے نیچے کا کوڈ جواب آنے تک نہیں چلتا
- پھر آپ کے پاس جواب ہوتا ہے

---

## غیر مطابقت پذیر پیٹرن (حقیقی، نان بلاکنگ)

حقیقی روبوٹکس میں، آپ بلاک نہیں کر سکتے۔ یہ غیر مطابقت پذیر پیٹرن ہے:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class AsyncClient(Node):
    def __init__(self):
        super().__init__('async_client')
        self.cli = self.create_client(SetBool, 'enable_robot')
        self.future = None

    def call_enable_async(self, enable: bool):
        """درخواست بھیجیں بغیر بلاک کیے."""
        # پہلے سروس کا انتظار کریں
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return

        # درخواست بنائیں
        request = SetBool.Request()
        request.data = enable

        # درخواست بھیجیں لیکن بلاک نہ ہوں
        self.future = self.cli.call_async(request)
        self.get_logger().info(f'Request sent, not blocked')

    def check_response(self):
        """چیک کریں کہ جواب آیا ہے یا نہیں۔"""
        if self.future is None:
            return None

        # کیا جواب تیار ہے؟
        if self.future.done():
            response = self.future.result()
            self.get_logger().info(f'Response arrived: {response.message}')
            return response
        else:
            self.get_logger().info('Still waiting for response...')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = AsyncClient()

    # درخواست بھیجیں (بلاک نہیں ہوتی)
    client.call_enable_async(True)

    # نوڈ یہاں دوسرا کام کر سکتا ہے
    # حقیقی روبوٹ میں، یہ سینسر ڈیٹا شائع کرنا ہو سکتا ہے

    # بعد میں، چیک کریں کہ جواب آیا ہے یا نہیں
    client.check_response()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

ہم آہنگ سے اہم فرق:

```python
# غیر مطابقت پذیر: فوری طور پر واپس آ جاتا ہے
self.future = self.cli.call_async(request)
# کوڈ یہاں جاری رہتا ہے - درخواست ابھی بھی پروسیس ہو رہی ہے!

# بعد میں:
if self.future.done():
    response = self.future.result()
```

`future` آبجیکٹ "میں نے درخواست بھیج دی ہے، جواب جلد ہی آ رہا ہے" کی نمائندگی کرتا ہے۔ آپ اسے بعد میں `.done()` کے ساتھ چیک کرتے ہیں۔

---

## پروڈکشن پیٹرن (Async with Spin)

حقیقی روبوٹس کال بیک کے ساتھ ROS 2 کے `rclpy.spin()` کا استعمال کرتے ہیں۔ یہ پیٹرن سب سے زیادہ استعمال ہوتا ہے:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class ProductionClient(Node):
    def __init__(self):
        super().__init__('production_client')
        self.cli = self.create_client(SetBool, 'enable_robot')

        # شروع میں ایک بار انتظار کریں
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for enable_robot service...')

        # وقتاً فوقتاً درخواستیں بھیجنے کے لیے ایک ٹائمر بنائیں
        self.timer = self.create_timer(2.0, self.send_request)
        self.request_count = 0

    def send_request(self):
        """ہر 2 سیکنڈ میں کال کیا جاتا ہے۔"""
        self.request_count += 1
        enable = self.request_count % 2 == 0  # True/False کو باری باری تبدیل کریں

        request = SetBool.Request()
        request.data = enable

        # غیر مطابقت پذیر بھیجیں اور جواب آنے پر کال بیک منسلک کریں
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

        self.get_logger().info(f'Request #{self.request_count} sent')

    def response_callback(self, future):
        """جواب آنے پر کال کیا جاتا ہے۔"""
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = ProductionClient()
    # چلتے رہیں، اسپن ٹائمرز اور کال بیکس کو سنبھالتا ہے
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

یہ **پروڈکشن پیٹرن** ہے:
1. کلائنٹ بنائیں اور شروع میں سروس کا انتظار کریں۔
2. وقتاً فوقتاً درخواستیں بھیجنے کے لیے ٹائمر استعمال کریں۔
3. `future.add_done_callback()` کے ساتھ کال بیک منسلک کریں۔
4. جب جواب آتا ہے، تو کال بیک خود بخود چل جاتا ہے۔
5. `rclpy.spin()` سب کچھ چلاتا رہتا ہے۔

---

## اپنے کلائنٹ کی جانچ کرنا

آئیے پچھلے سبق سے چلنے والے سرور کے ساتھ تجربہ کریں۔

**ٹرمینل 1: سرور شروع کریں**

```bash
ros2 run my_first_package enable_service
```

**ٹرمینل 2: پروڈکشن کلائنٹ چلائیں**

ایک فائل `service_client.py` بنائیں:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(SetBool, 'enable_robot')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not found, waiting...')

        # فوری طور پر درخواست بھیجیں
        request = SetBool.Request()
        request.data = True

        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        self.get_logger().info(f'Result: {response.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

`setup.py` میں شامل کریں:

```python
'service_client = my_first_package.service_client:main',
```

بلڈ اور رن کریں:

```bash
cd ~/ros2_ws && colcon build
source install/setup.bash
ros2 run my_first_package service_client
```

ٹرمینل 1 (سرور) دکھائے گا:
```
[INFO] [robot_enabler]: Robot has been ENABLED
```

ٹرمینل 2 (کلائنٹ) دکھائے گا:
```
[INFO] [client_node]: Result: Robot enabled. Battery: 85%
```

---

## غلطی سے بچاؤ کے پیٹرن (Error Handling Patterns)

کیا ہوتا ہے اگر:
1. سروس چل نہیں رہی ہے؟
2. سروس جواب کے بیچ میں کریش ہو جاتی ہے؟
3. نیٹ ورک سست ہے؟

```python
def send_request_safely(self, request):
    """مناسب غلطی سے بچاؤ کے ساتھ محفوظ درخواست۔"""
    try:
        # سروس موجود ہے یا نہیں چیک کریں
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return None

        # درخواست بھیجیں
        future = self.cli.call_async(request)

        # جواب کا انتظار کریں (زیادہ سے زیادہ 5 سیکنڈ)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

        # چیک کریں کہ ہمیں جواب ملا یا نہیں
        if future.result() is None:
            self.get_logger().error('Service call failed')
            return None

        return future.result()

    except Exception as e:
        self.get_logger().error(f'Error: {e}')
        return None
```

اہم فنکشنز:
- `wait_for_service()`: سروس کی موجودگی چیک کریں۔
- `spin_until_future_complete()`: جواب کے لیے N سیکنڈ تک انتظار کریں۔
- `future.result()`: اصل جواب حاصل کریں۔

---

## اہم نکات

**Wait_for_service ضروری ہے:**
کبھی یہ فرض نہ کریں کہ سروس چل رہی ہے۔ ہمیشہ انتظار کریں۔

**ہم آہنگ بمقابلہ غیر مطابقت پذیر فیصلہ:**
- **ہم آہنگ (Sync)**: سادہ ایک بار کی کالز (شروع کرنا، ابتدائی سیٹ اپ)
- **غیر مطابقت پذیر (Async)**: حقیقی نوڈز جو چلتے رہتے ہیں

**کال بیکس پیٹرن ہیں:**
حقیقی ROS 2 میں، آپ "انتظار" نہیں کرتے۔ آپ کال بیکس منسلک کرتے ہیں اور `rclpy.spin()` کو وقت کا انتظام کرنے دیتے ہیں۔

**ٹائم آؤٹس اہم ہیں:**
ہمیشہ ٹائم آؤٹس کی وضاحت کریں۔ ایک پھنسی ہوئی سروس کو آپ کے نوڈ کو منجمد نہیں کرنا چاہیے۔

---

## AI کے ساتھ تجربہ کریں

آپ کے پاس ایک کام کرنے والا غیر مطابقت پذیر کلائنٹ ہے۔ آئیے اسے بہتر بنائیں۔

**اپنے AI سے پوچھیں:**

> "میرے پاس ایک سروس کلائنٹ ہے جو ہر 2 سیکنڈ میں درخواستیں بھیجتا ہے اور کال بیک کے ساتھ جوابات سنبھالتا ہے۔ میں شامل کرنا چاہتا ہوں: (1) اگر سروس دستیاب نہیں ہے تو دوبارہ کوشش کرنے کا منطق (retry logic)، (2) ایک کاؤنٹر جو کامیاب بمقابلہ ناکام درخواستوں کو ٹریک کرتا ہے، (3) درخواست/جواب کے اوقات کو لاگ کرنا۔"

**متوقع نتیجہ:** AI تجویز کرے گا:
- سروس کی دستیابی کے لیے ایکسپوننشل بیک آف کے ساتھ لوپ
- کامیابی/ناکامی کاؤنٹر شامل کرنا
- راؤنڈ ٹرپ ٹائم کی پیمائش کے لیے time.time() استعمال کرنا
- ہر 10 درخواستوں کے بعد میٹرکس لاگ کرنا

**نفاذ کو چیلنج کریں:**

> "اچھا، لیکن کیا ہوگا اگر سروس کلائنٹ چلتے ہوئے غیر دستیاب ہو جائے؟ کیا مجھے ہمیشہ کے لیے دوبارہ کوشش کرتے رہنا چاہیے یا ہار مان لینی چاہیے؟"

**متوقع نتیجہ:** AI وضاحت کرے گا:
- آپشن 1: N بار کوشش کریں پھر ناکام ہو جائیں
- آپشن 2: مسلسل کوشش کرتے رہیں لیکن وارننگ لاگ کریں
- آپشن 3: اسے قابل ترتیب بنانے کے لیے ros2 param کا استعمال کریں
- مضبوطی بمقابلہ وسائل کے استعمال میں ٹریڈ آف پر بحث کریں۔

**مل کر تکرار کریں:**

> "مجھے آپشن 3 پسند ہے۔ مجھے دکھائیں کہ 'max_retries' پیرامیٹر کیسے شامل کیا جائے جسے صارف سیٹ کر سکے۔ اگر کوششیں ختم ہو جائیں تو ایک غلطی لاگ کریں لیکن کریش نہ ہوں۔"

یہ AI کے پیٹرن سکھانے، آپ کی ضروریات کو بہتر بنانے، اور مل کر بہترین حل تک پہنچنے کا مظاہرہ کرتا ہے۔

---

## مشقیں

1. **ایک سادہ کلائنٹ بنائیں** جو روبوٹ کو فعال کرتا ہے، 2 سیکنڈ انتظار کرتا ہے، پھر اسے غیر فعال کرتا ہے۔
2. **دوبارہ کوشش کرنے کا منطق شامل کریں** جو ناکام ہونے سے پہلے سروس کال کو 3 بار تک کرنے کی کوشش کرے۔
3. **جواب کا وقت** ماپیں اور لاگ کریں کہ درخواست میں کتنا وقت لگا۔
4. **ایک ہی پیکیج میں دو کلائنٹس** بنائیں جو مختلف سروسز کو کال کرتے ہیں (آپ کو دوسرے سرور کی ضرورت ہوگی)
5. **درخواست کی فریکوئنسی کے لیے ایک پیرامیٹر شامل کریں** تاکہ اسے چلنے کے وقت تبدیل کیا جا سکے۔

---

## عکاسی

اگلے سبق پر جانے سے پہلے، غور کریں:

- ایک حقیقی روبوٹ ہم آہنگ سروس کالز کیوں استعمال نہیں کر سکتا؟
- `call()` اور `call_async()` کے درمیان کیا فرق ہے؟
- آپ `wait_for_service()` کا استعمال کب کریں گے بمقابلہ صرف یہ امید کرنا کہ سروس موجود ہے؟

اگلا سبق: اپنی مرضی کے پیغام اور سروس کی اقسام۔ اب تک آپ نے `AddTwoInts` اور `SetBool` جیسی بنی بنائی اقسام استعمال کی ہیں۔ کیا ہوگا اگر آپ کو اپنی مرضی کے ڈیٹا ڈھانچے بھیجنے کی ضرورت ہو؟ تب آپ `.srv` فائلیں بناتے ہیں۔