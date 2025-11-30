---
id: lesson-6-1-parameters
title: 'Lesson 6.1: Parameters (Configurable Nodes)'
sidebar_position: 1
sidebar_label: 6.1 Parameters
description: >-
  Learn how to make ROS 2 nodes configurable using parameters that can be
  modified at runtime without recompiling code.
duration_minutes: 60
proficiency_level: B1
layer: L3
hardware_tier: 1
learning_objectives:
  - Declare and read ROS 2 parameters in nodes
  - Modify parameters at runtime using ros2 param commands
  - Implement parameter validation callbacks
  - Design parameter-driven configuration systems
  - Apply parameters in multi-node launch configurations
---


# سبق 6.1: پیرامیٹرز — نوڈز کو قابلِ ترتیب بنانا

آپ نے پہلے جو پبلشر اور سبسکرائبر مثالیں بنائی تھیں، ان میں پبلش ریٹ اور دیگر رویے پائتھن سورس میں ہارڈکوڈ کیے گئے تھے۔ کیا ہوگا اگر آپ کوڈ میں ترمیم اور دوبارہ کمپائل کیے بغیر ریٹ تبدیل کرنا چاہیں؟ یہیں پر ROS 2 پیرامیٹرز کام آتے ہیں۔

پیرامیٹرز رن ٹائم کنفیگریشن ویلیوز (runtime configuration values) ہیں جو آپ کو سسٹم چلتے ہوئے نوڈ کے رویے کو ایڈجسٹ کرنے کی اجازت دیتے ہیں۔ ایک نوڈ ڈیفالٹ ویلیوز کے ساتھ پیرامیٹرز کو **اعلان (declare)** کر سکتا ہے، ضرورت پڑنے پر ان پیرامیٹرز کو **پڑھ (read)** سکتا ہے، اور آپ `ros2 param` کمانڈز کا استعمال کرتے ہوئے انہیں کمانڈ لائن سے **تبدیل (modify)** کر سکتے ہیں۔

## پیرامیٹرز کو سمجھنا

پیرامیٹرز کو نوڈ لیول کی سیٹنگز سمجھیں—وہ ہر نوڈ کے لیے مخصوص ہوتے ہیں، پیرامیٹر سرور میں محفوظ ہوتے ہیں، اور پبلش/سبسکرائب میسج فلو سے آزاد ہوتے ہیں۔

### پیرامیٹرز کیوں اہم ہیں

ہمارے پہلے پبلشر مثال میں، ٹائمر کا دورانیہ مقرر تھا:

```python
timer_period = 0.5  # سیکنڈز → ہارڈکوڈڈ
```

اگر آپ پبلش ریٹ کو 2 Hz (0.5 Hz کے بجائے) میں تبدیل کرنا چاہتے تھے، تو آپ کو یہ کرنا پڑتا تھا:
1. پائتھن فائل میں ترمیم کریں
2. `colcon build` کے ساتھ پیکیج کو دوبارہ بنائیں
3. نوڈ کو روکیں اور دوبارہ شروع کریں

پیرامیٹرز کے ساتھ، آپ بس یہ چلاتے ہیں:
```bash
ros2 param set /minimal_publisher publish_rate 2.0
```

نوڈ فوری طور پر نئی ویلیو اٹھا لیتا ہے (اگر اسے وقتاً فوقتاً پیرامیٹر چیک کرنے کے لیے کوڈ کیا گیا ہو)۔

### پیرامیٹر کا دائرہ کار (Scope)

پیرامیٹرز **نوڈ کے مخصوص** ہوتے ہیں۔ نوڈ کی ہر چلتی ہوئی مثال (instance) کے پیرامیٹرز کا اپنا سیٹ ہوتا ہے۔ اگر آپ ایک ہی نوڈ ایگزیکیوٹیبل کی دو مثالیں چلاتے ہیں، تو وہ پیرامیٹر کی الگ الگ ویلیوز برقرار رکھتے ہیں۔

## پیرامیٹرز کا اعلان اور پڑھنا

### پیرامیٹر کا اعلان (Parameter Declaration)

پیرامیٹر استعمال کرنے کا پہلا قدم نوڈ کے کنسٹرکٹر میں اس کا **اعلان** کرنا ہے۔ یہ پیرامیٹر کو پیرامیٹر سرور کے ساتھ رجسٹر کرتا ہے اور ایک ڈیفالٹ ویلیو سیٹ کرتا ہے۔

**حل شدہ مثال (Worked Example):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # ڈیفالٹ ویلیو کے ساتھ ایک پیرامیٹر کا اعلان کریں
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # پیرامیٹر ویلیو پڑھیں
        rate = self.get_parameter('publish_rate').value

        # پبلشر بنائیں
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # ریٹ کا استعمال کرتے ہوئے ٹائمر بنائیں
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.i}'
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurablePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**آؤٹ پٹ (دو ٹرمینلز سے):**

ٹرمینل 1 (نوڈ چلاتے ہوئے):
```
[INFO] [configurable_publisher]: Starting publisher at 1.0 Hz
[INFO] [configurable_publisher]: Published: Hello 0
[INFO] [configurable_publisher]: Published: Hello 1
[INFO] [configurable_publisher]: Published: Hello 2
```

ٹرمینل 2 (ٹاپک کو سنتے ہوئے):
```bash
ros2 topic echo /topic
---
data: 'Hello 0'
---
data: 'Hello 1'
---
```

### پیرامیٹرز کو متحرک طور پر پڑھنا (Reading Parameters Dynamically)

پیرامیٹرز کو کسی بھی وقت پڑھا جا سکتا ہے۔ **پیرامیٹر ویلیو پڑھنے کے لیے**، یہ استعمال کریں:

```python
rate = self.get_parameter('publish_rate').value
```

اگر آپ ایک پیرامیٹر کا اعلان کرتے ہیں لیکن اس کی ویلیو کو کئی بار استعمال کرنا چاہتے ہیں، تو آپ اسے کیش (cache) کر سکتے ہیں (جیسا کہ اوپر دکھایا گیا ہے) یا اسے ہر کال بیک سائیکل میں پڑھ سکتے ہیں اگر پیرامیٹر تبدیل ہو سکتا ہے۔

## رن ٹائم پر پیرامیٹرز میں ترمیم کرنا

قابلِ ترتیب پبلشر کے چلنے کے ساتھ، آپ دوبارہ شروع کیے بغیر پبلش ریٹ تبدیل کر سکتے ہیں:

```bash
# نوڈ کے لیے تمام پیرامیٹرز کی فہرست بنائیں
ros2 param list /configurable_publisher
# آؤٹ پٹ: /configurable_publisher:
#   publish_rate

# موجودہ ویلیو حاصل کریں
ros2 param get /configurable_publisher publish_rate
# آؤٹ پٹ: publish_rate: 1.0

# نئی ویلیو سیٹ کریں
ros2 param set /configurable_publisher publish_rate 5.0

# تبدیلی کی تصدیق کریں
ros2 param get /configurable_publisher publish_rate
# آؤٹ پٹ: publish_rate: 5.0
```

نوڈ اب 1 Hz کے بجائے 5 Hz پر پبلش کرے گا۔

## گائیڈڈ پریکٹس: ایک نوڈ کو قابلِ ترتیب بنانا

آئیے ایک نوڈ کے ساتھ تصور کو بڑھاتے ہیں جو متعدد پیرامیٹرز استعمال کرتا ہے۔

### کام: ایک لچکدار میسج پبلشر بنائیں

ایک نوڈ بنائیں جو مندرجہ ذیل قابلِ ترتیب خصوصیات کے ساتھ پیغامات شائع کرتا ہے:
- **publish\_rate** (float): فی سیکنڈ پیغامات (1-10 Hz)
- **message\_prefix** (string): ہر میسج سے پہلے شامل کیا جانے والا متن
- **enable\_logging** (bool): آیا ہر اشاعت کو لاگ کرنا ہے

**حل:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FlexiblePublisher(Node):
    def __init__(self):
        super().__init__('flexible_publisher')

        # پیرامیٹرز کا اعلان کریں
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('message_prefix', 'Message')
        self.declare_parameter('enable_logging', True)

        # پبلشر بنائیں
        self.publisher_ = self.create_publisher(String, 'flexible_topic', 10)

        # ٹائمر سیٹ اپ کریں
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.counter = 0

    def timer_callback(self):
        prefix = self.get_parameter('message_prefix').value
        enable_log = self.get_parameter('enable_logging').value

        msg = String()
        msg.data = f'{prefix}: {self.counter}'
        self.publisher_.publish(msg)

        if enable_log:
            self.get_logger().info(f'Published: {msg.data}')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = FlexiblePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**آزمائیں:**

```bash
# ٹرمینل 1: نوڈ چلائیں
ros2 run my_first_package flexible_publisher

# ٹرمینل 2: پیرامیٹرز میں ترمیم کریں
ros2 param set /flexible_publisher publish_rate 5.0
ros2 param set /flexible_publisher message_prefix "Status Update"
ros2 param set /flexible_publisher enable_logging false
```

**متوقع رویہ**: نوڈ فوری طور پر اپنے رویے کو اپ ڈیٹ کرتا ہے—تیزی سے پبلش کرنا، میسج کے مواد کو تبدیل کرنا، اور لاگنگ کو ٹوگل کرنا، یہ سب دوبارہ شروع کیے بغیر۔

## پیرامیٹر ڈیزائن پیٹرنز

### بہترین عمل: شروع میں اعلان کریں، ضرورت پڑنے پر پڑھیں

```python
def __init__(self):
    # __init__ میں تمام پیرامیٹرز کا اعلان کریں
    self.declare_parameter('param1', default_value_1)
    self.declare_parameter('param2', default_value_2)

def timer_callback(self):
    # ضرورت پڑنے پر پیرامیٹر ویلیوز پڑھیں
    value = self.get_parameter('param2').value
    # ویلیو استعمال کریں...
```

یہ پیٹرن یقینی بناتا ہے:
- نوڈ شروع ہونے پر تمام پیرامیٹرز رجسٹر ہو جاتے ہیں
- پیرامیٹر سرور کو معلوم ہوتا ہے کہ کون سی ویلیوز موجود ہیں
- آپ رن ٹائم پر پیرامیٹرز تبدیل کر سکتے ہیں
- نوڈ ہمیشہ موجودہ ویلیو استعمال کرتا ہے

### پیرامیٹر کی توثیق (Parameter Validation)

جب آپ رن ٹائم پر کسی پیرامیٹر میں ترمیم کرتے ہیں، تو کیا آپ کو اس کی توثیق کرنی چاہیے؟ ہاں، خاص طور پر حفاظت کے لحاظ سے اہم ویلیوز کے لیے۔

**توثیق کے ساتھ:**

```python
class SafePublisher(Node):
    def __init__(self):
        super().__init__('safe_publisher')

        # حدود کے ساتھ اعلان کریں (ROS 2 پیرامیٹر حدود محدود ہیں)
        self.declare_parameter('publish_rate', 1.0)

        # پیرامیٹر تبدیلیوں کے لیے ایک کال بیک بنائیں
        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'publish_rate':
                # توثیق کریں: publish_rate 0.1 سے 100 Hz کے درمیان ہونا چاہیے
                if param.value < 0.1 or param.value > 100:
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
```

## آزادانہ پریکٹس

### مشق 1: قابلِ ترتیب ڈیٹا لاگر

ایک نوڈ بنائیں جو:
1. `/sensor_data` ٹاپک پر پیغامات کو سبسکرائب کرتا ہے
2. ایک پیرامیٹر `log_frequency` رکھتا ہے (کتنے پیغامات کے درمیان لاگنگ کرنی ہے)
3. ایک پیرامیٹر `output_file` (لاگ کرنے کے لیے فائل کا نام) رکھتا ہے
4. ہر Nویں میسج کو ٹائم سٹیمپ کے ساتھ لاگ کرتا ہے

آزمائیں:
- `/sensor_data` پر ایک پبلشر چلائیں۔
- رن ٹائم پر `log_frequency` تبدیل کریں۔
- تصدیق کریں کہ لاگنگ کا رویہ بدل گیا ہے۔

### مشق 2: پیرامیٹر ریفلیکشن

ایک چھوٹا یوٹیلیٹی نوڈ لکھیں جو:
1. کمانڈ لائن آرگومنٹ کے طور پر نوڈ کا نام لیتا ہے
2. `ros2 param list` کا استعمال کرتے ہوئے اس کے تمام پیرامیٹرز سے پوچھ گچھ کرتا ہے
3. انہیں ایک فارمیٹ شدہ ٹیبل میں پرنٹ کرتا ہے

یہ مشق آپ کو یہ سمجھنے میں مدد کرتی ہے کہ پیرامیٹر انسپیکشن (introspection) کیسے کام کرتا ہے۔

## AI تعاون: ترتیب کو بڑھانا

اب دیکھتے ہیں کہ AI آپ کو زیادہ نفیس پیرامیٹر سسٹم ڈیزائن کرنے میں کیسے مدد کر سکتا ہے۔

**آپ کی درخواست:**
"میرے پاس ایک نوڈ ہے جو سینسر ڈیٹا شائع کرتا ہے۔ میں میسج کی قسم کو قابلِ ترتیب بنانا چاہتا ہوں—کبھی میں String پیغامات شائع کرتا ہوں، کبھی حسب ضرورت RobotStatus پیغامات۔ میں یہ کیسے کروں گا؟"

**مشاہدہ**: یہ ایک زیادہ پیچیدہ ڈیزائن کا مسئلہ ہے۔ پیرامیٹر کو نہ صرف ایک ویلیو کو کنٹرول کرنا چاہیے، بلکہ اس میسج کی قسم کو بھی جس کو نوڈ شائع کرتا ہے۔ یہ مشکل ہے کیونکہ C++ اور پائتھن میں ٹائپ ہینڈلنگ مختلف ہوتی ہے۔

**AI کا طریقہ کار:**
"یہاں آپ کو رن ٹائم پیرامیٹرز کی ایک حد کا سامنا کرنا پڑتا ہے—وہ پائتھن ٹائپس کو متحرک طور پر تبدیل نہیں کر سکتے۔ تاہم، آپ یہ کر سکتے ہیں:

1. **آپشن A**: دو الگ الگ ایگزیکیوٹیبل بنائیں (ایک String کے لیے، ایک RobotStatus کے لیے)
2. **آپشن B**: ایک **یونین میسج** استعمال کریں جس میں دونوں ٹائپس شامل ہوں (کم صاف)
3. **آپشن C**: ہمیشہ RobotStatus (سپرسٹ) شائع کریں، جس سے سبسکرائبرز غیر استعمال شدہ فیلڈز کو نظر انداز کر دیں

زیادہ تر روبوٹکس سسٹم کے لیے، آپشن C—**ایک واحد میسج ٹائپ ڈیزائن کرنا جو تمام صورتوں کا احاطہ کرتا ہو**—بہترین کام کرتا ہے۔ پبلشر سادہ رہتا ہے، سبسکرائبرز کو جو چاہیے وہ مل جاتا ہے، اور آپ رن ٹائم ٹائپ سوئچنگ کی پیچیدگی سے بچ جاتے ہیں۔"

**اہم بصیرت**: پیرامیٹرز **ویلیوز** کو ترتیب دیتے ہیں، **ٹائپس** کو نہیں۔ ساختی تبدیلیوں (میسج ٹائپس، سروس ڈیفینیشنز) کے لیے، آپ مختلف نوڈز بناتے ہیں یا پولی مورفک میسج ٹائپس ڈیزائن کرتے ہیں۔ اس گفتگو نے پیرامیٹر پر مبنی کنفیگریشن کی حدود کو ظاہر کیا۔

## AI کے ساتھ کوشش کریں

**سیٹ اپ:** اپنے AI ٹول کے ساتھ ایک چیٹ کھولیں اور اسے پیرامیٹر سے چلنے والے نوڈ کنفیگریشن کو ڈیزائن کرنے میں مدد کرنے دیں۔

**پرامپٹ سیٹ:**

```
Prompt 1: "I want to create a ROS 2 node that publishes at different rates depending on robot mode (fast, normal, slow). How would I use parameters for this?"

Prompt 2: "My node publishes sensor data, but different robots have different sensor configurations. Some have cameras, some have LIDAR, some have both. Can I use parameters to configure what sensors to include?"

Prompt 3: "What's a good way to validate parameter values in ROS 2? For example, ensuring publish_rate is between 0.1 and 100 Hz?"
```

**متوقع نتائج:**
- AI پیرامیٹر سے چلنے والے برانچنگ لاجک کو ڈیزائن کرنے کا طریقہ دکھاتا ہے۔
- AI مشروط سینسر شمولیت کے پیٹرن کی وضاحت کرتا ہے۔
- AI ROS 2 میں پیرامیٹر توثیق کال بیکس کا مظاہرہ کرتا ہے۔

**حفاظتی نوٹ**: پیرامیٹر میں ترمیم رن ٹائم پر نوڈ کو دوبارہ شروع کیے بغیر ہوتی ہے—اس بات کو یقینی بنائیں کہ روبوٹ کی حرکت یا حفاظت کے لحاظ سے اہم رویے کو متاثر کرنے والے پیرامیٹرز استعمال کرنے سے پہلے کوئی بھی توثیق موجود ہو۔

**اختیاری توسیع:**
پیرامیٹرز کے لیے ایک نوڈ بنائیں:
- روبوٹ کا نام (string)
- بیٹری تھریشولڈ (float, 0-100)
- موٹر اسپیڈ کی حد (float, 0-1.0)
- سیفٹی موڈ (bool)

پھر ایک چھوٹی اسکرپٹ لکھیں جو تمام پیرامیٹرز کو تبدیل کرے اور تصدیق کے لیے نتائج کو لاگ کرے۔