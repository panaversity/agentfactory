---
id: lesson-4-2-writing-publisher
title: "Lesson 4.2: Writing a Publisher"
sidebar_position: 2
sidebar_label: "4.2 Writing Publisher"
description: "Write your first ROS 2 publisher node using rclpy with timer callbacks."
chapter: 4
lesson: 2
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 2
learning_objectives:
  - "Write a ROS 2 publisher node using rclpy"
  - "Understand the Node class and lifecycle (init → spin → shutdown)"
  - "Implement periodic publishing with timer callbacks"
  - "Verify publication using ros2 topic echo"
  - "Extend publisher code based on feedback"
skills:
  - "ros2-publisher-subscriber"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 4.2: Writing a Publisher

You now have a workspace and package. It's time to write your first ROS 2 code: a **publisher** — a node that sends data to a topic.

Think of a publisher as a speaker at a conference. It stands on stage and broadcasts information. Anyone in the audience (subscribers) can listen. The publisher doesn't care who's listening or how many people hear it — it just keeps speaking.

In this lesson, you'll write a simple publisher that sends a text message every 0.5 seconds. You'll see the code, understand how it works, test it, and then modify it based on feedback.

---

## The Publisher Pattern

All ROS 2 publishers follow the same pattern:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's break this down line by line.

---

## Understanding the Code

### Imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- **rclpy**: ROS 2 Python library. Everything ROS starts here.
- **Node**: The base class. Every ROS 2 node inherits from Node.
- **String**: A message type. This particular type just contains text data.

### Creating the Node Class

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

You create a class that inherits from Node. The `super().__init__()` call tells ROS 2 the node's name: 'minimal_publisher'. This name will appear in `ros2 node list`.

### Creating the Publisher

```python
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

This line creates a publisher that:
- **Sends String messages** (the type)
- **Sends to the topic named 'topic'** (the channel name)
- **Has a queue depth of 10** (how many unsent messages to buffer)

The underscore at the end (`publisher_`) is a Python convention — it tells readers this is an internal attribute.

### Creating the Timer

```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
```

This creates a timer that calls `timer_callback()` every 0.5 seconds. Think of this as "every 0.5 seconds, do something."

### The Callback

```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.i}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
```

This is called every 0.5 seconds:
1. **Create a message**: `msg = String()`
2. **Fill in the data**: `msg.data = ...`
3. **Publish it**: `self.publisher_.publish(msg)`
4. **Log it**: `self.get_logger().info(...)` prints to the console
5. **Increment counter**: `self.i += 1`

### The Main Function

```python
def main(args=None):
    rclpy.init(args=args)              # Initialize ROS 2
    minimal_publisher = MinimalPublisher()  # Create the node
    rclpy.spin(minimal_publisher)      # Run the node (block forever)
    minimal_publisher.destroy_node()   # Clean up
    rclpy.shutdown()                   # Shut down ROS 2
```

This is the standard ROS 2 startup sequence:
1. **rclpy.init()**: Set up ROS 2
2. **Create your node**
3. **rclpy.spin()**: Run forever. This is where the timer callbacks happen.
4. **Cleanup**: When spin() exits (e.g., user presses Ctrl+C), clean up gracefully

---

## Creating the Publisher File

Now let's create this file in your package.

**Step 1: Navigate to your package**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
```

Notice there are TWO `my_first_package` directories — the outer one is the folder, the inner one is the Python package.

**Step 2: Create the publisher file**

```bash
cat > minimal_publisher.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**Step 3: Add the executable to package.xml**

Edit `setup.py` in the outer `my_first_package` folder:

```python
# Find this section:
entry_points={
    'console_scripts': [
    ],
},

# Add this line inside the list:
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
    ],
},
```

This tells ROS 2 that there's an executable called `minimal_publisher` that runs the `main()` function from `minimal_publisher.py`.

**Step 4: Build the workspace**

```bash
cd ~/ros2_ws
colcon build
```

**Step 5: Source the workspace**

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the Publisher

**Terminal 1: Start the publisher**

```bash
ros2 run my_first_package minimal_publisher
```

You should see output:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

The node is running and publishing! But how do you verify data is actually being sent? Let's check.

**Terminal 2: Listen to the topic**

```bash
ros2 topic echo /topic
```

You should see:

```
data: Hello World: 0
---
data: Hello World: 1
---
data: Hello World: 2
---
```

Each line separated by `---` is one message. The publisher is successfully sending data!

**Terminal 3: Inspect the topic**

```bash
ros2 topic info /topic
```

Output:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1 (from ros2 topic echo)
```

This confirms:
- There's 1 publisher (your node)
- There's 1 subscriber (the ros2 topic echo command)

---

## Modifying the Publisher

Now that you have a working publisher, let's modify it. Change the message interval from 0.5 seconds to 2 seconds:

**Edit minimal_publisher.py:**

```python
timer_period = 2.0  # Changed from 0.5 to 2.0
```

**Rebuild:**

```bash
cd ~/ros2_ws && colcon build
```

**Restart the publisher:**

```bash
ros2 run my_first_package minimal_publisher
```

Now it publishes every 2 seconds instead of 0.5. You can verify with `ros2 topic echo /topic`.

---

## Extending the Publisher

Let's make it more realistic. A real sensor might publish multiple fields: timestamp, value, and confidence.

**Create a new version:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.reading = 0.0

    def timer_callback(self):
        msg = String()
        # Simulate sensor data: temp in celsius
        self.reading += 0.1
        msg.data = f'temperature: {self.reading:.2f}C'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sensor reading: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This simulates a temperature sensor that gradually increases temperature.

---

## Key Insights

**Timing:** The `create_timer()` callback is called at regular intervals. Callbacks are never called simultaneously — ROS 2 ensures they run one at a time.

**Logging:** Always use `self.get_logger()` for diagnostic output. Never use `print()` in ROS nodes. The logger integrates with ROS 2's logging system.

**Naming:** Node names are important. They appear in `ros2 node list` and `rqt_graph`. Use descriptive names like `sensor_publisher` not `node1`.

**Quality of Service (QoS):** The `10` in `create_publisher()` is the queue depth. If a subscriber falls behind, ROS 2 keeps the last 10 messages. For real-time systems, this number might be different.

---

## Try With AI

You have a working publisher. Now let's use AI to improve it.

**Ask your AI:**

> "I have a simple ROS 2 publisher that sends String messages every 0.5 seconds. I want to make it production-ready. What should I add? Give me specific code improvements for error handling, logging configuration, and graceful shutdown."

**Expected outcome:** AI will suggest:
- Add try/except blocks for error handling
- Configure logging levels (INFO, DEBUG, WARNING)
- Add a shutdown hook that stops publishing on exit
- Add docstrings explaining the code

**Challenge AI's suggestions:**

> "My publisher needs to work in low-bandwidth conditions where we can only send 5 messages per second. How would you add that constraint to the code? Should it be a hard limit or a configurable parameter?"

**Expected outcome:** AI will explain:
- Use a counter to skip publishing every Nth callback
- OR use a smaller timer period with conditional publishing
- Make it a configurable parameter in package.xml or via ros2 param set

**Iterate together:**

> "Good point. Let's implement the configurable parameter approach. Show me how to add a 'publish_rate' parameter that a user can set at runtime using ros2 param set."

**Expected outcome:** AI will show you how to:
- Declare the parameter in `__init__`
- Read it in the callback
- Allow runtime modification

Through this dialogue, AI suggested patterns, you provided constraints, and together you converged on a better solution.

---

## Exercises

1. **Modify the publisher to send a counter that increments by 10 each time** (instead of 1)
2. **Create a second publisher node in the same package that sends different data**
3. **Use `ros2 topic info` to verify both publishers are sending**

---

## Reflect

Consider these questions:

1. **Why does the node use a timer instead of a loop?** What advantage does `create_timer()` provide over `while True: publish()`?

2. **What happens to messages if no one subscribes?** Does the publisher know? Should it care?

3. **How would you modify the publish rate?** What code changes are required, and what would be a better approach?

In the next lesson, you'll write a subscriber that listens to what this publisher sends.
