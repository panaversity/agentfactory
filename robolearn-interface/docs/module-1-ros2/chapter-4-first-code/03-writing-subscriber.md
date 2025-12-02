---
id: lesson-4-3-writing-subscriber
title: "Lesson 4.3: Writing a Subscriber"
sidebar_position: 3
sidebar_label: "4.3 Writing Subscriber"
description: "Write your first ROS 2 subscriber node using rclpy with message callbacks."
chapter: 4
lesson: 3
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 2
learning_objectives:
  - "Write a ROS 2 subscriber node using rclpy"
  - "Implement message callbacks"
  - "Understand the subscriber queue depth parameter"
  - "Test pub/sub communication between two nodes"
  - "Process and respond to incoming messages"
skills:
  - "ros2-publisher-subscriber"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 4.3: Writing a Subscriber

In the last lesson, you wrote a publisher — a node that sends data. Publishers are only useful if someone is listening. In this lesson, you'll write a **subscriber** — a node that receives messages from a topic.

Think of a subscriber as an audience member at the publisher's conference. The audience member hears what the speaker broadcasts and reacts to it. The speaker doesn't know or care who's in the audience — the audience member just shows up and listens.

By the end of this lesson, you'll have a publisher and subscriber communicating, and you'll understand how to process received messages.

---

## The Subscriber Pattern

All ROS 2 subscribers follow the same pattern:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's understand each part.

---

## Understanding the Code

### Imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

Same as the publisher — we need rclpy, Node, and String messages.

### Creating the Subscriber

```python
self.subscription = self.create_subscription(
    String,           # Message type
    'topic',          # Topic name (must match publisher)
    self.listener_callback,  # Function to call when message arrives
    10)               # Queue depth
```

This creates a subscription that:
- **Listens to String messages** (the message type)
- **On the topic named 'topic'** (must match what the publisher sends to)
- **Calls self.listener_callback()** whenever a message arrives
- **Has a queue depth of 10** (if messages arrive faster than you process them, buffer up to 10)

### The Callback

```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

This function is called automatically when a message arrives. The `msg` parameter contains the received message. You can access the data with `msg.data`.

### The Line That Looks Odd

```python
self.subscription  # prevent unused variable warning
```

This is a quirk of Python linters. Without this line, some IDEs might warn "you created self.subscription but never used it." Actually, ROS 2 is using it internally — the subscription keeps listening in the background. This line prevents the false warning.

---

## Creating the Subscriber File

Let's create this in your package.

**Step 1: Create the subscriber file**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
cat > minimal_subscriber.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**Step 2: Update setup.py to add the executable**

Edit `~/ros2_ws/src/my_first_package/setup.py`:

```python
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
        'minimal_subscriber = my_first_package.minimal_subscriber:main',  # Add this line
    ],
},
```

**Step 3: Build the workspace**

```bash
cd ~/ros2_ws
colcon build
```

**Step 4: Source the workspace**

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Testing Publisher + Subscriber

Now you have both a publisher and subscriber. Let's test them together.

**Terminal 1: Run the publisher**

```bash
ros2 run my_first_package minimal_publisher
```

Output:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
...
```

**Terminal 2: Run the subscriber**

```bash
ros2 run my_first_package minimal_subscriber
```

Output:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
...
```

Success! The publisher is sending messages, and the subscriber is receiving them. Look at the output — the subscriber receives every message the publisher sends.

**Terminal 3: Inspect the system**

```bash
ros2 node list
```

Output:

```
/minimal_publisher
/minimal_subscriber
```

Both nodes are running.

```bash
ros2 topic list
```

Output:

```
/topic
```

There's one topic (the publisher and subscriber are on the same topic).

```bash
ros2 topic info /topic
```

Output:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

Perfect — 1 publisher and 1 subscriber on the same topic.

---

## Understanding Message Flow

Here's what happens inside ROS 2:

1. **Publisher publishes**: Node 1 calls `publisher.publish(msg)`
2. **ROS 2 middleware**: The ROS 2 message broker receives it and stores it
3. **Subscriber receives**: ROS 2 calls `listener_callback(msg)` in Node 2
4. **Process the message**: Node 2's callback runs and logs the data

This happens automatically — you don't manually pass messages between nodes. The middleware (DDS, ROS 2's underlying communication system) handles it.

---

## Processing Messages: A More Complex Subscriber

The simple subscriber just logs what it hears. Let's make it do something more interesting — process the data.

**Create a more complex subscriber:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ProcessingSubscriber(Node):
    def __init__(self):
        super().__init__('processing_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.message_count = 0

    def listener_callback(self, msg):
        self.message_count += 1

        # Log the message
        self.get_logger().info(f'Message #{self.message_count}: {msg.data}')

        # Extract the number from "Hello World: X"
        parts = msg.data.split(': ')
        if len(parts) == 2:
            try:
                number = int(parts[1])
                self.get_logger().info(f'  → Number extracted: {number}')

                # React based on the number
                if number % 5 == 0:
                    self.get_logger().warning(f'  → Milestone! Number {number} is divisible by 5')
            except ValueError:
                self.get_logger().error(f'  → Could not parse number from {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    processing_subscriber = ProcessingSubscriber()
    rclpy.spin(processing_subscriber)
    processing_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This subscriber:
1. **Counts messages** (keeps a running counter)
2. **Parses the message** (extracts the number)
3. **Reacts conditionally** (logs a warning every 5 messages)

When you run this with the publisher, the output becomes:

```
[INFO] [processing_subscriber]: Message #1: Hello World: 0
[INFO] [processing_subscriber]:   → Number extracted: 0
[WARNING] [processing_subscriber]:   → Milestone! Number 0 is divisible by 5
[INFO] [processing_subscriber]: Message #2: Hello World: 1
[INFO] [processing_subscriber]:   → Number extracted: 1
...
```

---

## Connecting Multiple Subscribers

One publisher can send to multiple subscribers. Let's test this.

**Terminal 1: Run the publisher**

```bash
ros2 run my_first_package minimal_publisher
```

**Terminal 2: Run the simple subscriber**

```bash
ros2 run my_first_package minimal_subscriber
```

**Terminal 3: Run the processing subscriber**

```bash
ros2 run my_first_package processing_subscriber  # You need to add this executable to setup.py
```

Both subscribers receive the same messages simultaneously. The publisher doesn't care how many subscribers are listening.

**Verify with:**

```bash
ros2 topic info /topic
```

Output:

```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 2
```

One publisher, two subscribers, one topic.

---

## Understanding Queue Depth

The `10` in `create_subscription()` is the queue depth. Here's what it means:

**If a subscriber is slow:**

```
Message arrives → Subscriber is busy → ROS 2 queues the message
Message arrives → Still busy → Queue now has 2 messages
...
Message arrives → Queue is full (10 messages) → Oldest message is dropped
```

**For real-time sensors** (camera, LIDAR), dropping old messages is fine — you only care about the latest. So queue depth might be 1-2.

**For critical commands** (navigation goals, safety stops), you never want to drop messages. So queue depth might be 100 or higher.

For now, use `10` as a reasonable default.

---

## Debugging Subscriber Issues

**Subscriber not receiving messages?**

1. Check that publisher is running: `ros2 node list`
2. Check topic name matches: `ros2 topic list`
3. Check message type matches: `ros2 topic info /topic`

**Callback not being called?**

1. Verify publisher is sending: `ros2 topic echo /topic`
2. Check for exceptions in your callback (any errors will be logged)
3. Ensure you're calling `rclpy.spin()` — without it, callbacks never run

**Messages arriving slowly?**

This is often expected. Network latency, system load, and message size all affect speed. Verify expected behavior with `ros2 topic hz /topic` to measure actual message rate.

---

## Try With AI

You have a working subscriber. Let's improve it with AI's help.

**Ask your AI:**

> "I have a ROS 2 subscriber that processes String messages. I want to add robustness: error handling for malformed messages, a timeout if the publisher stops sending (node goes silent), and statistics (how many messages received, average time between messages). Show me the code."

**Expected outcome:** AI will suggest:
- Try/except blocks in the callback
- A timeout timer that fires if no message arrives within X seconds
- Statistics tracking (message count, timestamps)
- A method to print statistics periodically

**Challenge AI:**

> "Your timeout approach uses a timer. Won't that be inefficient if the publisher is sending frequently but then stops for 5 minutes? Is there a better way?"

**Expected outcome:** AI will explain:
- Reset the timer in the callback (only counts silence)
- Trade-off: More complex code but better efficiency
- Alternative: Just track the last message timestamp and check it periodically

**Iterate together:**

> "Let me implement the timer-reset approach. But I also want to log a warning only once when the timeout first happens, not repeatedly. How would you prevent repeated warnings?"

**Expected outcome:** AI will show you how to:
- Track state (has_warned flag)
- Set it to True on first timeout
- Reset it when a message arrives
- Only log the warning if state changed

This is the Three Roles pattern in action — AI teaches you patterns, you guide it with your constraints, and you converge on a good solution.

---

## Exercises

1. **Modify the subscriber to count total messages received and print the count every 10 messages**
2. **Create a subscriber that extracts and processes numeric data (like temperature values)**
3. **Run 1 publisher with 3 different subscribers simultaneously**

---

## Reflect

Consider these questions:

1. **Why is callback-based design better than polling?** What would be wrong with checking for messages in a loop?

2. **What happens if your callback is slow?** If processing takes longer than the publish interval, what might occur?

3. **How does decoupling benefit system design?** The publisher doesn't know subscribers exist. Why is this an advantage?

You now understand the pub/sub pattern — the core communication mechanism in ROS 2. In the next lesson, you'll combine everything with AI collaboration to build more sophisticated systems.
