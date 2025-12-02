---
id: lesson-4-4-try-with-ai
title: "Lesson 4.4: Try With AI"
sidebar_position: 4
sidebar_label: "4.4 Try With AI"
description: "Collaborate with AI to extend pub/sub systems with validation and production refinement."
chapter: 4
lesson: 4
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 2
learning_objectives:
  - "Collaborate with AI to extend pub/sub systems"
  - "Validate and refine AI-generated code"
  - "Recognize when AI knowledge exceeds your own (Teacher role, invisible)"
  - "Guide AI using constraints and domain context (Student role, invisible)"
  - "Iterate toward better designs through bidirectional refinement (Co-Worker, invisible)"
  - "Understand production-readiness concepts"
skills:
  - "ros2-publisher-subscriber"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 4.4: Try With AI

You now have working publisher and subscriber nodes. They send and receive messages. But there's a gap between "working code" and "production code."

Production code handles errors gracefully, logs diagnostics, configures parameters, and recovers from failures. Writing all that by hand is tedious and error-prone. This is where AI collaboration shines.

In this lesson, you'll work with AI to improve your pub/sub system. You'll discover how AI can suggest patterns you didn't know existed, how you can refine those suggestions with your constraints, and how iteration produces better designs than either of you could create alone.

---

## Three Scenarios: Discovering Bidirectional Learning

### Scenario 1: AI Teaches You (Pattern Discovery)

You have a simple publisher. It works but feels incomplete:

```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.i}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
```

**You ask your AI:**

> "I have a simple ROS 2 publisher that sends String messages. It works, but I feel like I'm missing something for production use. What would a production-ready publisher look like? Give me specific code improvements."

**AI responds** (teaching you patterns):

Here's a production-ready version:

```python
def timer_callback(self):
    try:
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Log at appropriate level (DEBUG for verbose, INFO for normal)
        self.get_logger().debug(f'Publishing message #{self.i}')

        self.publisher_.publish(msg)
        self.i += 1

        # Track metrics
        if self.i % 100 == 0:
            self.get_logger().info(f'Published {self.i} messages total')

    except Exception as e:
        self.get_logger().error(f'Error in callback: {e}')
        # Don't crash, just log and continue
```

**What you learned:**
- Error handling (try/except in callbacks)
- Logging levels (DEBUG vs INFO vs ERROR)
- Metrics tracking (how many total?)
- Graceful degradation (log errors, don't crash)

The AI suggested patterns from thousands of ROS 2 projects — patterns you might not discover on your own for months.

---

### Scenario 2: You Teach AI (Constraint Refinement)

AI's suggestion is good but doesn't fit your constraints:

**You respond:**

> "That looks good, but our system has tight bandwidth constraints. We can only send 10 messages per second maximum. Your code publishes every 0.5 seconds (2x per second). Also, we don't need DEBUG logging — just the milestone messages. Can you show me how to make it configurable so we can change the rate without rebuilding?"

**AI adapts** (learning from your constraints):

Here's a configurable version:

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare configurable parameter
        self.declare_parameter('publish_rate_hz', 2.0)
        rate = self.get_parameter('publish_rate_hz').value

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        try:
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.i += 1

            # Only log milestones (every 10 messages)
            if self.i % 10 == 0:
                self.get_logger().info(f'Milestone: {self.i} messages published')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
```

**Now you can control the rate without rebuilding:**

```bash
# Rebuild once
cd ~/ros2_ws && colcon build

# Run with default rate (2 Hz)
ros2 run my_first_package minimal_publisher

# Run with custom rate (1 Hz - respects bandwidth limit)
ros2 run my_first_package minimal_publisher --ros-args -p publish_rate_hz:=1.0
```

**What happened:**
- You provided your constraint (10 messages/second max)
- AI refined its suggestion (made it configurable)
- You now control rate at runtime without recompiling

Your feedback shaped the solution. The dialogue converged on something better than the original suggestion.

---

### Scenario 3: Iterative Refinement (Convergence)

You realize the parameter approach could be even better:

**You ask:**

> "Good! Now I want to take this further. Different robots use different message types (String for simple, geometry_msgs/Twist for velocity commands). Can we design a system where I can change both the rate AND the message type without editing code? I'm thinking: a base class or abstraction that makes this easy."

**AI suggests:**

```python
from abc import ABC, abstractmethod

class ConfigurablePublisher(Node, ABC):
    """Base class for production publishers."""

    def __init__(self, node_name, topic_name, msg_type, queue_size=10):
        super().__init__(node_name)

        # Configurable parameters
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('topic_name', topic_name)

        rate = self.get_parameter('publish_rate_hz').value
        topic = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(msg_type, topic, queue_size)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
        self.message_count = 0

    @abstractmethod
    def create_message(self):
        """Subclasses implement this to create appropriate message type."""
        pass

    def timer_callback(self):
        try:
            msg = self.create_message()
            self.publisher_.publish(msg)
            self.message_count += 1

            if self.message_count % 10 == 0:
                self.get_logger().info(f'Published {self.message_count} messages')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


# Subclass for String messages
class StringPublisher(ConfigurablePublisher):
    def __init__(self):
        super().__init__('string_publisher', 'topic', String)
        self.counter = 0

    def create_message(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.counter += 1
        return msg


# Subclass for Twist messages (velocity commands)
from geometry_msgs.msg import Twist

class VelocityPublisher(ConfigurablePublisher):
    def __init__(self):
        super().__init__('velocity_publisher', 'cmd_vel', Twist)

    def create_message(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward velocity
        msg.angular.z = 0.1  # Rotation
        return msg
```

**Now you have reusable infrastructure:**

```python
# For String messages
string_pub = StringPublisher()

# For Velocity commands
vel_pub = VelocityPublisher()

# Both respect the same rate parameter
# Both log milestones
# Both handle errors the same way
```

**Iteration complete:**
- Iteration 1: Simple working code
- Iteration 2: Production-ready with error handling and logging
- Iteration 3: Configurable rate parameter
- Iteration 4: Abstraction for multiple message types

**What emerged:** A reusable pattern that neither of you designed independently. AI suggested the pattern, you refined it with constraints, and together you converged on an elegant abstraction.

Through iteration, the design improved beyond what either starting point offered.

---

## What Just Happened

Notice the flow:
1. **You asked** → AI suggested production patterns
2. **You pushed back** → AI adapted to your bandwidth constraint
3. **You iterated** → The design converged on an elegant abstraction

This dialogue pattern repeats throughout your career. The more specific your questions and constraints, the more useful AI becomes.

---

## Guided Practice: Your Turn

Now you try. Pick one of these scenarios and ask your AI:

### Practice Option 1: Subscriber Robustness

You have a subscriber that receives messages. Ask your AI:

> "I have a ROS 2 subscriber that receives String messages and prints them. I want to make it robust: detect when the publisher dies (no messages for 5 seconds), log statistics (total received, rate), and handle malformed messages gracefully. Show me the code."

Expected outcome: AI suggests timeout detection, metrics, and error handling. You refine based on your needs. You iterate toward a robust design.

### Practice Option 2: Multi-Node Coordination

You have a publisher and subscriber. Ask your AI:

> "I want to expand my system: one publisher sends velocity commands, three different subscribers handle motor control, odometry logging, and safety monitoring. How would you structure this for maintainability? Should each subscriber be a separate node or can they share one?"

Expected outcome: AI explains the trade-offs. You decide based on your use case. You iterate on the architecture.

### Practice Option 3: Hardware Abstraction

You have a simple publisher. Ask your AI:

> "This publisher works on my laptop but needs to work on three different robot platforms (Jetson, Raspberry Pi, and a cloud VM). The message type and rate are the same, but the network path is different. How would I abstract this so the publisher code doesn't change between platforms?"

Expected outcome: AI suggests configuration files or environment variables. You refine based on your deployment needs. You iterate toward a portable design.

---

## Validating AI-Generated Code

When AI suggests code, always verify it works:

```python
# AI suggests this — does it make sense?
self.declare_parameter('publish_rate_hz', 2.0)
rate = self.get_parameter('publish_rate_hz').value
self.timer = self.create_timer(1.0/rate, self.timer_callback)

# Test it:
# Can I understand what each line does?
# ✓ Declare parameter named 'publish_rate_hz' with default 2.0 Hz
# ✓ Read the parameter value
# ✓ Create a timer with period = 1.0/rate

# Does it compile/run?
# ✓ Yes, at least in isolation

# Does it match ROS 2 Humble patterns?
# ✓ Yes, parameters are declared, read, and used this way

# Could there be edge cases?
# ✓ What if rate is 0? (would crash on 1.0/0)
# ✓ Better to validate: if rate <= 0, log error and use default

# Refine the code based on your analysis
```

Always test suggested code. AI is right most of the time but can miss edge cases that your domain knowledge catches.

---

## Common AI Mistakes to Watch For

1. **Outdated API**: "That's the ROS 1 way. ROS 2 Humble changed that. What's the new pattern?"
2. **Missing imports**: "Your code uses Twist but doesn't import it. What package does Twist come from?"
3. **Unsafe defaults**: "Your code doesn't validate parameters. What should happen if the rate is negative?"
4. **Overengineering**: "That's a lot of abstraction for a simple need. Can we simplify?"

When you catch a mistake, correct it and show AI the right approach. AI learns from your corrections.

---

## Using the ros2-publisher-subscriber Skill

Throughout this chapter, you've been using patterns from the `ros2-publisher-subscriber` skill. This skill captured:

- **Publisher pattern**: Node class, timer callback, publish cycle
- **Subscriber pattern**: Node class, subscription callback, message handling
- **Common mistakes**: Missing cleanup, wrong imports, queue depth confusion
- **Debugging approaches**: Using ros2 topic echo, ros2 topic info

The skill exists so that whenever you write another pub/sub node, you don't start from scratch. You reference the proven pattern and adapt it.

This pattern will compound across your learning:
- **Module 1**: You learn pub/sub
- **Module 2**: You use pub/sub for sensor data (LIDAR, camera)
- **Module 3**: You compose multiple pub/sub nodes into a system
- **Module 4**: You use pub/sub as part of a sophisticated autonomous system

---

## Intelligence Created in This Chapter

### ros2-publisher-subscriber Skill

**What it captures:**
- Anatomy of a ROS 2 node (init → timer/callback → spin → shutdown)
- Publisher pattern (create_publisher → publish)
- Subscriber pattern (create_subscription → callback)
- Common parameters (QoS queue depth, topic names)
- Debugging techniques (ros2 topic echo, ros2 topic info)

**When to use it:**
- Writing any ROS 2 publisher or subscriber
- Extending existing pub/sub nodes
- Teaching others the pattern

**Cross-book value:**
- ANY robotics course needs this pattern
- ROS 2 is the standard middleware in industry
- This pattern compounds across future books

---

## Reflection: What You've Accomplished

**In this lesson, you:**
- Worked with AI to improve code
- Guided AI with your specific constraints
- Iterated toward better designs through dialogue
- Validated AI-generated code critically
- Created production-ready patterns

**In this chapter, you:**
- Created a ROS 2 workspace from scratch
- Built packages with colcon
- Wrote a publisher (sending data)
- Wrote a subscriber (receiving data)
- Collaborated with AI to improve both

**In Module 1, you:**
- Understand physical vs software AI (Chapter 1)
- Know how robots sense and move (Chapter 2)
- Explored ROS 2 via CLI (Chapter 3)
- Can write ROS 2 code in Python (Chapter 4)

You're building toward the capstone project in Chapter 7, where you'll integrate all these pieces into a complete multi-node system.

---

## Try With AI (Final Challenge)

This is your opportunity to practice the Three Roles pattern without guidance. Pick a scenario that excites you and collaborate with AI:

### Challenge 1: Performance Optimization

> "My publisher and subscriber are working but feel slow. The publisher publishes 100 times per second but the subscriber can only process 50 messages per second. Messages are getting dropped. How would you diagnose this? What are the solutions? Let's design one together."

Expected outcome: AI suggests diagnostics (ros2 topic hz), possible solutions (increase queue depth, optimize callback, reduce publish rate), trade-offs for each. You converge on a solution for your needs.

### Challenge 2: Error Recovery

> "If the publisher crashes, the subscriber tries to access data that's no longer being sent. What's the best way to handle this? Should the subscriber timeout and exit? Log a warning? Try to reconnect? Teach me the patterns."

Expected outcome: AI teaches timeout patterns, exponential backoff, health monitoring. You refine based on your system's criticality. You iterate toward a robust solution.

### Challenge 3: System Architecture

> "I have a simple pub/sub system now, but the final product needs 5 publishers and 8 subscribers all communicating. How would you organize this code? Should it be one giant package or multiple packages? How do I keep it maintainable?"

Expected outcome: AI explains modularity trade-offs. You decide based on your team structure. You iterate toward an architecture that scales.

---

## Reflect

Consider these questions:

1. **When did AI's suggestion surprise you?** What patterns or techniques did you learn that you wouldn't have discovered on your own?

2. **When did you push back?** How did providing your specific constraints change the AI's suggestions?

3. **What emerged through iteration?** How did the final design differ from both your starting idea and AI's first suggestion?

4. **Where might you apply this dialogue pattern?** What other problems in your work could benefit from this kind of iterative refinement?

---

You've mastered pub/sub. In Chapter 5, you'll learn services (request/response), custom message types, and design patterns for choosing between topics and services.
