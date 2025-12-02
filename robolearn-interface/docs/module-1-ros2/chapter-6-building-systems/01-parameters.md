---
id: lesson-6-1-parameters
title: "Lesson 6.1: Parameters (Configurable Nodes)"
sidebar_position: 1
sidebar_label: "6.1 Parameters"
description: "Learn how to make ROS 2 nodes configurable using parameters that can be modified at runtime without recompiling code."
duration_minutes: 60
proficiency_level: "B1"
layer: "L3"
hardware_tier: 1
learning_objectives:
  - "Declare and read ROS 2 parameters in nodes"
  - "Modify parameters at runtime using ros2 param commands"
  - "Implement parameter validation callbacks"
  - "Design parameter-driven configuration systems"
  - "Apply parameters in multi-node launch configurations"
---

# Lesson 6.1: Parameters — Making Nodes Configurable

In the publisher and subscriber examples you built earlier, the publish rate and other behaviors were hardcoded in the Python source. What if you wanted to change the rate without editing and recompiling the code? That's where ROS 2 parameters come in.

Parameters are runtime configuration values that allow you to adjust node behavior while the system is running. A node can **declare** parameters with default values, **read** those parameters when needed, and you can **modify** them from the command line using `ros2 param` commands.

## Understanding Parameters

Think of parameters as node-level settings—they're specific to each node, stored in a parameter server, and independent of the publish/subscribe message flow.

### Why Parameters Matter

In our earlier publisher example, the timer period was fixed:

```python
timer_period = 0.5  # seconds → hardcoded
```

If you wanted to change the publish rate to 2 Hz (instead of 0.5 Hz), you had to:
1. Edit the Python file
2. Rebuild the package with `colcon build`
3. Stop and restart the node

With parameters, you simply run:
```bash
ros2 param set /minimal_publisher publish_rate 2.0
```

The node picks up the new value immediately (if coded to check the parameter periodically).

### Parameter Scope

Parameters are **node-specific**. Each running instance of a node has its own set of parameters. If you run two instances of the same node executable, they maintain separate parameter values.

## Declaring and Reading Parameters

### Parameter Declaration

The first step in using a parameter is to **declare** it in the node's constructor. This registers the parameter with the parameter server and sets a default value.

**Worked Example:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare a parameter with a default value
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # Read the parameter value
        rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create timer using the rate
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

**Output (from two terminals):**

Terminal 1 (running the node):
```
[INFO] [configurable_publisher]: Starting publisher at 1.0 Hz
[INFO] [configurable_publisher]: Published: Hello 0
[INFO] [configurable_publisher]: Published: Hello 1
[INFO] [configurable_publisher]: Published: Hello 2
```

Terminal 2 (echoing the topic):
```bash
ros2 topic echo /topic
---
data: 'Hello 0'
---
data: 'Hello 1'
---
```

### Reading Parameters Dynamically

Parameters can be read at any time. To **read a parameter value**, use:

```python
rate = self.get_parameter('publish_rate').value
```

If you declare a parameter but want to use its value multiple times, you can cache it (as shown above) or read it every callback cycle if the parameter might change.

## Modifying Parameters at Runtime

With the configurable publisher running, you can change the publish rate without restarting:

```bash
# List all parameters for the node
ros2 param list /configurable_publisher
# Output: /configurable_publisher:
#   publish_rate

# Get the current value
ros2 param get /configurable_publisher publish_rate
# Output: publish_rate: 1.0

# Set a new value
ros2 param set /configurable_publisher publish_rate 5.0

# Verify the change
ros2 param get /configurable_publisher publish_rate
# Output: publish_rate: 5.0
```

The node will now publish at 5 Hz instead of 1 Hz.

## Guided Practice: Making a Node Configurable

Let's extend the concept with a node that uses multiple parameters.

### Task: Create a Flexible Message Publisher

Create a node that publishes messages with the following configurable properties:
- **publish_rate** (float): Messages per second (1-10 Hz)
- **message_prefix** (string): Text to prepend to each message
- **enable_logging** (bool): Whether to log each publication

**Solution:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FlexiblePublisher(Node):
    def __init__(self):
        super().__init__('flexible_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('message_prefix', 'Message')
        self.declare_parameter('enable_logging', True)

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'flexible_topic', 10)

        # Set up timer
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

**Test it:**

```bash
# Terminal 1: Run the node
ros2 run my_first_package flexible_publisher

# Terminal 2: Modify parameters
ros2 param set /flexible_publisher publish_rate 5.0
ros2 param set /flexible_publisher message_prefix "Status Update"
ros2 param set /flexible_publisher enable_logging false
```

**Expected behavior**: The node updates its behavior instantly—publishing faster, changing message content, and toggling logging, all without restarting.

## Parameter Design Patterns

### Best Practice: Declare at Startup, Read When Needed

```python
def __init__(self):
    # Declare all parameters in __init__
    self.declare_parameter('param1', default_value_1)
    self.declare_parameter('param2', default_value_2)

def timer_callback(self):
    # Read parameter values when needed
    value = self.get_parameter('param2').value
    # Use value...
```

This pattern ensures:
- All parameters are registered when the node starts
- The parameter server knows what values exist
- You can change parameters at runtime
- The node always uses the current value

### Parameter Validation

When you modify a parameter at runtime, should you validate it? Yes, especially for safety-critical values.

**With validation:**

```python
class SafePublisher(Node):
    def __init__(self):
        super().__init__('safe_publisher')

        # Declare with constraints (ROS 2 parameter constraints are limited)
        self.declare_parameter('publish_rate', 1.0)

        # Create a callback for parameter changes
        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'publish_rate':
                # Validate: publish_rate must be 0.1 to 100 Hz
                if param.value < 0.1 or param.value > 100:
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
```

## Independent Practice

### Exercise 1: Configurable Data Logger

Create a node that:
1. Subscribes to messages on `/sensor_data` topic
2. Has a parameter `log_frequency` (how many messages between logging)
3. Has a parameter `output_file` (filename to log to)
4. Logs every Nth message with a timestamp

Test by:
- Running a publisher on `/sensor_data`
- Changing `log_frequency` at runtime
- Verifying the logging behavior changes

### Exercise 2: Parameter Reflection

Write a small utility node that:
1. Takes a node name as a command-line argument
2. Queries all its parameters using `ros2 param list`
3. Prints them in a formatted table

This exercise helps you understand how parameter introspection works.

## AI Collaboration: Extending Configurability

Now let's see how AI can help you design more sophisticated parameter systems.

**Your request:**
"I have a node that publishes sensor data. I want to make the message type configurable—sometimes I publish String messages, sometimes custom RobotStatus messages. How would I do that?"

**Observation**: This is a more complex design problem. The parameter would need to control not just a value, but which message type the node publishes. This is tricky because C++ and Python have different type handling.

**AI's approach:**
"This is where you hit a limitation of runtime parameters—they can't change Python types dynamically. However, you could:

1. **Option A**: Create two separate executables (one for String, one for RobotStatus)
2. **Option B**: Use a **union message** that contains both types (less clean)
3. **Option C**: Always publish RobotStatus (superset), letting subscribers ignore unused fields

For most robotics systems, Option C—designing a **single message type that covers all cases**—works best. The publisher is simple, subscribers get what they need, and you avoid runtime type switching complexity."

**Key insight**: Parameters configure **values**, not **types**. For structural changes (message types, service definitions), you create different nodes or design polymorphic message types. This discussion revealed the boundaries of parameter-based configuration.

## Try With AI

**Setup:** Open a chat with your AI tool and have it help you design a parameter-driven node configuration.

**Prompt Set:**

```
Prompt 1: "I want to create a ROS 2 node that publishes at different rates depending on robot mode (fast, normal, slow). How would I use parameters for this?"

Prompt 2: "My node publishes sensor data, but different robots have different sensor configurations. Some have cameras, some have LIDAR, some have both. Can I use parameters to configure what sensors to include?"

Prompt 3: "What's a good way to validate parameter values in ROS 2? For example, ensuring publish_rate is between 0.1 and 100 Hz?"
```

**Expected Outcomes:**
- AI shows how to design parameter-driven branching logic
- AI explains conditional sensor inclusion patterns
- AI demonstrates parameter validation callbacks

**Safety Note**: Parameter modification happens at runtime without restarting the node—ensure any validation is in place before using parameters that affect robot motion or safety-critical behavior.

**Optional Stretch:**
Create a node with parameters for:
- Robot name (string)
- Battery threshold (float, 0-100)
- Motor speed limit (float, 0-1.0)
- Safety mode (bool)

Then write a small script that changes all parameters and logs the results to verify they took effect.
