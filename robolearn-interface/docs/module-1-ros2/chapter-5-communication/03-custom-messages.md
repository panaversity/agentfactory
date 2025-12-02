---
id: custom-messages
title: "Lesson 5.3: Custom Message and Service Definitions"
sidebar_label: "5.3 Custom Interfaces"
sidebar_position: 3
chapter: 5
lesson: 3
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 5
learning_objectives:
  - "Understand why custom message types are needed"
  - "Create .msg and .srv interface files"
  - "Configure CMakeLists.txt and package.xml for interfaces"
  - "Build and use custom interfaces in nodes"
  - "Design interfaces that communicate intent clearly"
skills:
  - "ros2-custom-interfaces"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 5.3: Custom Message and Service Definitions

So far you've used built-in message types: `String`, `SetBool`, `AddTwoInts`. These work for simple cases. But what if you need to send something more complex?

Imagine a robot that reports its status. It needs to send:
- Battery level (float)
- Temperature (float)
- Active task (string)
- Joint angles (array of floats)
- Timestamp (builtin time)

Using `String` to pack all this would be ugly. You'd have to parse strings, no type safety, error-prone.

**Custom messages** let you define a data structure: "A RobotStatus contains a float battery, float temperature, string task, array joint_positions, and time stamp." Then ROS 2 generates Python classes automatically.

This lesson teaches you to:
1. Create `.msg` files (message definitions)
2. Create `.srv` files (service request/response definitions)
3. Configure build files so ROS 2 generates the code
4. Use custom types in your nodes

---

## Why Custom Messages Matter

### Without Custom Messages (Bad)

```python
# Publisher sending "battery:85.5,temp:45.2,task:moving"
msg = String()
msg.data = "battery:85.5,temp:45.2,task:moving"
self.publisher_.publish(msg)

# Subscriber receiving and parsing
if msg.data:
    parts = msg.data.split(',')
    battery = float(parts[0].split(':')[1])  # Ugly parsing
    temp = float(parts[1].split(':')[1])
    task = parts[2].split(':')[1]
```

Problems:
- Error-prone parsing
- No type safety
- No documentation of what fields exist
- Hard to extend (add new field = break existing code)

### With Custom Messages (Good)

```python
# Publisher
msg = RobotStatus()
msg.battery = 85.5
msg.temperature = 45.2
msg.task = "moving"
self.publisher_.publish(msg)

# Subscriber receiving
self.get_logger().info(
    f'Battery: {msg.battery}, Temp: {msg.temperature}, Task: {msg.task}')
```

Advantages:
- Type-safe (Python knows `battery` is a float)
- Self-documenting (clear field names)
- IDE autocomplete works
- Easy to extend

---

## Creating a Custom Message

Let's build the `RobotStatus` message. First, you need a package dedicated to interfaces.

### Step 1: Create an Interface Package

ROS 2 convention: interface packages are separate from executable packages. Create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_interfaces
cd my_robot_interfaces
```

This creates:
```
my_robot_interfaces/
├── CMakeLists.txt
├── package.xml
├── src/          (you can delete this)
└── include/      (you can delete this)
```

Add message and service folders:

```bash
mkdir msg
mkdir srv
```

### Step 2: Create the Message File

Create `msg/RobotStatus.msg`:

```
# File: msg/RobotStatus.msg
# Custom message for reporting robot status

# Standard timestamp
builtin_interfaces/Time stamp

# Basic status
bool is_active
float64 battery_level    # Range: 0.0 to 100.0

# Sensor readings
float64 temperature      # Celsius
float64[] joint_angles   # Array of joint positions in radians

# Current task
string current_task
```

Each line is a field. Format: `type name` with optional `# comment`

**Data types available:**
- `bool`, `byte`, `char` (single character)
- `int8`, `int16`, `int32`, `int64` (signed)
- `uint8`, `uint16`, `uint32`, `uint64` (unsigned)
- `float32`, `float64`
- `string` (UTF-8 text)
- `builtin_interfaces/Time` (timestamp)
- `geometry_msgs/Point`, `geometry_msgs/Pose` (standard geometry)

**Arrays:**
- `float64[]` — unbounded array (any size)
- `float64[4]` — fixed array (exactly 4 elements)
- `float64[<=10]` — bounded array (maximum 10 elements)

### Step 3: Create a Custom Service

Now create a service that controls the robot. It should take a command and return success/failure.

Create `srv/RobotCommand.srv`:

```
# Request: What the client sends
string command_type      # e.g. "start", "stop", "pause"
float64 parameter        # e.g. speed factor

---
# Response: What the server sends back
bool success
string message           # Feedback or error message
float64 execution_time   # How long it took (seconds)
```

The `---` separates request (above) from response (below).

### Step 4: Update CMakeLists.txt

The CMakeLists.txt needs to tell ROS 2 to generate code from your .msg and .srv files.

Edit `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/RobotCommand.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Required for ament build system
ament_package()
```

**Key parts:**
- `find_package(rosidl_default_generators REQUIRED)` — enables code generation
- `rosidl_generate_interfaces()` — generates Python/C++ code from .msg/.srv files
- `DEPENDENCIES builtin_interfaces geometry_msgs` — list packages your interfaces depend on

### Step 5: Update package.xml

Edit `package.xml` to declare build dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.1</version>
  <description>Custom interfaces for robot communication</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

**Critical line:**
```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

This tells the ROS 2 build system this is an interface package.

### Step 6: Build the Interface Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

If it succeeds, you now have:
- `from my_robot_interfaces.msg import RobotStatus`
- `from my_robot_interfaces.srv import RobotCommand`

Verify:

```bash
ros2 interface show my_robot_interfaces/msg/RobotStatus
```

Output:

```
builtin_interfaces/Time stamp
bool is_active
float64 battery_level
float64 temperature
float64[] joint_angles
string current_task
```

---

## Using Custom Messages in Nodes

Now let's create a node that publishes `RobotStatus`:

Create `robot_status_publisher.py` in your executable package:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import RobotStatus
from builtin_interfaces.msg import Time

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_pub')
        self.publisher_ = self.create_publisher(
            RobotStatus, 'robot_status', 10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.battery = 100.0

    def publish_status(self):
        """Publish robot status every second."""
        msg = RobotStatus()

        # Set timestamp (simplified—normally use get_clock())
        msg.stamp.sec = 0  # You'd get this from ROS clock
        msg.stamp.nanosec = 0

        # Set other fields
        msg.is_active = True
        msg.battery_level = self.battery
        msg.temperature = 42.5
        msg.joint_angles = [0.1, 0.2, 0.3]  # List becomes array
        msg.current_task = "moving"

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: battery={msg.battery_level}, temp={msg.temperature}')

        # Simulate battery drain
        self.battery -= 0.5

def main(args=None):
    rclpy.init(args=args)
    pub = RobotStatusPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And a subscriber:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import RobotStatus

class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_sub')
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10)

    def status_callback(self, msg):
        """Called when status message arrives."""
        self.get_logger().info(
            f'Battery: {msg.battery_level:.1f}%, '
            f'Temp: {msg.temperature:.1f}C, '
            f'Task: {msg.current_task}')

def main(args=None):
    rclpy.init(args=args)
    sub = RobotStatusSubscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Important:** You must import from your interfaces package:

```python
from my_robot_interfaces.msg import RobotStatus
```

---

## Using Custom Services

Create a node that uses the `RobotCommand` service:

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import RobotCommand
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.service = self.create_service(
            RobotCommand,
            'control_robot',
            self.command_callback)

    def command_callback(self, request, response):
        """Handle robot commands."""
        start_time = time.time()

        self.get_logger().info(f'Command: {request.command_type}, param: {request.parameter}')

        if request.command_type == 'start':
            response.success = True
            response.message = 'Robot started'
        elif request.command_type == 'stop':
            response.success = True
            response.message = 'Robot stopped'
        else:
            response.success = False
            response.message = f'Unknown command: {request.command_type}'

        response.execution_time = time.time() - start_time
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Call it:

```python
from my_robot_interfaces.srv import RobotCommand

cli = node.create_client(RobotCommand, 'control_robot')

request = RobotCommand.Request()
request.command_type = 'start'
request.parameter = 0.5

future = cli.call_async(request)
```

---

## Build Procedure (Complete)

When you have custom interfaces:

```bash
# 1. Build interface package first
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces

# 2. Source to use new interfaces
source install/setup.bash

# 3. Build packages that use the interfaces
colcon build --packages-select my_first_package

# 4. Source again (new generated code available)
source install/setup.bash
```

**Order matters:** Interface package must build before any node that uses it.

---

## Design Principles for Messages

**Message Design Checklist:**
1. ✅ All fields have clear names and comments
2. ✅ Consistent units (meters, radians, Hz, etc.)
3. ✅ Use standard types when possible (geometry_msgs, sensor_msgs)
4. ✅ Include timestamp if time-sensitive
5. ✅ Document value ranges (e.g., battery 0.0-100.0)
6. ✅ Consider extensibility (room for new fields)

**Bad message design:**
```
float64 value1
float64 value2
string status
```

No one knows what these mean. What are the units?

**Good message design:**
```
float64 battery_level        # Range: 0.0-100.0 percent
float64 motor_temperature    # Celsius
string operational_status    # Options: idle, moving, charging, error
```

Clear, self-documenting, extensible.

---

## Key Insights

**Separate Interface Packages:**
Keep interfaces in their own package. Nodes that use them depend on the interface package.

**Build Order:**
Interfaces first, then everything else.

**Standard Dependencies:**
Always use standard types from `geometry_msgs`, `sensor_msgs`, `builtin_interfaces` when available.

**Arrays vs Fields:**
- `float64[]` for variable-size arrays (joint positions vary by robot)
- `float64[4]` for fixed-size when count never changes

---

## Try With AI

You have custom messages and services. Let's extend them.

**Ask your AI:**

> "I've created RobotStatus and RobotCommand interfaces. Now I want to add a third interface: 'SensorReading' that captures: timestamp, sensor_id (int), reading_type (string like 'temperature', 'pressure'), value (float64), and confidence (0.0-1.0). Show me the .msg file and explain the design."

**Expected outcome:** AI will provide:
- Complete .msg file with proper types
- Comments explaining each field
- Rationale for chosen types

**Challenge the design:**

> "Good, but should confidence be a float between 0-1, or should I use an enum like RELIABLE, MODERATE, UNRELIABLE?"

**Expected outcome:** AI will explain:
- Float: More expressive, precise quantification
- Enum: Simpler clients, no interpretation needed
- Tradeoffs for your use case

**Iterate:**

> "Let me add a 'quality' string field that captures MORE than just confidence—things like 'sensor_warm_up', 'noisy_environment', etc. Show me the updated interface."

This is AI teaching interface design patterns, you refining requirements, collaborating on the best structure.

---

## Exercises

1. **Create a `SensorReading` message** with: timestamp, sensor_id, reading_type, value, confidence
2. **Create a `RobotState` service** that takes no request and returns current battery, temperature, and active task
3. **Build both** and verify with `ros2 interface show`
4. **Create a publisher** that sends sensor readings every 2 seconds
5. **Create a client** that queries robot state once per second and logs results

---

## Reflection

Before the next lesson, think about:

- Why are interface packages separate from executable packages?
- When would you use `builtin_interfaces/Time` vs just a float64 for seconds?
- How do you design messages so they're extensible (new fields can be added later)?

In the next lesson, we'll pull everything together: topics, services, and custom interfaces. You'll learn the design patterns for complex multi-node systems.
