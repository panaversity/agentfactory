---
id: design-patterns
title: "Lesson 5.4: Communication Design Patterns"
sidebar_label: "5.4 Design Patterns"
sidebar_position: 4
chapter: 5
lesson: 4
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 6
learning_objectives:
  - "Design communication architectures using topics and services"
  - "Apply decision frameworks for topics vs services"
  - "Compose multi-node systems with mixed communication patterns"
  - "Debug communication issues in complex systems"
  - "Refine communication architecture through AI iteration"
skills:
  - "ros2-service-pattern"
  - "ros2-custom-interfaces"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct) with multiple terminal windows"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 5.4: Communication Design Patterns

You now know how to build topics (continuous streams), services (request/response), and custom interfaces (type-safe communication). But how do you put them together into a real system?

In this lesson, you'll learn **communication architecture patterns**—how real robots design systems that coordinate multiple nodes, each with a specific job. You'll see how NASA designs rover communication, how warehouse robots organize sensor data, and how humanoid robots orchestrate body control.

The key insight: **Good architecture is invisible. You don't notice it until something breaks.**

---

## The Decision Framework: Topics vs Services (Revisited)

Let's formalize the decision you've been making intuitively:

```
START: Do we need to send data?
  └─ YES
       └─ Does the receiver need to respond/confirm?
            ├─ NO  →  Use TOPIC (fire and forget)
            └─ YES
                 └─ Can the sender block while waiting?
                      ├─ YES  →  Use SERVICE (synchronous)
                      └─ NO   →  Use ACTION (Module 2)
```

**Real examples:**

| Scenario | Pattern | Why |
|----------|---------|-----|
| Camera publishes images 30x/second | Topic | Continuous, no response needed |
| Battery publishes status every 2 sec | Topic | Continuous stream |
| Move robot forward 1 meter | Service | Needs confirmation, relatively rare |
| Enable/disable robot | Service | Command with response, infrequent |
| Publish odometry (position updates) | Topic | Continuous, multiple subscribers |
| Get current battery level | Service | Query response, on-demand |

**Decision tree for robotics:**

```
Sensor reading? → TOPIC
Movement command? → SERVICE
Emergency stop? → SERVICE (fast, synchronous)
Telemetry data? → TOPIC
Configuration change? → SERVICE
Motor speed updates? → TOPIC
Pick object (on/off)? → SERVICE
```

---

## Architectural Pattern 1: Layered Communication (Hub and Spoke)

Most robots have a **central coordinator** that talks to specialized modules.

```
                    ┌─────────────┐
                    │  Main       │
                    │  Coordinator│
                    └──────┬──────┘
            ┌───────┬──────┼──────┬────────┐
            │       │      │      │        │
        SUBSCRIBE SUBSCRIBE SUBSCRIBE PUBLISH
        TO STATUS  TO SENSOR TO BATTERY
            │       │      │      │
         ┌──▼──┐ ┌─▼──┐ ┌─▼──┐ ┌─▼───┐
         │Motor│ │Arm │ │Head│ │Legs │
         │Ctrl │ │Ctrl│ │Ctrl│ │Ctrl │
         └─────┘ └────┘ └────┘ └─────┘
```

**Pattern:**
- Central node subscribes to status from all modules
- Central node sends commands to specific modules via services
- Modules continuously publish sensor data via topics
- Emergency stops use dedicated high-priority topic

**Advantage:** Coordinator has complete picture. Modules are independent.

**Disadvantage:** Central node becomes bottleneck for high-frequency updates.

---

## Architectural Pattern 2: Distributed Pub/Sub (Many-to-Many)

When you don't need a central coordinator:

```
┌────────────┐    ┌─────────────┐    ┌──────────┐
│  Sensors   │    │  Planning   │    │  Actuators
│            │    │             │    │
│ Publishes: │    │ Subscribes: │    │ Subscribes:
│ - camera   │────▶ - camera    │    │ - motion_cmd
│ - lidar    │    │ - lidar     │    │
│ - imu      │    │             │    │ Publishes:
│            │    │ Publishes:  │    │ - motor_state
│            │    │ - motion_cmd────▶
│            │    │ - gesture   │    │
│            │    │             │    │
└────────────┘    └─────────────┘    └──────────┘
```

**Pattern:**
- Each node publishes its output data
- Each node subscribes to the data it needs
- No central coordinator
- Nodes are loosely coupled

**Advantage:** Scales well, any node can come/go, no bottleneck.

**Disadvantage:** Hard to debug (complex interconnections).

---

## Architectural Pattern 3: Request/Response Clusters

Use services for discrete tasks:

```
┌──────────────────────────────────────┐
│          Robot Main Loop             │
│                                      │
│  1. Call /get_sensor_reading         │
│  2. Call /plan_motion                │
│  3. Call /execute_command            │
│  4. Call /report_status              │
└────┬─────────┬────────────┬──────────┘
     │         │            │
     │         │            │
   SERVICE   SERVICE      SERVICE
     │         │            │
  ┌──▼──┐  ┌──▼──┐      ┌──▼──┐
  │Sensor│  │Planner   │Motor │
  │Server│  │Server    │Server│
  └──────┘  └────────┘  └─────┘
```

**Pattern:**
- Main loop orchestrates via service calls
- Services execute discrete tasks
- Synchronous, deterministic

**Advantage:** Clear control flow, easy to sequence operations.

**Disadvantage:** Slower (services wait for response).

---

## Real Example: Two-Node Communication

Let's build a complete system with two nodes:
- **Sensor Node**: Publishes sensor readings and responds to queries
- **Control Node**: Subscribes to sensors and commands them

### Interface Package (Shared)

Create interfaces (`my_robot_interfaces`):

**msg/SensorReading.msg:**
```
builtin_interfaces/Time timestamp
string sensor_name
float64 value
```

**srv/GetSensorData.srv:**
```
string sensor_id
---
float64 latest_value
string status
```

**srv/SetMotorSpeed.srv:**
```
string motor_id
float64 speed_rpm
---
bool success
string message
```

### Sensor Node

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorReading
from my_robot_interfaces.srv import GetSensorData

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Publish continuous sensor data
        self.publisher_ = self.create_publisher(
            SensorReading, 'sensor_data', 10)

        # Respond to queries
        self.service = self.create_service(
            GetSensorData,
            'query_sensor',
            self.query_callback)

        # Simulate sensor data
        self.temperature = 25.0
        self.timer = self.create_timer(1.0, self.publish_sensor)

    def publish_sensor(self):
        """Publish sensor readings continuously."""
        msg = SensorReading()
        msg.timestamp.sec = 0  # Simplified
        msg.sensor_name = 'temperature'
        self.temperature += 0.1  # Simulate gradual increase
        msg.value = self.temperature

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: temperature={msg.value:.1f}')

    def query_callback(self, request, response):
        """Respond to sensor queries."""
        self.get_logger().info(f'Query for sensor: {request.sensor_id}')

        if request.sensor_id == 'temperature':
            response.latest_value = self.temperature
            response.status = 'OK'
        else:
            response.latest_value = 0.0
            response.status = 'UNKNOWN_SENSOR'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Control Node

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorReading
from my_robot_interfaces.srv import GetSensorData, SetMotorSpeed

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Subscribe to continuous sensor stream
        self.subscription = self.create_subscription(
            SensorReading,
            'sensor_data',
            self.sensor_callback,
            10)

        # Create clients for on-demand queries
        self.sensor_client = self.create_client(
            GetSensorData, 'query_sensor')
        self.motor_client = self.create_client(
            SetMotorSpeed, 'set_motor')

        # Control loop
        self.timer = self.create_timer(2.0, self.control_loop)
        self.latest_temp = 0.0

    def sensor_callback(self, msg):
        """Handle streaming sensor data."""
        self.latest_temp = msg.value
        self.get_logger().debug(
            f'Received {msg.sensor_name}: {msg.value:.1f}')

    def control_loop(self):
        """Main control logic every 2 seconds."""
        self.get_logger().info(
            f'Control loop: latest temp = {self.latest_temp:.1f}')

        # Decision: if temp high, slow motor
        if self.latest_temp > 30.0:
            self.command_motor('motor_0', 50.0)  # Reduce to 50 RPM
        else:
            self.command_motor('motor_0', 100.0)  # Normal speed

    def command_motor(self, motor_id, speed_rpm):
        """Send motor command via service."""
        if not self.motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Motor service unavailable')
            return

        request = SetMotorSpeed.Request()
        request.motor_id = motor_id
        request.speed_rpm = speed_rpm

        future = self.motor_client.call_async(request)
        future.add_done_callback(self.motor_response_callback)

    def motor_response_callback(self, future):
        """Handle motor command response."""
        response = future.result()
        self.get_logger().info(f'Motor response: {response.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Motor Service Node

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetMotorSpeed

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.service = self.create_service(
            SetMotorSpeed,
            'set_motor',
            self.motor_callback)

        self.motor_speeds = {}
        self.get_logger().info('Motor service ready')

    def motor_callback(self, request, response):
        """Execute motor commands."""
        self.motor_speeds[request.motor_id] = request.speed_rpm

        self.get_logger().info(
            f'Set {request.motor_id} to {request.speed_rpm} RPM')

        response.success = True
        response.message = f'Motor set to {request.speed_rpm} RPM'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing

**Terminal 1: Sensor node**
```bash
ros2 run my_first_package sensor_node
```

**Terminal 2: Motor node**
```bash
ros2 run my_first_package motor_node
```

**Terminal 3: Control node**
```bash
ros2 run my_first_package control_node
```

You'll see:
- Sensor continuously publishes temperature
- Control receives it and adjusts motor based on temperature
- Motor service responds to commands

---

## Debugging Communication Issues

### Problem 1: "Service Not Available"

```python
# Symptom: Call fails immediately
# Cause: Service not running
# Fix: Check it's launched
ros2 service list
# Should see your service
```

### Problem 2: "Message Type Mismatch"

```python
# Symptom: RuntimeError about message types
# Cause: Publishing wrong type to topic
msg = String()  # WRONG
msg = RobotStatus()  # CORRECT

# Fix: Verify publisher and subscriber types match
```

### Problem 3: "Slow Response"

```python
# Symptom: Service takes 2+ seconds
# Cause: Callback does heavy computation
def callback(self, request, response):
    response.value = expensive_calculation()  # Blocks!
    return response

# Fix: Do expensive work asynchronously
self.executor.submit(expensive_calculation, response)
```

### Problem 4: "Dead Nodes"

```python
# Symptom: One node crashes, system hangs
# Cause: Other nodes waiting forever
# Fix: Always use timeouts
cli.wait_for_service(timeout_sec=2.0)

# Or use try/except
try:
    response = cli.call(request)
except Exception:
    self.get_logger().error('Service failed')
```

### Visualization Tools

```bash
# See all nodes and connections
rqt_graph

# See all topics
ros2 topic list

# See all services
ros2 service list

# Monitor single topic
ros2 topic echo /topic_name

# Monitor service call rate
ros2 service call /service_name ServiceType "{field: value}"
```

---

## Design Antipatterns (What NOT to Do)

### Antipattern 1: Service for Continuous Data

```python
# BAD: Service publishing temperature every 0.1 seconds
self.client.call(GetTemperature)  # Blocks until response
self.client.call(GetTemperature)  # Blocks again
```

**Fix:** Use topic instead. Services are for infrequent calls.

### Antipattern 2: Nested Services

```python
# BAD: Service handler calls another service
def callback(self, request, response):
    # This blocks waiting for another service!
    response = self.other_client.call(other_request)  # Dangerous!
    return response
```

**Fix:** Use topics for data flow, services only for simple queries.

### Antipattern 3: Synchronous Everything

```python
# BAD: Main loop calls 5 services sequentially
self.service1.call()  # Block 100ms
self.service2.call()  # Block 100ms
self.service3.call()  # Block 100ms
# Total: 300ms for each loop iteration
```

**Fix:** Use async calls and callbacks, or restructure with topics.

### Antipattern 4: Hardcoded Topic Names

```python
# BAD: Topic name buried in code
self.create_publisher(Type, '/robot/sensor/temp', 10)
# Hard to rename or reconfigure

# GOOD: Parameterized
self.declare_parameter('sensor_topic', '/robot/sensor/temp')
topic = self.get_parameter('sensor_topic').value
self.create_publisher(Type, topic, 10)
```

---

## Key Principles for Good Architectures

1. **Clear Ownership**: Each topic/service is owned by one publisher/service
2. **Loose Coupling**: Nodes don't depend on internal implementation of others
3. **Graceful Degradation**: System handles missing nodes (timeouts, fallbacks)
4. **Observable**: Use ROS 2 tools to visualize and debug
5. **Testable**: Mock out nodes, test communication in isolation

---

## Try With AI

You have a working multi-node system. Let's improve it architecturally.

**Ask your AI:**

> "I have sensor, control, and motor nodes communicating via topics and services. As the robot gets more complex, I want to add a planning node, a safety monitor, and a logging service. Where should each one fit? What topics/services should be added? Draw a communication diagram and explain the design rationale."

**Expected outcome:** AI will suggest:
- Safety monitor as separate high-priority listener
- Planning node subscribes to sensors, publishes motion plans
- Logger service that multiple nodes call
- Rationale for each choice

**Challenge the design:**

> "What if the planner gets slow? Should the control node wait for planning or proceed with old plan?"

**Expected outcome:** AI will explain:
- Async patterns so control doesn't block
- Fallback behaviors
- Timeout handling
- Tradeoffs between freshness and responsiveness

**Iterate together:**

> "Got it. Show me the code for a control loop that: (1) publishes status topic, (2) calls planning service async, (3) calls motor service only when safe, (4) has fallback if planner unavailable. Full implementation with error handling."

This demonstrates architecture design through collaboration.

---

## Exercises

1. **Extend the two-node system** with a third "safety monitor" node that subscribes to sensor data and publishes warnings
2. **Add a parameter service** that lets you change motor speed limits without restarting
3. **Create a launch file** that starts all three nodes together
4. **Visualize communication** with `rqt_graph`
5. **Implement error handling** so the control node continues if motor service unavailable

---

## Reflection

Before the capstone (Lesson 6), think about:

- When would you use a topic vs service in your design?
- How do you debug a system with 5+ nodes?
- What breaks most often in multi-node systems?

**Next chapter (Chapter 6): Building Systems**

You now know all the communication patterns. Next you'll learn how to launch multiple nodes, organize large projects, and build real robot systems that scale.
