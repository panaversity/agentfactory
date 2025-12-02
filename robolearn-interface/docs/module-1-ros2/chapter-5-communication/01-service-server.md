---
id: service-server
title: "Lesson 5.1: Building Service Servers"
sidebar_label: "5.1 Service Servers"
sidebar_position: 1
chapter: 5
lesson: 1
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 3
learning_objectives:
  - "Understand request/response communication patterns in ROS 2"
  - "Create a service server node using rclpy"
  - "Handle synchronous service requests with callbacks"
  - "Compare services to topics based on use cases"
  - "Extend service implementations using AI feedback"
skills:
  - "ros2-service-pattern"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 5.1: Building Service Servers

You've learned how publishers and subscribers send continuous streams of data. But what if you need a synchronous request and response? What if a robot needs to execute a command and report back whether it succeeded?

Welcome to services.

Think of it like making a phone call. When you publish to a topic, it's like broadcasting over a megaphone—anyone listening hears you, but no one replies. When you call a service, it's like calling a friend on the phone—you ask a question, wait for an answer, and then continue.

In this lesson, you'll build a service server—a node that receives requests, processes them, and sends back responses. By the end, you'll understand when to use services instead of topics, and how to handle synchronous requests from other nodes.

---

## Topics vs Services: When to Use Each

Before diving into code, let's clarify the difference. You already know topics. Services are different because they're **synchronous** and **request/response** based.

| Aspect | Topics | Services |
|--------|--------|----------|
| **Pattern** | Publish/Subscribe | Request/Response |
| **Flow** | Fire and forget | Wait for answer |
| **Frequency** | Continuous (high rate) | Infrequent (on demand) |
| **Use Case** | Sensor data, continuous streams | Commands, queries, configuration |
| **Example** | Temperature sensor publishing every 0.1s | "Move robot forward 1 meter" command |

**Decision Rule:**
- **Use Topics** if the receiver doesn't need to respond
- **Use Services** if the sender needs confirmation that something happened

Real robotics examples:
- **Topic**: Camera publishes images continuously
- **Service**: Robot executes "pick up object" and confirms success/failure

---

## The Service Server Pattern

All ROS 2 service servers follow the same structure. Here's a complete example:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        # Create the service that will handle requests
        self.srv = self.create_service(
            AddTwoInts,           # Service type (request/response format)
            'add_two_ints',       # Service name (how clients call it)
            self.add_callback)    # Function to handle requests

    def add_callback(self, request, response):
        """Called when a client sends a request."""
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

Let's break down what's happening:

### Understanding the Code

**Imports:**

```python
from example_interfaces.srv import AddTwoInts
```

This imports the `AddTwoInts` service type. It's a built-in ROS 2 service with:
- **Request**: Two integers `a` and `b`
- **Response**: One integer `sum`

**Creating the Service:**

```python
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_callback)
```

This line creates a service server:
- **First argument** (`AddTwoInts`): The service type (defines request/response format)
- **Second argument** (`'add_two_ints'`): The service name (how clients reference this service)
- **Third argument** (`self.add_callback`): The callback function (runs when a request arrives)

**The Callback Function:**

```python
def add_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(...)
    return response
```

When a client sends a request:
1. `request` contains the data the client sent (here: `a` and `b`)
2. You process it and fill in `response` (here: the sum)
3. Return the `response` object
4. ROS 2 automatically sends it back to the client

**The Difference from Topics:**

With topics, you publish and forget. With services, you **wait** for the response object. The client blocks until your callback returns.

---

## Creating Your First Service Server

Let's create a real example. We'll build a service that the robot can use to "enable" or "disable" itself—a simple control command.

**Step 1: Navigate to your package**

```bash
cd ~/ros2_ws/src/my_first_package/my_first_package
```

**Step 2: Create the service server file**

```bash
cat > enable_service.py << 'EOF'
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class RobotEnabler(Node):
    def __init__(self):
        super().__init__('robot_enabler')
        self.robot_enabled = False

        # Create service: clients call 'enable_robot' with True/False
        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_callback)

        self.get_logger().info('Robot enabler service ready')

    def enable_callback(self, request, response):
        """Handle enable/disable requests."""
        self.robot_enabled = request.data  # Request contains True or False
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

**Step 3: Add it to setup.py**

Edit `setup.py` (in the outer `my_first_package` folder) and add the executable:

```python
entry_points={
    'console_scripts': [
        'minimal_publisher = my_first_package.minimal_publisher:main',
        'minimal_subscriber = my_first_package.minimal_subscriber:main',
        'enable_service = my_first_package.enable_service:main',  # Add this line
    ],
},
```

**Step 4: Build and source**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the Service

**Terminal 1: Start the service server**

```bash
ros2 run my_first_package enable_service
```

You should see:

```
[INFO] [rclpy]: Robot enabler service ready
```

The service is now waiting for requests. It doesn't exit—it keeps running.

**Terminal 2: Call the service**

Now let's send it a request. We'll tell it to enable:

```bash
ros2 service call /enable_robot std_srvs/SetBool "data: true"
```

You should see:

```
response:
  success: true
  message: 'Robot enabled'
```

And in Terminal 1:

```
[INFO] [robot_enabler]: Robot has been ENABLED
```

**Terminal 2: Call it again with False**

```bash
ros2 service call /enable_robot std_srvs/SetBool "data: false"
```

Response:

```
response:
  success: true
  message: 'Robot disabled'
```

And in Terminal 1:

```
[INFO] [robot_enabler]: Robot has been DISABLED
```

---

## What Happened Here

The key difference from topics:

1. **Service created** and waiting for requests
2. **Client called** the service with data (`data: true`)
3. **Server received** the request in the callback
4. **Server processed** it and filled in the response
5. **Client blocked** until response arrived
6. **Client printed** the response
7. **Server continued** running, ready for next request

Unlike publishing to a topic, the client **waits** for an answer. The service is **synchronous**.

---

## Understanding Service Types

What's `SetBool`? It's a built-in service type with:
- **Request field**: `data` (boolean)
- **Response fields**: `success` (boolean) and `message` (string)

Other common built-in services:
- `AddTwoInts`: adds two integers
- `SetBool`: boolean control (like enable/disable)
- `Trigger`: no request data, just "do it" (response: success + message)

---

## Key Insights

**Synchronous vs Asynchronous:**
Services are synchronous from the client's perspective—the client waits. Inside the server callback, be fast. Heavy computation blocks other requests.

**Named Services:**
The service name appears in `ros2 service list`. Naming conventions:
- Use lowercase with underscores
- Avoid generic names like `/service` or `/command`
- Use `/robot_enable`, `/motor_status`, `/calculate_path`

**Response Structure:**
Always fill in the entire response object before returning. The client expects all fields populated.

---

## Extending the Service

Now let's make it more realistic. What if we want to report the robot's battery level when enabled?

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class RobotEnablerV2(Node):
    def __init__(self):
        super().__init__('robot_enabler_v2')
        self.robot_enabled = False
        self.battery_level = 85.0  # Simulate battery

        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_callback)

        self.get_logger().info('Robot enabler v2 ready')

    def enable_callback(self, request, response):
        """Enable/disable with battery check."""
        if request.data and self.battery_level < 10.0:
            # Don't enable if battery too low
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

Now the service checks battery before enabling. It responds with a message about the battery level. The client can read that message and decide what to do.

---

## Try With AI

You have a working service server. Now let's improve it with AI collaboration.

**Ask your AI:**

> "I have a ROS 2 service server that enables/disables a robot and checks battery level. It's working but I want to make it production-ready. What error handling and logging improvements would you suggest? Give me specific code for graceful shutdown and parameter configuration."

**Expected outcome:** AI will suggest:
- Add try/except blocks for safety
- Log when service gets called vs responds
- Add parameters for battery threshold
- Add graceful shutdown with cleanup

**Challenge the suggestion:**

> "Good ideas, but the battery check seems too strict. What if I want it configurable—allow the user to set a 'minimum_battery' parameter at runtime?"

**Expected outcome:** AI will explain:
- Use `declare_parameter()` in `__init__`
- Read it in the callback with `get_parameter()`
- Show how to change it with `ros2 param set`

**Iterate together:**

> "Perfect. Show me the full code with configurable battery threshold AND a second service called 'get_status' that just returns the current battery level without enabling/disabling."

**Expected outcome:** AI will provide:
- Full implementation with both services
- Parameter declaration and reading
- Two separate callbacks
- Example ros2 commands to test both

This is the Three Roles pattern: AI suggests architecture (Teacher), you refine constraints (Student), together you build the solution (Co-Worker).

---

## Exercises

1. **Modify the basic service** to accept a string name and respond with a greeting
2. **Create a second service** in the same node called `get_robot_status` that returns battery level without changing any state
3. **Add logging** before and after the callback executes to track service calls
4. **Test with ros2 service list** to verify both services are available

---

## Reflection

Before moving to the next lesson, think about:

- When would a real robot prefer a service over a topic?
- What happens if a service callback takes 10 seconds to execute?
- How is the client "blocked" differently than in traditional function calls?

The next lesson teaches you how to **call** services from another node—how to be the client, not the server.
