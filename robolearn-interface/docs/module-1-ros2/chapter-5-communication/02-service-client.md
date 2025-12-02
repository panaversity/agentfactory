---
id: service-client
title: "Lesson 5.2: Service Clients and Async Patterns"
sidebar_label: "5.2 Service Clients"
sidebar_position: 2
chapter: 5
lesson: 2
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
cognitive_load:
  new_concepts: 4
learning_objectives:
  - "Create a service client node using rclpy"
  - "Understand async vs sync service calls"
  - "Implement wait_for_service() pattern for robustness"
  - "Handle service responses and failures gracefully"
  - "Improve client code through AI-guided iteration"
skills:
  - "ros2-service-pattern"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 5.2: Service Clients and Async Patterns

In the previous lesson, you built a service server—a node that waits for requests. Now you'll build the other side: a service client that sends requests and waits for responses.

A service client is like making a phone call. You dial (make a request), wait on hold (block until response), then the other person responds. You need to handle the case where no one is home (service not available), and you need to know what to do with the answer.

In this lesson, you'll learn both **synchronous** (simpler, blocking) and **asynchronous** (more complex, non-blocking) patterns. Most real robots use asynchronous because they can't block waiting for a response while other critical tasks run.

---

## Synchronous vs Asynchronous Service Calls

### Synchronous (Simple)

```python
# Call service, block until response
response = node.call_service(request)
print(response.result)
# Continue here - response already arrived
```

**Pros:**
- Simple code: "call, then use response"
- Easy to understand flow

**Cons:**
- Blocks the entire node while waiting
- If service hangs, the whole node freezes

### Asynchronous (Realistic)

```python
# Send request, don't block
future = node.call_service_async(request)
# Continue doing other work here
# ...
# Later, check if response arrived
if future.done():
    response = future.result()
    print(response.result)
```

**Pros:**
- Node keeps running while waiting
- Can handle multiple requests
- More like real robots

**Cons:**
- More complex code
- Need to check when response arrives

**Real robotics reason:** A robot can't freeze while waiting for one service. It needs to keep publishing sensor data, listening to commands, updating the display—all at the same time.

---

## The Synchronous Pattern (Simple, Blocking)

Here's the simplest way to call a service:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        # Create a client (not a service!)
        self.cli = self.create_client(SetBool, 'enable_robot')

    def call_enable_robot(self, enable: bool):
        """Send enable/disable request to robot server."""
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create the request
        request = SetBool.Request()
        request.data = enable

        # Call service and BLOCK until response arrives
        response = self.cli.call(request)

        # Now we have the response
        self.get_logger().info(f'Response: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    client = SimpleClient()

    # Enable the robot
    client.call_enable_robot(True)

    # Later...
    client.call_enable_robot(False)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's understand each part:

**Creating the Client:**

```python
self.cli = self.create_client(SetBool, 'enable_robot')
```

This creates a client (different from a server!):
- **Argument 1** (`SetBool`): Service type
- **Argument 2** (`'enable_robot'`): Service name to call

**Waiting for Service:**

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting...')
```

**Critical:** Always wait before calling. The service might not be running yet:
- If service is available, returns immediately
- If not, waits up to 1 second
- If still not available, loops and waits again
- If you skip this and service isn't running, the call fails

**Creating the Request:**

```python
request = SetBool.Request()
request.data = enable
```

Create an empty request object and fill in the fields.

**Calling Synchronously:**

```python
response = self.cli.call(request)
```

This **blocks** the entire node:
- Sends request to server
- Waits for response
- Code below this line doesn't run until response arrives
- Then you have the response

---

## The Asynchronous Pattern (Realistic, Non-blocking)

In real robotics, you can't block. Here's the async pattern:

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
        """Send request WITHOUT blocking."""
        # Wait for service first
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return

        # Create request
        request = SetBool.Request()
        request.data = enable

        # Send request but DON'T BLOCK
        self.future = self.cli.call_async(request)
        self.get_logger().info(f'Request sent, not blocked')

    def check_response(self):
        """Check if response has arrived yet."""
        if self.future is None:
            return None

        # Is response ready?
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

    # Send request (doesn't block)
    client.call_enable_async(True)

    # Node can do other work here
    # In a real robot, this might be publishing sensor data

    # Later, check if response arrived
    client.check_response()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key differences from sync:**

```python
# Async: returns immediately
self.future = self.cli.call_async(request)
# Code continues here - request still being processed!

# Later:
if self.future.done():
    response = self.future.result()
```

The `future` object represents "I sent the request, response coming soon." You check it later with `.done()`.

---

## The Production Pattern (Async with Spin)

Real robots use ROS 2's `rclpy.spin()` with a callback. Here's the pattern most used:

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class ProductionClient(Node):
    def __init__(self):
        super().__init__('production_client')
        self.cli = self.create_client(SetBool, 'enable_robot')

        # Wait once at startup
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for enable_robot service...')

        # Create a timer to send requests periodically
        self.timer = self.create_timer(2.0, self.send_request)
        self.request_count = 0

    def send_request(self):
        """Called every 2 seconds."""
        self.request_count += 1
        enable = self.request_count % 2 == 0  # Alternate True/False

        request = SetBool.Request()
        request.data = enable

        # Send async and attach callback for when response arrives
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

        self.get_logger().info(f'Request #{self.request_count} sent')

    def response_callback(self, future):
        """Called when response arrives."""
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = ProductionClient()
    # Keep running, spin handles timers and callbacks
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This is the **production pattern**:
1. Create client and wait for service at startup
2. Use a timer to send requests periodically
3. Attach a callback with `future.add_done_callback()`
4. When response arrives, callback executes automatically
5. `rclpy.spin()` keeps everything running

---

## Testing Your Client

Let's test with the server from the previous lesson running.

**Terminal 1: Start the server**

```bash
ros2 run my_first_package enable_service
```

**Terminal 2: Run the production client**

Create a file `service_client.py`:

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

        # Send request immediately
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

Add to `setup.py`:

```python
'service_client = my_first_package.service_client:main',
```

Build and run:

```bash
cd ~/ros2_ws && colcon build
source install/setup.bash
ros2 run my_first_package service_client
```

Terminal 1 (server) will show:
```
[INFO] [robot_enabler]: Robot has been ENABLED
```

Terminal 2 (client) will show:
```
[INFO] [client_node]: Result: Robot enabled. Battery: 85%
```

---

## Error Handling Patterns

What happens if:
1. Service is not running?
2. Service crashes mid-response?
3. Network is slow?

```python
def send_request_safely(self, request):
    """Safe request with proper error handling."""
    try:
        # Check service exists
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return None

        # Send request
        future = self.cli.call_async(request)

        # Wait for response (max 5 seconds)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

        # Check if we got a response
        if future.result() is None:
            self.get_logger().error('Service call failed')
            return None

        return future.result()

    except Exception as e:
        self.get_logger().error(f'Error: {e}')
        return None
```

The key functions:
- `wait_for_service()`: Check service exists
- `spin_until_future_complete()`: Wait up to N seconds for response
- `future.result()`: Get the actual response

---

## Key Insights

**Wait_for_service is Essential:**
Never assume a service is running. Always wait.

**Sync vs Async Decision:**
- **Sync**: Simple one-off calls (startup, initialization)
- **Async**: Real nodes that keep running

**Callbacks are the Pattern:**
In real ROS 2, you don't "wait". You attach callbacks and let `rclpy.spin()` manage the timing.

**Timeouts Matter:**
Always specify timeouts. A frozen service shouldn't freeze your node.

---

## Try With AI

You have a working async client. Let's improve it.

**Ask your AI:**

> "I have a service client that sends requests every 2 seconds and handles responses with a callback. I want to add: (1) retry logic if service isn't available, (2) a counter that tracks successful vs failed requests, (3) logging of request/response times."

**Expected outcome:** AI will suggest:
- Loop with exponential backoff for service availability
- Add success/failure counters
- Use time.time() to measure round-trip time
- Log metrics every 10 requests

**Challenge the implementation:**

> "Good, but what if the service becomes unavailable while the client is running? Should I keep retrying forever or give up?"

**Expected outcome:** AI will explain:
- Option 1: Retry N times then fail
- Option 2: Keep retrying but log warnings
- Option 3: Use ros2 param to make it configurable
- Discuss tradeoffs in robustness vs resource usage

**Iterate together:**

> "I like option 3. Show me how to add a 'max_retries' parameter that the user can set. If retries exceeded, log an error but don't crash."

This demonstrates AI teaching patterns, you refining requirements, and together reaching the best solution.

---

## Exercises

1. **Create a simple client** that enables the robot, waits 2 seconds, then disables it
2. **Add retry logic** that attempts the service call up to 3 times before failing
3. **Time the response** and log how long the request took
4. **Create two clients** in the same package calling different services (you'll need a second server)
5. **Add a parameter** for request frequency so it can be changed at runtime

---

## Reflection

Before moving to the next lesson, consider:

- Why can't a real robot use synchronous service calls?
- What's the difference between `call()` and `call_async()`?
- When would you use `wait_for_service()` vs just hoping the service exists?

Next lesson: Custom message and service types. So far you've used built-in types like `AddTwoInts` and `SetBool`. What if you need to send custom data structures? That's when you create `.srv` files.
