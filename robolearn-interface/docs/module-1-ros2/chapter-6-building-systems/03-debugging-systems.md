---
id: lesson-6-3-debugging-systems
title: "Lesson 6.3: Debugging Multi-Node Systems"
sidebar_position: 3
sidebar_label: "6.3 Debugging Systems"
description: "Learn systematic debugging techniques for diagnosing and fixing issues in ROS 2 multi-node systems using ros2doctor, rqt_graph, and logger level modification."
duration_minutes: 60
proficiency_level: "B1"
layer: "L3"
hardware_tier: 1
learning_objectives:
  - "Use ros2doctor to diagnose system health issues"
  - "Visualize node architecture with rqt_graph"
  - "Enable and interpret debug-level logging"
  - "Trace data flow using ros2 topic and ros2 service commands"
  - "Systematically isolate and fix integration problems"
---

# Lesson 6.3: Debugging Multi-Node Systems

When you launch a system with multiple nodes, communicating through topics and services, things can go wrong in subtle ways:

- A node crashes silently
- Nodes can't find each other (network issues)
- Data isn't flowing (topic mismatch)
- System is slow (latency bottleneck)
- Parameters aren't being read correctly

Without the right debugging tools, you'd be stuck adding print statements and guessing. ROS 2 provides powerful diagnostic tools that let you see what's happening in the system.

## The Debugging Workflow

The key to efficient debugging is **systematic diagnosis**:

```
1. Check System Health     (ros2doctor)
   ↓
2. Visualize Architecture  (rqt_graph)
   ↓
3. Enable Detailed Logging (ros2 run ... --ros-args --log-level debug)
   ↓
4. Trace Data Flow         (ros2 topic echo, ros2 service list)
   ↓
5. Test Components         (Isolate and test each node)
   ↓
6. Fix and Validate        (Verify the fix works)
```

Let's walk through each tool.

## 1. System Health Check: ros2doctor

`ros2doctor` is your first line of defense. It performs automated checks on your ROS 2 installation and running system.

### Basic Usage

```bash
ros2 doctor
```

**Sample output:**
```
ROS 2 Distribution: Humble
Platform: Linux
Architecture: x86_64

system check passed:

ROS 2 version: Humble
Python: 3.10.12
distro: humble
ROS 2 path: /opt/ros/humble

All checks passed!
```

### Detailed Diagnostic Report

```bash
ros2 doctor --report
```

This generates a comprehensive report including:
- Environment variables (ROS_DISTRO, ROS_DOMAIN_ID)
- Network configuration (localhost resolution)
- Package paths (can packages be found?)
- Middleware configuration (DDS settings)

**Useful for detecting:**
- Missing environment setup (forgot to `source install/setup.bash`?)
- Network configuration issues
- Package installation problems

### When Nodes Can't Find Each Other

If nodes aren't communicating, `ros2doctor` can help identify the issue:

**Scenario:** You launch two nodes, but they don't see each other's topics.

```bash
# Check if both nodes are running
ros2 node list

# Check topics
ros2 topic list

# Run doctor
ros2 doctor
# Look for network issues or middleware warnings
```

**Common diagnosis:**
```
WARNING: DDS network issues detected
- localhost resolution failed
- Try: ping 127.0.0.1
- Set ROS_LOCALHOST_ONLY=1 if using local-only communication
```

## 2. Visualize Architecture: rqt_graph

`rqt_graph` shows the **node graph**—all nodes and their connections. This is invaluable for understanding what's connected and finding broken links.

### Launch rqt_graph

With your launch system running:

```bash
ros2 run rqt_graph rqt_graph
```

A window opens showing:
- **Nodes** as rectangular boxes
- **Topics** as circles
- **Connections** as arrows (publisher → subscriber)

**Example graph for robot system:**

```
┌──────────────────┐
│  camera_sensor   │ ──→ /image_raw ──→ ┌──────────────┐
└──────────────────┘                     │   processor  │
                                         └──────────────┘
┌──────────────────┐                            ↓
│   lidar_sensor   │ ──→ /scan ───────────────→ │
└──────────────────┘

┌──────────────────┐
│   odometry       │ ──→ /odom ────────────────→ │
└──────────────────┘
```

### Interpreting the Graph

**What should you see?**
- All expected nodes present
- Arrows showing data flow
- No disconnected components (unless intentional)

**Red flags:**
- **Missing node**: Expected node not in graph → Node crashed or failed to start
- **Missing topic**: No arrow from publisher → Topic name mismatch or publisher bug
- **Disconnected node**: Node with no connections → Not published by others or not subscribed to

### Debugging Example: Missing Topic

**Problem:** Your subscriber isn't receiving data from a publisher.

**Diagnosis with rqt_graph:**
1. Open rqt_graph
2. Look for the publisher node and the expected topic
3. Look for the subscriber node
4. Check if arrow connects them

**If no arrow:**
- Are they using the same topic name?
- Is the publisher actually publishing?
- Run: `ros2 topic list` to see all active topics
- Run: `ros2 topic echo /expected_topic` to see if data flows

## 3. Enable Detailed Logging

By default, ROS 2 nodes only log INFO level messages. For debugging, you need DEBUG logs that show execution details.

### Logger Levels

ROS 2 uses five logging levels (from least to most verbose):

| Level | When to Use | Example |
|-------|-----------|---------|
| ERROR | Critical failures | "Motor command failed: timeout" |
| WARN | Unusual but recoverable | "Parameter value out of range, using default" |
| INFO | Normal operation info | "Node started", "Message published" |
| DEBUG | Detailed execution flow | "Entering function X", "Parameter value is 42.0" |
| TRACE | Very detailed internal state | "Loop iteration 1000", "Buffer contents" |

### Setting Log Level at Runtime

While a node is running, change its logging level **without restarting**:

```bash
# Start a node with DEBUG logging
ros2 run my_first_package minimal_publisher --ros-args --log-level debug

# Or change logging for running node
ros2 param set /minimal_publisher rcl_logging_level DEBUG
```

### Adding Debug Output to Your Code

In your node code, use the logger to emit messages:

```python
import rclpy
from rclpy.node import Node

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        self.get_logger().error("Critical error - stopping")
        self.get_logger().warn("Warning - proceeding with caution")
        self.get_logger().info("Normal operation message")
        self.get_logger().debug("Detailed debug info - only in debug mode")
        self.get_logger().trace("Very detailed tracing info")
```

**Logging levels are especially useful in callbacks:**

```python
def timer_callback(self):
    rate = self.get_parameter('publish_rate').value

    self.get_logger().debug(f"Timer fired, rate={rate}")  # Only shown in debug mode

    msg = String()
    msg.data = f'Message {self.counter}'
    self.publisher_.publish(msg)

    self.get_logger().info(f"Published: {msg.data}")  # Always shown
    self.counter += 1
```

## 4. Trace Data Flow: ros2 Commands

Once you identify that a node exists and the graph looks right, trace the actual data flow.

### Check if Data is Being Published

```bash
# List all active topics
ros2 topic list

# See detailed topic info (message type, publishers, subscribers)
ros2 topic list -t

# Example output:
/image_raw [sensor_msgs/Image]
/scan [sensor_msgs/LaserScan]
/odom [nav_msgs/Odometry]
```

### Echo Topic Data in Real-Time

```bash
# See messages as they're published
ros2 topic echo /image_raw

# Example output (continuous):
---
header:
  stamp:
    sec: 1700000001
    nsec: 500000000
  frame_id: camera
encoding: rgb8
width: 640
height: 480
data: [255, 128, 64, ...]
---
```

**If no output:**
- Topic exists but no publisher is active
- Check if publisher is running: `ros2 node list`
- Check if publisher is really publishing to that topic: `ros2 topic info /image_raw`

### Test Services

```bash
# List available services
ros2 service list

# Call a service
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5, y: 5, theta: 0, name: 'turtle2'}"

# Example output:
result:
  x: 5.0
  y: 5.0
```

**If service call fails:**
- Is the service server running? Check: `ros2 service list`
- Is the service name correct?
- Try calling with `--verbose` flag: `ros2 service call /spawn ... --verbose`

## 5. Debugging Scenario: Broken System

Let's walk through a complete debugging example.

### The Problem

You launch a robot system with:
- 1 camera sensor (publishes to `/image`)
- 1 processor (subscribes from `/image`, publishes to `/processed`)
- 1 display (subscribes from `/processed`)

The system starts, but the display isn't showing images. No error messages. What's wrong?

### Step 1: Check if Nodes are Running

```bash
ros2 node list

# Output:
/camera_sensor
/processor
/display
```

✓ All nodes present and running.

### Step 2: Check the Graph

```bash
ros2 run rqt_graph rqt_graph
```

**Visual inspection shows:**
- Camera publishes to `/image`
- Processor subscribes from `/image` ✓
- Processor publishes to `/processed`
- Display subscribes from `/processed` ✓

Graph looks correct!

### Step 3: Check Data Flow

```bash
# Is camera really publishing?
ros2 topic echo /image
# Output: (no messages)
```

**Aha!** Camera node is running but not publishing. Check the camera node with logging.

### Step 4: Enable Debug Logging

```bash
# Kill the camera node and restart with debug logging
ros2 run my_pkg camera_sensor --ros-args --log-level debug

# Output:
[DEBUG] [camera_sensor]: Camera initialized
[DEBUG] [camera_sensor]: Timer callback starting
[ERROR] [camera_sensor]: Failed to capture frame: device not found
```

**Found it!** Camera hardware not found (maybe we're in cloud environment without actual camera). The node runs but fails silently.

### Step 5: Fix and Validate

Solution: Use a mock camera for Tier 1 environment.

```python
def get_image():
    # Mock image data when running in cloud
    import os
    if os.getenv('ROS_LOCALHOST_ONLY'):
        # Simulate image data
        return [0] * (640 * 480)
    else:
        # Real camera capture
        ...
```

Restart the system:
```bash
ros2 run my_pkg camera_sensor --ros-args --log-level debug

# Now shows:
[DEBUG] [camera_sensor]: Using mock camera (cloud environment)
[DEBUG] [camera_sensor]: Publishing frame...
[INFO] [camera_sensor]: Published image (640x480)
```

Verify data flow:
```bash
ros2 topic echo /processed
# Output: (now shows processed images flowing)
```

✓ Fixed!

## Debugging Workflow Diagram

```
                           System Not Working?
                                  ↓
                           Run ros2doctor
                                  ↓
                    ┌─────────────────────────┐
                    │  Health Check Passed?   │
                    └─────────────────────────┘
                         ↙ No              Yes ↘
                        ↓                        ↓
            Fix Environment Issues        Open rqt_graph
                        ↓                        ↓
                        │              ┌──────────────────┐
                        │              │  Graph Correct?  │
                        │              └──────────────────┘
                        │                ↙ No          Yes ↘
                        │               ↓                    ↓
                        │    Check Node Names          Enable DEBUG
                        │    & Topics                  Logging
                        │               ↓                    ↓
                        │               │         Echo Topics & Check
                        │               │         Data Flow
                        │               │                    ↓
                        │               │         ┌──────────────────┐
                        │               │         │  Data Flowing?   │
                        │               │         └──────────────────┘
                        │               │           ↙ No          Yes ↘
                        │               ↓ ← → ↓                        ↓
                        │        Debug Individual           System Working!
                        │        Node
                        │               ↓
                        └───────────────┘
```

## Guided Practice: Debugging Exercise

### Create a Broken System

Create a launch system with intentional bugs:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publisher on topic 'data'
        Node(
            package='my_first_package',
            executable='minimal_publisher',
            name='pub',
            parameters=[{'publish_rate': 2.0}],
        ),
        # Subscriber expects 'data' but will it get it?
        Node(
            package='my_first_package',
            executable='minimal_subscriber',
            name='sub',
        ),
    ])
```

**Hidden bug**: minimal_publisher publishes to `/topic`, minimal_subscriber subscribes to `/topic`, but the minimal_subscriber code was modified to expect `/sensor_data`.

### Debug It

1. Launch the system
2. Run `ros2doctor`
3. Open `rqt_graph` — note the disconnection
4. `ros2 topic echo /topic` — see data flowing
5. `ros2 topic echo /sensor_data` — see nothing
6. Enable DEBUG logging on subscriber
7. Look for error message about topic mismatch
8. Fix by updating subscriber to use correct topic name

## Independent Practice

### Exercise 1: Node Lifecycle Debugging

Create two nodes:
- Node A: Publishes to `/control` every 1 second
- Node B: Subscribes to `/control`, prints every message

Kill Node B in the middle of execution. What does `rqt_graph` show? What does `ros2 topic list` show?

### Exercise 2: Parameter Propagation

Create a node that declares a parameter but never uses it. Launch with a parameter value. Verify the parameter was set with `ros2 param get`. Then add code to read and use the parameter.

### Exercise 3: Service Debugging

Create a service server and client. Make the client call the service with wrong argument types. Use debugging tools to identify the mismatch.

## Common Issues and Solutions

| Issue | Diagnosis | Solution |
|-------|-----------|----------|
| **Node not starting** | `ros2 node list` shows fewer nodes than expected | Check logs, verify package built correctly, check executable name |
| **No data flowing** | `rqt_graph` shows no arrows between nodes | Verify topic names match (case-sensitive!), check message types |
| **Slow/laggy system** | `ros2 doctor --report` shows middleware warnings | Check network, set `ROS_LOCALHOST_ONLY=1` if on same machine |
| **Service call timeout** | Service in `ros2 service list` but call fails | Enable DEBUG logging on server, verify request format |
| **Parameters not applying** | Parameter set with `ros2 param set` but node ignores | Check if node reads parameter in right place (declare vs get) |

## Try With AI

**Setup:** Use your AI assistant to work through a debugging challenge.

**Prompt Set:**

```
Prompt 1: "I have a multi-node system where Node A publishes sensor data but Node B (the subscriber) isn't receiving it. The nodes are both running. What systematic steps would you take to debug this?"

Prompt 2: "My launch file starts 5 nodes, but only 3 of them appear in rqt_graph. The other 2 must be crashing silently. How do I find which ones and why they're failing?"

Prompt 3: "My system works fine with debug logging on, but when I turn logging off it breaks. What could cause this? How would I fix it?"
```

**Expected Outcomes:**
- AI walks through systematic debugging (ros2doctor → rqt_graph → logging → echo)
- AI suggests checking node startup logs and stderr output
- AI identifies potential race conditions or initialization order issues

**Safety Note:** When debugging robot systems, be prepared for nodes to start in unexpected order. Always verify system health with `ros2doctor` and `rqt_graph` before operating physical hardware. Never assume data is flowing correctly without verification.

**Optional Stretch:**
Create a complex debugging scenario:
1. Multiple sensor nodes with different message types
2. Add an intermediate processor that remaps topic names
3. Intentionally create a bug (topic name mismatch, parameter type error)
4. Use all debugging tools to locate and fix the bug
5. Document your debugging workflow as a reference guide
