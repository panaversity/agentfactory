---
id: lesson-3-3-nodes-topics
title: "Lesson 3.3: Nodes and Topics (CLI Exploration)"
sidebar_position: 3
sidebar_label: "3.3 Nodes & Topics"
description: "Explore ROS 2 nodes and topics using CLI tools to understand publisher/subscriber communication."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "List running nodes using ros2 node list"
  - "List active topics using ros2 topic list -t"
  - "Echo topic data in real-time using ros2 topic echo"
  - "Understand publisher/subscriber pattern through observation"
skills:
  - "ros2-fundamentals"
  - "ros2-cli"
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Nodes and Topics (CLI Exploration)

In Lesson 3.2, you saw turtlesim and teleop working together. Now you'll **explore the system systematically** using command-line tools.

By the end of this lesson, you'll have a precise understanding of:
- What **nodes** are running
- What **topics** they use to communicate
- What **data** flows through those topics

**Duration**: 60 minutes | **Hardware Tier**: Tier 1 | **CLI exploration only—no coding yet**

---

## Prerequisites

Before starting, have turtlesim running from Lesson 3.2:

**Terminal 1**:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2**:
```bash
ros2 run turtlesim turtle_teleop_key
```

Keep both running while you explore in Terminal 3.

---

## Core Concepts

Before you explore, understand what you're looking for:

### What is a Node?

A **node** is an independent process running ROS logic. Think of it as a program.

Examples:
- `turtlesim_node`: A program that simulates a turtle
- `turtle_teleop_key`: A program that reads keyboard input
- `rqt_graph`: A program that visualizes the system

Each node:
- Has a name (e.g., `/turtlesim`, `/teleop_turtle`)
- Publishes to topics (sends data)
- Subscribes to topics (receives data)
- Can offer services (request/response, covered in Lesson 3.4)

---

### What is a Topic?

A **topic** is a named channel for messages. Think of it as a "subject" for pub/sub communication.

Examples:
- `/turtle1/cmd_vel`: Commands telling turtle to move
- `/turtle1/odometry`: Position/velocity feedback from turtle
- `/turtle1/color_sensor`: (hypothetical) Color data from a sensor

Each topic:
- Has a name starting with `/` (e.g., `/turtle1/cmd_vel`)
- Has a message type (e.g., `geometry_msgs/Twist` for velocity)
- Can have multiple publishers (usually just 1)
- Can have multiple subscribers (usually several)

---

### Publisher vs Subscriber

When a node publishes to a topic, it sends messages.
When a node subscribes to a topic, it receives messages.

**Example**:
- `turtle_teleop_key` **publishes** keyboard input to `/turtle1/cmd_vel`
- `turtlesim_node` **subscribes** to `/turtle1/cmd_vel` and acts on the messages

The two nodes never directly interact—ROS 2 middleware handles message delivery.

---

## Step 1: List All Running Nodes

Open **Terminal 3** and run:

```bash
ros2 node list
```

**Expected output**:
```
/teleop_turtle
/turtlesim
```

**What this shows**:
- Two nodes are running
- `/teleop_turtle`: The keyboard teleop process
- `/turtlesim`: The simulator process

**Names start with `/`** — this is ROS 2 convention for node names.

---

## Step 2: Inspect a Node

Get detailed information about the `/turtlesim` node:

```bash
ros2 node info /turtlesim
```

**Expected output**:
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /turtle1/color_sensor: std_msgs/msg/UInt32
    /turtle1/odometry: nav_msgs/msg/Odometry
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Reset
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:
    (none)
```

**What this tells you**:

- **Subscribers**: `/turtlesim` listens to `/turtle1/cmd_vel` (velocity commands)
- **Publishers**: `/turtlesim` sends odometry (position) and color sensor data
- **Service Servers**: `/turtlesim` offers services like `/spawn`, `/kill`, `/reset` (covered in Lesson 3.4)

---

## Step 3: List All Topics

See all topics in the system:

```bash
ros2 topic list
```

**Expected output**:
```
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/odometry
```

**What you see**:
- `/rosout` and `/rosout_agg`: System logging (ignore for now)
- `/turtle1/cmd_vel`: Velocity commands (keyboard teleop → turtlesim)
- `/turtle1/color_sensor`: Sensor data (hypothetical color reading)
- `/turtle1/odometry`: Position/velocity feedback from turtlesim

---

## Step 4: Get Topic Details

List topics WITH their message types:

```bash
ros2 topic list -t
```

**Expected output**:
```
/rosout [rcl_interfaces/msg/Log]
/rosout_agg [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [std_msgs/msg/UInt32]
/turtle1/odometry [nav_msgs/msg/Odometry]
```

**What `-t` shows**:
- Message type for each topic
- `geometry_msgs/msg/Twist`: Velocity commands (linear + angular velocity)
- `nav_msgs/msg/Odometry`: Position data (x, y, theta, velocities)
- `std_msgs/msg/UInt32`: Single integer (color value)

---

## Step 5: Echo a Topic (See Real-Time Data)

See the actual data flowing through `/turtle1/cmd_vel`:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Now go to Terminal 2 (where teleop is running) and **press arrow keys**.

**In Terminal 3, you'll see real-time updates**:

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

**What you see**:
- Each `---` separator is a new message
- `linear.x = 2.0`: Turtle moving forward at 2 m/s
- `angular.z`: Rotation (0 means no rotation)
- When you press ← or →, `angular.z` changes

**What's happening**:
- You press a key in Terminal 2
- Teleop publishes a Twist message to `/turtle1/cmd_vel`
- You see it here in Terminal 3 (you're subscribing)
- Meanwhile, turtlesim also subscribes and updates the turtle position

**Press Ctrl+C to stop echoing.**

---

## Step 6: Echo Odometry (Position Feedback)

See where the turtle is:

```bash
ros2 topic echo /turtle1/odometry
```

Go back to Terminal 2 and move the turtle around.

**In Terminal 3, you'll see position updates**:

```
header:
  stamp:
    sec: 123
    nanosec: 456789012
  frame_id: world
child_frame_id: turtle1
pose:
  pose:
    position:
      x: 5.544444
      y: 5.544444
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071
      w: 0.7071
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
---
```

**Key data**:
- `position.x, position.y`: Turtle's current location
- `orientation` (quaternion): Turtle's rotation
- `twist.linear, twist.angular`: Current velocity (usually 0 when turtle stops)

**What's happening**:
- Turtlesim maintains turtle state (position, orientation)
- Every update cycle, it publishes this state as an odometry message
- You're receiving and seeing this data

**Press Ctrl+C to stop.**

---

## Putting It Together: The Message Flow

Now visualize the complete flow:

```
Terminal 2 (teleop_turtle)           ROS 2 Middleware              Terminal 1 (turtlesim_node)
┌──────────────────────┐             ┌──────────────────┐          ┌───────────────────┐
│  Reads keyboard      │             │  /turtle1/cmd_vel│          │  Reads cmd_vel    │
│  input               │───Publishes→│   [Twist msg]    │─Subscribe│  Updates position │
│                      │             │                  │          │  Publishes odometry
└──────────────────────┘             │ /turtle1/odometry│          │  on /turtle1/      │
                                     │   [Odometry msg] │←Subscribe│  odometry          │
                                     │                  │          │                   │
Your Keyboard                        └──────────────────┘          Canvas updates with │
(↑ ↓ ← →)                                                          new turtle position
```

**Key insight**: The two nodes are completely decoupled. Teleop doesn't know about turtlesim's implementation. Turtlesim doesn't know about keyboard input. They only know about topics. ROS 2 middleware handles all message routing.

---

## Hands-On: Information Gathering

Now that you know the tools, gather information about your system:

### Task 1: How many publishers does `/turtle1/cmd_vel` have?

```bash
ros2 topic info /turtle1/cmd_vel
```

**Expected output**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

**Answer**: 1 publisher (teleop_turtle)

---

### Task 2: How many subscribers does `/turtle1/odometry` have?

```bash
ros2 topic info /turtle1/odometry
```

**Expected output**:
```
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
```

**Answer**: 0 subscribers right now (you echo'd it, but echo is temporary and doesn't count as a permanent subscription)

---

### Task 3: What is the structure of a Twist message?

```bash
ros2 interface show geometry_msgs/msg/Twist
```

**Output**:
```
# This expresses velocity in free space with only linear and angular parts.
Vector3  linear
  float64 x
  float64 y
  float64 z
Vector3  angular
  float64 x
  float64 y
  float64 z
```

**What it shows**: A Twist has 6 components—3 linear (x,y,z) and 3 angular (x,y,z). Turtlesim only uses `linear.x` and `angular.z`.

---

### Task 4: What topics does `teleop_turtle` publish to?

```bash
ros2 node info /teleop_turtle
```

**Expected output**:
```
/teleop_turtle
  Subscribers:
    (none)
  Publishers:
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    (none)
  Service Clients:
    (none)
```

**Answer**: Publishes to `/turtle1/cmd_vel` and `/rosout`

---

## Why This Matters

You've now **systematically explored** a ROS 2 system without writing any code:

1. **Nodes are discoverable**: `ros2 node list`
2. **Topics are queryable**: `ros2 topic list -t`, `ros2 topic info`
3. **Data is observable**: `ros2 topic echo` shows real-time messages
4. **Relationships are clear**: Node info shows what publishes/subscribes what

This is powerful because:
- **Debugging**: You can inspect running systems without touching code
- **Understanding**: Newcomers can learn how systems work by exploring
- **Modularity**: You can replace nodes (e.g., swap teleop for a mouse controller) without changing turtlesim

---

## Try With AI

**Setup**: Keep turtlesim and teleop running. Open your AI tool.

**Prompt 1** (Understanding Decoupling):

Ask your AI:

```
In the turtlesim system, /teleop_turtle publishes to /turtle1/cmd_vel,
and /turtlesim subscribes to the same topic.

Why is this better than teleop calling a function directly on turtlesim?
What problems would arise if teleop directly called turtlesim functions?
```

**Expected insight**: Loose coupling, extensibility, and testability are key benefits. This design philosophy prepares you for Chapter 4 when you write your own nodes.

---

**Prompt 2** (Topic Design):

Ask your AI:

```
Turtlesim publishes odometry on /turtle1/odometry.
Right now, nothing subscribes to it.

If I wanted to log all turtle positions to a file, how would I do that?
Would I need to modify teleop or turtlesim? How would a logging node get the data?
```

**What you learn**: Writing a new node that subscribes to odometry is independent of existing nodes. The AI explains that you'd write a subscriber without touching teleop/turtlesim code.

---

**Prompt 3** (Message Types):

Ask your AI:

```
I echoed /turtle1/cmd_vel and saw a Twist message with linear.x and angular.z.
What do the 6 fields of a Twist message represent in a real robot?
When would you use linear.y or linear.z in 3D robots?
```

**What you learn**: Twist is designed for 6 DOF (6-degree-of-freedom) motion. Turtlesim only uses 2 (linear forward, angular rotation). Real robots use more fields.

---

## Checkpoint Before Next Lesson

Before Lesson 3.4 (Services), verify you can:

- [ ] Run `ros2 node list` and see `/teleop_turtle` and `/turtlesim`
- [ ] Run `ros2 node info /turtlesim` and understand the output
- [ ] Run `ros2 topic list -t` and see all active topics
- [ ] Run `ros2 topic echo /turtle1/cmd_vel` and see data flow when moving turtle
- [ ] Understand that nodes communicate through topics, not function calls

If all checkboxes pass, you're ready for Lesson 3.4, where you'll explore **services**—a different communication pattern for request/response interactions.

---

**Next lesson**: [→ Lesson 3.4: Services and Parameters (CLI Exploration)](./04-services-parameters.md)
