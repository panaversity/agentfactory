---
id: lesson-3-4-services-parameters
title: "Lesson 3.4: Services and Parameters (CLI Exploration)"
sidebar_position: 4
sidebar_label: "3.4 Services & Parameters"
description: "Explore ROS 2 services (request/response) and parameters (runtime configuration) using CLI tools."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "List available services using ros2 service list"
  - "Call services with arguments using ros2 service call"
  - "List and modify parameters using ros2 param list/set"
  - "Understand services vs topics conceptually"
skills:
  - "ros2-fundamentals"
  - "ros2-cli"
  - "ros2-service-pattern"
tier_1_path: "Cloud ROS 2 (TheConstruct)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Services and Parameters (CLI Exploration)

In Lessons 3.2 and 3.3, you explored **topics**—continuous data streams using pub/sub.

Now you'll explore **services**—request/response interactions (like calling a function) and **parameters**—runtime configuration variables.

By the end of this lesson, you'll understand the complete toolkit ROS 2 provides for node communication.

**Duration**: 60 minutes | **Hardware Tier**: Tier 1 | **CLI exploration only—no coding yet**

---

## Prerequisites

Before starting, have turtlesim running from Lesson 3.2:

**Terminal 1**:
```bash
ros2 run turtlesim turtlesim_node
```

Keep it running while you explore in Terminal 2 (you don't need teleop for this lesson).

---

## Core Concepts

### What is a Service?

A **service** is a synchronous request/response interaction. Unlike topics (which stream continuously), services are **RPC-like** (Remote Procedure Call).

**Analogy**:
- **Topic**: News broadcast (continuous stream, anyone listening gets it)
- **Service**: Phone call (I ask you a question, you answer, call ends)

**Characteristics**:
- Client sends a request
- Server processes it
- Server sends back a response
- Communication blocks (client waits for response)

**Examples in turtlesim**:
- `/spawn`: "Create a new turtle at position X, Y" (no response data needed)
- `/kill`: "Delete a turtle" (responds with success/failure)
- `/reset`: "Clear all drawings, reset to start" (responds with result)
- `/teleport_absolute`: "Move turtle to exact position X, Y" (confirms success)

---

### What is a Parameter?

A **parameter** is a runtime configuration variable scoped to a node.

**Examples in turtlesim**:
- `background_r`, `background_g`, `background_b`: Canvas background color (RGB values 0-255)
- `pen_r`, `pen_g`, `pen_b`: Drawing pen color

**Characteristics**:
- Owned by a node (e.g., `/turtlesim` owns its color parameters)
- Can be read anytime
- Can be modified at runtime (node responds immediately to new values)
- Scoped to the node (parameters under `/turtlesim` are separate from other nodes)

---

### Services vs Topics: When to Use Each?

**Use Topics when**:
- Data is continuous (sensor streams, odometry)
- Multiple nodes need the same data (broadcast)
- Real-time feedback preferred over guaranteed delivery
- Example: `/turtle1/odometry` (position updates stream continuously)

**Use Services when**:
- You need a response (request/response interaction)
- Action is discrete (spawn, kill, reset)
- Blocking call appropriate (wait for server response)
- Example: `/spawn` (client waits for server to create turtle)

**Rule of thumb**: Topics for continuous data, services for discrete actions.

---

## Step 1: List Available Services

```bash
ros2 service list
```

**Expected output**:
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

**What these do**:
- `/clear`: Erase all drawings (blank canvas)
- `/kill`: Delete a turtle (name required)
- `/reset`: Reset to initial state
- `/spawn`: Create a new turtle (position/rotation required)
- `/turtle1/set_pen`: Change drawing pen properties
- `/turtle1/teleport_absolute`: Move turtle to exact location
- `/turtle1/teleport_relative`: Move turtle relative to current position

---

## Step 2: Get Service Details

See what arguments a service expects:

```bash
ros2 service type /spawn
```

**Output**:
```
turtlesim/srv/Spawn
```

**What does `Spawn` service expect?** Get the interface:

```bash
ros2 interface show turtlesim/srv/Spawn
```

**Output**:
```
float32 x
float32 y
float32 theta
string name
---
uint32 success
string message
```

**What this means**:
- **Request** (above the `---`):
  - `x`: X coordinate
  - `y`: Y coordinate
  - `theta`: Rotation angle (radians)
  - `name`: Name for new turtle
- **Response** (below the `---`):
  - `success`: 1 if successful, 0 if failed
  - `message`: Text response

---

## Step 3: Call a Service

Create a new turtle using the `/spawn` service:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 1.57, name: 'turtle2'}"
```

**What you see**:
- In the turtlesim canvas, a **second turtle appears** at position (5, 5)
- Terminal shows response:
  ```
  requester: making request: x=5.0 y=5.0 theta=1.57 name='turtle2'
  response:
  success: True
  message: ''
  ```

**What happened**:
1. You sent a request to `/spawn` with coordinates and name
2. Turtlesim received it, created a new turtle
3. Sent back a response: `success: True`
4. Service call completed

---

## Step 4: Call Another Service (Teleport)

Move a turtle to an exact location:

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 0.0}"
```

**Result**: Turtle 1 jumps to position (8, 8) without drawing a path.

**Compare to topics**: If you had published velocity commands (topic), the turtle would move smoothly. With the service, it jumps instantly.

---

## Step 5: Clear the Canvas

```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

**Result**: All drawings disappear. Canvas blank again.

**Note**: Empty service (no request data needed, hence `{}`)

---

## Step 6: List Parameters

See all configuration parameters on the `/turtlesim` node:

```bash
ros2 param list
```

**Expected output**:
```
/turtlesim:
  background_b
  background_g
  background_r
  pen_r
  pen_g
  pen_b
```

**What these are**:
- `background_r/g/b`: Red, green, blue components of background (0-255)
- `pen_r/g/b`: Red, green, blue components of drawing pen (0-255)

---

## Step 7: Get Parameter Values

```bash
ros2 param get /turtlesim background_r
```

**Output**:
```
Integer value is: 69
```

The background red component is currently 69.

Get all color parameters:

```bash
ros2 param list /turtlesim
```

---

## Step 8: Modify a Parameter

Change the background color to green:

```bash
ros2 param set /turtlesim background_r 0
ros2 param set /turtlesim background_g 255
ros2 param set /turtlesim background_b 0
```

**Result**: Turtlesim canvas background turns bright green!

Change pen color to red:

```bash
ros2 param set /turtlesim pen_r 255
ros2 param set /turtlesim pen_g 0
ros2 param set /turtlesim pen_b 0
```

Now draw on the canvas (run teleop or echo a velocity topic). The turtle draws in red.

---

## Understanding Services vs Topics

Let's compare the two communication patterns:

```
┌─────────────────────────────────────────────────────────────┐
│                    TOPICS (Lesson 3.3)                     │
├─────────────────────────────────────────────────────────────┤
│ Publisher (Teleop)    →  /turtle1/cmd_vel  →  Subscriber   │
│                            (Topic)          (Turtlesim)     │
│                                                              │
│ Characteristics:                                             │
│ • Continuous stream                                          │
│ • Publish once, receive anywhere                           │
│ • No waiting for response                                   │
│ • One-way (usually)                                         │
│ • Fire-and-forget                                           │
│                                                              │
│ Use cases:                                                   │
│ • Sensor data (LIDAR, camera, IMU)                          │
│ • Continuous feedback (odometry, velocity)                  │
│ • Asynchronous updates                                      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                   SERVICES (This Lesson)                    │
├─────────────────────────────────────────────────────────────┤
│ Client (You)          →  /spawn  ↔  Server                 │
│                       (Service) (Turtlesim)                 │
│ Sends Request                                                │
│ Waits for Response  ←────────────                           │
│                                                              │
│ Characteristics:                                             │
│ • Request/response pattern                                  │
│ • Synchronous (client waits)                                │
│ • Gets back data from server                                │
│ • Two-way                                                   │
│ • Transaction-like                                          │
│                                                              │
│ Use cases:                                                   │
│ • Discrete actions (spawn, kill, reset)                     │
│ • Configuration commands                                    │
│ • When you need confirmation                                │
└─────────────────────────────────────────────────────────────┘
```

---

## Hands-On Challenges

### Challenge 1: Create Three Turtles

Using the `/spawn` service, create two more turtles at different positions:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle3'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 9.0, y: 2.0, theta: 0.785, name: 'turtle4'}"
```

Result: Four turtles on the canvas (original + 3 new ones).

---

### Challenge 2: Find the Teleport Service for turtle2

You created `turtle2` earlier. Can you find its teleport service?

```bash
ros2 service list | grep turtle2
```

Or explore:

```bash
ros2 service list
```

Look for services related to `turtle2`.

Once found, try teleporting it:

```bash
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 1.0, y: 1.0, theta: 3.14159}"
```

---

### Challenge 3: Design a Color with Parameters

Using `ros2 param set`, create a custom background color:

```bash
ros2 param set /turtlesim background_r 200
ros2 param set /turtlesim background_g 150
ros2 param set /turtlesim background_b 100
```

Also change pen color and draw. Watch the visual result change in real-time.

---

## Why This Matters

You've now explored ROS 2's complete communication toolkit:

1. **Topics** (Lesson 3.3): Continuous pub/sub data streams
2. **Services** (this lesson): Request/response actions
3. **Parameters**: Runtime configuration

Real robot systems use all three:
- **Topics**: Sensor data (LIDAR, camera)
- **Services**: High-level commands (move arm, open gripper)
- **Parameters**: Configuration (max speed, safety limits)

---

## Design Decision Framework

When building a ROS 2 system, ask:

**Is this continuous or discrete?**
- Continuous → Topic (stream data)
- Discrete → Service (one-off action)

**Do I need a response?**
- Yes → Service (wait for confirmation)
- No → Topic (fire and forget)

**Does every node need this data?**
- Yes → Topic (broadcast)
- No → Service (specific client/server)

**Example decisions**:
- Motor velocity: Topic (continuous, broadcast)
- Gripper command (open/close): Service (discrete, needs confirmation)
- Temperature sensor: Topic (continuous stream)
- Emergency stop: Service (discrete, needs confirmation)

---

## Try With AI

**Setup**: Keep turtlesim running. Open your AI tool.

**Prompt 1** (Topic vs Service Trade-offs):

Ask your AI:

```
In turtlesim, velocity is published as a topic (/turtle1/cmd_vel).
Could we instead implement it as a service?
("Please move the turtle forward" → server moves it → responds "done")

What are pros and cons of using a service instead of a topic for velocity control?
```

**Expected insight**: Topics are better for continuous control (fires repeatedly), while services would block on each call. This reasoning will guide your future design decisions.

---

**Prompt 2** (Parameter Design):

Ask your AI:

```
Turtlesim has parameters for pen color (pen_r, pen_g, pen_b) and background color.
Why are these parameters instead of service calls?
What's the benefit of changing them at runtime rather than restarting the node?
```

**What you learn**: Parameters enable runtime tuning without stopping/restarting nodes. Services would work but require blocking calls. Topics would broadcast unnecessarily.

---

**Prompt 3** (Designing a Robot System):

Ask your AI:

```
I'm designing a mobile robot with:
- A LIDAR sensor
- A motor controller
- An emergency stop button

For each, should I use a topic, service, or parameter?
Explain your reasoning.
```

**What you learn**: Real-world design decisions—sensor data (topic), discrete commands (service), configuration (parameter).

---

## Checkpoint: Chapter 3 Mastery

Before moving to Chapter 4 (Your First Code), verify you can:

- [ ] **Setup (Lesson 3.1)**: Access ROS 2 environment and run diagnostic commands
- [ ] **Turtlesim (Lesson 3.2)**: Launch turtlesim and teleop, control the turtle
- [ ] **Nodes & Topics (Lesson 3.3)**: List nodes, topics, and echo real-time data
- [ ] **Services & Parameters (Lesson 3.4)**: Call services and modify parameters
- [ ] **Understand the difference** between topics (continuous) and services (discrete)
- [ ] **Visualize the system** using rqt_graph

If all boxes are checked, you have the **mental model** needed for Chapter 4, where you'll write your first ROS 2 code (Python publisher/subscriber).

---

## Chapter 3 Mastery Gate

You're ready for **Chapter 4: Your First ROS 2 Code** when you can:

✓ Launch turtlesim and control it with teleop
✓ List all nodes using `ros2 node list`
✓ List all topics using `ros2 topic list -t`
✓ Echo topic data using `ros2 topic echo`
✓ Call a service using `ros2 service call`
✓ Modify a parameter using `ros2 param set`
✓ Visualize nodes/topics with `rqt_graph`
✓ Explain the difference between topics and services

Congratulations on completing **Chapter 3: Meet ROS 2**! You've experienced ROS 2 from the outside. In Chapter 4, you'll experience it from the inside—by writing your own nodes.

---

**Next Chapter**: [→ Chapter 4: Your First ROS 2 Code](../chapter-4-first-code/README.md)
