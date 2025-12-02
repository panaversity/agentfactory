---
id: lesson-3-2-turtlesim-action
title: "Lesson 3.2: Turtlesim in Action"
sidebar_position: 2
sidebar_label: "3.2 Turtlesim"
description: "Launch and control ROS 2's turtlesim simulator to visualize node communication and data flow."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Launch turtlesim node and see it run"
  - "Understand turtlesim as a ROS node (executable + messaging logic)"
  - "Use rqt_graph to visualize node connections"
skills:
  - "ros2-fundamentals"
tier_1_path: "Cloud ROS 2 with VNC or local with GUI"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Turtlesim in Action

You have a working ROS 2 environment. Now let's see ROS 2 do something visible.

**Turtlesim** is a simple 2D robot simulator built into ROS 2. You'll launch it, control it with your keyboard, and watch the data flow between nodes. By the end of this lesson, you'll have a visceral understanding of how ROS 2 coordinates multiple processes.

**Duration**: 60 minutes | **Hardware Tier**: Tier 1 (cloud ROS 2 with VNC OR local with GUI)

---

## What is Turtlesim?

Turtlesim is a minimal robot simulator that comes with every ROS 2 installation. It simulates:

- **A turtle** (robot) on a 2D canvas
- **Velocity commands** (forward/rotate)
- **Odometry** (position feedback)
- **Services** (spawn/kill/reset)

Think of it as your first robot. It's not fancy, but it demonstrates every core ROS 2 concept: nodes, topics, services, and messaging.

---

## Terminal Setup (Both Paths)

Before you launch anything, ensure you have two terminals open:

### For Cloud Path (The Construct)

1. Click **Open Shell** (creates a new terminal)
2. Click **Open Shell** again (creates a second terminal)
3. You now have two side-by-side terminals

---

### For Local Path (Ubuntu/WSL)

1. Open your first terminal: `Ctrl+Alt+T` or open a terminal app
2. Open a second terminal: Same method in new window
3. Source both: In each terminal, run `source ~/.bashrc` (if you set it up in Lesson 3.1)

---

## Step 1: Launch Turtlesim Node

In **Terminal 1**, run:

```bash
ros2 run turtlesim turtlesim_node
```

**What happens**:

Terminal output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [1234]
```

A new window opens showing a blue canvas with a yellow turtle in the center.

**Keep this running** (don't close Terminal 1).

---

## Step 2: Launch Keyboard Teleop (Control)

In **Terminal 2**, run:

```bash
ros2 run turtlesim turtle_teleop_key
```

Terminal output:
```
This node takes keypresses on stdin and publishes them
as Twist messages on the 'turtle1/cmd_vel' topic.
---------------------------
Reading from keyboard
Use arrow keys to move the turtle.
Press 'q' to quit.
```

**You're now ready to control the turtle.**

---

## Step 3: Control the Turtle

In **Terminal 2** (where the keyboard teleop is running), press keys:

- **↑ (Up Arrow)**: Turtle moves forward
- **↓ (Down Arrow)**: Turtle moves backward
- **← (Left Arrow)**: Turtle turns left
- **→ (Right Arrow)**: Turtle turns right

**Try this**:

1. Press **↑** several times → turtle draws a line forward
2. Press **← ← ←** → turtle rotates left
3. Press **↑** → turtle moves in new direction
4. Continue drawing a shape (square, spiral, whatever)

**What's happening**:

- Your keypresses go into the teleop process
- Teleop converts them into ROS 2 **Twist messages** (velocity commands)
- These messages are published to the `/turtle1/cmd_vel` topic
- Turtlesim subscribes to that topic
- Turtlesim receives the velocity, updates turtle position
- Canvas redraws the turtle at new location

**You're seeing ROS 2 in action:** Process A (teleop) → Topic → Process B (turtlesim)

---

## Understanding the Flow

Let me visualize what's happening:

```
┌─────────────────────────────────────────────────────┐
│              Your Computer / Cloud                  │
├─────────────────────────────────────────────────────┤
│                                                     │
│  Terminal 1            Topic               Canvas   │
│  ┌──────────────┐    (Message Bus)       ┌──────┐  │
│  │ turtlesim    │←────────────────────────│ Pose │  │
│  │ node         │  /turtle1/odometry     │  X,Y │  │
│  │              │                        │  θ    │  │
│  └──────────────┘←────────────────────────│      │  │
│         ↑        /turtle1/cmd_vel         │      │  │
│         │         (Twist: linear,angular) └──────┘  │
│         │                                           │
│         │                                           │
│  Terminal 2                                         │
│  ┌──────────────┐                                   │
│  │ teleop_key   │─────────┐                         │
│  │ (reads       │         │                         │
│  │  keyboard)   │    Publishes Twist                │
│  └──────────────┘    messages                       │
│      ↑                                              │
│      │                                              │
│    Your                                             │
│   Keyboard                                          │
│                                                     │
└─────────────────────────────────────────────────────┘
```

**Key insight**: The teleop process doesn't directly control the turtle. Instead, it publishes velocity messages. Turtlesim **listens** for those messages and acts on them. This is the **publish/subscribe pattern**—the foundation of ROS 2.

---

## Step 4: Visualize with RQT Graph

Open a **third terminal** (keep the first two running):

```bash
ros2 run rqt_graph rqt_graph
```

A window opens showing a visual graph of nodes and topics.

**You should see**:

```
      ┌──────────────┐
      │  /teleop_key │
      └────────┬─────┘
               │
        /turtle1/cmd_vel (Twist)
               │
               ↓
      ┌──────────────┐
      │  /turtlesim  │
      └──────────────┘
```

**Legend**:
- **Rectangles**: Nodes (processes)
- **Lines**: Topic connections
- **Direction**: Arrows show data flow (publisher → topic → subscriber)

Try pressing keyboard keys while watching the graph. The graph doesn't animate, but you can see the **structure**: teleop publishes, turtlesim subscribes.

---

## Hands-On Challenge: Draw a Square

Using only keyboard arrow keys, draw a square in turtlesim.

**Strategy**:
1. Move forward (↑) for ~3-4 seconds
2. Turn left (←) to rotate 90 degrees (roughly)
3. Move forward again
4. Repeat 4 times

**What you learn**: You're controlling a robot through discrete actions. Timing is imprecise (depends on how fast you press keys), but you see how velocity commands accumulate into position.

---

## Why This Matters

This lesson demonstrates the core insight of ROS 2:

1. **Modularity**: Teleop and turtlesim are independent processes
2. **Communication**: They coordinate through topics (pub/sub)
3. **Decoupling**: Turtlesim doesn't know about teleop; it just listens to messages
4. **Extensibility**: You could write a *different* process that publishes to `/turtle1/cmd_vel`, and turtlesim would respond the same way

This is why ROS 2 is powerful: build complex systems by connecting independent, reusable components.

---

## Cleanup

When done exploring:

1. In Terminal 2 (teleop): Press **q** to quit
2. In Terminal 1 (turtlesim): Press **Ctrl+C** to stop
3. Optional: In Terminal 3 (rqt_graph): Close the window

All nodes stop cleanly.

---

## Try With AI

**Setup**: Keep your turtlesim running (Terminal 1 and 2), and open your AI tool in another window.

**Prompt 1** (Understanding the Architecture):

Ask your AI:

```
In ROS 2, the teleop process publishes to /turtle1/cmd_vel,
and turtlesim subscribes to the same topic.

Explain why the teleop process doesn't need to know the details
of how turtlesim is implemented. How does ROS 2 enable this decoupling?
```

**Expected insight**: The **publish/subscribe pattern** is better than direct function calls. This mental model is crucial for Chapter 4 when you write your own nodes.

---

**Prompt 2** (Scalability):

Ask your AI:

```
Right now, only turtlesim subscribes to /turtle1/cmd_vel.
What if I wrote a second node that ALSO subscribed to the same topic?
Would both nodes receive the velocity commands?
What would be the benefit or problem of that design?
```

**What you learn**: In ROS 2, a topic can have multiple subscribers. The AI might suggest use cases (logging, redundancy, simulation + real robot switching) or caveats (both nodes react independently).

---

**Prompt 3** (Extension):

Ask your AI:

```
Turtlesim publishes odometry (position updates) on /turtle1/odometry.
Currently, nothing subscribes to that.

If I wrote a new node that subscribed to odometry and printed
"Turtle is at position X, Y", how would it receive data?
Would it get every position update, or would it miss some?
```

**What you learn**: Multiple subscribers can listen to one topic, and ROS 2 handles the message routing. The AI might mention buffering/queue depth, which you'll encounter in Chapter 4.

---

## Checkpoint Before Next Lesson

Before proceeding to Lesson 3.3, verify you understand:

- [ ] Turtlesim is a **node** (process running ROS logic)
- [ ] Teleop is a **different node** that publishes keyboard input as messages
- [ ] They communicate through a **topic** (`/turtle1/cmd_vel`)
- [ ] RQT graph showed the **node-topic structure** visually
- [ ] Both nodes run **independently**—teleop doesn't call turtlesim functions directly

If all checkboxes are clear, you're ready for Lesson 3.3, where you'll explore ALL the topics and services in the system using CLI tools.

---

**Next lesson**: [→ Lesson 3.3: Nodes and Topics (CLI Exploration)](./03-nodes-topics.md)
