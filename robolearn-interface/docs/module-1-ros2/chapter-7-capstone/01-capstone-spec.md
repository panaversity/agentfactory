---
id: lesson-7-1-capstone-spec
title: "Lesson 7.1: Capstone Specification"
sidebar_position: 1
sidebar_label: "7.1 Capstone Specification"
description: "Write specification FIRST—define system intent, architecture, interfaces, and success criteria before implementing."
duration_minutes: 90
proficiency_level: "B1"
layer: "L4"
hardware_tier: 1
learning_objectives:
  - "Write clear, unambiguous system specifications"
  - "Define system architecture with explicit responsibilities"
  - "Specify topics, services, and message types precisely"
  - "Create measurable success criteria for validation"
  - "Identify non-goals to prevent scope creep"
---

# Lesson 7.1: Capstone Specification

## The Key Insight

In professional robotics, this principle is absolute: **Specification comes FIRST. Code comes second.**

Before you write a single line of Python, before you create a single publisher or service, you define: What is the system supposed to do? What messages flow where? How do we know when success happens?

This lesson teaches you that skill. By the end, you'll have written a complete specification that's so clear, precise, and unambiguous that someone else could implement it—and get the right system.

---

## Why Specs Matter More Than Code

Think about why this matters for a robot system:

**Scenario 1: No Specification**
- You start coding a Navigator node
- Halfway through, you realize: "Should this take goal requests via service or topic?"
- You've already written half the code for a service
- You refactor, lose time, maybe introduce bugs

**Scenario 2: Specification First**
- You write: "Navigator accepts goals via `/navigate_to` service"
- You implement exactly that
- No surprises, no refactoring, clear design decision captured
- Someone else reads your spec and understands your intent

**Professional roboticists live by specifications.** The goal is clear before implementation starts.

---

## The Specification Template

Here's the template you'll use. Every line matters—vagueness causes implementation failures.

### 1. Intent

**What problem does this system solve?** One paragraph. Be specific about what the robot does and why.

**Bad intent**: "Create a robot controller"
**Good intent**: "Build a multi-node ROS 2 system that navigates a turtlesim turtle to goal positions, reports its status continuously, and gracefully handles obstacle warnings from a mock sensor."

Why is the second better? It's **concrete**:
- What's the scope? Multi-node, ROS 2
- What's the goal? Navigate to positions
- What data flows? Status reports, obstacle warnings
- What's the graceful handling? Doesn't crash, continues operating

### 2. System Architecture

**How many nodes? What does each do?** List them like:

```
Navigator Node
  Input: Goal position (x, y, theta) via service request
  Output: Velocity commands to turtle
  Purpose: Accepts navigation goals, computes paths, issues movement commands

Status Monitor Node
  Input: Current turtle state (from topics, odometry simulators)
  Output: Robot status (position, velocity, battery) published every 100ms
  Purpose: Aggregates state information, publishes to external subscribers

Obstacle Detector Node
  Input: (None—generates mock sensor data)
  Output: Obstacle warnings on topic
  Purpose: Simulates obstacle detection, publishes warnings for Navigator to handle
```

Each node has **input, output, and purpose**. No mystery. No ambiguity.

### 3. Interfaces (Topics & Services)

**What's the contract between nodes?** Define every topic and service:

```
Service: /navigate_to
  Request: x (float), y (float), theta (float)
  Response: success (bool), time_taken (float)
  Purpose: Accept navigation goal, return completion status and time

Topic: /robot/status (custom TurtleStatus message)
  Message fields: x (float), y (float), theta (float), vel_x (float), vel_y (float), battery (float)
  Frequency: 10 Hz (every 100ms)
  Subscribers: Status monitor, or external monitoring nodes
  Purpose: Publish robot state continuously

Topic: /obstacles (sensor_msgs/LaserScan mock)
  Message fields: angle_min, angle_max, distances (array)
  Frequency: 5 Hz
  Publishers: Obstacle detector
  Purpose: Simulated obstacle data for Navigator to process
```

**Why this detail?** When implementing, the developer reads this and knows:
- Exactly what message types to use
- How often data flows
- Who publishes, who subscribes
- What fields go in custom messages

### 4. Success Criteria

**How do you know it worked?** Measurable, testable conditions:

```
✅ Turtle reaches goal position within 2 seconds of service call
✅ Status messages published every 100ms (±10ms tolerance)
✅ System continues operating when 3+ obstacles detected (no crash, no freeze)
✅ Launch file with `ros2 launch` starts all nodes with correct parameters
✅ Navigator node runs without errors for 5+ minutes
✅ Service responds with completion time within 0.1s of actual movement time
```

Why measurable? You'll test each one explicitly in Lesson 7.3.

### 5. Non-Goals

**What's NOT in scope?** Explicitly excluding these prevents scope creep:

```
❌ Real obstacle avoidance (only detection, no path planning around obstacles)
❌ Real LIDAR simulation (mock sensor only, not full physics)
❌ Multiple turtles (single turtle only)
❌ URDF or robot description (deferred to Module 2)
❌ Real hardware deployment (simulation only)
❌ Visual path visualization (logging/debugging only)
```

Writing "what's NOT included" is as important as writing "what IS included."

---

## Your Specification Template

Now you fill it in. Use this template:

```markdown
# Turtle Robot Controller Specification

## Intent
[1 paragraph describing what this system does, concretely]

## System Architecture

### Navigator Node
Input: [What does it receive?]
Output: [What does it produce?]
Purpose: [Why does it exist?]

### Status Monitor Node
Input: [What does it receive?]
Output: [What does it produce?]
Purpose: [Why does it exist?]

### Obstacle Detector Node
Input: [What does it receive?]
Output: [What does it produce?]
Purpose: [Why does it exist?]

## Interfaces

### Service: /navigate_to
Request: [Fields]
Response: [Fields]
Purpose: [Why this service?]

### Topic: /robot/status
Message Type: [Custom or standard?]
Fields: [List each field]
Frequency: [Hz or ms?]
Purpose: [Why this topic?]

### Topic: /obstacles
Message Type: [Custom or standard?]
Fields: [List each field]
Frequency: [Hz or ms?]
Purpose: [Why this topic?]

## Success Criteria

✅ [Specific, measurable criterion 1]
✅ [Specific, measurable criterion 2]
✅ [Specific, measurable criterion 3]
✅ [Specific, measurable criterion 4]
✅ [Specific, measurable criterion 5]

## Non-Goals

❌ [What's NOT in scope 1]
❌ [What's NOT in scope 2]
❌ [What's NOT in scope 3]
```

---

## What Makes a Specification GOOD?

### Test It Against These Criteria

**Criterion 1: Clarity**
- Could a developer who hasn't met you implement this from your spec alone?
- No hand-waving ("intelligently handles obstacles") — be specific
- No vague terms ("optimal path") — define exactly what you mean

**Criterion 2: Completeness**
- Every topic has a message type defined
- Every service has request/response types defined
- Every node has inputs and outputs
- Nothing left for someone to guess about

**Criterion 3: Testability**
- Every success criterion is measurable
- You can test it with `ros2 topic echo` or service calls
- You can write a test script that validates each criterion

**Criterion 4: Realism**
- Is the spec achievable in 90 minutes?
- Are the success criteria reasonable?
- Does it stretch your skills without breaking them?

---

## Common Spec Mistakes to Avoid

### Mistake 1: Vague Success Criteria

**Bad**: "Turtle navigates to goal"
**Good**: "Turtle reaches goal position within 2 seconds, with position error < 0.1m"

Why? The first one is ambiguous. What counts as "reached"? The second is testable.

### Mistake 2: Missing Node Responsibilities

**Bad**: "System controls turtle"
**Good**: "Navigator node computes velocity commands; Status Monitor publishes state; Obstacle Detector publishes warnings"

Why? The bad version doesn't clarify who does what. The good version makes clear separation of concerns.

### Mistake 3: No Message Types

**Bad**: "Nodes communicate via topics"
**Good**: "Status Monitor publishes TurtleStatus (custom message with x, y, theta, vel_x, vel_y, battery fields) at 10 Hz"

Why? Without message types, the implementer has to invent them. With types, implementation is straightforward.

### Mistake 4: Unrealistic Time Constraints

**Bad**: "Turtle reaches goal in 0.1 seconds"
**Good**: "Turtle reaches goal in 2 seconds" (realistic for turtlesim movement)

Why? If your spec is impossible, implementation fails. Make success criteria achievable.

---

## The Turtle Specification Example (Reference)

Here's what a completed specification looks like:

```markdown
# Turtle Robot Controller Specification

## Intent
Build a multi-node ROS 2 system that controls a turtlesim turtle to achieve
autonomous navigation goals. The system accepts goal positions via service call,
publishes continuous status updates, and gracefully handles simulated obstacle
warnings without crashing or losing control.

## System Architecture

### Navigator Node
Input: Goal position (x, y, theta) via /navigate_to service request
Output: Velocity commands (cmd_vel) to turtle; Goal completion response
Purpose: Orchestrates movement toward goal. Accepts new goals via service.
         Computes velocity commands based on goal. Returns success/time when done.

### Status Monitor Node
Input: Turtle odometry (pose, velocity) — either from /turtle1/pose topic
       or computed from cmd_vel/odom if available
Output: /robot/status topic (custom TurtleStatus message)
Purpose: Continuously publishes robot state (position, velocity, battery) for
         external monitoring. Enables other nodes to react to state changes.

### Obstacle Detector Node
Input: (None—generates simulated data)
Output: /obstacles topic (sensor_msgs/LaserScan mock structure)
Purpose: Simulates obstacle detection. Publishes periodic "detections" that
         Navigator can handle. Tests system robustness.

## Interfaces

### Service: /navigate_to
Request:
  - x (float): Goal X position in turtle coordinate system
  - y (float): Goal Y position in turtle coordinate system
  - theta (float): Goal orientation in degrees (0-360)
Response:
  - success (bool): True if goal reached, false if aborted
  - time_taken (float): Seconds elapsed from request to completion
Purpose: Primary command interface. Clients request navigation to position.
         Navigator handles the goal and returns status.

### Topic: /robot/status
Message Type: TurtleStatus (custom)
Fields:
  - x (float): Current X position
  - y (float): Current Y position
  - theta (float): Current orientation (degrees)
  - vel_x (float): Velocity in X direction
  - vel_y (float): Velocity in Y direction
  - battery (float): Simulated battery level (0-100%)
Frequency: 10 Hz (every 100ms)
Subscribers: Status monitor dashboard, external decision systems
Purpose: Real-time state stream. Enables reactive behavior and monitoring.

### Topic: /obstacles
Message Type: sensor_msgs/LaserScan (simplified mock)
Fields:
  - angle_min, angle_max: Detection angle range
  - ranges: Array of distance measurements (simulated)
Frequency: 5 Hz (every 200ms)
Publishers: Obstacle Detector node
Purpose: Mock sensor data. Navigator subscribes and avoids/stops if obstacle
         detected. Tests handling of sensor input.

## Success Criteria

✅ Service /navigate_to accepts goal and returns success=true when turtle reaches position within 0.2m
✅ Turtle moves to goal within 2 seconds from service request
✅ Status topic publishes TurtleStatus message every 100ms (±10ms variance acceptable)
✅ System receives and logs 10+ obstacle messages without crashing
✅ Launch file ros2 launch robot_controller start_system.launch.py starts all nodes and parameters
✅ Navigator node runs continuously for 5+ minutes without errors
✅ Custom TurtleStatus message compiles and serializes correctly

## Non-Goals

❌ Real obstacle avoidance (system detects obstacles but doesn't plan around them)
❌ Realistic LIDAR simulation (mock sensor only, not physics-based)
❌ Multiple turtles or coordination between agents
❌ Robot URDF or kinematics (direct velocity commands only)
❌ Real hardware deployment (turtlesim simulation only)
❌ Path planning or trajectory optimization (straight-line movement is acceptable)
❌ Visual RViz visualization (CLI validation only)
```

---

## System Architecture Diagram

Here's how the nodes interact:

```
┌─────────────────────────────────────────────────────┐
│         Turtle Robot Controller System               │
├─────────────────────────────────────────────────────┤
│                                                       │
│  Client App                                          │
│     │                                                │
│     │ (service call: /navigate_to)                   │
│     ↓                                                │
│  ┌──────────────────────────────────────────┐       │
│  │ Navigator Node                            │       │
│  │  ├─ Input: Goal (x,y,theta) via service  │       │
│  │  ├─ Computes: velocity commands          │       │
│  │  └─ Output: cmd_vel to turtlesim         │       │
│  └──────────────────────────────────────────┘       │
│     │ (cmd_vel)          │ (/navigate_to response)   │
│     │                    └────────────► Client       │
│     ↓                                                │
│  [TurtleSim Turtle]◄────────────────────────────┐   │
│     │ (updated pose)                            │   │
│     ↓                                            │   │
│  ┌──────────────────────────────────────────┐   │   │
│  │ Status Monitor Node                       │   │   │
│  │  ├─ Input: turtle pose/odometry          │   │   │
│  │  ├─ Aggregates: position, velocity, battery
│  │  └─ Output: /robot/status (10 Hz)        │───┤   │
│  └──────────────────────────────────────────┘   │   │
│                                                  │   │
│  ┌──────────────────────────────────────────┐   │   │
│  │ Obstacle Detector Node                    │   │   │
│  │  ├─ Input: (generated simulated data)    │   │   │
│  │  └─ Output: /obstacles (5 Hz) ───────────┼───┴──→ Navigator
│  └──────────────────────────────────────────┘       │
│                                                       │
└─────────────────────────────────────────────────────┘
```

---

## Try With AI

**Setup**: You'll use a chat AI (Claude, ChatGPT, or GitHub Copilot) to help refine your specification.

**Phase 1: Draft Your Specification**

First, write your rough specification using the template above. You can fill it with:
- Concrete details (real node names, real message types like `sensor_msgs/LaserScan`)
- Or high-level ideas (you can refine in phase 2)

**Phase 2: AI Refinement Prompts**

Once you have a draft, use these prompts with your AI:

**Prompt 1: Clarity Check**
```
I've written this ROS 2 specification for a turtle controller:

[paste your spec]

Are there any parts that are vague or ambiguous?
Where would a developer be confused implementing this?
What details are missing?
```

**Prompt 2: Completeness Check**
```
Looking at my specification, did I define:
- All message types (what fields, what types?)
- All topics (who publishes, who subscribes, at what frequency?)
- All services (what request/response structure?)
- All node responsibilities (what does each node do?)

Where are the gaps?
```

**Prompt 3: Testability Check**
```
Can each of these success criteria be tested with ROS 2 CLI tools?

[paste your success criteria]

Which ones are testable? Which ones are too vague to test?
How would I write a test for each one?
```

**Expected Outcome**: Your refined specification should be clear enough that:
- A developer could implement it without asking questions
- You could test each success criterion with concrete ROS 2 commands
- You have no ambiguous terms (no "intelligently", "optimally", "well")

**Safety Note**: Your AI will sometimes suggest overly complex specs (adding features beyond the module scope). Remember: Non-Goals exclude URDF, Actions, tf2, and real obstacle avoidance. Keep it simple—focus on pub/sub, services, parameters, and launch files from Chapters 4-6.

**Optional Stretch**: Write your specification in PlantUML or Mermaid diagram format showing node communication. This forces you to think visually about system architecture.

---

**Next Step**: [Lesson 7.2: Building the Controller →](./02-building-controller.md)
