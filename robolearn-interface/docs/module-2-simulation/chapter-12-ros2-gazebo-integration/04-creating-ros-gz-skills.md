---
id: lesson-12-4-creating-ros-gz-skills
title: "Lesson 12.4: Creating ros_gz Skills"
sidebar_position: 4
sidebar_label: "12.4 Creating Skills"
description: "Crystallizing ros_gz patterns into reusable AI skills"
duration_minutes: 60
proficiency_level: "B1"
layer: "L3"
hardware_tier: 1
learning_objectives:
  - "Identify recurring patterns in ros_gz configuration"
  - "Structure a skill using Persona + Questions + Principles"
  - "Create a reusable skill for ROS 2-Gazebo integration"
  - "Apply skill design for cross-project intelligence compounding"
skills:
  - "skill-creation"
  - "intelligence-design"
cognitive_load:
  new_concepts: 6
tier_1_path: "Browser-based skill documentation"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 12.4: Creating ros_gz Skills

## From Lessons to Reusable Intelligence

Over Lessons 12.1-12.3, you mastered a specific workflow:

1. **Lesson 12.1**: Configure bridge from ROS 2 to Gazebo
2. **Lesson 12.2**: Spawn robot from launch file
3. **Lesson 12.3**: Implement control loop with sensor feedback

But you only did this once. What happens when you build your next robot? Do you repeat all the same steps? Or do you have reusable patterns?

This is **Layer 3: Intelligence Design**. You transform tacit knowledge (what you learned by doing) into explicit knowledge (reusable skills that guide future projects).

A **skill** is more than just code. It's documented reasoning about:
- **When to apply this pattern** (what problems does it solve?)
- **How to make decisions** (what are the key choice points?)
- **Why certain choices matter** (what are the tradeoffs?)

---

## What Makes a Good Skill?

Compare three artifacts:

**❌ Code Snippet (Not Reusable)**
```python
# Hard-coded for one specific robot
publisher = self.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5
self.publisher.publish(msg)
```

**❌ Documentation (Generic)**
```
To implement a control loop:
1. Create publisher for velocity commands
2. Subscribe to sensor data
3. Publish commands at regular interval
```

**✅ Skill (Reasoning-Activated)**
```
PERSONA: Think like a robotics engineer designing a control system
that must work for different robots and different sensor configurations.

QUESTIONS:
- What's the control frequency? (10Hz for slow robots, 50Hz for agile)
- Which sensors drive decisions? (LIDAR, camera, IMU—how do they conflict?)
- What safety constraints exist? (max velocity, emergency stop conditions)
- How do you test without a real robot? (simulation fidelity matters)

PRINCIPLES:
1. Decoupling: Sensor reading, decision logic, and motor commands are separate
2. Frequency Alignment: All parts run at the same frequency
3. Safety First: Default to stop, fail-safe on errors
4. Simulation Fidelity: Test in Gazebo first, then real hardware
```

**Why the skill is better**:
- **Persona** activates the right cognitive stance
- **Questions** force you to think through your specific context
- **Principles** guide decisions across different robots

A good skill works for your second robot without modification.

---

## Skill Structure: Persona + Questions + Principles

A reusable skill has this structure:

### 1. Skill Name and Purpose
```
Skill: ros2-gazebo-bridge

Purpose: Configure topic translation between ROS 2 and Gazebo
for any robot type and sensor configuration.

When to use: Whenever connecting ROS 2 nodes to a Gazebo simulation.
```

### 2. Persona (Cognitive Stance)
```
PERSONA: Think like a systems integrator connecting legacy middleware
(ROS 2) to modern physics simulation (Gazebo). Your goal is to
minimize friction points and maximize reusability.
```

The persona answers: **What mindset helps you make good decisions here?**

Not: "You are an expert" (too vague)

But: "Think like a systems integrator optimizing for cross-robot reusability"

### 3. Questions (Decision Points)
```
ANALYSIS QUESTIONS:

1. Sensor Mapping
   - Which sensors exist in simulation? (camera, LIDAR, IMU, contact)
   - Which ROS topic names will downstream nodes expect?
   - What's the message type mapping? (geometry_msgs → gz.msgs)

2. Command Flow
   - Where do velocity commands originate? (teleop, autonomy node, test script)
   - What message type do publishers send? (Twist, Quaternion, Float64)
   - What does Gazebo expect? (matching Gazebo message type)

3. Frequency and Timing
   - At what frequency must commands update? (10Hz minimum, 50Hz typical)
   - Are sensor rates synchronized with command rates?
   - Do you need buffering for non-synchronized topics?

4. Debugging and Monitoring
   - How do you verify the bridge is working? (topic echo, plot)
   - What happens if the bridge crashes? (robot stops immediately)
   - How do you monitor for message drops or delays?

5. Testing Strategy
   - Can you test in pure simulation first?
   - What's the minimal test: one robot, one sensor?
   - How do you scale from one to five robots?
```

The questions answer: **What decisions determine success?**

### 4. Principles (Decision Frameworks)
```
PRINCIPLES:

1. YAML Over Command-Line
   Store all bridges in a single YAML config, not scattered across terminals.
   Rationale: Reproducibility, version control, easy team handoff.

2. Message Type Symmetry
   For bidirectional bridges, ensure ROS and Gazebo types are equivalent.
   Rationale: Prevents silent data corruption from type mismatch.

3. Frequency Alignment
   All sensors and commands run at same frequency. No sensor at 10Hz and
   commands at 50Hz.
   Rationale: Coordination problems emerge at frequency boundaries.

4. Explicit Fallback
   Document what happens if bridge fails (does robot stop? does it drift?).
   Rationale: Safety-critical systems need predictable failure modes.

5. Single Source of Truth
   Define message types once in .yaml, reference by name, never hardcode.
   Rationale: Reduces bugs when changing sensor configurations.
```

The principles answer: **What guidelines prevent common mistakes?**

---

## Skill: ros2-gazebo-bridge (Complete Example)

Here's a production-grade skill you can adapt:

### ros2-gazebo-bridge

**Purpose**: Configure ROS 2 to Gazebo topic bridges for any robot and sensor suite.

**When to use**: Whenever a ROS 2 application must communicate with a Gazebo simulation.

**Persona**: Think like a simulation specialist who knows ROS 2 semantics deeply and Gazebo's message model equally well. Your goal: translate between them with zero semantic loss.

**Analysis Questions**:

1. **What sensors are in your simulation?**
   - Each sensor publishes on a Gazebo internal topic
   - You must expose that data to ROS 2 (GZ_TO_ROS direction)
   - Map each sensor to its corresponding ROS message type

2. **What commands does your robot accept?**
   - Velocity commands? (Twist)
   - Joint position commands? (JointState)
   - Gripper commands? (something custom?)
   - Each command type has a specific ROS message

3. **What's the message type mapping?**
   - Always: ROS package/msg/Type ↔ gz.msgs.Type
   - Examples:
     - geometry_msgs/msg/Twist ↔ gz.msgs.Twist
     - sensor_msgs/msg/Image ↔ gz.msgs.Image
     - sensor_msgs/msg/LaserScan ↔ gz.msgs.LaserScan

4. **What's the direction?**
   - GZ_TO_ROS: Gazebo publishes, ROS 2 consumes (sensors)
   - ROS_TO_GZ: ROS 2 publishes, Gazebo consumes (commands)
   - BIDIRECTIONAL: Both directions (rare, mainly for synchronized state)

5. **What's your testing strategy?**
   - Single sensor → multiple sensors → multiple robots?
   - Test each bridge independently before composing them?
   - How do you verify messages are flowing?

**Principles**:

1. **YAML centralization**: One YAML file lists all bridges. No scattered command-lines.

2. **Type safety**: If a bridge uses wrong message types, it silently fails. Always verify types with `ros2 interface show`.

3. **Frequency verification**: Topic frequency must match your control loop. LIDAR at 10Hz + commands at 50Hz = latency problems.

4. **Debugging first**: Before scaling to complex scenarios, verify simple case (one sensor, one command) works end-to-end.

5. **Fail-safe**: If bridge crashes, robot defaults to safe state (zero velocity, not continued motion).

---

## Building Your Own Skill

Choose a pattern you've discovered in Lessons 12.1-12.3. Create a skill that future projects can reuse.

**Step 1: Identify the Pattern**

From Lessons 12.1-12.3, what recurred?

Examples:
- "Every time I spawn a robot, I followed this launch file pattern"
- "Every obstacle avoidance node had these three components: LIDAR reader, decision logic, velocity publisher"
- "Debugging the bridge always followed the same checklist"

**Step 2: Name the Skill**

Choose a name that describes the pattern:
- `ros2-gazebo-bridge` (what it does)
- `robot-spawning-launch` (what it enables)
- `closed-loop-obstacle-avoidance` (specific behavior)
- `sensor-fusion-decision-logic` (technical pattern)

**Step 3: Document Persona**

Write a single paragraph describing the cognitive stance someone needs to apply this skill well:

```
PERSONA: Think like an autonomy engineer building control systems
that must work across hardware variations. Your goal is to decouple
sensor input, decision logic, and motor commands so each can be
tested independently and swapped for alternatives.
```

**Step 4: Write Analysis Questions**

What decisions does someone need to make when applying this skill?

List 4-5 questions that force context-specific thinking:

```
1. What sensors are available? (which dictate decision inputs)
2. What's the control frequency? (which determines system responsiveness)
3. What's the failure mode? (stop, drift, retry)
4. How do you test incrementally? (one sensor → all sensors → multiple robots)
```

**Step 5: State Principles**

What guidelines prevent common mistakes?

List 3-5 principles with brief rationale:

```
1. Sensor-Decision-Motor separation: Keep reading, computing, and publishing
   in separate methods.
   Rationale: Easier to test and debug each component independently.

2. Fail-safe defaults: If any sensor is unavailable, default to safe state
   (usually: stop moving).
   Rationale: Prevents "best guess" behavior that might be wrong.
```

---

## Cross-Project Reusability

A skill is reusable when it works for your next project with minimal adaptation.

**Same robot, different world**:
- Skill: ros2-gazebo-bridge
- Change: Different YAML config (different world, different sensor positions)
- Reuse: 100% (same code, different parameters)

**Different robot, same task**:
- Skill: closed-loop-obstacle-avoidance
- Change: Robot-specific thresholds (0.5m vs 0.3m obstacle distance)
- Reuse: 95% (same logic, tuned parameters)

**Different task, same patterns**:
- Skill: decision-logic-template
- Change: Different rules (obstacle avoidance → target seeking → path following)
- Reuse: 90% (same structure, different rules)

**Platform-Level Skills**:
Some patterns are so general they apply across ALL robotics books:
- `ros2-middleware-concepts` (messaging, nodes, topics)
- `simulation-first-verification` (test in simulation before hardware)
- `sensor-fusion-architecture` (combining multiple inputs)

These become shared platform skills that compound value across books.

---

## Exercise: Document Your First Skill

**Goal**: Create a reusable skill from patterns you've learned.

**Step 1: Choose a pattern**
Pick one from Lessons 12.1-12.3 that felt important:
- Bridge configuration (12.1)
- Robot spawning (12.2)
- Control loop design (12.3)

**Step 2: Name it**
```
Skill name: [something-descriptive]
Purpose: [one sentence what it enables]
When to use: [what problems does it solve?]
```

**Step 3: Write Persona**
```
PERSONA: Think like a [role] who [goal].
You care about [what matters].
```

**Step 4: Write 4 questions**
```
ANALYSIS QUESTIONS:

1. [Decision point 1 specific to your skill]
   - Sub-question
   - Sub-question

2. [Decision point 2]
   - Sub-question
```

**Step 5: Write 3 principles**
```
PRINCIPLES:

1. [Guideline with rationale]
2. [Guideline with rationale]
3. [Guideline with rationale]
```

**Example completion**:

```
Skill: closed-loop-obstacle-avoidance

Purpose: Enable any robot to safely detect and avoid obstacles using LIDAR feedback.

When to use: Building autonomous navigation systems that must not collide with static obstacles.

PERSONA: Think like a safety engineer who knows that obstacles in the
path are more important than reaching maximum speed. Your goal is to
build behavior that prioritizes not crashing over performance.

ANALYSIS QUESTIONS:

1. Safety Thresholds
   - At what distance do you warn? (e.g., 1.0m)
   - At what distance do you stop? (e.g., 0.5m)
   - How does terrain affect these distances?

2. Behavior on Obstacle
   - Stop in place?
   - Back up and turn?
   - Lateral slide?
   - Each has different implications for robot type.

3. Testing and Validation
   - How do you verify the thresholds are safe?
   - What happens if LIDAR fails? (fallback behavior)
   - How do you test on physical robot safely?

4. Integration with Other Behaviors
   - Does obstacle avoidance override other goals?
   - Can the robot ignore obstacles if ordered to?
   - How do you rank multiple competing behaviors?

PRINCIPLES:

1. Fail-safe defaults: If obstacle distance is uncertain, assume obstacle
   exists. Better to stop unnecessarily than crash.

2. Clear thresholds: Document all distance and velocity thresholds in code
   comments with rationale. Magic numbers are bugs.

3. Sensor fusion: Don't rely on LIDAR alone. Combine with other sensors
   (camera, contact) for robust behavior.

4. Incremental testing: Test obstacle detection separately from avoidance
   behavior. Verify each part works before combining.
```

---

## Try With AI

**Setup**: Open ChatGPT or Claude and refine your skill design.

**Prompt 1** (Skill refinement):
```
I'm documenting a skill for ROS 2-Gazebo bridge configuration.
Here's my first draft:

PERSONA: [your persona]
QUESTIONS: [your questions]
PRINCIPLES: [your principles]

What questions am I missing? What principles would save a future me
from common mistakes?
```

**Prompt 2** (Cross-project validation):
```
I documented a skill for obstacle avoidance that works for differential
drive robots. What would I need to change to make it work for:
- Humanoid robots with foot placement?
- Aerial drones that need to avoid obstacles above/below?
- Manipulator arms with large reach?

Show me what stays the same and what changes.
```

**Prompt 3** (Skill composition):
```
I have these three skills:
1. ros2-gazebo-bridge (bridge configuration)
2. robot-spawning (launch file patterns)
3. closed-loop-control (decision logic)

How would I combine them into a higher-level skill like
"full-robot-simulation-workflow"? What new decisions would that
higher-level skill need to address?
```

**Expected Outcomes**:
- AI identifies missing questions from your skill
- AI helps adapt skills across different robot types
- AI suggests skill hierarchies (composing simple skills into complex ones)

---

## Why Skills Matter

Skills are how intelligence compounds across projects:

**Without skills** (Procedural approach):
- Project 1: Build obstacle avoidance (20 hours)
- Project 2: Build obstacle avoidance again (20 hours)
- Project 3: Build obstacle avoidance again (20 hours)
- Total: 60 hours of repeated work

**With skills** (Intelligence design):
- Project 1: Build obstacle avoidance + document skill (25 hours)
- Project 2: Use skill, adapt to new robot (5 hours)
- Project 3: Use skill, adapt to new sensors (3 hours)
- Total: 33 hours, and skill works for future projects too

Skills are the bridge between individual learning and organizational knowledge.

---

## Next Steps

You've completed Layer 2 (AI collaboration) and Layer 3 (intelligence design) for ROS 2-Gazebo integration. Chapter 13 will orchestrate these skills in a capstone project.

**What emerged from this lesson**: Reusable intelligence is more valuable than reusable code. A well-documented skill guides you through decision-making across different contexts. Skills compound—each new skill makes the next skill easier to build.

---

## Key Concepts Checkpoint

Before moving to the capstone, verify you understand:

- **Skill vs code**: Skills document reasoning, not just implementation
- **Persona**: How cognitive stance shapes decision-making
- **Analysis questions**: Forcing context-specific thinking
- **Principles**: Decision frameworks that prevent mistakes
- **Cross-project reuse**: How skills scale across different robots
- **Skill composition**: Building higher-level skills from simpler ones

If these are clear, you're ready for Chapter 13: Capstone Project.

---

## Appendix: Skill Template

Use this template to document your own skills:

```markdown
# Skill: [Name]

**Purpose**: [One sentence describing what this skill enables]

**When to use**: [What problems does this skill solve?]

**When NOT to use**: [What problems is this NOT for?]

---

## Persona

Think like a [role] who [describes cognitive stance].
Your goal is [primary objective].
You care about [what matters most].

---

## Analysis Questions

What decisions determine success when applying this skill?

### Category 1: [Decision Area]
1. [Question 1]
   - [Sub-question]
   - [Sub-question]

2. [Question 2]

### Category 2: [Decision Area]
1. [Question 1]

---

## Principles

What guidelines prevent common mistakes?

### Principle 1: [Name]
[Statement of principle]

**Rationale**: [Why this matters]

**Counterexample**: [What goes wrong without this]

---

## Cross-Project Reusability

How does this skill scale?

- **Same context, different details**: [% reuse]
- **Different context, same task**: [% reuse]
- **Different task, same patterns**: [% reuse]

---

## Example Application

[Show how to apply this skill in a real project]

---

## Related Skills

[What other skills does this compose with?]
```

Use this template for any pattern you want to document and share.
