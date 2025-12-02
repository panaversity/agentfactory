---
id: lesson-13-1-capstone-specification
title: "Lesson 13.1: Capstone Specification"
sidebar_position: 1
sidebar_label: "13.1 Capstone Specification"
description: "Write clear specifications for simulation projects before building them"
duration_minutes: 60
proficiency_level: "B1"
layer: "L4"
hardware_tier: 1
learning_objectives:
  - "Write clear intent statements that define the problem you're solving"
  - "List robot, world, sensor, and behavior requirements systematically"
  - "Define measurable success criteria that you can validate later"
  - "Identify constraints that shape your design decisions"
  - "Evaluate a vague specification and identify missing information"
skills:
  - "specification-writing"
cognitive_load:
  new_concepts: 6
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 13.1: Capstone Specification

In AI-native development, **specifications are the new syntax**. You don't jump straight to code or URDF files. You write intent first. A clear specification answers the question: "What problem am I solving, and how will I know when I've solved it?"

This lesson teaches the skill you'll use throughout your career: writing specifications that guide design and validate results.

---

## The Specification-First Principle

Think of a specification like a contract between you (designer) and your future self (implementer):

**Your future self asks**: "What exactly did I agree to build?"
**Specification answers**: "Build a robot that navigates a warehouse, avoids obstacles, and reaches a goal location within 5 minutes. Success means the robot reaches the goal 8 out of 10 times."

Without this clarity, implementation wanders. You build features that don't solve the problem. You implement optimizations that don't matter. You validate things that weren't the goal.

**With specification clarity**:
- You know exactly what to build
- You can validate when you're done
- You can explain design choices to others

---

## Specification Structure: Five Questions

A complete specification answers these five questions:

### Question 1: What is the intent?

One or two sentences describing the problem you're solving. Not HOW you'll solve it, but WHAT you're solving.

**Example (delivery robot)**:
> "Build a robot that autonomously delivers packages around a warehouse, navigating corridors and avoiding obstacles, delivering to designated drop-off points."

**Example (inspection robot)**:
> "Create a robot that inspects warehouse infrastructure (shelves, beams, ceiling) using onboard camera, collecting images for later analysis."

**What to avoid**:
- ❌ Implementation details ("uses ROS 2 and Gazebo")
- ❌ Technology choices ("deep learning" or "URDF models")
- ❌ Process descriptions ("iterates 100 times")

**What to include**:
- ✅ The goal (deliver, inspect, navigate, map)
- ✅ The environment (warehouse, office, outdoor)
- ✅ The constraints (obstacles, narrow spaces, time limits)

---

### Question 2: What are the robot requirements?

List the physical structure your robot needs:
- **Links** (chassis, wheels, arm, gripper, sensors)
- **Joints** (wheel motors, arm joints, gripper articulation)
- **Sensors** (camera, LIDAR, IMU, bumpers)
- **Actuators** (what moves: wheels, arm, gripper)

**Example (delivery robot)**:
```
Robot Requirements:
- Chassis: 50cm x 30cm x 40cm footprint (fits warehouse corridors)
- Wheels: 2 differential drive wheels (enable turning)
- Bumpers: Front and rear contact sensors (obstacle detection)
- Camera: Forward-facing for navigation
- LIDAR: 2D scanner for obstacle mapping
```

**Example (inspection robot)**:
```
Robot Requirements:
- Arm: 5-joint robotic arm (reach 1.5m height for shelf inspection)
- Camera: High-resolution mounted on arm end-effector
- IMU: Track arm vibration and acceleration
- Base: Wheeled platform for mobility
```

---

### Question 3: What are the world requirements?

Describe the environment where your robot operates:
- **Layout** (warehouse, office building, outdoor path)
- **Obstacles** (shelves, walls, objects on ground)
- **Surface properties** (concrete, carpet, rough terrain)
- **Lighting** (indoor, outdoor, variable shadows)
- **Initial conditions** (starting position, goal location)

**Example (delivery robot)**:
```
World Requirements:
- Warehouse floor: 30m x 20m rectangular space
- Shelves: 4 rows of 10m shelves creating corridors 2m wide
- Obstacles: Cardboard boxes randomly placed on floor (10-20 per simulation)
- Physics: Gravity 9.81 m/s², friction coefficient 0.5
- Starting position: Dock at (0, 0)
- Goal positions: 5 drop-off points at (30, 5), (30, 10), (30, 15), (30, 20), (30, 25)
```

---

### Question 4: What are the behavior requirements?

Describe what the robot DOES:
- **Perception** (what does it sense and understand?)
- **Decision-making** (what logic determines its actions?)
- **Execution** (what commands does it send?)

**Example (delivery robot)**:
```
Behavior Requirements:
- Perception: Use LIDAR to detect obstacles within 2m radius
- Decision: If obstacle closer than 0.5m, stop; otherwise move forward
- Navigation: Follow path using odometry (dead reckoning) to reach goal
- Execution: Publish velocity commands to drive toward goal waypoints
```

---

### Question 5: What are the success criteria?

Define measurable outcomes you'll validate later:
- **Must be testable** (can you measure it in simulation?)
- **Must be quantifiable** (percentage, time, count, distance)
- **Must relate to the problem** (not implementation details)

**Example (delivery robot)**:
```
Success Criteria:
1. Robot reaches goal location within 5 minutes (time limit)
2. Robot successfully avoids all obstacles (never collides)
3. Robot reaches goal 8 out of 10 times (reliability)
4. Robot path is reasonably efficient (not wandering randomly)
```

**Example (inspection robot)**:
```
Success Criteria:
1. Arm reaches all shelf heights (0.5m to 2.0m)
2. Camera captures clear images from minimum 0.5m distance
3. Inspection completes in under 10 minutes
4. Robot detects and reports placement of 5 marked targets
```

---

## Example Specification: Delivery Robot

Here's a complete specification you'll implement in Lesson 13.2:

```
CAPSTONE SPECIFICATION: Autonomous Warehouse Delivery Robot

INTENT
------
Build a robot that autonomously delivers packages from a central dock
to designated drop-off points in a warehouse, navigating corridors while
avoiding obstacles.

ROBOT REQUIREMENTS
------------------
- Chassis: 50cm x 30cm x 40cm (fits 2m-wide warehouse corridors)
- Wheels: 2 differential drive wheels (rear) + caster (front)
- Motors: Wheel velocity control (0 to 1.0 m/s)
- Sensors:
  * LIDAR (2D, 10m range, 270° field of view)
  * Camera (front-facing, for navigation landmarks)
  * IMU (track acceleration, orientation)
  * Bumpers (front and rear contact switches)
- Payload: Gripper to hold packages (simplified as model attachment)

WORLD REQUIREMENTS
------------------
- Environment: Warehouse 30m x 20m
- Obstacles:
  * Shelves: 4 rows x 10m long, 1.5m high, 2m spacing creates corridors
  * Floor objects: 15-20 cardboard boxes randomly distributed
  * Walls: Perimeter enclosure
- Physics: Standard gravity (9.81 m/s²), friction 0.5, soft contacts
- Lighting: Uniform indoor lighting
- Initial State:
  * Robot starts at dock (position 0, 0)
  * Goal locations: (30, 5), (30, 10), (30, 15), (30, 20), (30, 25)
  * Boxes placed randomly in corridors

SENSOR REQUIREMENTS
-------------------
- LIDAR: Publish obstacle distance to /scan topic (sensor_msgs/LaserScan)
- Camera: Publish images to /camera/image_raw (sensor_msgs/Image)
- IMU: Publish acceleration/orientation to /imu/data (sensor_msgs/Imu)
- Bumper: Publish contact events to /bumper (sensor_msgs/BumperEvent)

BEHAVIOR REQUIREMENTS
---------------------
- Perception: Use LIDAR to detect obstacles within 3m radius
- Navigation:
  * From dock, navigate to first goal using odometry
  * Detect obstacles; adjust course to avoid
  * Reach goal location (within 0.5m tolerance)
  * Return to dock for next delivery
- Safety: Never collide with obstacles (bumper reading = failure)
- Communication: ROS 2 node subscribes to /goal and publishes to /cmd_vel

CONSTRAINTS
-----------
- Only ROS 2 Humble available (no custom packages)
- Simulation limited to 10 minute run time per trial
- No ML/neural networks (behavior is rule-based)
- Gazebo Harmonic physics simulator

SUCCESS CRITERIA
----------------
1. Deliverability: Robot reaches all 5 goal locations (100% success rate in single run)
2. Safety: No collisions with obstacles (bumper never triggered)
3. Efficiency: Complete all 5 deliveries in under 5 minutes
4. Reliability: Repeat simulation 3 times; succeed in 3/3 runs
5. ROS 2 Integration: All sensor topics publish data; cmd_vel commands work
```

This specification is specific enough that you know exactly what to implement, yet flexible enough that you can make design decisions (exact LIDAR tuning, obstacle avoidance algorithm, etc.).

---

## What Makes a Specification Good?

**Good specification** (clear, testable):
```
Success Criteria:
- Robot reaches goal location within 5 minutes
- Robot avoids 100% of obstacles (zero collisions)
- Robot completes 5 deliveries in single run
- Behavior succeeds in 3/3 trial runs
```

Measurable? YES (time, collision count, success rate)
Testable? YES (can validate each criterion in simulation)
Related to problem? YES (all criteria matter for delivery task)

**Bad specification** (vague, untestable):
```
Success Criteria:
- Robot works well
- Behavior is good
- Simulation runs
```

Measurable? NO (what does "works well" mean?)
Testable? NO (what does "good" mean in meters or seconds?)
Related to problem? NO (these don't measure delivery capability)

---

## Exercise: Write Your Capstone Specification

**Your task**: Write a complete capstone specification for a simulation project of your choice.

**Options** (pick one, or invent your own):

**Option 1: Delivery Robot** (follow the example above)
- Modify the warehouse layout or goals
- Change sensor configuration
- Add constraints (time limits, path restrictions)

**Option 2: Inspection Robot**
- Robot arm inspects objects at different heights
- Camera captures images for analysis
- Success: arm reaches all target heights, captures 10+ images

**Option 3: Exploration Robot**
- Robot navigates unknown environment building a map
- Uses LIDAR to detect walls and obstacles
- Success: map covers 80% of environment, no collisions

**Option 4: Search and Rescue Robot**
- Robot finds and "rescues" objects in rubble-like environment
- Uses camera to detect colored targets
- Success: finds 5 target objects in under 10 minutes

**Your specification should have**:
- ✅ Intent statement (1-2 sentences)
- ✅ Robot requirements (physical structure, sensors, actuators)
- ✅ World requirements (environment, obstacles, initial conditions)
- ✅ Sensor requirements (which sensors, which topics)
- ✅ Behavior requirements (what the robot does)
- ✅ Success criteria (5 measurable outcomes)
- ✅ Constraints (what you're NOT doing)

**Evaluation**:
- Is your intent clear? (Can someone understand the goal in 30 seconds?)
- Are requirements specific? (Can you implement them from the spec?)
- Are success criteria measurable? (Can you validate them in simulation?)
- Is the project reasonable? (Can you implement in 90 minutes? See Lesson 13.2.)

---

## Try With AI

**Setup**: Open ChatGPT or Claude and refine your capstone specification.

**Prompt Set:**

**Prompt 1** (Validating your specification):
```
I've written a capstone specification for a robot simulation project:

[Paste your specification here]

Review this specification and identify:
1. Ambiguous requirements (what's unclear?)
2. Missing information (what's not specified?)
3. Unrealistic success criteria (what's too hard?)
4. How to make it more specific and measurable
```

**Prompt 2** (Learning from good specifications):
```
Show me an example of a well-written specification for a robotic arm
task (e.g., bin picking, object placement). Include:
- Clear intent
- Robot and world requirements
- 5 measurable success criteria
- Explain what makes each criterion testable
```

**Prompt 3** (Refining your own):
```
I'm building a [YOUR_ROBOT_TYPE] in Gazebo. Here's my current spec:

[Paste your specification]

What should I change or clarify to make this easier to implement?
Which success criteria will be hardest to achieve? What's a realistic time frame?
```

**Expected Outcomes**:
- AI identifies vagueness in your spec
- AI provides examples of well-written specifications
- AI suggests realistic constraints based on your simulation capabilities

**Safety Note**: Your specification guides your implementation. A vague spec leads to wasted time building the wrong thing. A clear spec saves 10 hours of confusion.

---
