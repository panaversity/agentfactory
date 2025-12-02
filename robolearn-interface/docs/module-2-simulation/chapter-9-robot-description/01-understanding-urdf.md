---
id: lesson-9-1-understanding-urdf
title: "Lesson 9.1: Understanding URDF"
sidebar_position: 1
sidebar_label: "9.1 Understanding URDF"
description: "Introduction to Unified Robot Description Format (URDF) for describing robot structure"
chapter: 9
lesson: 1
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
cognitive_load:
  new_concepts: 6
learning_objectives:
  - "Explain what URDF is and why it is needed in robotics"
  - "Identify the basic XML structure of a URDF file"
  - "Distinguish between links and joints in robot descriptions"
  - "Recognize visual and collision geometry definitions"
  - "Understand coordinate frames and transformations"
  - "Validate URDF syntax using basic tools"
skills:
  - "urdf-basics"
hardware_tier: 1
tier_1_path: "The Construct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 9.1: Understanding URDF

A robot is not just code—it's a physical structure. Before Gazebo can simulate your robot, it needs a blueprint. That blueprint is written in **URDF**: Unified Robot Description Format.

URDF is an XML language. Don't let that scare you. XML is just structured text with tags. Think of URDF as a recipe for a robot: you list ingredients (links—the rigid parts), describe how to combine them (joints—the connections), and specify their appearance (geometry). The simulator reads this recipe and assembles your robot.

In this lesson, you'll learn what URDF is, why it matters, and how to read URDF files. You won't write complex URDFs yet—you'll understand the concepts so that when you write them in Lesson 9.2, they make sense.

---

## Why Do We Need URDF?

Imagine you're building a robot in the real world. You'd specify:
- What parts exist: "I have a chassis, two wheels, and a camera"
- How they connect: "Each wheel attaches to the chassis with a revolving joint"
- What they look like: "The chassis is a rectangular box; wheels are cylinders"
- How heavy they are: "Chassis weighs 5 kg; each wheel weighs 0.5 kg"

URDF captures all this information in a structured, machine-readable format. The simulator reads the URDF file and understands your robot's structure instantly.

**Without URDF:** You'd describe your robot in plain English, and the simulator wouldn't understand.

**With URDF:** The simulator automatically constructs your robot, calculates where it can move, applies physics forces correctly, and renders it on screen.

---

## Core Concept 1: Links (Rigid Bodies)

A **link** is a rigid body—a part of your robot that doesn't bend or deform. Examples:
- Chassis (main body)
- Wheel
- Arm segment
- Camera mount

In URDF, a link has:
- A **name** (how you refer to it in your code)
- **Visual geometry** (how it looks: shape, size, color)
- **Collision geometry** (what shape the physics engine treats it as)
- **Inertial properties** (mass, how weight is distributed)

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
  </inertial>
</link>
```

**Key insight:** Every robot has at least one link called `base_link`. This is the reference frame—the origin where measurements start.

---

## Core Concept 2: Joints (Connections)

A **joint** connects two links. It specifies how one link can move relative to the other.

Types of joints:
- **revolute**: Rotates around an axis (like a wheel spinning)
- **continuous**: Unlimited rotation (like a spinning motor)
- **prismatic**: Slides along an axis (like a linear actuator)
- **fixed**: Cannot move (rigidly attached)

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.15 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

This joint says:
- It connects `base_link` (parent) to `left_wheel` (child)
- The left wheel is positioned 0.15 meters to the left of the chassis
- It can spin continuously around the Y axis

**Key insight:** Joints are directional. The parent link is the reference; the child link moves relative to it.

---

## Core Concept 3: Coordinate Frames

Every link has a **coordinate frame**—an origin (0, 0, 0) and three axes (X, Y, Z).

```
      Z (up)
      ^
      |
      o----- X (forward)
     /
    Y (left)
```

When you specify a joint's `origin`, you're saying:
- `xyz="0 0.15 0"`: The child link's origin is 0.15 meters along the Y axis
- `rpy="0 0 0"`: No rotation (Pitch=0, Roll=0, Yaw=0)

This system lets you describe complex robots unambiguously.

---

## Core Concept 4: Visual vs Collision Geometry

**Visual geometry** is what humans see. It looks pretty.

**Collision geometry** is what the physics engine uses. It can be simplified for faster simulation.

Example: A robot arm might have a detailed 3D model (visual) but use simple boxes for collision (faster physics calculations).

```xml
<link name="arm_link">
  <visual>
    <mesh filename="arm_detailed.stl"/>
  </visual>
  <collision>
    <box size="0.1 0.05 0.3"/>
  </collision>
</link>
```

---

## Reading a Simple URDF File

Let's look at a complete minimal robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base link: the chassis -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

**What this URDF describes:**
- A robot with three links: base (chassis) and two wheels
- The left wheel is positioned 0.15m to the left (+Y direction)
- The right wheel is positioned 0.15m to the right (-Y direction)
- Both wheels can spin continuously (motors)
- The `rpy="1.5708 0 0"` rotates the wheel 90 degrees so it faces the right direction

**When you load this in Gazebo:**
```
Expected output:
[INFO] Loaded robot with 3 links and 2 joints
[INFO] robot_name: simple_robot
[INFO] Links: base_link, left_wheel, right_wheel
[INFO] Joints: left_wheel_joint, right_wheel_joint
```

---

## Understanding XML Tags

URDF is XML, which means everything is wrapped in tags: `<tag>content</tag>`

Basic structure:
- `<robot>`: The root element, wraps everything
- `<link>`: Defines a rigid body
- `<joint>`: Connects two links
- `<geometry>`: Specifies shape
- `<visual>`: How it looks
- `<collision>`: How physics treats it

Attributes are properties inside tags:
```xml
<joint name="my_joint" type="continuous">
```
Here, `name` and `type` are attributes.

**Key patterns:**
- Every attribute goes in quotes: `name="value"`
- Tags must be properly closed: `<tag>content</tag>` or `<tag/>`
- Indentation helps humans read it but doesn't matter to the computer

---

## Checking Your Understanding

Before moving to writing URDFs, verify you can answer:

1. **What is a link?** (A rigid body—a part of your robot)
2. **What is a joint?** (A connection between two links, specifies how they move)
3. **Why separate visual and collision geometry?** (Visual is pretty; collision is fast)
4. **What does `base_link` mean?** (The reference frame, the origin of the robot)
5. **What happens if you forget the `<axis>` element in a joint?** (The simulator doesn't know which direction to rotate)

---

## Try With AI

**Setup:** Open your preferred AI tool (ChatGPT, Claude, etc.) and have a conversation about URDF.

**Prompt Set:**

```
Prompt 1: "Explain what a URDF file is like you're teaching a beginner.
Use an analogy (like cooking, building, etc.) to help me understand."

Prompt 2: "Show me a URDF file for a simple wheeled robot.
Then explain each section—what does each tag do?"

Prompt 3: "If I have a robot with 4 wheels instead of 2,
how would the URDF change? Show me the code."
```

**Expected Outcomes:**
- AI should explain URDF as a blueprint or recipe
- Code example should include links for wheels and a joint for each wheel
- You should understand how to modify the URDF for different wheel counts

**Safety Note:** URDF is just data—it describes structure but doesn't control anything yet. You can't break anything with URDF alone. When we add physics (Lesson 9.3) and motors (Chapter 12), that's when safety becomes important.

**Optional Stretch:** Ask your AI tool: "What's the difference between URDF and SDF? When would you use each one?" (You'll encounter SDF later when building Gazebo worlds.)
