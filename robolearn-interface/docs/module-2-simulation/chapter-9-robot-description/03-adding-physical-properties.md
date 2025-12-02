---
id: lesson-9-3-adding-physical-properties
title: "Lesson 9.3: Adding Physical Properties"
sidebar_position: 3
sidebar_label: "9.3 Physical Properties"
description: "Adding mass, inertia, and collision geometry for realistic physics simulation"
chapter: 9
lesson: 3
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
cognitive_load:
  new_concepts: 6
learning_objectives:
  - "Add mass and inertia properties to robot links"
  - "Calculate inertia tensors for common shapes"
  - "Configure collision geometry separate from visual geometry"
  - "Understand why physics properties matter for simulation"
  - "Add material properties for friction and restitution"
  - "Debug physics issues in Gazebo"
skills:
  - "urdf-basics"
  - "physics-properties"
hardware_tier: 1
tier_1_path: "The Construct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 9.3: Adding Physical Properties

You've built a robot with geometry—shape and appearance. But without physics properties, your robot doesn't behave like a real object. It floats. It doesn't fall. Gravity has no effect. Collisions are geometric but not physical.

In this lesson, you'll add the numbers that make simulation realistic: **mass** (how heavy), **inertia** (how weight is distributed), and **friction** (how surfaces interact). These properties transform your URDF from a visual model into a physically accurate simulation.

---

## Why Physics Properties Matter

Imagine a robot in Gazebo with no mass. If you apply a force, what happens? **Nothing.** Physics engines assume massless objects have infinite inertia—they don't accelerate. Your wheels spin but don't roll the chassis. The robot doesn't fall when unsupported.

With mass, the physics engine applies gravity. Without correct inertia, rotation is wrong—the robot flips unexpectedly or spins too easily.

**Example: Tipping Over**
- Robot without inertia: Tilts even slightly and tips over immediately
- Robot with correct inertia: Stable on its wheels, tips only at realistic angles

---

## Core Concept 1: Mass

**Mass** is how much "stuff" is in a link. Measured in kilograms.

```xml
<inertial>
  <mass value="5.0"/>
</inertial>
```

This says the link weighs 5 kilograms.

**Rule of thumb for robots:**
- Small wheel: 0.2 - 0.5 kg
- Medium chassis: 2 - 5 kg
- Large robot: 10 - 50 kg

---

## Core Concept 2: Inertia

**Inertia** describes how mass is distributed. It affects how hard it is to rotate an object.

A heavy ball and a heavy rod can weigh the same, but rotating the rod is harder because the mass is far from the center.

Inertia is specified as a **tensor** (a 3×3 matrix). For simplicity, we use the diagonal values:

```xml
<inertia ixx="0.01" ixy="0" ixz="0"
         iyy="0.02" iyz="0"
         izz="0.03"/>
```

The values are:
- **ixx, iyy, izz**: Rotation resistance around each axis
- **ixy, ixz, iyz**: Cross terms (usually 0 for symmetric objects)

### Common Shape Formulas

For **box** (length L, width W, height H, mass M):
```
ixx = (1/12) * M * (W² + H²)
iyy = (1/12) * M * (L² + H²)
izz = (1/12) * M * (L² + W²)
```

For **cylinder** (radius R, length L, mass M):
```
ixx = (1/12) * M * (3*R² + L²)
iyy = (1/12) * M * (3*R² + L²)
izz = (1/2) * M * R²
```

For **sphere** (radius R, mass M):
```
ixx = iyy = izz = (2/5) * M * R²
```

---

## Calculating Inertia for Your Robot

From Lesson 9.2, your robot has:

### Chassis (box: 0.3m × 0.2m × 0.1m, mass 5 kg)

```
ixx = (1/12) * 5 * (0.2² + 0.1²) = (1/12) * 5 * 0.05 = 0.0208
iyy = (1/12) * 5 * (0.3² + 0.1²) = (1/12) * 5 * 0.1 = 0.0417
izz = (1/12) * 5 * (0.3² + 0.2²) = (1/12) * 5 * 0.13 = 0.0542
```

### Each Wheel (cylinder: 0.05m radius, 0.05m length, mass 0.3 kg)

```
ixx = (1/12) * 0.3 * (3*0.05² + 0.05²) = 0.000625
iyy = (1/12) * 0.3 * (3*0.05² + 0.05²) = 0.000625
izz = (1/2) * 0.3 * 0.05² = 0.000375
```

### Caster Wheel (sphere: 0.025m radius, mass 0.1 kg)

```
ixx = iyy = izz = (2/5) * 0.1 * 0.025² = 0.000025
```

---

## Step 1: Add Mass and Inertia to Base Link

Modify your `base_link` from Lesson 9.2 to include inertial properties:

```xml
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0208" ixy="0" ixz="0"
               iyy="0.0417" iyz="0"
               izz="0.0542"/>
    </inertial>
  </link>
```

**What this does:**
- Mass of 5 kg pulls the robot down with gravity
- Inertia values make rotation realistic

---

## Step 2: Add Properties to Left and Right Wheels

```xml
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.000625" ixy="0" ixz="0"
               iyy="0.000625" iyz="0"
               izz="0.000375"/>
    </inertial>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.000625" ixy="0" ixz="0"
               iyy="0.000625" iyz="0"
               izz="0.000375"/>
    </inertial>
  </link>
```

---

## Step 3: Add Properties to Caster Wheel

```xml
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000025" ixy="0" ixz="0"
               iyy="0.000025" iyz="0"
               izz="0.000025"/>
    </inertial>
  </link>
```

---

## Step 4: Add Friction (Gazebo Extension)

Standard URDF doesn't specify friction. Gazebo uses a custom extension:

Add this to each wheel's link, inside the `<collision>` element:

```xml
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
```

**What this does:**
- `<mu>0.8</mu>`: Friction coefficient (0 = slippery ice, 1.0 = rough rubber)
- Wheels with 0.8 friction grip the ground realistically

---

## Before and After Comparison

### BEFORE (No Physics Properties)

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
</link>
```

**When loaded in Gazebo:**
- Robot floats in space
- Gravity has no effect
- Wheels don't roll (no inertia to resist motion)
- Collisions are detected but not realistically

### AFTER (With Physics Properties)

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
    <inertia ixx="0.0208" ixy="0" ixz="0"
             iyy="0.0417" iyz="0"
             izz="0.0542"/>
  </inertial>
</link>
```

**When loaded in Gazebo:**
- Robot falls to the ground (gravity pulls)
- Wheels support the weight
- When wheels are powered, robot rolls forward (inertia makes it accelerate/decelerate realistically)
- Collisions cause realistic forces

---

## Testing Physics in Gazebo

Load your updated URDF:

```bash
gazebo my_first_robot.urdf
```

**Expected visual output:**
1. Robot falls to the ground (gravity effect)
2. Robot sits stable on three wheels (balance)
3. If you spin a wheel motor (using commands), the robot should roll

**Common Physics Issues:**

| Issue | Cause | Fix |
|-------|-------|-----|
| Robot flips upside down | Inertia values wrong or too light | Recalculate inertia, increase mass |
| Wheels slip excessively | Friction too low | Increase `<mu>` to 0.8-1.0 |
| Robot sinks into ground | Collision geometry too small | Expand `<box>` or `<cylinder>` size |
| Wheels don't spin when powered | No inertia on wheels | Add `<inertial>` to each wheel |

---

## Exercise: Add Physics to Your Robot

Take your URDF from Lesson 9.2 and:

1. **Add inertial properties** to all three links (base_link, left_wheel, right_wheel, caster_wheel)
2. **Use the formulas** provided to calculate correct values
3. **Load in Gazebo** and verify the robot falls to the ground
4. **Test stability**: The robot should sit flat, not tip over

---

## Try With AI

**Setup:** You now have a robot with physics. Let's use AI to validate and improve it.

**Prompt Set:**

```
Prompt 1: "I calculated inertia for my robot using the formulas.
Here are my values: [paste your inertial values]
Do these seem reasonable for a small mobile robot?
Are there any that look wrong?"

Prompt 2: "What would happen if I doubled the mass of my wheels
but kept the inertia the same? Would the robot behave differently?"

Prompt 3: "Explain friction in simulation and how it affects robot motion.
What friction value should I use for wheels on carpet vs. hard floor?"
```

**Expected Outcomes:**
- AI should validate calculations and flag unrealistic values
- You should understand why changing mass alone breaks physics
- You should learn that friction varies by surface

**Safety Note:** Physics properties are critical for accurate simulation. Incorrect inertia can cause:
- Robot tips unexpectedly (safety risk if relying on simulation)
- Wheels spin but don't grip (broken locomotion simulation)
- Jerky, unrealistic motion (wrong control tuning)

Always validate physics against real-world robot behavior.

**Optional Stretch:** Ask your AI tool: "How do I add damping to my robot to make movement less jerky?" or "What's the difference between `<friction>` and `<damping>`?"
