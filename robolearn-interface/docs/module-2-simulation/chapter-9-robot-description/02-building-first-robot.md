---
id: lesson-9-2-building-first-robot
title: "Lesson 9.2: Building Your First Robot"
sidebar_position: 2
sidebar_label: "9.2 Building First Robot"
description: "Create a simple two-wheeled mobile robot from scratch using URDF"
chapter: 9
lesson: 2
duration_minutes: 75
proficiency_level: "A2"
layer: "L1"
cognitive_load:
  new_concepts: 7
learning_objectives:
  - "Create a complete URDF file for a two-wheeled robot"
  - "Define chassis, wheels, and caster link properly"
  - "Configure continuous joints for wheel rotation"
  - "Set correct coordinate frame origins and rotations"
  - "Validate URDF syntax"
  - "Load and visualize robot in Gazebo"
  - "Debug common URDF errors"
skills:
  - "urdf-basics"
  - "robot-modeling"
hardware_tier: 1
tier_1_path: "The Construct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 9.2: Building Your First Robot

Now you understand URDF concepts. Time to write your first complete robot description.

You'll build a **differential-drive robot**—the simplest mobile platform. Two powered wheels control movement: spin both forward to go straight, spin left faster to turn left. Add a caster wheel for stability, and you have a working mobile base.

This is hands-on. You'll write URDF from scratch, load it in Gazebo, see your robot on screen, and iterate when something goes wrong. By the end, you'll have written a real, functional robot description.

---

## The Robot Design

Your robot:
- **Chassis**: A rectangular box (30 cm × 20 cm × 10 cm) representing the main body
- **Left wheel**: A cylinder (5 cm radius, 5 cm wide) on the left side
- **Right wheel**: A cylinder (5 cm radius, 5 cm wide) on the right side
- **Caster wheel**: A small sphere (2.5 cm radius) at the back for balance

This is a classic differential-drive design used in robots from TurtleBot to custom builds.

---

## Step 1: Create the URDF File

In The Construct cloud environment, create a new file:

**File name:** `my_first_robot.urdf`

**Location:** Your ROS 2 workspace, likely in `src/my_package/urdf/`

Start with the basic structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- We'll add content here -->

</robot>
```

**Expected output after typing:**
```
File created successfully: ~/catkin_ws/src/my_package/urdf/my_first_robot.urdf
```

---

## Step 2: Add the Base Link (Chassis)

The chassis is a rigid box. Add this inside the `<robot>` tags:

```xml
  <!-- Chassis: main body of the robot -->
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
  </link>
```

**What this does:**
- Defines a link named `base_link`
- Visual: A blue box 30cm long, 20cm wide, 10cm tall
- Collision: Same box shape (physics engine treats it this way)

**Expected output when loaded:**
You'll see a blue rectangular box in Gazebo.

---

## Step 3: Add the Left Wheel

```xml
  <!-- Left wheel -->
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
  </link>
```

**What this does:**
- Defines a black wheel (cylinder)
- 5cm radius, 5cm wide (length)

**Expected output when loaded:**
You'll see a black cylinder, but it will float in space until we connect it with a joint.

---

## Step 4: Add the Right Wheel

```xml
  <!-- Right wheel -->
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
  </link>
```

Same as the left wheel.

---

## Step 5: Add the Caster Wheel

```xml
  <!-- Caster wheel for balance -->
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
  </link>
```

**What this does:**
- A small gray sphere (2.5cm radius) for the caster
- No motor—it just rolls freely

---

## Step 6: Connect the Left Wheel with a Joint

Now connect the left wheel to the chassis. Add this after all the link definitions:

```xml
  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
```

**What this does:**
- Joint name: `left_wheel_joint`
- Type: `continuous` (can spin indefinitely—a motor)
- Parent: `base_link` (the chassis)
- Child: `left_wheel` (what we're connecting)
- Origin: Position at (x=0, y=0.15, z=-0.05) relative to the chassis center
  - Y=0.15 places it 15cm to the left
  - Z=-0.05 places it at the bottom of the chassis
- `rpy="1.5708 0 0"`: Rotates the wheel 90 degrees (1.5708 radians = 90°) so it faces sideways
- Axis: `0 0 1` means it spins around the Z axis

**Expected output when loaded:**
The left wheel appears attached to the left side of the chassis, positioned correctly to roll forward.

---

## Step 7: Connect the Right Wheel with a Joint

```xml
  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
```

**Difference from left wheel:**
- Y=-0.15 places it 15cm to the right (opposite side)

---

## Step 8: Connect the Caster Wheel with a Joint

```xml
  <!-- Caster wheel joint (fixed to prevent tipping) -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.1 0 -0.075" rpy="0 0 0"/>
  </joint>
```

**What this does:**
- Type: `fixed` (cannot move, rigidly attached)
- Position: x=-0.1 (at the back), z=-0.075 (at the bottom)
- No `<axis>` element needed because it doesn't rotate

---

## Complete URDF File

Here's the full file with all pieces together:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Chassis: main body -->
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
  </link>

  <!-- Left wheel -->
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
  </link>

  <!-- Right wheel -->
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
  </link>

  <!-- Caster wheel -->
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
  </link>

  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster wheel joint -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.1 0 -0.075" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Step 9: Validate and Load in Gazebo

Save your file. Then check for syntax errors:

```bash
check_urdf my_first_robot.urdf
```

**Expected output if valid:**
```
robot name is: my_robot
---------- Link my_robot ----------
parent
child(1): base_link
---------- Link base_link ----------
parent
child(3): left_wheel, right_wheel, caster_wheel
---------- Link left_wheel ----------
parent: base_link
---------- Link right_wheel ----------
parent: base_link
---------- Link caster_wheel ----------
parent: base_link
```

This output means your URDF is syntactically correct and the structure makes sense.

**If you see errors:**
- Check XML tags are properly closed
- Verify all `<link>` and `<joint>` names are unique
- Ensure `<parent>` and `<child>` references exist

Then load in Gazebo:

```bash
gazebo --verbose -u -g libgazebo_ros_init.so my_first_robot.urdf
```

**Expected visual output:**
You'll see:
- A blue rectangular chassis in the center
- Two black wheels on the sides
- A small gray sphere at the back

The robot sits on the ground, ready to simulate.

---

## Common Mistakes and How to Fix Them

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Missing `<axis>` in continuous joint | Joint spins but nothing rotates | Add `<axis xyz="0 0 1"/>` |
| Wrong rotation values `rpy` | Wheels point backward | Use `rpy="1.5708 0 0"` for sideways cylinders |
| Negative Z values wrong | Wheels float above ground | Use `z=-0.05` to place them at wheel height |
| Missing `</link>` closing tag | XML parse error | Every `<link>` must close with `</link>` |
| Joint parent/child doesn't exist | Missing link error | Check all link names are spelled correctly |

---

## Try With AI

**Setup:** You now have a working URDF. Let's iterate with AI to improve it.

**Prompt Set:**

```
Prompt 1: "Here's my URDF for a two-wheeled robot.
[paste your complete URDF file]
What could go wrong when I use this in Gazebo?
What improvements would you suggest?"

Prompt 2: "How would I modify this URDF to add a camera
mounted on the front of the chassis? Show me the XML
I need to add."

Prompt 3: "I want to add a fourth wheel (another caster at the front).
Show me how to define the link and joint for it."
```

**Expected Outcomes:**
- AI should identify potential issues (missing physics properties from Lesson 9.3, for example)
- Code for camera should include a new link and fixed joint
- Fourth caster should follow same pattern as the first caster

**Safety Note:** At this stage, the URDF is just geometry—your robot doesn't move yet. In Chapter 12, you'll connect it to ROS 2 controllers that actually command motion. The safety considerations come when motors get involved.

**Optional Stretch:** Ask: "How would I add color materials to my robot in Gazebo?" or "What's the difference between `<visual>` and `<collision>` and why do I need both?"
