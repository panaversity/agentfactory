---
id: lesson-10-1-sdf-world-basics
title: "Lesson 10.1: SDF World Basics"
sidebar_position: 1
sidebar_label: "10.1 SDF World Basics"
description: "Understanding Simulation Description Format (SDF) for creating robot environments with ground planes, lighting, and physics configuration."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Explain the difference between URDF and SDF formats"
  - "Describe the structure of SDF world files"
  - "Create a basic SDF world with ground plane and lighting"
  - "Configure world physics properties including gravity and step size"
skills:
  - "sdf-world-creation"
  - "gazebo-configuration"
cognitive_load:
  new_concepts: 6
tier_1_path: "The Construct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 10.1: SDF World Basics

You now understand URDF—how to describe a robot's structure, joints, and physics. But a robot exists in an environment. Before your robot can move, it needs a world to move through.

This lesson introduces **SDF** (Simulation Description Format), the language Gazebo uses to describe complete simulation environments. While URDF describes a single robot, SDF describes the entire stage: ground, sky, lighting, physics rules, and all objects in the world.

## Understanding URDF vs SDF

You've seen URDF before (Chapter 9). Let's clarify the distinction:

**URDF (Unified Robot Description Format)**
- Describes **one robot** only
- Specifies links (rigid bodies) and joints (connections)
- Includes visual geometry and collision shapes
- Encodes physics properties (mass, inertia, friction)
- **Limitation**: Cannot describe environments or multiple robots
- **Use case**: "What does this robot look like and how does it move?"

**SDF (Simulation Description Format)**
- Describes **entire worlds** (robot + environment + objects + physics + lighting)
- Can include multiple robots and environmental objects
- Specifies world-level properties: gravity direction, physics step size, ambient lighting
- Includes sun/sky/lighting configuration
- Supports physics engine selection and tuning
- **Use case**: "What does this robot's entire operating environment look like?"

**Practical relationship**: Your URDF-described robot is placed INTO an SDF-described world. The robot blueprint (URDF) must exist before you can add it to a world (SDF).

## The Structure of an SDF World File

An SDF world file is XML. Here's the basic skeleton:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default_world">

    <!-- Physics configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane (floor) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Let's break down each section:

### Section 1: Physics Configuration

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**What this does:**
- **type**: Physics engine to use ("dart", "ode", or "bullet")
- **max_step_size**: How large each simulation step is (in seconds). Smaller = more accurate but slower. 0.001 seconds is typical.
- **real_time_factor**: Speed relative to wall-clock time. 1.0 means "run at real speed". 2.0 means "run twice as fast"

### Section 2: Lighting

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <direction>-0.5 0.5 -1</direction>
</light>
```

**What this does:**
- **type**: "directional" (like the sun), "point" (like a lamp), or "spot" (like a flashlight)
- **pose**: Position of the light source (x, y, z, roll, pitch, yaw)
- **direction**: Where the light is shining (as a direction vector)
- **cast_shadows**: Whether the light creates shadows (more realistic but slower)

### Section 3: Ground Plane

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane>...</plane></geometry>
    </collision>
    <visual name="visual">
      <geometry><plane>...</plane></geometry>
    </visual>
  </link>
</model>
```

**What this does:**
- **model**: A named object in the world (ground_plane is the floor)
- **static**: "true" means it never moves (ground stays put)
- **collision**: The physics shape (plane is infinite flat surface)
- **visual**: How it looks to the camera (appearance)

## Creating Your First World

Let's create a minimal but complete world file. This is the simplest valid SDF world:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">

    <!-- Physics: DART engine, 1ms steps, real-time speed -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Gravity: 9.81 m/s^2 downward (standard Earth) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Sky appearance -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun-like directional light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane (infinite flat floor) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**Output**: If you save this as `empty_world.sdf` and open it in Gazebo, you'll see:
- A flat gray ground plane extending to the horizon
- Bright lighting from a directional light (the sun)
- Objects you place in this world will experience Earth-like gravity (9.81 m/s downward)

## Configuring Gravity

Gravity is specified as a 3D vector (x, y, z). Each component is acceleration in that direction in m/s^2.

**Examples:**

Standard Earth gravity (downward):
```xml
<gravity>0 0 -9.81</gravity>
```

Reduced gravity (Mars-like, about 1/3 Earth):
```xml
<gravity>0 0 -3.71</gravity>
```

No gravity (space):
```xml
<gravity>0 0 0</gravity>
```

Gravity in an unusual direction (for special scenarios):
```xml
<gravity>-9.81 0 0</gravity>  <!-- Gravity pulls toward negative X -->
```

Why does gravity matter? If your robot is designed for Earth gravity but simulates with Mars gravity, it will behave differently—heavier steps, different balance dynamics. For accurate sim-to-real transfer, use Earth gravity unless you have a specific reason otherwise.

## Physics Step Size and Real-Time Factor

Two critical parameters control simulation accuracy and speed:

**max_step_size**: How large each timestep is (in seconds)
- 0.001s (1 millisecond) = accurate but slow
- 0.01s (10 milliseconds) = standard for most robotics
- 0.1s (100 milliseconds) = fast but may be inaccurate

**real_time_factor**: Simulation speed relative to wall-clock time
- 1.0 = "real-time" (1 second of simulation = 1 second of wall time)
- 2.0 = "2x speed" (1 second of simulation = 0.5 seconds of wall time)
- 0.5 = "slow motion" (1 second of simulation = 2 seconds of wall time)

If your robot physics seem wrong (objects bouncing strangely, robots tipping over unexpectedly), the step size is often the culprit. Smaller step sizes are more accurate but slower. Start with 0.001s and adjust if needed.

## Lighting and Visual Quality

SDF worlds support multiple light types:

**Directional Light** (like the sun)
```xml
<light type="directional" name="sun">
  <direction>-0.5 0.5 -1</direction>
  <diffuse>0.8 0.8 0.8 1</diffuse>
</light>
```

**Point Light** (like a lamp or bulb)
```xml
<light type="point" name="lamp">
  <pose>0 0 2 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <attenuation>
    <range>5</range>
    <constant>1</constant>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

**Spot Light** (like a flashlight or stage light)
```xml
<light type="spot" name="spotlight">
  <pose>0 0 5 0 0 0</pose>
  <direction>0 0 -1</direction>
  <spot>
    <inner_angle>0.1</inner_angle>
    <outer_angle>1</outer_angle>
    <falloff>1</falloff>
  </spot>
</light>
```

Color values are RGBA (Red, Green, Blue, Alpha) ranging from 0 to 1:
- (1, 0, 0, 1) = Red
- (0, 1, 0, 1) = Green
- (0, 0, 1, 1) = Blue
- (1, 1, 1, 1) = White
- (0, 0, 0, 1) = Black

## Putting It Together: A Complete Minimal World

Here's a world file you can copy and use immediately:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_first_world">

    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**What to do next:**
1. Copy this code
2. Save it as `my_first_world.sdf`
3. Open it in Gazebo: `gazebo my_first_world.sdf`
4. You should see a gray ground plane with lighting and shadows

## Key Takeaways

**SDF vs URDF**: URDF describes robots, SDF describes worlds that contain robots.

**World structure**: Every SDF file needs physics configuration, lighting, and a ground plane.

**Physics parameters matter**: Step size and real-time factor control accuracy and speed. Match them to your needs.

**Gravity is configurable**: Use Earth gravity (0, 0, -9.81) for realistic behavior. Change it only intentionally.

**Lighting affects simulation**: More lights = more realistic but slower. Start simple (one directional light) and add complexity as needed.

## Try With AI

**Setup**: Open your preferred AI tool (ChatGPT, Claude, or similar) and have a world file (the code above) ready to paste.

**Prompt Set**:

**Prompt 1** (Understanding):
```
I'm learning Gazebo worlds (SDF format). Here's my basic world file:
[Paste the empty_world.sdf above]

Explain what each major section does in 2-3 sentences per section.
```

**Expected outcome**: AI explains physics, lighting, and ground plane in plain language.

**Prompt 2** (Modification):
```
How would I modify this world to:
1. Have half gravity (Mars-like)
2. Use a faster physics step size (for quick testing)
3. Add dimmer lighting (less bright sun)

Show me the modified sections of the SDF file.
```

**Expected outcome**: AI provides modified XML sections with brief explanations.

**Prompt 3** (Troubleshooting):
```
If I opened this world in Gazebo and objects kept falling through the ground plane,
what are three possible causes in the SDF file?
For each cause, what should I check or modify?
```

**Expected outcome**: AI identifies likely issues (incorrect physics engine, bad collision geometry, wrong gravity) and suggests fixes.

