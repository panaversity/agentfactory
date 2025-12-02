---
id: lesson-10-3-physics-configuration
title: "Lesson 10.3: Physics Configuration"
sidebar_position: 3
sidebar_label: "10.3 Physics Configuration"
description: "Configuring physics engines, friction, contact properties, and debugging physics instability in Gazebo simulations."
duration_minutes: 60
proficiency_level: "B1"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Understand how physics engines simulate motion and collisions"
  - "Configure physics engine selection and step size for accuracy"
  - "Set surface friction and contact parameters for realistic behavior"
  - "Debug common physics instability issues (falling through floors, jittering, explosions)"
  - "Validate physics configuration through simulation testing"
skills:
  - "physics-configuration"
  - "simulation-debugging"
cognitive_load:
  new_concepts: 7
tier_1_path: "The Construct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 10.3: Physics Configuration

You can create beautiful SDF worlds with models and lighting, but if physics is wrong, your robot will behave incorrectly. A robot trained to walk in a world with wrong gravity or friction won't work when deployed to the real world.

This lesson teaches you to configure Gazebo's physics engine so simulation matches reality. You'll learn to choose physics engines, set step sizes for accuracy, configure friction and contact properties, and debug common physics problems.

## Physics Engines in Gazebo

Gazebo supports three physics engines, each with different accuracy and performance characteristics:

### DART (Recommended)
**Name**: Dynamic Animation and Robotics Toolkit
**Strengths**: Stable, accurate, good for multi-body systems
**Weaknesses**: Can be slower with very large worlds
**Best for**: Most robotics applications (default choice)

**Configuration**:
```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### ODE (Open Dynamics Engine)
**Name**: Open Dynamics Engine
**Strengths**: Fast, good for simple scenes
**Weaknesses**: Less stable with complex collisions, may exhibit jittering
**Best for**: Simpler worlds where speed matters

**Configuration**:
```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <solver>
    <type>quick</type>
    <iters>20</iters>
  </solver>
</physics>
```

### Bullet
**Name**: Bullet Physics Engine
**Strengths**: Very stable, good for soft-body physics
**Weaknesses**: Can be slower than ODE
**Best for**: Complex contact scenarios, soft-body objects

**Configuration**:
```xml
<physics name="default_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Recommendation**: Use DART for most robotics work. Switch to ODE only if DART is too slow. Use Bullet if you need advanced contact modeling.

## Understanding Step Size (max_step_size)

The physics engine works like a movie. It divides time into discrete frames called "steps". Each step, the engine:
1. Reads the current state (positions, velocities)
2. Calculates forces and accelerations
3. Integrates motion equations to find new positions and velocities
4. Detects collisions
5. Applies collision forces

The **step size** is how long each timestep is (in seconds).

### Small Step Size (Accurate but Slow)
```xml
<max_step_size>0.0001</max_step_size>  <!-- 0.1 millisecond steps -->
```
**Pros**: Very accurate, smooth motion, stable collisions
**Cons**: Slow simulation (takes 10,000 steps per second)
**Use case**: Critical physics experiments, precise control algorithms

### Medium Step Size (Balanced)
```xml
<max_step_size>0.001</max_step_size>  <!-- 1 millisecond steps -->
```
**Pros**: Good balance of accuracy and speed (standard for robotics)
**Cons**: None significant for most applications
**Use case**: Almost all robotics simulations (recommended)

### Large Step Size (Fast but Inaccurate)
```xml
<max_step_size>0.01</max_step_size>  <!-- 10 millisecond steps -->
```
**Pros**: Very fast simulation, quick iteration
**Cons**: May miss collisions, objects can pass through each other, jerky motion
**Use case**: Quick testing where high accuracy isn't needed

**Rule of thumb**: Use 0.001s (1 millisecond) as your default. If simulation is too slow, try 0.002s or 0.005s. If physics seems wrong (objects passing through each other), try smaller step sizes.

## Real-Time Factor

The **real_time_factor** controls simulation speed relative to wall-clock time:

```xml
<real_time_factor>1.0</real_time_factor>
```

**real_time_factor = 1.0**
- 1 second of simulation = 1 second of wall time
- Robot walks at normal speed
- Comfortable for watching and iterating

**real_time_factor = 2.0**
- 1 second of simulation = 0.5 seconds of wall time
- Everything happens twice as fast
- Good for quick testing

**real_time_factor = 0.5**
- 1 second of simulation = 2 seconds of wall time
- Everything happens in slow motion
- Good for observing fast dynamics in detail

**Important**: Changing real_time_factor does NOT affect the step size or simulation accuracy. It only changes how fast the simulation runs relative to your wall clock. Use 1.0 for normal operation and development.

## Friction and Contact Properties

Objects interact through friction and contact forces. Gazebo lets you configure these properties per material.

### Setting Friction in SDF

Friction is defined at the collision level (the invisible physics shape, distinct from the visual shape):

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu>
        <mu2>0.5</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

**mu** and **mu2**: Friction coefficients (0 to very high)
- **0**: No friction (frictionless ice)
- **0.1 - 0.3**: Very slippery (glass, lubricated surfaces)
- **0.5 - 0.7**: Normal friction (wood, concrete)
- **1.0+**: High friction (rubber, sandpaper)

**Typical values by material**:
- Steel on steel: mu = 0.6
- Rubber on concrete: mu = 0.8
- Plastic on plastic: mu = 0.5
- Wheel on ground: mu = 1.0

### Contact Properties

Beyond friction, you can configure contact stiffness and damping:

```xml
<surface>
  <friction>
    <ode>
      <mu>0.5</mu>
      <mu2>0.5</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>10000000</kp>  <!-- Contact stiffness (spring constant) -->
      <kd>1000</kd>      <!-- Contact damping (friction in contact) -->
    </ode>
  </contact>
</surface>
```

**kp** (contact stiffness):
- Higher values = stiffer contact (objects don't deform much)
- Lower values = softer contact (objects deform more, bouncy)
- Typical range: 1,000,000 to 10,000,000

**kd** (contact damping):
- Higher values = more energy dissipation (less bouncy)
- Lower values = less damping (bouncier)
- Typical range: 100 to 10,000

**Practical tuning**:
```xml
<!-- Soft, bouncy contact (ball, tennis ball) -->
<kp>1000000</kp>
<kd>100</kd>

<!-- Normal, stable contact (most objects) -->
<kp>10000000</kp>
<kd>1000</kd>

<!-- Hard, non-bouncy contact (metal on metal) -->
<kp>100000000</kp>
<kd>10000</kd>
```

## Common Physics Problems and Solutions

### Problem 1: Objects Falling Through the Ground

**Symptom**: Your robot or objects fall through the ground plane and disappear.

**Causes**:
1. Ground plane collision geometry is missing or misconfigured
2. Physics step size is too large (collisions missed)
3. Object is created with initial velocity pointing downward

**Solutions**:
```xml
<!-- Fix 1: Ensure ground plane has proper collision -->
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
  </link>
</model>

<!-- Fix 2: Reduce step size -->
<physics name="default_physics" type="dart">
  <max_step_size>0.0005</max_step_size>  <!-- Smaller steps -->
</physics>

<!-- Fix 3: Place object above ground at startup -->
<include>
  <uri>model://openrobotics/table</uri>
  <pose>0 0 1.0 0 0 0</pose>  <!-- z=1.0 ensures height above ground -->
</include>
```

### Problem 2: Objects Jittering or Vibrating

**Symptom**: Objects shake uncontrollably, vibrating on the ground.

**Causes**:
1. Step size too large (instability in integration)
2. Contact stiffness too high (oscillating forces)
3. Friction coefficients unrealistic

**Solutions**:
```xml
<!-- Fix 1: Reduce step size -->
<physics name="default_physics" type="dart">
  <max_step_size>0.0005</max_step_size>  <!-- Half the default -->
</physics>

<!-- Fix 2: Tune contact properties -->
<contact>
  <ode>
    <kp>5000000</kp>    <!-- Lower stiffness -->
    <kd>2000</kd>       <!-- Higher damping to absorb energy -->
  </ode>
</contact>

<!-- Fix 3: Use realistic friction -->
<friction>
  <ode>
    <mu>0.7</mu>        <!-- Normal friction coefficient -->
    <mu2>0.7</mu2>
  </ode>
</friction>
```

### Problem 3: Objects Bouncing Unexpectedly

**Symptom**: Objects bounce when they hit the ground, like they're made of rubber.

**Causes**:
1. Contact damping (kd) too low
2. Friction coefficients too low
3. Objects have high restitution (bounciness)

**Solutions**:
```xml
<!-- Reduce bouncing -->
<contact>
  <ode>
    <kp>10000000</kp>
    <kd>10000</kd>      <!-- Much higher damping -->
  </ode>
</contact>

<!-- High friction to prevent sliding -->
<friction>
  <ode>
    <mu>1.0</mu>
    <mu2>1.0</mu2>
  </ode>
</friction>

<!-- Alternative: Use Bullet engine (more stable damping) -->
<physics name="default_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
</physics>
```

### Problem 4: Simulation Too Slow

**Symptom**: Gazebo runs slowly, making development tedious.

**Causes**:
1. Step size too small
2. Physics engine doing unnecessary calculations
3. World too complex (many objects, detailed meshes)

**Solutions**:
```xml
<!-- Increase step size (trade accuracy for speed) -->
<physics name="default_physics" type="dart">
  <max_step_size>0.002</max_step_size>  <!-- 2ms instead of 1ms -->
</physics>

<!-- Use ODE for faster simulation -->
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <solver>
    <type>quick</type>
  </solver>
</physics>

<!-- Simplify world: remove unnecessary objects, use simpler meshes -->
<!-- Disable shadows if not needed -->
<scene>
  <shadows>false</shadows>  <!-- Faster rendering -->
</scene>
```

## A Complete Physics Configuration Example

Here's a world with carefully tuned physics for a mobile robot:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="physics_tuned_world">

    <!-- DART physics with moderate step size -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Earth gravity -->
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
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane with good friction -->
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
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
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
          </material>
        </visual>
      </link>
    </model>

    <!-- Table: stable contact properties -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>2 2 0 0 0 0</pose>
      <name>work_table</name>
    </include>

  </world>
</sdf>
```

## Physics Validation: Testing Your Configuration

After setting up physics, validate it:

**Test 1: Gravity Test**
- Place a small object (model) at z=1.0
- Run the simulation
- Confirm it falls to the ground and stops (doesn't bounce or jitter)

**Test 2: Friction Test**
- Push an object horizontally (using ROS 2 or Gazebo GUI)
- Confirm it slides then stops (doesn't slide forever or stop immediately)

**Test 3: Collision Test**
- Drop one object onto another
- Confirm they collide and stay in contact (don't vibrate or pass through)

**Test 4: Multi-Body Test**
- If your robot has multiple moving parts (wheels, arms)
- Confirm all parts move smoothly together

## Try With AI

**Setup**: Have a text editor ready with SDF world files. Access your AI assistant.

**Prompt 1** (Selection):
```
I'm building a Gazebo simulation for a mobile robot on concrete.
Should I use DART, ODE, or Bullet physics engine?
What step size would you recommend?
Explain the tradeoffs.
```

**Expected outcome**: AI explains engine differences and recommends DART with 0.001s step size for balanced accuracy/speed.

**Prompt 2** (Configuration):
```
I want to configure realistic friction for:
1. Concrete ground (mu = ?)
2. Rubber wheels (mu = ?)
3. Metal robot frame (mu = ?)

Write the SDF friction configuration for each.
```

**Expected outcome**: AI suggests realistic friction coefficients and provides XML snippets.

**Prompt 3** (Troubleshooting):
```
My robot is jittering badly when it sits on the ground.
I'm using DART with step_size=0.001, and contact properties:
kp=10000000, kd=1000

What's likely wrong and how should I fix it?
```

**Expected outcome**: AI suggests increasing damping (kd) and possibly using Bullet engine for better stability.

