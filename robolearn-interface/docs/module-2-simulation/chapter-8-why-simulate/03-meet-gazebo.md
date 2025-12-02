---
id: lesson-8-3-meet-gazebo
title: "Lesson 8.3: Meet Gazebo Harmonic"
sidebar_position: 3
sidebar_label: "8.3 Meet Gazebo"
description: "Introduction to Gazebo Harmonic (gz-sim), the open-source robotics simulator"
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Describe Gazebo Harmonic's client-server architecture"
  - "Identify the role of plugins in Gazebo simulation"
  - "Access Gazebo through TheConstruct cloud environment"
skills:
  - "gazebo-basics"
cognitive_load:
  new_concepts: 7
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Meet Gazebo Harmonic

**Duration**: 60 minutes | **Layer**: L1 (Manual Foundation) | **Tier**: 1 (Cloud)

A digital twin is a concept. A simulator is the tool that realizes that concept. For professional robotics worldwide, that simulator is **Gazebo**.

Gazebo is:
- **Open-source**: Free to use, community-driven, trusted by NASA, DARPA, universities
- **Physics-accurate**: Real gravity, friction, collisions, sensor physics
- **ROS 2 integrated**: Your ROS 2 code controls virtual robots the same way it controls physical robots
- **Industry standard**: The most widely used robotics simulator globally

This lesson introduces you to Gazebo Harmonic (the modern version, also called gz-sim), how it works, and how to access it through TheConstruct cloud environment.

## Why Gazebo?

Before exploring how to use Gazebo, understand why it's the industry choice.

### The Gazebo Ecosystem

Gazebo started in 2002 as part of the Robot Operating System (ROS) project. Over two decades, it became the standard simulator for academic robotics, commercial robotics, and research institutions worldwide.

**Organizations using Gazebo**:
- NASA (Mars rover simulation, lunar missions)
- DARPA (robotics competition simulation)
- Boston Dynamics (dynamics research)
- University robotics labs (worldwide)
- Robotics startups (most use Gazebo)

**Why this matters**: If you learn Gazebo, you learn with the tools professional engineers use.

### Gazebo vs. Competitors

Other simulators exist (Unity, Unreal, CoppeliaSim, MuJoCo). Gazebo's advantages:

| Feature | Gazebo | Unity | Unreal | Why it Matters |
|---------|--------|-------|--------|----------------|
| **Cost** | Free | Freemium | Freemium | Academic budgets prefer free |
| **ROS 2 Integration** | Native | Plugin | Plugin | ROS 2 is robotics standard |
| **Physics Fidelity** | High | Medium | Medium | Sim-to-real transfer requires accurate physics |
| **Open Source** | Yes | No | No | Transparency, community contributions |
| **Linux Support** | Excellent | Good | Fair | Most robotics work on Linux |

Gazebo isn't universally "better"—Unity excels at visualization, Unreal at realism—but for robotics + ROS 2 development, Gazebo is the natural choice.

## Gazebo Harmonic Architecture

Understanding Gazebo's design helps you use it effectively.

### Client-Server Model

Gazebo uses a **client-server architecture**:

```
Server (gz-sim)
  - Simulates physics
  - Updates robot state
  - Processes ROS 2 messages
  - Runs plugins
  - Stores world state

       |
       | TCP/IP (network communication)
       |

Clients
  - gz-gui (graphical interface)
  - ROS 2 nodes (your code)
  - External tools (recording, visualization)
```

**Why this design?**
- **Separation of concerns**: Physics computation separate from visualization
- **Network transparency**: Server and clients can run on different machines
- **Multiple clients**: One physics engine can serve multiple visualization clients
- **Remote simulation**: Server in cloud, client on your laptop

For you, this means:
- The simulation server runs on TheConstruct cloud
- You connect to it via your web browser
- Your ROS 2 code communicates with the server via network messages
- Visualization displays in your browser

### The Gazebo Server (gz-sim)

The server handles the physics computation:

1. **Load world**: Parse SDF (Simulation Description Format) files defining robots and environments
2. **Run plugins**: Load simulator plugins that add functionality
3. **Simulate physics**: Update positions, velocities, forces each timestep
4. **Process messages**: Receive ROS 2 messages from control code
5. **Update state**: Send sensor data back to ROS 2 nodes
6. **Communicate**: Send world state to visualization clients

The server runs the simulation loop continuously, usually at 1000 Hz (1000 physics updates per second).

### The Gazebo GUI Client (gz-gui)

The GUI client provides visualization:

1. **3D rendering**: Display robots, environments, and sensor data
2. **Interaction**: Click to select objects, drag to move them
3. **Introspection**: Inspect joint states, sensor readings, world properties
4. **Simulation control**: Play/pause, speed up/slow down simulation
5. **Camera control**: Rotate, zoom, pan the view

For you:
- You access this through your browser on TheConstruct
- No installation needed (all computation happens in cloud)

### Plugins: The Extensibility System

Gazebo's power comes from plugins. A plugin is code that extends the simulator:

**Sensor plugins**: Simulate cameras, LIDAR, IMUs, force sensors
- Capture virtual images, point clouds, acceleration data
- Publish to ROS 2 topics

**Actuator plugins**: Control motors and grippers
- Subscribe to ROS 2 topics for motor commands
- Update joint positions in simulation

**System plugins**: Custom behaviors
- Implement controllers, AI agents, test scenarios
- Run arbitrary code within simulation

**Example plugin flow**:

```
Your ROS 2 Code                Gazebo Server
     |                              |
     |--- publishes motor cmd ----->|
     |                         Physics Update
     |                         (plugin processes cmd)
     |                         Motor moves
     |<--- publishes sensor data ----|
     |                              |
```

The sensor plugin captures the new motor position and publishes it as a ROS 2 topic your code reads.

## Gazebo and ROS 2 Integration

The bridge between Gazebo and ROS 2 is transparent:

### How It Works

1. **Your ROS 2 code** publishes a motor command to topic `/cmd_vel`
2. **Gazebo plugin** subscribes to `/cmd_vel`
3. **Plugin updates simulation** - moves the robot
4. **Sensor plugin** captures new robot state
5. **Plugin publishes** to ROS 2 topic `/odom` (odometry)
6. **Your ROS 2 code** subscribes to `/odom` and reads the state

From your perspective, it's identical to controlling a physical robot:
- You publish commands to `/cmd_vel`
- You read sensor data from `/odom`
- Whether the endpoint is a physical robot or Gazebo is transparent

This is the power of the ROS 2 abstraction layer. **The same code works in simulation and reality.**

## Accessing Gazebo on TheConstruct

TheConstruct (theconstructsim.com) is a cloud robotics platform that provides free and paid access to Gazebo and ROS 2.

### Tier 1 Path (Cloud): TheConstruct

1. **Sign up**: Visit theconstructsim.com and create an account (free tier available)
2. **Launch environment**: Create a new ROSject (TheConstruct's term for a cloud ROS 2 environment)
3. **Select Gazebo Harmonic**: Choose the Gazebo Harmonic image
4. **Access via browser**: TheConstruct provides a browser-based IDE and Gazebo GUI
5. **Run roslaunch**: Bring up Gazebo simulations without local installation

**Advantages for you**:
- No installation needed
- All computation in cloud
- Access from any device with a browser
- Pre-configured ROS 2 + Gazebo environment
- Free tier sufficient for learning

### What You'll See

When you launch Gazebo on TheConstruct, you'll see:

**The main viewport**:
- 3D visualization of the robot and environment
- Grid showing world coordinates
- Multiple robots can appear simultaneously

**Sidebar panels**:
- **Simulation**: Play/pause controls, world properties
- **Entities**: Tree of all objects (robots, models, lights)
- **Inspector**: Properties of selected objects (position, velocity, mass)
- **Terminal**: Command-line access for debugging

## Gazebo Basics: Your First Simulation

Here's what happens when you launch a Gazebo world with a robot:

### Step 1: Load World File

A `.sdf` file (Simulation Description Format) describes:

```xml
<!-- Example simplified SDF -->
<sdf version="1.6">
  <world name="default">
    <!-- Gravity pointing down -->
    <gravity>0 0 -9.81</gravity>

    <!-- Ground plane -->
    <model name="ground_plane">
      <link name="link">
        <collision><geometry><plane/></geometry></collision>
        <visual><geometry><plane/></geometry></visual>
      </link>
    </model>

    <!-- Robot model (loaded from URDF) -->
    <model name="my_robot">
      <pose>0 0 1 0 0 0</pose>
      <include>
        <uri>model://my_robot/my_robot.urdf</uri>
      </include>
    </model>
  </world>
</sdf>
```

The server parses this file, creates the ground, loads the robot.

### Step 2: Initialize ROS 2 Bridges

Plugins subscribe to ROS 2 topics:
- `/cmd_vel`: Receives motion commands from your ROS 2 code
- `/odom`: Publishes robot odometry (position, velocity)
- `/scan`: Publishes LIDAR data
- `/camera/image_raw`: Publishes camera images

Your ROS 2 code subscribes to sensor topics and publishes command topics.

### Step 3: Run Simulation Loop

The server updates physics repeatedly:

```
for each timestep (1000 times per second):
  1. Receive messages from ROS 2 topics
  2. Update forces/torques from control commands
  3. Compute new positions, velocities (physics math)
  4. Check collisions
  5. Update sensor outputs
  6. Publish to ROS 2 topics
  7. Render visualization (send to GUI client)
```

Each step is ~1 millisecond of simulated time.

### Step 4: Visualize and Interact

The GUI client displays the result. You see:
- Robot moving in response to your ROS 2 commands
- Sensor data flowing in real time
- Collisions preventing robot from passing through walls
- Gravity pulling objects downward

## Key Concepts Summary

| Concept | Meaning | Example |
|---------|---------|---------|
| **World** | Complete simulation environment (ground, robots, objects) | warehouse_simulation.sdf |
| **Model** | Individual object in world (robot, obstacle, landmark) | my_robot, ground_plane |
| **Link** | Rigid body within a model (no flexibility) | robot_arm_base, robot_arm_link1 |
| **Joint** | Connection between links (allows motion) | robot_arm_joint1 (rotating joint) |
| **Plugin** | Code extending simulator (sensors, controllers) | sensor_plugin, control_plugin |
| **SDF** | XML format describing worlds and models | defines geometry, physics, joints |
| **URDF** | XML format for robot structure (human-readable) | alternative to SDF, commonly used |

## Try With AI

**Setup**: Open ChatGPT (chat.openai.com) or your preferred AI tool and explore Gazebo and robotics simulation.

**Prompt Set 1 (Basic):**
```
What is Gazebo and why do robotics companies use it?
Compare it to other robotics simulators like Unity or CoppeliaSim.
```

**Prompt Set 2 (Intermediate):**
```
Explain how Gazebo's client-server architecture works.
Why is this design useful for robotics simulation?
```

**Prompt Set 3 (Advanced):**
```
I'm writing ROS 2 code to control a robot's arm.
How does the code differ if the robot is in Gazebo simulation vs a physical robot?
What role does the plugin system play?
```

**Expected Outcomes**: You should understand that:
- Gazebo is the industry-standard open-source robotics simulator
- Its client-server architecture separates physics from visualization
- ROS 2 integration makes simulation transparent (same code works for physical and virtual robots)
- TheConstruct cloud makes Gazebo accessible without local installation

**Hands-On Next Step**: In Chapter 9, you'll write robot description files (URDF) and create simulation worlds. In Chapter 12, you'll connect actual ROS 2 code to Gazebo.

---

**Next**: [Chapter 9: Robot Description Formats →](../chapter-9-robot-description/README.md)
