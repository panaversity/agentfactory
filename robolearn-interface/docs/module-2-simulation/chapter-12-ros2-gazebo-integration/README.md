---
id: chapter-12-ros2-gazebo-integration
title: "Chapter 12: ROS 2 + Gazebo Integration"
sidebar_position: 12
sidebar_label: "12. ROS 2 + Gazebo Integration"
description: "Connecting ROS 2 to Gazebo for closed-loop robot control and simulation"
---

# Chapter 12: ROS 2 + Gazebo Integration

You've built robots in URDF and created Gazebo worlds. Now comes the critical step: connecting them. A robot in Gazebo is just a 3D model until ROS 2 can command it and read its sensors. The `ros_gz_bridge` is the translation layer that connects ROS 2 topics to Gazebo entity properties and sensor outputs.

In this chapter, you'll master the bridge that brings your robots to life. You'll configure topic mapping between ROS 2 and Gazebo message types, spawn robots from ROS 2 launch files, implement closed-loop control (commanding motion and reading feedback), and crystallize recurring patterns into reusable skills.

By the end, you'll understand how ROS 2 applications communicate with physics-based simulators—a fundamental skill for developing autonomous systems before deploying to hardware.

**Duration**: 4 lessons, 4.5 hours total
**Layer Breakdown**: L2: 75% (AI collaboration), L3: 25% (Intelligence Design)
**Hardware Tier**: Tier 1 (cloud Gazebo), Tier 2 (local GPU optional)
**Prerequisites**: Chapter 11 (Sensors in Simulation), Module 1 (ROS 2 fundamentals)
**Reusable Skills Created**: `ros2-gazebo-bridge`, `closed-loop-control`

## Learning Objectives

By the end of this chapter, you will be able to:

- **Configure ros_gz_bridge** to map ROS 2 topics to Gazebo message types
- **Write YAML configuration files** for complex topic bridging scenarios
- **Spawn robot models** from ROS 2 using the ros_gz spawn service
- **Implement velocity control** via cmd_vel commanding and sensor feedback loops
- **Create simple reactive behaviors** (obstacle avoidance, stop-on-obstacle)
- **Visualize robot state** in RViz2 with real-time sensor data
- **Design and document reusable skills** for ROS 2-Gazebo integration patterns

## Lessons

### Lesson 12.1: The ros_gz Bridge (75 minutes)

Learn the architecture of ros_gz_bridge, how it translates between ROS 2 and Gazebo message types, and configure topic bridging for your first robot-simulator connection.

**Core Concepts**:
- ros_gz_bridge purpose and architecture
- Bridge syntax: `/TOPIC@ROS_MSG@GZ_MSG`
- Direction: `[gz_to_ros]`, `[ros_to_gz]`, `[bidirectional]`
- Command-line usage vs YAML configuration
- Common message type mappings
- Verification and debugging
- Layer: L2 (AI collaboration for troubleshooting)

---

### Lesson 12.2: Spawning Robots from ROS 2 (60 minutes)

Instead of pre-placing robots in Gazebo worlds, spawn them dynamically from ROS 2. This enables modular testing—one world, many robot configurations launched on demand.

**Core Concepts**:
- ros_gz spawn service interface
- URDF to SDF conversion
- Launch file integration (Python launch API)
- robot_description parameter and topic
- Joint state publishing
- Position and orientation parameters
- Layer: L2 (AI collaboration for launch configuration)

---

### Lesson 12.3: Closed-Loop Control (75 minutes)

Implement the control loop: sense from Gazebo, decide in ROS 2, act back to Gazebo. Build a simple obstacle avoidance behavior that demonstrates all three components working together.

**Core Concepts**:
- Control loop architecture (sense → decide → act)
- Publishing velocity commands via cmd_vel
- Subscribing to sensor feedback (LIDAR scans, images)
- Reactive behavior logic (thresholds, decision trees)
- Manual teleoperation with keyboard
- Visualization in RViz2
- Layer: L2 (AI collaboration for behavior design and debugging)

---

### Lesson 12.4: Creating ros_gz Skills (60 minutes)

Move beyond one-off solutions to reusable intelligence. Crystallize the patterns from Lessons 12.1-12.3 into documented skills that you and others can apply to future projects.

**Core Concepts**:
- Pattern recognition (what recurs across projects?)
- Skill design using Persona + Questions + Principles
- Documenting decision points and variations
- Cross-project reusability
- Skill composition (when to combine vs separate)
- Layer: L3 (Intelligence Design, creating reusable components)

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 0% | Foundation in Chapter 11 |
| **L2: AI Collab** | 75% | Bridge configuration, spawn logic, behavior design, AI partnership in debugging |
| **L3: Intelligence** | 25% | Skill documentation, pattern encapsulation, reusable components |
| **L4: Spec-Driven** | 0% | Not this chapter (foundation for capstone in Chapter 13) |

This chapter emphasizes **collaborative problem-solving with AI** (L2) as you master complex configuration, then **crystallizing patterns into reusable skills** (L3) that compound across projects.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Cloud Gazebo via TheConstruct)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud Gazebo with ros_gz_bridge, all exercises supported |
| **2** | RTX GPU | Local Gazebo for faster iteration, optional for advanced physics |

All core exercises work on Tier 1. Tier 2 optional for development speed.

## Prerequisites

- **Chapter 11** (Sensors in Simulation)
- **Module 1** (ROS 2 fundamentals: nodes, topics, messages, launch files)
- **Comfort with SDF/URDF** and robot configuration

## Mastery Gate

Before proceeding to **Chapter 13** (Capstone Project), you should be able to:

- **Configure ros_gz_bridge** via YAML for multiple topics with proper direction
- **Verify bridge connectivity** using `ros2 topic list` and `gz topic list`
- **Write launch files** that spawn robots with proper URDF/SDF conversion
- **Implement basic closed-loop control** (command motion, read feedback)
- **Build simple reactive behaviors** with decision logic and sensor thresholds
- **Visualize robot state** in RViz2 with TF frames and sensor data
- **Document a reusable skill** with clear Persona, Questions, and Principles
- **Apply AI collaboration** to troubleshoot integration issues

If you can do these, you're ready for the capstone project.

---

## Key Patterns

### Bridge Pattern (Topic Translation)
```
ROS 2 Topic          ros_gz_bridge          Gazebo Topic
/cmd_vel             (geometry_msgs/msg/ →  /cmd_vel
(Publisher)          Twist @ gz.msgs.Twist)  (Subscriber)
```

### Spawn Pattern (Dynamic Robot Addition)
```
Launch File → URDF/SDF → spawn_entity Service → Gazebo World
              (Description)      (Instantiate)        (Active Robot)
```

### Control Pattern (Closed-Loop Behavior)
```
Gazebo Sensors → ROS 2 Subscription → Decision Logic → ROS 2 Publish → Bridge → Gazebo Command
(LIDAR/Camera)       (Read Feedback)    (Compute)      (cmd_vel)       (Act)  (Execute)
```

---

## ros_gz Bridge Command Reference

**Command-line single topic**:
```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

**YAML configuration file**:
```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

**Common message mappings**:
- `geometry_msgs/msg/Twist` ↔ `gz.msgs.Twist` (velocity commands)
- `sensor_msgs/msg/Image` ↔ `gz.msgs.Image` (camera feeds)
- `sensor_msgs/msg/LaserScan` ↔ `gz.msgs.LaserScan` (LIDAR data)
- `std_msgs/msg/Float64` ↔ `gz.msgs.Double` (scalar values)

---

## Navigation

**Previous Chapter**: [← Chapter 11: Sensors in Simulation](../chapter-11-sensors-simulation/README.md)

**Next Chapter**: [Chapter 13: Capstone Project →](../chapter-13-capstone/README.md)

**Module Overview**: [← Back to Module 2](../README.md)

**Start Lesson 12.1**: [The ros_gz Bridge →](./01-ros-gz-bridge.md)
