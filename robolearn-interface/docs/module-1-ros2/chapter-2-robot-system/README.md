---
id: chapter-2-robot-system
title: "Chapter 2: The Robot System"
sidebar_position: 2
sidebar_label: "2. Robot System"
description: "Understand sensors, actuators, control loops, and why middleware is essential"
---

# Chapter 2: The Robot System

Now that you understand embodied intelligence, let's look inside a robot. What makes it perceive? How does it move? What's the "glue" that coordinates all the pieces?

In this chapter, you'll explore the three essential layers of every robot:
1. **Perception** (sensors that tell the robot what's happening)
2. **Actuation** (motors and joints that let the robot move)
3. **Coordination** (middleware that ties everything together)

You'll also take a crucial step: identifying your own hardware tier. This determines what you can build in future chapters and guides which exercises are available to you.

**Duration**: 4 lessons, 3 hours total
**Layer Breakdown**: L1: 85%, L2: 15% (brief AI perspective on middleware)
**Hardware Tier**: Tier 1 (conceptual diagrams + interactive tier selector)
**Prerequisites**: Chapter 1 (physical AI foundations)

## Learning Objectives

By the end of this chapter, you will be able to:

- **Categorize sensor types** (proprioceptive vs. exteroceptive) and understand what each type tells the robot
- **Explain actuator control** (servo motors, control loops, feedback) and why open-loop control fails
- **Understand why middleware exists** and what problems it solves in multi-node systems
- **Identify your hardware tier** (Tier 1-4) and understand what learning paths are accessible to you
- **Recognize that robot systems are ecosystems**, not monolithic units

## Lessons

### Lesson 2.1: How Robots See (Sensors) (45 minutes)

Explore the sensor ecosystem. From encoders that tell you joint position to LIDARs that map the world, sensors are your robot's eyes, ears, and proprioceptive sense.

**Core Concepts**:
- Sensor types (proprioceptive: encoders, IMU; exteroceptive: LIDAR, camera, force/torque)
- Sensor data flow and uncertainty

---

### Lesson 2.2: How Robots Move (Actuators) (45 minutes)

Motors aren't just "spinners." Understand servo control, feedback loops, joint limits, and why a motor's specification sheet tells you what movements are possible.

**Core Concepts**:
- Actuator types (servo motors, stepper motors, brushless DC) and their properties
- Control loops (commanded position → motor → feedback → error correction)

---

### Lesson 2.3: The Glue (Middleware) (45 minutes)

Why does a robot need ROS 2? Explore the problem of coordinating 10+ sensors, motors, and controllers without a unified framework.

**Core Concepts**:
- The middleware problem (N sensors × M readers = complexity explosion)
- ROS 2 as the solution (pub/sub, standard interfaces, decoupling)

---

### Lesson 2.4: Your Hardware Tier (45 minutes)

Take a self-assessment and identify which tier of hardware you have access to. This determines what content is available and how fast you can run exercises.

**Core Concepts**:
- Hardware tier classification (Tier 1: laptop/cloud, Tier 2: GPU, Tier 3: Jetson, Tier 4: physical robot)
- Learning path selection based on equipment

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 85% | Direct teaching with diagrams, sensor visualizations, motor specifications, architecture comparisons |
| **L2: AI Collab** | 15% | Brief AI perspective on middleware design trade-offs (your first taste of AI collaboration) |
| **L3: Intelligence** | 0% | Not yet—too early for creating reusable patterns |
| **L4: Spec-Driven** | 0% | Not yet—specifications require coding experience |

This chapter remains primarily **manual foundation**, but you'll get a brief introduction to AI collaboration in Lesson 2.3.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Laptop/Browser)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | All lessons via cloud/browser simulation, hardware tier selector |
| **2** | RTX GPU Workstation | Tier 1 + local Gazebo simulation (later chapters) |
| **3** | Jetson Orin Kit | Tier 2 + real sensor data (later chapters) |
| **4** | Physical Robot (Unitree Go2/G1) | Tier 3 + real-world testing (capstone and Module 2+) |

**All Tier 1 content is fully accessible.** Higher tiers enable faster execution and more advanced experiments.

## Prerequisites

- **Chapter 1** (physical AI foundations)
- **No hardware needed** (all diagrams and simulators are browser-based)
- **Basic physics intuition** (gravity, motion, forces—nothing formal)

## Mastery Gate

Before proceeding to **Chapter 3**, you should be able to:

- **Match 5 sensor types** to their use cases (e.g., "I need to know the robot's orientation" → IMU)
- **Explain one actuator** (e.g., servo motor) and its control mechanism
- **Answer this question**: "Why is ROS 2 better than writing custom code for each sensor/motor combination?"
- **Identify your hardware tier** from the Lesson 2.4 self-assessment
- **Understand that hardware tier affects content availability** in Chapters 4-7

If you can do these, you're ready for Chapter 3.

---

## Navigation

**Previous Chapter**: [← Chapter 1: Physical AI](../chapter-1-physical-ai/README.md)

**Next Chapter**: [Chapter 3: Meet ROS 2 →](../chapter-3-meet-ros2/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 2.1**: [How Robots See →](./01-how-robots-see.md)
