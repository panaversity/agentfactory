---
id: module-2-simulation
title: "Module 2: Gazebo/Unity Simulation"
sidebar_position: 2
sidebar_label: "Module 2: Simulation"
description: "Learn robotics simulation using Gazebo Harmonic - from digital twins to ROS 2 integration"
---

# Module 2: Gazebo/Unity Simulation

**Duration**: ~4 weeks (22 lessons) | **Proficiency**: A2 → B1 | **Hardware**: Tier 1 (Cloud)

Welcome to Module 2! You've mastered ROS 2 fundamentals in Module 1. Now you'll learn to simulate robots before deploying to physical hardware—the industry standard for professional robotics development.

## What You'll Learn

By completing this module, you will be able to:

1. **Explain** why simulation precedes physical deployment in professional robotics
2. **Create** robot models using URDF (Unified Robot Description Format)
3. **Build** simulation worlds with physics using SDF
4. **Configure** sensors (camera, LIDAR, IMU) on simulated robots
5. **Connect** Gazebo simulations to ROS 2 using ros_gz_bridge
6. **Complete** a capstone project integrating all skills

## Prerequisites

- **Module 1**: ROS 2 Foundations (all 7 chapters)
- **Basic XML**: Understanding of tags, attributes, nesting
- **Command Line**: Comfortable with terminal operations

## Hardware Requirements

| Tier | Equipment | Required? |
|------|-----------|-----------|
| **Tier 1** | Browser + Cloud | Yes (all content works here) |
| **Tier 2** | Local GPU | Optional (faster iteration) |

**Cloud Path**: All content works via [TheConstruct](https://www.theconstructsim.com/) cloud environment. No local installation required.

## Chapter Overview

### Chapter 8: Why Simulate?
*3 lessons | Layer L1 | Proficiency A2*

Understand the simulation-first philosophy and Gazebo's architecture.

- [8.1 The Digital Twin Concept](./chapter-8-why-simulate/01-digital-twin-concept.md)
- [8.2 Simulation-First Development](./chapter-8-why-simulate/02-simulation-first.md)
- [8.3 Meet Gazebo Harmonic](./chapter-8-why-simulate/03-meet-gazebo.md)

### Chapter 9: Robot Description Formats
*4 lessons | Layer L1→L2 | Proficiency A2*

Create robot models using URDF with links, joints, and physical properties.

- [9.1 Understanding URDF](./chapter-9-robot-description/01-understanding-urdf.md)
- [9.2 Building Your First Robot](./chapter-9-robot-description/02-building-first-robot.md)
- [9.3 Adding Physical Properties](./chapter-9-robot-description/03-adding-physical-properties.md)
- [9.4 URDF with AI](./chapter-9-robot-description/04-urdf-with-ai.md)

### Chapter 10: Building Simulation Worlds
*4 lessons | Layer L1→L2 | Proficiency A2→B1*

Design simulation environments with ground, obstacles, and physics.

- [10.1 SDF World Basics](./chapter-10-simulation-worlds/01-sdf-world-basics.md)
- [10.2 Adding Models from Fuel](./chapter-10-simulation-worlds/02-adding-models-from-fuel.md)
- [10.3 Physics Configuration](./chapter-10-simulation-worlds/03-physics-configuration.md)
- [10.4 World Building with AI](./chapter-10-simulation-worlds/04-world-building-with-ai.md)

### Chapter 11: Sensors in Simulation
*4 lessons | Layer L1→L2 | Proficiency B1*

Add cameras, LIDAR, and IMU sensors to your robots.

- [11.1 Camera Simulation](./chapter-11-sensors-simulation/01-camera-simulation.md)
- [11.2 LIDAR Simulation](./chapter-11-sensors-simulation/02-lidar-simulation.md)
- [11.3 IMU and Contact Sensors](./chapter-11-sensors-simulation/03-imu-contact-sensors.md)
- [11.4 Sensor Debugging and Visualization](./chapter-11-sensors-simulation/04-sensor-debugging-visualization.md)

### Chapter 12: ROS 2 + Gazebo Integration
*4 lessons | Layer L2→L3 | Proficiency B1*

Connect Gazebo to ROS 2 for closed-loop control.

- [12.1 The ros_gz Bridge](./chapter-12-ros2-gazebo-integration/01-ros-gz-bridge.md)
- [12.2 Spawning Robots from ROS 2](./chapter-12-ros2-gazebo-integration/02-spawning-robots.md)
- [12.3 Closed-Loop Control](./chapter-12-ros2-gazebo-integration/03-closed-loop-control.md)
- [12.4 Creating ros_gz Skills](./chapter-12-ros2-gazebo-integration/04-creating-ros-gz-skills.md)

### Chapter 13: Module 2 Capstone
*3 lessons | Layer L4 | Proficiency B1*

Demonstrate integrated learning with a complete simulation project.

- [13.1 Capstone Specification](./chapter-13-capstone/01-capstone-specification.md)
- [13.2 Building the Simulation](./chapter-13-capstone/02-building-simulation.md)
- [13.3 Testing, Validation, and Sim-to-Real Preview](./chapter-13-capstone/03-testing-validation-preview.md)

## Learning Progression

```
Chapter 8 (L1): Understand WHY to simulate
    ↓
Chapter 9 (L1→L2): Create WHAT you simulate (robots)
    ↓
Chapter 10 (L1→L2): Build WHERE you simulate (worlds)
    ↓
Chapter 11 (L1→L2): Add HOW robots perceive (sensors)
    ↓
Chapter 12 (L2→L3): Connect to ROS 2 for control
    ↓
Chapter 13 (L4): Integrate everything in capstone
```

## Skills You'll Create

By completing this module, you'll have 4 reusable skills:

1. **urdf-robot-model**: Create robot descriptions with proper physics
2. **gazebo-world-builder**: Design simulation environments
3. **sensor-simulation**: Configure cameras, LIDAR, IMU
4. **ros2-gazebo-bridge**: Connect Gazebo to ROS 2

## What's Next

After completing Module 2, you're ready for:

- **Module 3**: NVIDIA Isaac Sim — AI-powered simulation with domain randomization
- **Module 4**: VLA/Embodied AI — Vision-language-action models for robot control

---

**Ready to begin?** Start with [Chapter 8: Why Simulate?](./chapter-8-why-simulate/README.md)
