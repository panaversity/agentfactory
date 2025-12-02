---
id: chapter-10-simulation-worlds
title: "Chapter 10: Building Simulation Worlds"
sidebar_position: 10
sidebar_label: "10. Building Simulation Worlds"
description: "Master Gazebo world creation using SDF format, model placement, physics configuration, and AI-accelerated workflows."
---

# Chapter 10: Building Simulation Worlds

Before a robot can move, it needs a world to move through. **SDF** (Simulation Description Format) defines not just the robot, but the entire environment: ground planes, obstacles, lighting, physics parameters, and sensor models.

Think of a Gazebo world file as a 3D stage. Your URDF-described robot is an actor on that stage. The stage needs scenery (walls, tables, trees), lighting (sun, lamps), physics rules (gravity direction, collision properties), and sometimes even other robots or interactive objects.

This chapter teaches you to build complete simulation worlds. You'll start by understanding SDF structure and world properties. Then you'll populate worlds with pre-built models from Gazebo Fuel (an online repository of 3D models). You'll configure physics engines for realistic behavior. Finally, you'll use AI collaboration to accelerate world design and iterate toward production-quality simulations.

---

## Lessons

### Lesson 10.1: SDF World Basics

Your entry into world description. You'll learn what SDF is, how it differs from URDF, and what components make up a complete world file. Key concepts: **world structure**, **ground plane**, **lighting**, **physics parameters**, **gravity configuration**.

**Duration:** 60 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud)

### Lesson 10.2: Adding Models from Fuel

Gazebo Fuel is an online repository of pre-built 3D models (tables, chairs, buildings, vegetation, robots). Instead of creating every object from scratch, you'll learn to find, include, and position models in your worlds. Hands-on: search Fuel, write include statements in SDF, position and scale models accurately.

**Duration:** 60 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud)

### Lesson 10.3: Physics Configuration

SDF files can specify physics engines (DART, ODE, Bullet), step sizes, friction coefficients, and contact parameters. Wrong physics settings cause instability: robots fall through floors, objects jitter violently, or simulations explode. You'll learn the parameters, understand what each does, and debug common physics issues.

**Duration:** 60 minutes | **Proficiency:** B1 | **Hardware:** Tier 1 (cloud)

### Lesson 10.4: World Building with AI

Now that you understand manual world creation, leverage AI collaboration. Describe desired environments in natural language, evaluate AI-generated SDF for correctness and completeness, iterate with AI to refine world configurations quickly. This lesson demonstrates the power of specification-first thinking in simulation design.

**Duration:** 60 minutes | **Proficiency:** B1 | **Hardware:** Tier 1 (cloud + AI assistant)

---

## Learning Outcomes

By completing Chapter 10, you will:

- Understand SDF structure and how it describes simulation environments
- Create SDF world files with ground planes, lighting, and physics configuration
- Use Gazebo Fuel to populate worlds with pre-built 3D models
- Position and scale models accurately in simulation space
- Configure physics engines for realistic robot-world interaction
- Debug common physics instability issues
- Use AI collaboration to accelerate world design and iterate effectively

---

## Prerequisites

**Chapter 9** (Robot Description Formats) complete. You understand URDF structure and can describe robots. Familiarity with XML notation helpful but not required—Lesson 10.1 teaches what you need.

**Recommended:** Chapter 8 (Simulation Concepts) for motivation about why simulation worlds matter.

---

## Hardware

| Tier | What You Need | What You Get |
|------|---------------|--------------|
| **1 (Cloud)** | Any laptop | Full course via [The Construct](https://theconstructsim.com) cloud Gazebo |
| **2 (Local)** | Ubuntu 22.04 + RTX GPU | Local Gazebo for faster iteration, NVIDIA Isaac Sim preparation |

---

## Module Progression

**Chapter 9 → Chapter 10 → Chapter 12 → Chapter 13**

1. **Chapter 9** teaches robot descriptions (URDF)
2. **Chapter 10** teaches environment descriptions (SDF) — *You are here*
3. **Chapter 12** connects robots and worlds to ROS 2
4. **Chapter 13** capstone: Complete autonomous systems in realistic simulation

Your URDF robots from Chapter 9 move through SDF worlds you build here. By Chapter 12, you'll publish ROS 2 commands that control those robots in these worlds.

---

## Key Concepts

### SDF vs URDF
- **URDF**: Describes individual robots only (links, joints, geometry, properties)
- **SDF**: Describes complete worlds (robots + environment + physics + lighting)

### Gazebo Fuel
Online model repository at https://app.gazebosim.org/fuel containing thousands of 3D models (free and open-source)

### Physics Engines
- **DART** (default, most stable)
- **ODE** (open dynamics engine)
- **Bullet** (physics engine)

Each has different stability characteristics and computation costs.

### Sim-to-Real Transfer
Simulation physics must approximate real-world physics accurately, or control algorithms trained in simulation will fail on real robots. This chapter emphasizes matching simulation to reality.

---

**[← Chapter 9: Robot Description](../chapter-9-robot-description/README.md)** · **[Start Lesson 10.1 →](./01-sdf-world-basics.md)**

**Next:** [Chapter 11: Sensors in Simulation →](../chapter-11-sensors-simulation/README.md)
