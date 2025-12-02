---
id: chapter-9-robot-description
title: "Chapter 9: Robot Description Formats"
sidebar_position: 9
sidebar_label: "9. Robot Description Formats"
description: "Learn URDF and SDF formats for describing robot structure, properties, and kinematics."
---

# Chapter 9: Robot Description Formats

Before a simulator can move your robot, it needs to know what the robot looks like. **URDF** (Unified Robot Description Format) and **SDF** (Simulation Description Format) are XML languages that describe robot structure: what parts exist (links), how they connect (joints), what they look like (geometry), and how they behave (physics).

Think of a URDF file as a blueprint. You specify a chassis, wheels, sensors, and how they're connected. The simulator reads this blueprint and builds your robot. When you change a joint type from `fixed` to `continuous`, the simulator automatically allows rotation. When you add mass and inertia, physics becomes realistic—your robot no longer floats.

This chapter starts with understanding: what is URDF, how does XML structure encode robot information. Then you'll build your first robot from scratch—a simple two-wheeled mobile base. Then add physics properties so simulation matches reality. Finally, you'll use AI collaboration to accelerate URDF development and catch errors that would cause simulation failures.

---

## Lessons

### Lesson 9.1: Understanding URDF

Your entry into robot description formats. You'll learn what URDF is, why simulators need it, and how XML structure encodes robot information. Key concepts: **links** (rigid bodies), **joints** (connections), **geometry** (visual shapes), **inertia** (physics properties).

**Duration:** 60 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud)

### Lesson 9.2: Building Your First Robot

Write a complete URDF file for a differential-drive robot. You'll define a chassis, two wheels, and a caster wheel. Hands-on practice: structure XML, configure joints, validate in Gazebo. When you finish, you'll have a working robot that you can visualize and simulate.

**Duration:** 75 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud)

### Lesson 9.3: Adding Physical Properties

URDF describes geometry (shape), but simulation needs physics (mass, friction, inertia). In this lesson, you'll add the numbers that make simulation realistic. Without mass, your robot floats. Without inertia, rotation is wrong. You'll learn the formulas, understand the values, and fix your robot to behave like a real object.

**Duration:** 60 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud)

### Lesson 9.4: URDF with AI

Now that you understand manual URDF creation, leverage AI collaboration. Ask an AI tool to generate URDF boilerplate for new robots. Evaluate outputs for common errors (wrong joint types, missing inertia, syntax mistakes). Iterate with AI to refine your robot descriptions quickly.

**Duration:** 60 minutes | **Proficiency:** A2 | **Hardware:** Tier 1 (cloud + AI assistant)

---

## Learning Outcomes

By completing Chapter 9, you will:

- Explain what URDF is and why simulators need robot descriptions
- Identify links, joints, geometry, and inertia in URDF files
- Write complete URDF files for simple robots
- Calculate and configure mass and inertia properties
- Use AI collaboration to accelerate URDF development
- Validate URDF files and debug common errors

---

## Prerequisites

**Chapter 8** (Simulation Concepts) complete. You understand why simulation matters and what Gazebo does.

**Helpful:** Basic XML knowledge. If you've never seen XML before, Lesson 9.1 teaches you enough.

---

## Hardware

| Tier | What You Need | What You Get |
|------|---------------|--------------|
| **1 (Cloud)** | Any laptop | Full course via [The Construct](https://theconstructsim.com) cloud Gazebo |
| **2 (Local)** | Ubuntu 22.04 | Local Gazebo for faster iteration |

---

## Capstone Connection

In Chapter 10, you'll build complete Gazebo worlds. Those worlds contain robots. The URDF files you write here become the building blocks for those worlds. By Chapter 12, you'll connect your URDF-defined robots to ROS 2 controllers. The robot description you write now powers everything that follows.

---

**[← Chapter 8: Simulation Concepts](../chapter-8-why-simulate/README.md)** · **[Start Lesson 9.1 →](./01-understanding-urdf.md)**

**Next:** [Chapter 10: Gazebo Worlds →](../chapter-10-simulation-worlds/README.md)
