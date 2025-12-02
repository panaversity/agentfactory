---
id: chapter-13-capstone
title: "Chapter 13: Module 2 Capstone Project"
sidebar_position: 13
sidebar_label: "13. Module 2 Capstone"
description: "Design and implement a complete robot simulation project meeting your own specification"
---

# Chapter 13: Module 2 Capstone Project

You've built robots in URDF, created simulation worlds, configured sensors, and connected everything to ROS 2. Now comes the capstone: designing and implementing a complete simulation project where your robot completes a meaningful task.

This is where the 4-Layer Teaching Method reaches its peak—**Layer 4 (Spec-Driven)**. You write the specification FIRST, then use accumulated skills from Chapters 8-12 to implement it. The specification is your contract: if your simulation meets all criteria, you've succeeded. This mirrors professional robotics development where specifications drive design, not the reverse.

Your capstone specification can be anything your simulation supports:
- **Delivery robot** navigating an obstacle-filled warehouse
- **Inspection robot** following a path while collecting sensor data
- **Sorting robot** identifying and manipulating objects
- **Exploration robot** mapping an unknown environment

The key constraint: you define the specification, you implement it, you validate it works.

**Duration**: 3 lessons, 3.5 hours total
**Layer Breakdown**: L4 (Spec-Driven) 100%
**Hardware Tier**: Tier 1 (cloud), Tier 2 (local optional)
**Prerequisites**: Chapter 12 (ROS 2 + Gazebo Integration), Chapters 8-11 (all core skills)
**Mastery Gate**: Working simulation meeting your own specification

## Learning Objectives

By the end of this chapter, you will be able to:

- **Write clear specifications** for simulation projects (intent, constraints, success criteria)
- **Design robot requirements** (links, joints, sensors, actuators)
- **Define world requirements** (environment, obstacles, initial conditions)
- **Implement using accumulated skills** from prior chapters (reference Chapter 9 `urdf-robot-model`, Chapter 10 `gazebo-world-builder`, Chapter 11 `sensor-simulation`, Chapter 12 `ros2-gazebo-bridge`)
- **Test and validate** your simulation against specification (multiple trials, measure success criteria)
- **Anticipate sim-to-real gaps** (preview for Module 3)
- **Complete a capstone project** that integrates all Module 2 learning

## Lessons

### Lesson 13.1: Capstone Specification (60 minutes)

Learn how to write clear, testable specifications for simulation projects. Specifications define what you want to build BEFORE you build it. A good specification has intent (what problem are you solving?), success criteria (how do you know it works?), and constraints (what are your limitations?).

**Core Concepts**:
- Specification structure: intent, requirements, constraints, success criteria
- Robot requirements: links, joints, sensors needed for the task
- World requirements: environment layout, obstacles, initial conditions
- Sensor requirements: which sensors does the task require?
- Behavior requirements: what should the robot do?
- Success criteria: measurable outcomes you'll validate against
- Layer: L4 (Spec-Driven)

---

### Lesson 13.2: Building the Simulation (90 minutes)

Implement your capstone using skills accumulated from Chapters 9-12. You won't build from scratch—you'll compose:
- Chapter 9 `urdf-robot-model` skill (create robot URDF)
- Chapter 10 `gazebo-world-builder` skill (create SDF world)
- Chapter 11 `sensor-simulation` skill (configure sensors)
- Chapter 12 `ros2-gazebo-bridge` skill (connect to ROS 2)

Testing happens incrementally: verify robot loads, verify world loads, verify sensors output data, verify ROS 2 bridge connectivity, verify behavior implementation.

**Core Concepts**:
- Implementation workflow referencing accumulated skills
- Incremental testing (robot → world → sensors → bridge → behavior)
- Debugging simulation issues systematically
- Integration testing (all components together)
- Performance optimization (physics stability, simulation speed)
- Layer: L4 (Spec-Driven implementation)

---

### Lesson 13.3: Testing, Validation, and Sim-to-Real Preview (60 minutes)

Validate your capstone against the specification you wrote in Lesson 13.1. Run your simulation multiple times, measure success criteria, iterate on failures. This is where you confirm your design works.

The final section previews the sim-to-real gap: why simulations differ from real robots, and how you'll bridge that gap in Module 3 (Isaac Sim).

**Core Concepts**:
- Validation methodology (multiple trials, statistical measures)
- Success criteria verification checklist
- Iteration on failures (what went wrong and why)
- Simulation vs. reality gaps (friction, sensor noise, latency)
- Module 2 completion celebration (you've built a complete simulation pipeline)
- Preview: Module 3 and sim-to-real transfer learning
- Layer: L4 (Spec-Driven validation)

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 0% | Foundation in Chapters 8-11 |
| **L2: AI Collab** | 0% | Collaboration patterns in Chapters 9-12 |
| **L3: Intelligence** | 0% | Skill encapsulation in Chapter 12 |
| **L4: Spec-Driven** | 100% | Specification-driven implementation and validation |

This chapter is **pure L4**: specification FIRST, implementation SECOND, validation THIRD. No lecturing, no passive content. Only action.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Cloud Gazebo via TheConstruct)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Full capstone via cloud Gazebo, all exercises supported |
| **2** | RTX GPU | Local Gazebo for faster iteration, optional for performance |

All core capstone work supported on Tier 1. Tier 2 optional for faster simulation cycles.

## Prerequisites

- **Chapter 12** (ROS 2 + Gazebo Integration) — you must understand bridge configuration
- **Chapter 11** (Sensors in Simulation) — you must know how to configure sensors
- **Chapter 10** (Building Simulation Worlds) — you must know SDF world structure
- **Chapter 9** (Robot Description Formats) — you must know URDF syntax
- **Module 1** (ROS 2 Fundamentals) — you must understand nodes, topics, services, launch files

If any of these are unclear, review the previous chapter before starting the capstone.

## Mastery Gate

Your capstone is complete when:

- ✅ **Specification written** (intent, constraints, success criteria clear)
- ✅ **Robot loads in Gazebo** (URDF valid, no errors)
- ✅ **World loads correctly** (physics stable, obstacles present)
- ✅ **Sensors output data** (camera/LIDAR/IMU topics publishing)
- ✅ **ROS 2 bridge configured** (bridged topics appear in `ros2 topic list`)
- ✅ **Behavior implemented** (robot executes the specified task)
- ✅ **Validation passes** (all success criteria from spec verified in 3+ trials)
- ✅ **Can explain sim-to-real gaps** (understand why sim differs from reality)

If all checks pass, you've completed Module 2.

---

## Key Patterns

### Spec-Driven Development (L4 Foundation)

```
Write Specification (Intent, Constraints, Success Criteria)
        ↓
Design Architecture (What robot? What world? What sensors?)
        ↓
Implement Using Skills (Compose URDF, SDF, Bridge)
        ↓
Test Incrementally (Robot → World → Sensors → Bridge)
        ↓
Validate Against Spec (Run 3+ trials, measure criteria)
        ↓
Iterate on Failures (Fix and re-validate)
        ↓
Specification Met ✓
```

### Skills Composition (Using L3 Accumulated Intelligence)

```
Chapter 9 Skill       Chapter 10 Skill       Chapter 11 Skill       Chapter 12 Skill
urdf-robot-model  +   gazebo-world-builder + sensor-simulation  +   ros2-gazebo-bridge
    ↓                      ↓                      ↓                      ↓
    └──────────────────────┴──────────────────────┴──────────────────────┘
                            ↓
                  Capstone Implementation
```

---

## Navigation

**Previous Chapter**: [← Chapter 12: ROS 2 + Gazebo Integration](../chapter-12-ros2-gazebo-integration/README.md)

**Module Overview**: [← Back to Module 2](../README.md)

**Next Module**: [Module 3: NVIDIA Isaac Sim →](../../module-3-isaac/README.md)

**Start Lesson 13.1**: [Capstone Specification →](./01-capstone-specification.md)
