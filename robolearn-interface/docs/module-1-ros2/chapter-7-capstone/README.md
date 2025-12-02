---
id: chapter-7-capstone
title: "Chapter 7: Capstone - Robot Controller"
sidebar_position: 7
sidebar_label: "7. Capstone"
description: "Specification-driven multi-node system integration, system testing and validation"
---

# Chapter 7: Capstone - Robot Controller

This is it. The culmination of everything you've learned: pub/sub, services, custom interfaces, parameters, launch files, debugging. In this capstone project, you'll build a complete multi-node robot control system **from specification**.

But here's what makes this different from previous chapters: **you write the specification FIRST**. Not the code. The specification. You define what your system should do, how nodes should communicate, what parameters it should have, what success looks like. Then you implement from that specification, validate against it, and iterate.

This is how professional roboticists work. Specifications drive implementation. And by the end, you'll have a working robot controller that you designed, built, tested, and validated.

**Duration**: 3 lessons, 4.5 hours total
**Layer Breakdown**: L3: 20%, L4: 80% (specification-first, orchestration of accumulated skills)
**Hardware Tier**: Tier 1 (Turtlesim simulation or MockROS in browser)
**Prerequisites**: Chapters 1-6 (all prior learning)
**Capstone Deliverable**: Complete multi-node ROS 2 system with specification documentation

## Learning Objectives

By the end of this capstone, you will be able to:

- **Write specifications** that clearly define system intent, interfaces, and success criteria
- **Implement systems from specifications** without explicit step-by-step instructions
- **Compose previously-learned skills** (Chapter 4's pub/sub, Chapter 5's services, Chapter 6's parameters and launch files)
- **Debug integration issues** systematically
- **Validate implementations** against their specifications
- **Reflect on design decisions** and articulate trade-offs made
- **Recognize specification quality** as the primary determinant of implementation quality

## Lessons

### Lesson 7.1: Capstone Specification (90 minutes)

Write your specification FIRST. Define what your robot controller should do: What commands should it accept? What data should it publish? How should nodes communicate? What makes success?

You'll fill in a specification template with:
- **Intent**: What problem does this system solve?
- **System Architecture**: How many nodes? What do they do?
- **Interfaces**: Topics, services, parameters (the contract between nodes)
- **Success Criteria**: How do you know it worked?
- **Non-Goals**: What's explicitly NOT in scope (deferred to Module 2)

**Core Concepts**:
- Specification-first thinking (intent before code)
- System architecture documentation
- Clear interface definitions
- Measurable success criteria

---

### Lesson 7.2: Building the Controller (90 minutes)

Now implement. But here's the key: you have a specification guiding you. You're not writing code and hoping it works. You're implementing a contract.

You'll use all the skills from Chapters 4-6:
- Publisher/subscriber nodes (Chapter 4)
- Service servers (Chapter 5)
- Custom interfaces (Chapter 5)
- Parameters for configuration (Chapter 6)
- Launch files to orchestrate everything (Chapter 6)

**Core Concepts**:
- Implementation from specification
- Composing multiple ROS 2 skills
- Integration testing
- Debugging against the specification

---

### Lesson 7.3: Testing, Validation, and Reflection (90 minutes)

Validate that your implementation matches your specification. Does it meet every success criterion? Test each one. Debug failures systematically. Reflect on what was harder than expected, what design decisions mattered, and how you'd improve the design.

This is the **reflection layer**—where learning crystallizes into understanding.

**Core Concepts**:
- Specification validation (does implementation match spec?)
- Systematic testing of success criteria
- Debugging integration issues
- Reflection on design and learning
- Preview of Module 2 concepts

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 10% | Specification template example, structure guidance |
| **L2: AI Collab** | 10% | AI assists with implementation and debugging |
| **L3: Intelligence** | 20% | Composing Chapter 4-6 skills, integration patterns |
| **L4: Spec-Driven** | 80% | FULL specification-first development cycle |

This is **Layer 4**—the deepest layer of the 4-Layer Teaching Method. You're orchestrating accumulated intelligence through clear specifications.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Laptop/Browser)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud ROS 2 (TheConstruct) with turtlesim visualization, full Python execution |
| **2+** | Local ROS 2 | Local installation with faster feedback |

**Capstone Simulation**: Turtlesim (built-in to ROS 2, no extra installation needed). Your controller will command a simulated turtle to move and respond to queries.

## Prerequisites

- **Chapters 1-6** (ALL prior learning—this integrates everything)
- **Understanding of ROS 2 architecture** (nodes, topics, services, parameters)
- **Python programming** (you're writing multi-node systems)
- **Comfortable with debugging** (integration issues are normal; you'll solve them systematically)

## Capstone Project Specification

Here's the template you'll fill in. Your system should:

```
TURTLE ROBOT CONTROLLER SPECIFICATION

Intent:
  Create a multi-node system that controls a turtlesim turtle to achieve
  autonomous navigation goals. The system accepts goal positions via service,
  publishes continuous status, and handles simulated obstacles.

System Architecture:
  - Navigator Node: Accepts navigation goals, computes velocity commands
  - Status Monitor Node: Publishes robot state (position, velocity, battery)
  - Obstacle Detector Node: Simulates obstacle detection, publishes warnings

Interfaces:
  Service: /navigate_to (request: x, y, theta; response: success, time_taken)
  Topic: /robot/status (RobotStatus message: position, velocity, battery)
  Topic: /obstacles (sensor_msgs/LaserScan mock data)

Success Criteria:
  - Turtle reaches goal position within 2 seconds
  - Status published every 100ms
  - System handles 3+ obstacles without crashing
  - Launch file starts all nodes with coordinated parameters

Non-Goals:
  - Real obstacle avoidance (mock sensor only)
  - Real LIDAR simulation (sensor_msgs/LaserScan is simulated)
  - Multiple robots (single turtle only)
```

Your job: Fill in the blanks, implement per spec, validate, reflect.

## Mastery Gate: Capstone Success

You've successfully completed the capstone when:

- ✅ **Specification is clear**: Someone else could implement it from your spec
- ✅ **Implementation is complete**: All 3+ nodes are running and connected
- ✅ **Communication works**: Data flows correctly through topics and services
- ✅ **All success criteria met**: Test each one explicitly
- ✅ **System is launchable**: One `ros2 launch` command starts everything
- ✅ **You can explain it**: Why you made each design decision
- ✅ **You learned something**: What was harder than expected? What surprised you?

## What You've Accomplished

By completing Module 1, you've:

1. **Understood embodied intelligence** (Chapters 1-2)
2. **Explored ROS 2's architecture** (Chapter 3)
3. **Written ROS 2 nodes** in Python (Chapters 4-6)
4. **Created 4 reusable skills**:
   - `ros2-publisher-subscriber`
   - `ros2-service-pattern`
   - `ros2-custom-interfaces`
   - `ros2-launch-system`
5. **Designed and built a complete system** from specification (Chapter 7)
6. **Applied the 4-Layer Teaching Method** from manual foundation to specification-driven integration

**You're now a ROS 2 developer.** Not advanced yet—but you understand the fundamentals deeply enough to build real systems.

## What Comes Next?

Module 2 builds on this foundation:
- **URDF**: Describe robot structure and kinematics
- **Gazebo Simulation**: More realistic physics simulation
- **Perception**: Add cameras, LIDAR processing
- **Navigation Stack**: Autonomous path planning
- **Sim-to-Real Transfer**: Deploy from simulation to physical robot

But you'll apply the same 4-Layer progression: conceptual foundation → hands-on collaboration → reusable patterns → spec-driven integration.

## Navigation

**Previous Chapter**: [← Chapter 6: Building Systems](../chapter-6-building-systems/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 7.1**: [Capstone Specification →](./01-capstone-spec.md)

---

## Capstone Reflection Prompts

As you work through the capstone, reflect on these questions:

1. **Specification First**: Did writing the spec before coding help? How?
2. **Composition**: Which Chapter 4-6 skills were most useful? Did you use them as expected?
3. **Debugging**: What went wrong during integration? How did you fix it?
4. **Design Trade-offs**: Why did you choose topics vs services for different communications?
5. **System Thinking**: How did you think differently about design compared to Chapter 4?
6. **What's Next**: How would you extend this system? What would Module 2 add?

Your answers to these questions show how deeply you've learned.

---

**Congratulations on reaching the capstone. You've come a long way.**

From not knowing what ROS 2 is (Chapter 3) to building complete systems (Chapter 7) in just 5 weeks.

**Now build something amazing.**
