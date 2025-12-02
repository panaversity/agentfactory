---
id: module-1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 1
sidebar_label: "Module 1: ROS 2"
description: "Foundation module covering Physical AI concepts and ROS 2 fundamentals across 5 weeks"
---

# Module 1: The Robotic Nervous System (ROS 2)

Welcome to the foundation of your Physical AI journey. Over the next 5 weeks, you'll bridge the gap between software AI (like ChatGPT) and embodied intelligence (robots that move and act in the physical world). You'll explore why robots are fundamentally different from software, understand the hardware and middleware that makes robotics possible, and gain hands-on experience with ROS 2—the middleware that powers professional robotics systems worldwide.

This module is designed for students with Python programming knowledge but no robotics background. Whether you have a laptop, a cloud environment, or access to expensive hardware, we've designed every lesson to be completable on Tier 1 (your laptop or a browser). You'll progress from conceptual foundations through hands-on ROS 2 programming to a real capstone project that demonstrates multi-node system integration.

## Learning Objectives

By completing Module 1, you will be able to:

- **Distinguish embodied AI from software AI** and explain why physical world constraints fundamentally change how we develop robots
- **Understand robot architecture**: sensors, actuators, control loops, and why middleware coordination is essential
- **Master ROS 2 communication patterns**: publish/subscribe messaging, service request/response, and custom interfaces
- **Design and implement multi-node systems** using Python, parameters, launch files, and systematic debugging
- **Apply specification-driven development** to robotics projects through the capstone system integration exercise

## Module Structure

This module contains **7 chapters** organized around a progressive pedagogical arc:

- **Chapters 1-2 (Week 1-2)**: Foundation concepts — no coding yet
- **Chapter 3 (Week 2)**: ROS 2 exploration through command-line interface
- **Chapters 4-5 (Week 3-4)**: Hands-on Python coding with ROS 2
- **Chapter 6 (Week 4-5)**: System integration and debugging
- **Chapter 7 (Week 5)**: Capstone project demonstrating full integration

## Chapters at a Glance

### Chapter 1: What is Physical AI?
**Week 1 | 3 Lessons | 2.25 hours | Tier 1 (Conceptual)**

Why are robots different from ChatGPT? Explore embodied intelligence, physical constraints (gravity, latency, safety), and the humanoid robotics ecosystem. No coding—just foundational thinking.

[→ Start Chapter 1](./chapter-1-physical-ai/README.md)

---

### Chapter 2: The Robot System
**Week 1-2 | 4 Lessons | 3 hours | Tier 1 (Conceptual + Interactive)**

Learn how robots perceive (sensors), move (actuators), and coordinate (middleware). Understand your hardware tier and how it affects what you can build.

[→ Start Chapter 2](./chapter-2-robot-system/README.md)

---

### Chapter 3: Meet ROS 2
**Week 2 | 4 Lessons | 4 hours | Tier 1 (Cloud ROS 2)**

Set up your ROS 2 environment and explore it through command-line tools. Control a robot (turtlesim), list nodes and topics, call services, modify parameters. No Python coding yet—pure exploration.

[→ Start Chapter 3](./chapter-3-meet-ros2/README.md)

---

### Chapter 4: Your First ROS 2 Code
**Week 3 | 4 Lessons | 4 hours | Tier 1 (Cloud ROS 2)**

Write your first Python ROS 2 nodes. Create publisher and subscriber nodes, understand callbacks, build packages, and collaborate with AI to extend your work.

[→ Start Chapter 4](./chapter-4-first-code/README.md)

---

### Chapter 5: Communication Mastery
**Week 3-4 | 4 Lessons | 4 hours | Tier 1 (Cloud ROS 2)**

Master ROS 2's communication patterns: services, custom message types, and design trade-offs. Learn when to use topics vs services through practical design exercises.

[→ Start Chapter 5](./chapter-5-communication/README.md)

---

### Chapter 6: Building Robot Systems
**Week 4-5 | 3 Lessons | 3 hours | Tier 1 (Cloud ROS 2)**

Tie everything together: configurable parameters, launch files that start multiple nodes, and systematic debugging of multi-node systems.

[→ Start Chapter 6](./chapter-6-building-systems/README.md)

---

### Chapter 7: Capstone - Robot Controller
**Week 5 | 3 Lessons | 4.5 hours | Tier 1 (Turtlesim Simulation)**

Write a specification FIRST, then implement a complete multi-node robot controller system. Integrate pub/sub, services, parameters, and launch files. Validate against your specification.

[→ Start Chapter 7](./chapter-7-capstone/README.md)

---

## Teaching Approach: The 4-Layer Method

This module progressively introduces more complex reasoning:

| **Layer** | **What Happens** | **Your Role** | **Chapters** |
|-----------|-----------------|---------------|------------|
| **L1: Manual Foundation** | Instructor teaches directly, you practice manually | Learn mental models | 1-2 |
| **L2: AI Collaboration** | You collaborate with AI through dialogue and iteration | Prompt, validate, iterate | 3-5 |
| **L3: Intelligence Design** | Pattern recognition, creating reusable skills | Generalize knowledge | 5-6 |
| **L4: Spec-Driven Integration** | Design systems through specifications, orchestrate components | Architect solutions | 7 |

You experience these layers through **action**, not through studying them. The framework remains invisible—you'll discover it through doing.

## Hardware Tiers & Your Learning Path

All Module 1 content is accessible to **Tier 1 students** (laptop/cloud only). Here's what each tier enables:

| **Tier** | **Equipment** | **Module 1 Path** | **Future Capability** |
|----------|---------------|-------------------|----------------------|
| **Tier 1** | Laptop + Browser | Cloud ROS 2 (TheConstruct), Pyodide Python, MockROS simulation | Master ROS 2 concepts (100% of Module 1) |
| **Tier 2** | RTX GPU Workstation | Local ROS 2 + Gazebo, faster execution | Module 2-3 locally with simulation |
| **Tier 3** | Jetson Orin Kit | Edge deployment, real sensors | Module 4 edge AI, real perception |
| **Tier 4** | Physical Robot (Unitree Go2/G1) | Everything above + real-world testing | Full embodied AI, sim-to-real transfer |

**No matter your tier, you can complete all of Module 1.** Higher tiers just execute faster and enable future modules.

Identify your tier in **Chapter 2, Lesson 4**, and the system will personalize content recommendations.

## What You'll Create

By the end of Module 1, you'll have:

1. **Conceptual Foundation**: Understand embodied intelligence and why robots require different thinking than software
2. **ROS 2 Proficiency**: Master publish/subscribe, services, custom interfaces, parameters, and launch systems
3. **4 Reusable Skills**: Components you can apply to future projects
   - `ros2-publisher-subscriber`
   - `ros2-service-pattern`
   - `ros2-custom-interfaces`
   - `ros2-launch-system`
4. **Capstone System**: A working multi-node robot controller system controlled through specifications

## Time Commitment

- **Total Duration**: 5 weeks
- **Lessons Per Week**: 5 lessons (Week 1: 3, Weeks 2-5: 4 each)
- **Per Lesson**: 45-60 minutes
- **Per Chapter**: 3-4.5 hours
- **Total Module Time**: 25-30 hours of active learning

## Prerequisites

- **Python programming**: Functions, classes, loops (you should be comfortable writing Python code)
- **Linux/Terminal basics**: `ls`, `cd`, `pwd`, text editing (you should be comfortable with command-line tools)
- **Patience and curiosity**: Robotics involves system thinking; you'll debug issues across multiple nodes

You do **NOT** need:
- Robotics background
- C++ knowledge
- Expensive hardware
- Previous ROS experience

## How to Use This Module

**For Students**:
1. Start with **Chapter 1, Lesson 1** (it takes ~1 hour)
2. Proceed sequentially—each chapter builds on prior learning
3. Complete the **mastery gate** at the end of each chapter before proceeding
4. Collaborate with AI in Chapters 4+ using the prompts provided
5. Finish with the **Chapter 7 capstone** to demonstrate full integration

**For Instructors**:
1. Review all 7 chapter READMEs for learning objectives and time commitment
2. Map chapters to your teaching calendar (5-week structure is suggested, adjustable)
3. Use chapter mastery gates as assessment checkpoints
4. The capstone project is graded with clear specification-based rubrics

**For Authors**:
1. Study this module's chapter architecture as a template for Module 2
2. The 4-layer progression (L1→L2→L3→L4) applies to all technical modules
3. The "Tier 1 fallback" principle ensures accessibility across hardware constraints
4. The reusable skills approach compounds intelligence across books

## Mastery Gates Between Chapters

Before progressing to the next chapter, confirm you can:

- **After Chapter 1**: Distinguish software AI from embodied AI, name 3 physical constraints
- **After Chapter 2**: Categorize sensor types, explain middleware role, identify your hardware tier
- **After Chapter 3**: Launch turtlesim, list nodes/topics, call services, modify parameters (via CLI)
- **After Chapter 4**: Write publisher/subscriber nodes, verify data flow, extend with AI assistance
- **After Chapter 5**: Design service interfaces, create custom messages, justify design trade-offs
- **After Chapter 6**: Create launch files, start multi-node systems, debug with ros2doctor and rqt
- **After Chapter 7**: Write specifications, implement from specs, validate completeness

## Technical Stack

- **ROS 2 Distribution**: Humble (Long-term support, current industry standard)
- **Language**: Python 3.10+ with rclpy (ROS 2 Python client library)
- **Simulation**: Turtlesim (Chapters 3-7), MockROS (browser simulation for Tier 1)
- **Cloud Option**: The Construct ROS (cloud ROS 2 environment, free tier available)
- **Tools**: ros2 CLI, rqt (visualization), colcon (build system), Docusaurus (this platform)

## Module Progression

**Chapter Flow:**

1. **Ch1-2 (Foundation)**: Physical AI concepts → Robot system anatomy
2. **Ch3 (Bridge)**: Meet ROS 2 through CLI exploration
3. **Ch4-5 (Hands-On)**: First code (pub/sub) → Communication (services + interfaces)
4. **Ch6 (Integration)**: Building systems with launch files and debugging
5. **Ch7 (Capstone)**: Multi-node spec-driven project

- **L1 Manual Foundation**: Ch1-2 (conceptual understanding)
- **L1→L2 Transition**: Ch3 (CLI exploration with AI assistance)
- **L2 AI Collaboration**: Ch4-5 (hands-on coding with AI)
- **L3 Intelligence Design**: Ch6 (system patterns)
- **L4 Spec-Driven**: Ch7 (capstone project)

## Support & Community

- **Questions about content?** Use the RAG chat on any lesson (bottom right corner)
- **Hardware setup help?** Chapter 2, Lesson 4 helps identify your tier
- **Stuck on a coding problem?** Leverage the AI collaboration prompts in Chapters 4-7
- **Capstone project guidance?** Chapter 7 provides specification templates and iteration frameworks

## What Comes Next?

After completing Module 1, you'll be ready for **Module 2: Simulation Mastery**, where you'll:
- Model robots using URDF (robot description format)
- Simulate complex systems in Gazebo
- Add perception (LIDAR, cameras) to your ROS 2 nodes
- Implement autonomous navigation algorithms

But first—master Module 1. The foundation you build here determines what's possible in Modules 2-4.

---

**Ready to begin?** [Start Chapter 1: What is Physical AI?](./chapter-1-physical-ai/README.md)

**Prefer to explore differently?** Jump to any chapter's README above to see what's inside before committing.

**Questions about this module?** Review the learning objectives, time commitment, and prerequisites above.
