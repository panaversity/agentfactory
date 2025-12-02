---
id: chapter-3-meet-ros2
title: "Chapter 3: Meet ROS 2"
sidebar_position: 3
sidebar_label: "3. Meet ROS 2"
description: "ROS 2 installation, CLI exploration, turtlesim, nodes, topics, services"
---

# Chapter 3: Meet ROS 2

You understand the problem: robots have many sensors and motors that need coordination. You understand the solution: middleware like ROS 2. Now it's time to experience ROS 2 directly—not by writing code yet, but by exploring it through your command line.

In this chapter, you'll install (or access) ROS 2, launch your first robot simulation (turtlesim), and explore ROS 2's core concepts using command-line tools. You'll see nodes running, watch topics flowing, call services, and modify parameters. By the end, you'll have a visceral understanding of how ROS 2 coordinates robot systems.

**Duration**: 4 lessons, 4 hours total
**Layer Breakdown**: L1: 60%, L2: 40% (AI perspective on CLI commands, architecture)
**Hardware Tier**: Tier 1 (cloud ROS 2 or local installation with fallback)
**Prerequisites**: Chapters 1-2 (physical AI, robot systems)
**Critical Note**: **NO Python coding in this chapter.** CLI exploration only.

## Learning Objectives

By the end of this chapter, you will be able to:

- **Set up a ROS 2 environment** locally or via cloud (TheConstruct)
- **Launch ROS 2 applications** using the `ros2 run` command
- **Explore the node graph** and understand how nodes publish and subscribe
- **Use ROS 2 CLI tools** to investigate running systems (nodes, topics, services, parameters)
- **Visualize multi-node systems** using rqt_graph

## Lessons

### Lesson 3.1: Setting Up Your ROS 2 Environment (60 minutes)

Install ROS 2 Humble locally OR access a cloud ROS 2 environment. Verify your installation works.

**Core Concepts**:
- ROS 2 installation and workspace setup
- Environment sourcing and package path management
- Tier 1 fallback: Cloud ROS 2 for students without local installation

---

### Lesson 3.2: Turtlesim in Action (60 minutes)

Launch turtlesim—a visual robot simulation built into ROS 2. Control it with your keyboard. See the pub/sub system in action through visual feedback.

**Core Concepts**:
- ROS 2 nodes (executables running ROS logic)
- Visual feedback from data flowing through ROS 2 topics

---

### Lesson 3.3: Nodes and Topics (CLI Exploration) (60 minutes)

Use `ros2 node list`, `ros2 topic list`, and `ros2 topic echo` to explore the system. Understand the publish/subscribe pattern through concrete observation.

**Core Concepts**:
- Nodes (independent processes)
- Topics (named channels for continuous data)

---

### Lesson 3.4: Services and Parameters (CLI Exploration) (60 minutes)

Go beyond pub/sub. Explore services (request/response pattern) and parameters (configuration variables) using ROS 2 CLI commands.

**Core Concepts**:
- Services (RPC-like request/response)
- Parameters (runtime configuration)

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 60% | Step-by-step CLI exploration, hands-on turtlesim control, command walkthroughs |
| **L2: AI Collab** | 40% | AI explains what each command does, architecture perspective, brief design rationale |
| **L3: Intelligence** | 0% | Not yet—you're still learning how to use tools |
| **L4: Spec-Driven** | 0% | Not yet—specifications come after coding experience |

This is the **bridge chapter** between conceptual understanding (Chapters 1-2) and hands-on coding (Chapters 4-7). You experience ROS 2 directly, and AI helps explain what you're seeing.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Laptop/Browser)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud ROS 2 via TheConstruct OR local installation via WSL/Linux |
| **2+** | GPU, Jetson | Local ROS 2 installation (faster than cloud) |

**Tier 1 Cloud Option**: The Construct provides free cloud ROS 2 (Humble) with browser-based terminal and VNC visualization. No installation required.

**Tier 1+ Local Option**: Ubuntu Linux (native or WSL on Windows) with `apt install ros-humble-desktop`

**All paths lead to the same ROS 2 CLI.** Choose what's easiest for you.

## Prerequisites

- **Chapters 1-2** (understand what robots and middleware are)
- **Terminal familiarity** (`cd`, `ls`, `pwd`, text editing)
- **ROS 2 Humble** (installed locally OR accessed via cloud)
- **Patience**: Installation sometimes has hiccups; cloud option sidesteps this

## Mastery Gate

Before proceeding to **Chapter 4** (your first coding chapter), you should be able to:

- **Launch turtlesim** and control it with keyboard teleop
- **List all running nodes** using `ros2 node list` and explain what they do
- **List all active topics** using `ros2 topic list -t` and understand their purpose
- **Echo topic data** using `ros2 topic echo` and interpret the output
- **Call a service** (e.g., `/spawn` to create a new turtle)
- **Modify a parameter** using `ros2 param set` (e.g., change turtle color)
- **Visualize the node graph** using `rqt_graph` and understand the publisher/subscriber connections

If you can do these, you're ready for Chapter 4.

---

## Navigation

**Previous Chapter**: [← Chapter 2: Robot System](../chapter-2-robot-system/README.md)

**Next Chapter**: [Chapter 4: Your First ROS 2 Code →](../chapter-4-first-code/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 3.1**: [Setting Up Your ROS 2 Environment →](./01-setup-environment.md)
