---
id: chapter-4-first-code
title: "Chapter 4: Your First ROS 2 Code"
sidebar_position: 4
sidebar_label: "4. First Code"
description: "Write your first ROS 2 nodes: publishers, subscribers, Python rclpy"
---

# Chapter 4: Your First ROS 2 Code

You've explored ROS 2 from the command line. You've seen turtlesim move. You've watched data flow through topics. Now it's time to write the code that makes it happen.

In this chapter, you'll write your first ROS 2 nodes in Python. You'll create publishers (nodes that send data), subscribers (nodes that receive data), and understand how they coordinate through ROS 2's pub/sub infrastructure. By the end, you'll have two nodes talking to each other—and you'll have created a reusable skill that applies to every ROS 2 project you'll ever build.

**Duration**: 4 lessons, 4 hours total
**Layer Breakdown**: L1: 40%, L2: 50%, L3: 10% (intro to creating reusable skills)
**Hardware Tier**: Tier 1 (cloud ROS 2 recommended)
**Prerequisites**: Chapters 1-3 (ROS 2 exploration)
**Reusable Skill Created**: `ros2-publisher-subscriber` (foundation for all ROS 2 development)

## Learning Objectives

By the end of this chapter, you will be able to:

- **Build ROS 2 packages** with proper workspace structure
- **Write a Python ROS 2 publisher** that sends data at regular intervals
- **Write a Python ROS 2 subscriber** that receives and processes data
- **Understand the rclpy Node API** and node lifecycle
- **Debug pub/sub communication** using `ros2 topic echo` and logging
- **Collaborate with AI** to extend your code with new features
- **Recognize reusable patterns** that you'll apply throughout Module 1+

## Lessons

### Lesson 4.1: Workspaces and Packages (60 minutes)

Create a ROS 2 workspace and your first package. Understand the directory structure (`src/`, `build/`, `install/`), the `package.xml` manifest, and how to build with `colcon`.

**Core Concepts**:
- Workspace structure (src, build, install directories)
- Package.xml metadata and dependencies
- Building with colcon

---

### Lesson 4.2: Writing a Publisher (60 minutes)

Create a node that publishes "Hello World" messages every 500ms. Understand the `rclpy.Node` API, timers, and how publishers send data.

**Core Concepts**:
- Publisher creation (`create_publisher`)
- Timer-based callbacks
- Node lifecycle (init, spin, shutdown)

---

### Lesson 4.3: Writing a Subscriber (60 minutes)

Create a node that listens for those messages and logs them. Understand callbacks, message handling, and subscriber creation.

**Core Concepts**:
- Subscriber creation (`create_subscription`)
- Callback pattern (processing incoming messages)
- Message handling and logging

---

### Lesson 4.4: Try With AI (60 minutes)

Collaborate with AI to extend your pub/sub system. Ask AI to help add error handling, configuration, logging levels. Experience productive AI dialogue through action.

**Core Concepts**:
- AI-assisted code generation and pattern discovery
- Validating AI suggestions against your constraints
- Iterating toward better design through dialogue
- Creating a reusable skill from the pattern

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 40% | Step-by-step code walkthroughs, package structure explanations |
| **L2: AI Collab** | 50% | AI collaboration to extend your code, validation, iteration |
| **L3: Intelligence** | 10% | Introduction: recognizing reusable patterns, skill creation preview |
| **L4: Spec-Driven** | 0% | Not yet—too early for complex specifications |

This is your **first hands-on coding chapter**. You write real Python code, run real ROS 2 nodes, and experience AI collaboration for the first time.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Cloud ROS 2)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud ROS 2 (TheConstruct) with Python execution and topic visualization |
| **2+** | Local ROS 2 | Local installation with faster execution |

**Recommended for beginners**: Cloud ROS 2 (TheConstruct) avoids local installation complexity. You get a terminal, Python IDE, and visualization all in one browser window.

## Prerequisites

- **Chapters 1-3** (understand ROS 2 concepts and CLI)
- **Python programming** (functions, classes, basic OOP)
- **ROS 2 environment set up** (from Chapter 3)

## Mastery Gate

Before proceeding to **Chapter 5**, you should be able to:

- **Create a ROS 2 package** from scratch using `ros2 pkg create`
- **Write a publisher node** that sends messages every N seconds
- **Write a subscriber node** that receives messages and processes them
- **Verify communication** using `ros2 topic echo` to see data flowing
- **Extend a node with AI** assistance (add logging, error handling, configuration)
- **Understand the publisher/subscriber pattern** well enough to explain it to someone else
- **Recognize that this pattern repeats** across all ROS 2 projects

If you can do these, you're ready for Chapter 5.

---

## Key Concept: Publisher/Subscriber Pattern

This pattern appears in **every ROS 2 system** you'll build:

```
Publisher Node → ROS 2 Topic → Subscriber Node(s)
```

**Publisher's job**: Create data, publish it periodically
**Subscriber's job**: Receive data, process it
**ROS 2's job**: Route data from publisher to all subscribers

You'll apply this pattern to:
- Sensor data (Chapter 5)
- Motor commands (Chapter 6)
- System status (Chapter 7 capstone)

Master this chapter, and you unlock the entire ROS 2 ecosystem.

---

## Navigation

**Previous Chapter**: [← Chapter 3: Meet ROS 2](../chapter-3-meet-ros2/README.md)

**Next Chapter**: [Chapter 5: Communication Mastery →](../chapter-5-communication/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 4.1**: [Workspaces and Packages →](./01-workspaces-packages.md)
