---
id: chapter-6-building-systems
title: "Chapter 6: Building Robot Systems"
sidebar_position: 6
sidebar_label: "6. Building Systems"
description: "Parameters, launch files, multi-node debugging, system integration"
---

# Chapter 6: Building Robot Systems

You can write individual nodes. You understand pub/sub and services. But real robots aren't single nodes—they're ecosystems of interconnected components. A humanoid robot might have 20+ nodes: motor controllers, sensor readers, perception pipelines, navigation stacks, and more.

In this chapter, you'll learn how to **scale** from individual nodes to complete systems. You'll make nodes configurable with parameters, create launch files that start multiple nodes with one command, and use debugging tools to diagnose what's happening when things go wrong.

**Duration**: 3 lessons, 3 hours total
**Layer Breakdown**: L2: 30%, L3: 50%, L4: 20% (beginning system-level design)
**Hardware Tier**: Tier 1 (Cloud ROS 2)
**Prerequisites**: Chapters 4-5 (pub/sub, services, custom interfaces)
**Reusable Skill Created**: `ros2-launch-system`

## Learning Objectives

By the end of this chapter, you will be able to:

- **Add parameters to nodes** for runtime configuration (instead of hardcoding values)
- **Write Python launch files** that start multiple nodes with different parameters
- **Debug multi-node systems** using `ros2doctor`, `rqt_graph`, and logging
- **Understand parameter scope** and how to pass parameters from launch files to nodes
- **Systematically diagnose issues** in multi-node systems
- **Scale from single nodes to complex systems** with confidence

## Lessons

### Lesson 6.1: Parameters (Configurable Nodes) (60 minutes)

Make your nodes configurable. Instead of changing code every time you want a different publish rate or threshold, use ROS 2 parameters. Declare parameters in your node, read them at startup, and modify them at runtime with `ros2 param set`.

**Core Concepts**:
- Parameter declaration (`declare_parameter`)
- Parameter reading (`get_parameter`)
- Runtime modification (`ros2 param set`)

---

### Lesson 6.2: Launch Files (Multi-Node Startup) (60 minutes)

Stop running nodes manually. Write Python launch files that start multiple nodes, pass parameters to each one, and remap topic names for coordination. Start entire systems with one command.

**Core Concepts**:
- Launch file structure (LaunchDescription, Node actions)
- Parameter passing from launch files
- Topic/service name remapping
- Conditional launching based on arguments

---

### Lesson 6.3: Debugging Multi-Node Systems (60 minutes)

When things break (and they will), use ROS 2's debugging tools. `ros2doctor` diagnoses environment issues. `rqt_graph` visualizes the entire node network. Logger levels let you see detailed trace information when needed.

**Core Concepts**:
- System health diagnosis (`ros2doctor`)
- Node graph visualization (`rqt_graph`)
- Logger level control (`ros2 param set`)
- Systematic troubleshooting methodology

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 20% | Step-by-step walkthroughs, structured examples |
| **L2: AI Collab** | 30% | AI helps with parameter validation, launch file design, debugging strategies |
| **L3: Intelligence** | 50% | Reusable patterns for configurable systems, debugging workflows |
| **L4: Spec-Driven** | 20% | Beginning to think about system architecture, how specifications guide system design |

This chapter **scales you up**. You're moving from individual nodes to thinking about complete robotic systems.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Cloud ROS 2)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud ROS 2 with full launch file support and visualization via rqt |
| **2+** | Local ROS 2 | Local installation with instant feedback |

All exercises work in cloud ROS 2. Local execution is faster but not required.

## Prerequisites

- **Chapters 4-5** (nodes, pub/sub, services, custom interfaces)
- **Python programming** (variable assignment, dictionaries, function calls)
- **Understanding of multi-process systems** (basic familiarity with running multiple programs simultaneously)

## Mastery Gate

Before proceeding to **Chapter 7** (the capstone), you should be able to:

- **Add parameters to a node** and read them at runtime
- **Modify parameters** using `ros2 param set` without restarting the node
- **Write a launch file** that starts 3+ nodes with different configurations
- **Remap topic names** in launch files to coordinate nodes
- **Visualize a node graph** using `rqt_graph` and explain the connections
- **Run `ros2doctor`** and interpret its output
- **Debug a broken multi-node system** by:
  - Checking which nodes are running (`ros2 node list`)
  - Verifying topics are publishing (`ros2 topic echo`)
  - Checking parameters are set correctly (`ros2 param list`)
  - Adjusting log levels to see detailed output
- **Understand why parameters and launch files matter** for system scalability

If you can do these, you're ready for the Chapter 7 capstone.

---

## Key Concept: System Orchestration

This chapter is about **orchestration**—coordinating multiple components:

```
Launch File
├── Node 1 (Motor Controller)
│   ├── Parameters: max_velocity, rate
│   └── Topics: /motor/command (sub), /motor/status (pub)
├── Node 2 (Sensor Reader)
│   ├── Parameters: sensor_rate, threshold
│   └── Topics: /sensor/raw (pub)
└── Node 3 (Logger)
    ├── Parameters: log_level
    └── Topics: /motor/status (sub), /sensor/raw (sub)
```

One `ros2 launch` command starts all of them with coordinated parameters. This scales to 20+ nodes in production systems.

---

## Navigation

**Previous Chapter**: [← Chapter 5: Communication](../chapter-5-communication/README.md)

**Next Chapter**: [Chapter 7: Capstone →](../chapter-7-capstone/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 6.1**: [Parameters →](./01-parameters.md)
