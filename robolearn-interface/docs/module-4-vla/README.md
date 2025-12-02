---
id: module-4-vla
title: "Part 4: Vision-Language-Action"
sidebar_position: 4
sidebar_label: "4. Humanoid VLA Systems"
description: "Build autonomous humanoids with VLA models - voice commands, LLM reasoning, motion planning, and end-to-end robot control."
---

# Part 4: Vision-Language-Action

**Autonomous Humanoid Systems**

This is where everything comes together. You've built the communication layer (ROS 2), the simulation environment (Gazebo), and the perception systems (Isaac). Now you'll create robots that understand natural language, reason about their environment, and execute complex tasks autonomously.

**VLA models** (Vision-Language-Action) represent the frontier of robotics AI. They combine what the robot sees (vision), what you tell it (language), and what it does (action) into unified systems. Say "pick up the cup on the table" and the robot figures out the rest: localize the cup, plan the arm trajectory, execute the grasp, verify success.

In this part, you'll build an autonomous humanoid that responds to voice commands, navigates real environments, and manipulates objects. This is the capstone of the entire course—every skill from Parts 1-3 combines into one working system.

---

## Chapters

### Chapter 9: Humanoid Kinematics

Humanoids are mechanically complex—dozens of joints forming kinematic chains from feet to fingertips. You'll model humanoid structure, implement inverse kinematics (given a target hand position, calculate all joint angles), and understand the unique challenges of bipedal balance and locomotion. Safety is paramount: you'll build safeguards before any motor moves.

### Chapter 10: Conversational Robotics

Make your robot listen and respond. You'll integrate Whisper for speech-to-text, connect LLMs for natural language understanding and task planning, and ground language in physical capabilities (the robot knows what it *can* do). When someone says "get me a drink," the robot decomposes this into navigation, search, grasp, and delivery subtasks.

### Chapter 11: Capstone — Autonomous Humanoid

The final integration. You'll build a complete system: voice command → LLM planning → VSLAM navigation → object detection → motion execution → task verification. Every component you've built across four parts connects into one autonomous agent. The capstone works in simulation (everyone) and optionally on physical Unitree humanoids (if available).

---

## Learning Outcomes

By completing Part 4, you will:

- Design humanoid kinematic models with inverse kinematics
- Implement voice-controlled robot interfaces
- Integrate LLMs for task planning and reasoning
- Compose skills from all parts into unified behaviors
- Deploy end-to-end autonomous humanoid systems

---

## Hardware

| Tier | What You Need | What You Get |
|------|---------------|--------------|
| **1 (Simulation)** | Any laptop | Complete course in simulation + cloud voice APIs |
| **4 (Physical)** | Unitree G1/Go2 | Deploy to real humanoid hardware |

All core content works in simulation. Physical robots are optional.

---

## Capstone: Autonomous Humanoid Agent

The course finale. Your humanoid receives a voice command ("find the red cup and bring it to me"), plans the task using an LLM, navigates using VSLAM, detects the object with vision, grasps with inverse kinematics, and delivers. One integrated system demonstrating everything you've learned.

---

## VLA Models Explored

- **OpenVLA** (Berkeley) — Open-source vision-language-action foundation
- **π0** (Physical Intelligence) — State-of-the-art manipulation
- **GR00T** (NVIDIA) — Sim-to-real humanoid control

---

## Prerequisites

**Parts 1-3** complete. You'll integrate ROS 2, simulation, and Isaac perception.

---

[**← Part 3: NVIDIA Isaac**](../module-3-isaac/README.md)

---

**Congratulations!** After Part 4, you'll have built an autonomous humanoid from scratch—ROS 2 middleware, physics simulation, AI perception, and voice-controlled autonomy working together. You're ready for real-world robotics.
