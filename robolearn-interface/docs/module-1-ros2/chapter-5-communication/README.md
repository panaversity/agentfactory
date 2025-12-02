---
id: chapter-5-communication
title: "Chapter 5: Communication Mastery"
sidebar_position: 5
sidebar_label: "5. Communication"
description: "Services, custom messages, design patterns, topic vs service trade-offs"
---

# Chapter 5: Communication Mastery

You've mastered pub/sub. Now it's time to expand your toolkit. ROS 2 offers more than just continuous data streams. Sometimes you need **request/response** communication (like calling a function), and sometimes you need **custom message types** (because String and Int64 aren't always enough).

In this chapter, you'll write service servers and clients, design your own message types, and learn the design patterns that determine whether to use topics or services for different scenarios. You'll create two more reusable skills that apply to advanced ROS 2 systems.

**Duration**: 4 lessons, 4 hours total
**Layer Breakdown**: L2: 50%, L3: 40%, L4: 10% (intro to spec-first design)
**Hardware Tier**: Tier 1 (Cloud ROS 2)
**Prerequisites**: Chapter 4 (publisher/subscriber patterns)
**Reusable Skills Created**: `ros2-service-pattern`, `ros2-custom-interfaces`

## Learning Objectives

By the end of this chapter, you will be able to:

- **Write service servers** that process requests and return responses
- **Write service clients** that send requests and handle responses
- **Design custom message types** (.msg files) and service definitions (.srv files)
- **Choose between topics and services** based on communication requirements
- **Understand design trade-offs** (synchronous vs. asynchronous, reliable vs. fast)
- **Create reusable interface packages** for custom message types

## Lessons

### Lesson 5.1: Writing a Service Server (60 minutes)

Create a node that implements a service. A service is like a function call: client sends a request, server processes it, and returns a response. Useful for commands, configuration changes, and one-time actions.

**Core Concepts**:
- Service pattern (request/response, synchronous RPC-like communication)
- Service server creation (`create_service`)
- Service callbacks

---

### Lesson 5.2: Writing a Service Client (60 minutes)

Create a node that calls the service from Lesson 5.1. Handle both successful responses and error cases (service not available, timeout).

**Core Concepts**:
- Service client creation (`create_client`)
- Async/sync request handling
- Error recovery patterns

---

### Lesson 5.3: Custom Messages and Services (60 minutes)

Stop using built-in message types. Create your own `.msg` files for custom messages (like `RobotStatus` with position, velocity, battery level) and `.srv` files for custom services.

**Core Concepts**:
- .msg file format (interface definitions)
- .srv file format (request/response structure)
- Building interface packages
- Using custom types in nodes

---

### Lesson 5.4: Design Patterns (60 minutes)

Go beyond implementation. Learn the **decision framework** for choosing between topics and services, and write mini-specifications before coding. This is your introduction to spec-driven thinking.

**Core Concepts**:
- Topic vs service trade-offs (continuous vs. discrete, async vs. sync)
- Design pattern selection framework
- Specification-first preview (write intent before code)

---

## 4-Layer Teaching Method

| Layer | % | What's Covered |
|-------|---|----------------|
| **L1: Manual** | 20% | Worked examples, step-by-step walkthroughs |
| **L2: AI Collab** | 50% | AI helps with error handling, design decisions, trade-off analysis |
| **L3: Intelligence** | 40% | Reusable pattern recognition, skill creation, interface design principles |
| **L4: Spec-Driven** | 10% | Preview: write mini-specs for Lesson 5.4 design exercise |

This chapter introduces **deeper design thinking**. You're not just writing code—you're making architectural decisions.

## Hardware Requirements

**Minimum Tier**: Tier 1 (Cloud ROS 2)

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **1** | Laptop + Browser | Cloud ROS 2 (TheConstruct) with full Python + custom interface support |
| **2+** | Local ROS 2 | Local installation with full capabilities |

All exercises run in cloud ROS 2. No special hardware needed.

## Prerequisites

- **Chapter 4** (publisher/subscriber patterns)
- **Python OOP** (classes, inheritance)
- **ROS 2 environment** with full colcon build support

## Mastery Gate

Before proceeding to **Chapter 6**, you should be able to:

- **Implement a service server** that processes requests and returns responses
- **Implement a service client** that sends requests and handles success/failure
- **Create custom .msg files** for your own message types
- **Create custom .srv files** for your own service definitions
- **Build interface packages** and use custom types in nodes
- **Explain when to use topics vs services** for different scenarios
- **Write a mini-spec** describing what your communication system should do before implementing

If you can do these, you're ready for Chapter 6.

---

## Key Patterns

### Topic Pattern
```
Publisher → ROS 2 Topic → Subscriber(s)
Continuous, asynchronous, loose coupling
Use for: sensor data, status streams, high-frequency updates
```

### Service Pattern
```
Client → Service Server → Response
Request/response, synchronous, tightly coupled
Use for: commands, one-time actions, when you need confirmation
```

### When to Choose Which

| **Scenario** | **Use** | **Why** |
|--------------|--------|--------|
| "Publish robot position continuously" | Topic | Continuous, doesn't need acknowledgment |
| "Request robot to move to (x,y)" | Service | One-time action, needs confirmation |
| "Stream sensor data (LIDAR points)" | Topic | High-frequency, multiple subscribers |
| "Configure robot speed limit" | Service | One-time config, needs confirmation |
| "Monitor robot battery level" | Topic | Continuous monitoring |
| "Emergency stop signal" | Service | Critical, needs immediate response |

Master these trade-offs, and your designs will scale efficiently.

---

## Navigation

**Previous Chapter**: [← Chapter 4: First Code](../chapter-4-first-code/README.md)

**Next Chapter**: [Chapter 6: Building Systems →](../chapter-6-building-systems/README.md)

**Module Overview**: [← Back to Module 1](../README.md)

**Start Lesson 5.1**: [Writing a Service Server →](./01-service-server.md)
