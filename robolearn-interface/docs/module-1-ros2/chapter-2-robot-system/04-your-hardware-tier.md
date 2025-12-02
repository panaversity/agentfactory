---
id: lesson-2-4-your-hardware-tier
title: "Lesson 2.4: Your Hardware Tier"
sidebar_position: 4
sidebar_label: "2.4 Your Hardware Tier"
description: "Identifying your hardware tier and understanding which learning paths and capabilities are available across Tier 1 cloud through Tier 4 physical robots."
duration_minutes: 45
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Identify which hardware tier you currently have access to"
  - "Understand the capabilities and limitations of each tier"
  - "Recognize how tier affects learning timeline and cost"
  - "Plan hardware progression pathways for future investment"
---

# Your Hardware Tier

This course meets you where you are. Some students have laptops; others have gaming PCs with RTX GPUs; others have embedded Jetson boards. Rather than pretending everyone has the same hardware, we've designed the curriculum to work across **hardware tiers**.

This lesson helps you identify your tier and understand which learning paths are available to you.

---

## The Four Hardware Tiers

### Tier 1: Laptop or Cloud (Cloud-Based Learning)

**Equipment**: Laptop/desktop (any OS) with internet access

**What you can do**:
- Run ROS 2 in cloud (TheConstruct)
- Write Python code in cloud terminal
- Use MockROS (browser-based simulation)
- Run Pyodide (Python in browser)
- Access visualization tools via remote desktop

**What you cannot do**:
- Run local GPU-intensive simulation
- Deploy to edge hardware
- Access local USB sensors (sensors are virtual/mocked)

**Timeline for Module 1**: ✅ Full module possible (4-5 weeks)
**Timeline for Module 2**: ⚠️ Partial (can use cloud Gazebo, slower iteration)
**Timeline for Module 3**: ⚠️ Limited (Isaac cloud slower than local)
**Timeline for Module 4**: ❌ Not without paid cloud resources

**Cost**: $0 (cloud ROS 2 has free tier) or $10-50/month for premium cloud access

---

### Tier 2: RTX GPU Workstation (Local Development)

**Equipment**: Laptop/desktop with NVIDIA RTX GPU (RTX 3050 or better)

**What you can do**:
- Run ROS 2 locally (no cloud dependency)
- Run Gazebo locally (full physics simulation)
- Run Isaac Sim locally (fast iteration)
- Work offline
- Deploy code quickly (seconds, not minutes)

**What you cannot do**:
- Deploy to real robots (no Jetson/robot hardware)
- Test edge constraints (no ARM architecture)
- Use real sensors (only simulated)

**Timeline for Module 1**: ✅ Full module in 3 weeks (faster iteration)
**Timeline for Module 2**: ✅ Full module locally
**Timeline for Module 3**: ✅ Full module, fast
**Timeline for Module 4**: ⚠️ Possible but slower than Tier 3/4

**Cost**: $300-1500 for GPU upgrade, or already have hardware

---

### Tier 3: Jetson Edge Device (Edge Deployment)

**Equipment**:
- NVIDIA Jetson board (Orin Nano $200, Orin AGX $3,000+)
- Plus laptop from Tier 1 or 2 for development
- Optional: Sensors (IMU, camera)

**What you can do**:
- Run ROS 2 on embedded ARM hardware
- Deploy code to edge device (no cloud dependency)
- Connect real sensors (USB cameras, serial IMU)
- Experience real latency and resource constraints
- Build autonomous robots (with Tier 4 hardware optional)

**What you cannot do**:
- Run heavy GPU workloads locally (Isaac Sim must run on workstation, results stream to Jetson)
- Iterate as fast as Tier 2 (Jetson is slower than desktop GPU)

**Timeline for Module 1**: ✅ Full module
**Timeline for Module 2**: ✅ Full module (development on workstation, deployment on Jetson)
**Timeline for Module 3**: ✅ Full module
**Timeline for Module 4**: ✅ Full module, realistic edge constraints

**Cost**: $200-3,000 for Jetson + $500-2,000 for sensors

---

### Tier 4: Physical Robot (Real-World Testing)

**Equipment**:
- Tier 2 or 3 hardware for development
- Plus physical robot:
  - **Affordable options**: Unitree Go2 ($1,500), Jetbot ($400)
  - **Expensive options**: Figure humanoid ($150,000+), Tesla Bot (not yet available)

**What you can do**:
- Test code on real hardware
- Experience real-world messiness (latency, sensor noise, actuator limits)
- Validate sim-to-real transfer (simulation matches reality?)
- Build production systems

**What you cannot do**:
- Iterate as quickly (real robots take time to test)
- Recover from software failures as easily (crashing code in simulation is free; in reality it may damage hardware)

**Timeline for Module 1**: ✅ Full module (foundation, no motor control yet)
**Timeline for Module 2**: ⚠️ Partial (can simulate in Gazebo, some tests on real robot)
**Timeline for Module 3**: ✅ Full module
**Timeline for Module 4**: ✅ Full module, with real-world validation

**Cost**: $1,500-$150,000+ depending on robot choice

---

## Hardware Tier Comparison Table

| Aspect | Tier 1 | Tier 2 | Tier 3 | Tier 4 |
|--------|--------|--------|--------|--------|
| **Equipment Cost** | $0-50 | $300-1,500 | $200-3,000 | $1,500-150,000+ |
| **ROS 2 Local** | Cloud | ✅ | ✅ | ✅ |
| **Gazebo Simulation** | Cloud | ✅ (local) | Partial | ✅ + Real |
| **Iteration Speed** | Slow (cloud) | Fast (local) | Medium | Slow (real hw) |
| **Module 1 Possible** | ✅ Full | ✅ Full | ✅ Full | ✅ Full |
| **Module 2 Possible** | ⚠️ Partial | ✅ Full | ✅ Full | ✅ Full |
| **Module 3 Possible** | ⚠️ Limited | ✅ Full | ✅ Full | ✅ Full |
| **Module 4 Possible** | ❌ (hard) | ⚠️ Difficult | ✅ Full | ✅ Full + Real |

---

## Tier Progression Paths

### Starting Tier 1 (Cloud Only)

```
Tier 1 (Weeks 1-5)
↓
Decision point:
  - Continue Tier 1 (cloud path) for Modules 2-4
  - Upgrade to Tier 2 (buy GPU laptop) for faster iteration
```

**Recommendation**: Complete Module 1 in Tier 1, then decide if cloud iteration speed is acceptable for Module 2. If frustrated by slow feedback loops, upgrade to Tier 2.

### Starting Tier 2 (GPU Workstation)

```
Tier 2 (Weeks 1-8, Modules 1-3)
↓
Decision point:
  - Continue Tier 2 for Module 4 (simulation-only humanoid)
  - Upgrade to Tier 3 (buy Jetson) to test real robot constraints
```

**Recommendation**: Tier 2 is sufficient for learning Modules 1-3. Upgrade to Tier 3 if you want to test edge deployment before Module 4.

### Starting Tier 3 (Jetson Edge Device)

```
Tier 3 (All modules possible)
↓
Optional: Add Tier 4 (physical robot) for real-world validation
```

**Recommendation**: Tier 3 covers everything. If you later want to test on real humanoid, add Tier 4.

### Starting Tier 4 (Physical Robot)

```
Tier 4 (All modules, with real-world validation)
↓
Use Tier 2 or 3 for faster simulation iteration
```

**Recommendation**: Real robots are expensive and break if you make mistakes. Always develop in simulation first (Tier 2 or 3), then test on real hardware (Tier 4).

---

## Cost-Benefit Analysis

### Should You Upgrade to Tier 2?

**Upgrade if:**
- ✅ You find cloud iteration speed frustrating (10+ second wait per code change)
- ✅ You want to work offline
- ✅ You plan to spend 100+ hours on this course

**Don't upgrade if:**
- ❌ You only have a few hours to learn
- ❌ You're just exploring robotics (not committing)
- ❌ Your budget is very tight

### Should You Upgrade to Tier 3?

**Upgrade if:**
- ✅ You want to deploy real edge code
- ✅ You plan to build actual robots
- ✅ You want to understand hardware constraints

**Don't upgrade if:**
- ❌ Simulation learning is sufficient for you
- ❌ You don't need real sensors
- ❌ Your budget doesn't support $1,500+

### Should You Upgrade to Tier 4?

**Upgrade if:**
- ✅ You want to test on real hardware
- ✅ You have a specific robot project in mind
- ✅ You're in a research/professional context

**Don't upgrade if:**
- ❌ You're learning (simulation is sufficient)
- ❌ You can't risk hardware damage from software bugs
- ❌ Budget is constrained (robots are expensive)

---

## Finding Your Tier

### Quick Self-Assessment

**Question 1**: What equipment do you have access to right now?
- Only laptop without GPU → **Tier 1**
- Laptop with NVIDIA RTX GPU → **Tier 2**
- Jetson board → **Tier 3**
- Physical robot (Unitree, Boston Dynamics, etc.) → **Tier 4**

**Question 2**: How much time can you invest?
- Less than 10 hours → Tier 1 cloud is fine
- 50-100 hours → Consider Tier 2 for better iteration
- 200+ hours → Invest in Tier 3 or 4

**Question 3**: What's your learning goal?
- Understand robotics concepts → Any tier works (Tier 1 sufficient)
- Build working code → Tier 2+ recommended
- Deploy to edge → Tier 3+ required
- Real-world robots → Tier 4 eventually

---

## What This Course Provides for Each Tier

### Tier 1 Path
- Cloud ROS 2 setup guide (TheConstruct)
- Browser-based visualizations (MockROS)
- All code examples work in cloud terminal
- Slower iteration (5-10 seconds per test)

### Tier 2 Path
- Local ROS 2 installation guide
- Gazebo/Isaac Sim setup
- All code examples work locally
- Fast iteration (1-2 seconds per test)

### Tier 3 Path
- Jetson setup guide
- ROS 2 ARM-specific considerations
- Sensor integration examples
- Edge deployment patterns

### Tier 4 Path
- Real robot setup guides (Unitree, etc.)
- Sim-to-real transfer validation
- Hardware safety patterns
- Real-world testing examples

---

## Interactive Hardware Tier Selector

Below is a conceptual hardware profile assessment. In the real platform, this would be interactive (you select equipment, the system recommends paths).

**Your Profile**:
- Laptop specs: [CPU, RAM, GPU]
- Budget for upgrade: [$0, $500, $1,000, $5,000+]
- Time available: [Under 10 hrs, 50-100 hrs, 200+ hrs]
- Learning goal: [Understand concepts, Build code, Deploy edge, Real robots]

**System Recommendation**:
- Start with: [Tier 1/2/3/4]
- Can complete: [Which modules fully vs partially]
- Timeline estimate: [Weeks for each module]

---

## Reflect

Think about your own situation:

- **Your tier**: Based on what you have right now, which tier are you starting in? Is that tier sufficient for what you want to learn?
- **Upgrade pathway**: If you wanted to upgrade tiers during the course, what's the minimal investment needed?
- **Constraints**: What's your biggest hardware constraint—budget, time, or something else? How does that affect your learning path?

These reflections help you make intentional decisions about hardware investment. Remember: **simulation learning is valid**. Many experts spend 90% of their time in simulation (Tier 1-2) and only 10% on real hardware (Tier 4). You don't need expensive equipment to learn effectively.
