# Feature Specification: Module 2 - Gazebo/Unity Simulation

**Feature Branch**: `004-module-2-simulation`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "Module 2: Gazebo/Unity Simulation - Create complete module covering robot simulation with Gazebo Harmonic, URDF/SDF robot descriptions, sensor simulation, ROS 2 integration, and sim-to-real concepts"

---

## Overview

Module 2 teaches robotics simulation using Gazebo Harmonic (gz-sim) as the primary simulation platform, with foundational concepts applicable to Unity. Students progress from understanding why simulation matters, through building robot models and worlds, to integrating simulations with ROS 2 for closed-loop control. This module bridges the ROS 2 fundamentals from Module 1 to the AI training workflows in Module 3 (Isaac Sim).

**Target Audience**: Students who completed Module 1 (ROS 2 Foundations)
**Duration**: ~4 weeks of content (22 lessons across 6 chapters)
**Proficiency Progression**: A2 (beginner) → B1 (intermediate)
**Hardware Tiers**: Tier 1 (cloud) primary, Tier 2 (local GPU) for advanced features

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learn Simulation Fundamentals (Priority: P1)

A student who completed Module 1 wants to understand why simulation is essential in robotics development before touching physical hardware. They need to grasp the "simulation-first" philosophy and understand Gazebo's architecture.

**Why this priority**: Foundation for all subsequent learning. Without understanding WHY to simulate, students lack motivation for the technical content.

**Independent Test**: Student can explain the simulation-first development workflow and identify 3 reasons why simulation precedes physical deployment.

**Acceptance Scenarios**:

1. **Given** a student has completed Module 1, **When** they complete Chapter 8 (Why Simulate?), **Then** they can articulate the digital twin concept and its benefits in robotics.
2. **Given** a student reads the Gazebo overview lesson, **When** they access a cloud simulation environment, **Then** they can identify Gazebo's client-server architecture and plugin system.

---

### User Story 2 - Create Robot Models with URDF (Priority: P1)

A student wants to create their own robot models using URDF (Unified Robot Description Format). They need to understand links, joints, visual geometry, and physical properties (mass, inertia, collision).

**Why this priority**: Robot models are the foundation of all simulation work. Without a robot model, no simulation exercises are possible.

**Independent Test**: Student can create a valid URDF file for a simple mobile robot with 2 wheels, chassis, and correct joint definitions that loads without errors in Gazebo.

**Acceptance Scenarios**:

1. **Given** a student understands XML basics, **When** they complete Chapter 9 (Robot Description Formats), **Then** they can write a URDF file with at least 3 links and 2 joints.
2. **Given** a student has a URDF file, **When** they add physical properties (mass, inertia), **Then** the robot behaves realistically under physics simulation (falls if top-heavy, rolls if wheeled).
3. **Given** a student uses AI collaboration (L2), **When** they prompt AI to generate URDF, **Then** they can evaluate and correct the AI output for errors.

---

### User Story 3 - Build Simulation Worlds (Priority: P2)

A student wants to create simulation environments (worlds) where robots operate. They need to configure ground planes, obstacles, lighting, and physics parameters.

**Why this priority**: Worlds provide context for robot testing. Essential for meaningful simulation but robots can be tested in default empty worlds initially.

**Independent Test**: Student can create an SDF world file with ground plane, 3+ obstacles, and correct physics configuration that loads in Gazebo.

**Acceptance Scenarios**:

1. **Given** a student understands SDF basics, **When** they complete Chapter 10 (Building Simulation Worlds), **Then** they can create a world with ground, walls, and imported Fuel models.
2. **Given** a student configures physics, **When** they drop an object in simulation, **Then** gravity, friction, and collision behave realistically.
3. **Given** a student uses AI collaboration (L2), **When** they describe a desired world, **Then** AI generates an SDF file they can refine and use.

---

### User Story 4 - Simulate Robot Sensors (Priority: P2)

A student wants to add sensors (camera, LIDAR, IMU) to their robot and receive realistic sensor data in simulation.

**Why this priority**: Sensors are how robots perceive the world. Critical for perception algorithms but depends on having a working robot model first.

**Independent Test**: Student can configure a camera and LIDAR sensor on their robot and view the sensor output (images, point clouds) in simulation.

**Acceptance Scenarios**:

1. **Given** a student has a robot model, **When** they add a camera sensor with correct SDF configuration, **Then** they can view simulated camera images from the robot's perspective.
2. **Given** a student configures a LIDAR sensor, **When** obstacles are present, **Then** the LIDAR topic publishes distance measurements matching obstacle positions.
3. **Given** a student configures an IMU sensor, **When** the robot moves, **Then** IMU data reflects acceleration and orientation changes.

---

### User Story 5 - Connect ROS 2 to Gazebo (Priority: P1)

A student wants to control their simulated robot using ROS 2 nodes they built in Module 1. They need to bridge Gazebo topics to ROS 2 topics.

**Why this priority**: ROS 2 integration is the core learning outcome. This connects Module 1 knowledge to simulation, enabling closed-loop control.

**Independent Test**: Student can send velocity commands from a ROS 2 node and see the simulated robot move; sensor data flows from Gazebo to ROS 2 topics.

**Acceptance Scenarios**:

1. **Given** a student has Gazebo running with a robot, **When** they configure ros_gz_bridge, **Then** Gazebo topics appear as ROS 2 topics.
2. **Given** bridge is configured, **When** student publishes cmd_vel from ROS 2, **Then** the simulated robot moves accordingly.
3. **Given** robot has sensors, **When** bridge is configured, **Then** sensor data (images, scans) is accessible via ROS 2 subscribers.

---

### User Story 6 - Complete Capstone Project (Priority: P3)

A student wants to demonstrate integrated learning by building a complete simulation: robot model + world + sensors + ROS 2 control for a specific task (e.g., navigate to goal, avoid obstacles).

**Why this priority**: Capstone validates learning but requires all prior skills. Lower priority because it depends on successful completion of P1/P2 stories.

**Independent Test**: Student can deliver a working simulation demo that meets a specification they wrote, with the robot completing a defined task.

**Acceptance Scenarios**:

1. **Given** a student completes all chapters, **When** they write a capstone specification, **Then** the spec includes robot requirements, world setup, sensor configuration, and success criteria.
2. **Given** a specification exists, **When** student implements using accumulated skills, **Then** the robot successfully completes the specified task in simulation.
3. **Given** capstone is complete, **When** evaluated against spec, **Then** all success criteria are met and student can explain sim-to-real considerations.

---

### Edge Cases

- What happens when URDF has syntax errors? System should provide clear error messages and lesson content covers debugging techniques.
- What happens when Gazebo physics becomes unstable? Lessons cover physics parameter tuning and common stability issues.
- What happens when ros_gz_bridge message types don't match? Lessons cover bridge configuration debugging and message type mapping.
- What happens when student has only Tier 1 (cloud) access? All core content works via TheConstruct cloud environment; Tier 2 features clearly marked.
- What happens when student's local GPU doesn't meet requirements? Clear hardware requirements documented; cloud fallback always available.

---

## Requirements *(mandatory)*

### Functional Requirements

**Content Structure**:
- **FR-001**: Module MUST contain 6 chapters (Chapters 8-13, continuing from Module 1 numbering)
- **FR-002**: Module MUST contain 22 lessons distributed across chapters as specified in architecture
- **FR-003**: Each lesson MUST follow the lesson-generator skill metadata requirements
- **FR-004**: All lessons MUST have Tier 1 (cloud) execution path available
- **FR-005**: Lessons requiring Tier 2+ MUST include `<HardwareGate>` markers

**Learning Progression**:
- **FR-006**: Chapter 8 MUST establish conceptual foundation (Layer 1, no AI collaboration yet)
- **FR-007**: Chapters 9-11 MUST progress from L1 (manual) to L2 (AI collaboration with Three Roles)
- **FR-008**: Chapter 12 MUST include L3 content (creating reusable skills for ros_gz integration)
- **FR-009**: Chapter 13 (Capstone) MUST follow L4 spec-driven approach
- **FR-010**: Proficiency MUST progress from A2 (Chapters 8-9) to B1 (Chapters 10-13)

**Technical Accuracy**:
- **FR-011**: All content MUST use Gazebo Harmonic (gz-sim) patterns, NOT Gazebo Classic
- **FR-012**: URDF content MUST use current best practices (xacro for modularity)
- **FR-013**: ROS 2 content MUST target Humble distribution (consistent with Module 1)
- **FR-014**: ros_gz_bridge syntax MUST match official documentation (`/TOPIC@ROS_MSG@GZ_MSG`)
- **FR-015**: Sensor configurations MUST use current gz-sim plugin names (not ignition-gazebo)

**Pedagogical Requirements**:
- **FR-016**: L2 lessons MUST demonstrate Three Roles framework (AI as Teacher/Student/Co-Worker) without exposing framework labels
- **FR-017**: Each chapter MUST have measurable learning objectives using Bloom's taxonomy verbs
- **FR-018**: Code examples MUST include expected output within 5 lines
- **FR-019**: Lessons MUST end with "Try With AI" section (no trailing summary/takeaways)
- **FR-020**: Cognitive load MUST respect tier limits (A2: 5-7 concepts, B1: 7-10 concepts)

**Platform Compliance**:
- **FR-021**: All files MUST use `.md` extension (not `.mdx`)
- **FR-022**: Index files MUST be named `README.md` (not `index.md`)
- **FR-023**: No Mermaid diagrams (plugin not configured)
- **FR-024**: No `<` characters in prose outside code blocks
- **FR-025**: All metadata MUST use standardized fields (`proficiency_level`, `duration_minutes`, etc.)

**Skills to Create**:
- **FR-026**: Module MUST produce `urdf-robot-model` skill in `.claude/skills/authoring/`
- **FR-027**: Module MUST produce `gazebo-world-builder` skill in `.claude/skills/authoring/`
- **FR-028**: Module MUST produce `sensor-simulation` skill in `.claude/skills/authoring/`
- **FR-029**: Module MUST produce `ros2-gazebo-bridge` skill in `.claude/skills/authoring/`

### Key Entities

- **Module**: Top-level content unit containing multiple chapters; has title, description, learning outcomes, hardware requirements
- **Chapter**: Collection of lessons on a focused topic; has number, title, layer designation, proficiency level
- **Lesson**: Individual learning unit; has metadata (id, title, duration, objectives, tier), content sections, exercises
- **Skill**: Reusable AI intelligence pattern; has persona, questions, principles structure
- **Robot Model (URDF)**: XML description of robot structure; has links, joints, visual/collision geometry, physical properties
- **World (SDF)**: XML description of simulation environment; has ground, models, physics config, lighting
- **Sensor**: Simulated perception device; has type (camera/LIDAR/IMU), configuration, output topic
- **Bridge Configuration**: ros_gz_bridge setup; has topic mappings, direction (GZ_TO_ROS/ROS_TO_GZ/BIDIRECTIONAL)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Content Delivery**:
- **SC-001**: All 22 lessons pass Docusaurus build without errors
- **SC-002**: All lessons have complete metadata (0 missing required fields)
- **SC-003**: 100% of lessons have Tier 1 cloud execution path documented
- **SC-004**: All code examples include expected output

**Learning Outcomes**:
- **SC-005**: Student can create valid URDF robot model from scratch (measurable: loads in Gazebo without errors)
- **SC-006**: Student can build SDF world with physics (measurable: objects fall, collide, slide with friction)
- **SC-007**: Student can configure camera and LIDAR sensors (measurable: sensor topics publish data)
- **SC-008**: Student can bridge Gazebo to ROS 2 (measurable: ros2 topic list shows bridged topics)
- **SC-009**: Student completes capstone project meeting their own specification (measurable: all spec criteria pass)

**Pedagogical Quality**:
- **SC-010**: 0 lessons expose Three Roles framework labels to students (grep validation)
- **SC-011**: 100% of L2+ lessons demonstrate bidirectional AI collaboration
- **SC-012**: All lessons end with student action section (no trailing summaries)
- **SC-013**: Cognitive load within tier limits for all lessons

**Platform Intelligence**:
- **SC-014**: 4 new skills created and documented (URDF, world, sensor, bridge)
- **SC-015**: Skills follow standard structure (persona + questions + principles)
- **SC-016**: Skills are reusable for future robotics content

---

## Assumptions

1. **TheConstruct cloud environment** provides adequate Gazebo Harmonic + ROS 2 Humble support for Tier 1 students
2. **Gazebo Fuel** models are available and accessible for world building lessons
3. **Students have completed Module 1** and understand ROS 2 nodes, topics, services, and launch files
4. **Ubuntu 22.04** is the assumed local development environment for Tier 2 students
5. **Unity content** is secondary/optional and will be covered briefly; Gazebo is the primary focus
6. **Sim-to-real gap** concepts are introduced but detailed transfer learning is deferred to Module 3 (Isaac Sim)

---

## Constraints

1. **No Gazebo Classic patterns** - All content must use gz-sim (Gazebo Harmonic) APIs and patterns
2. **ROS 2 Humble only** - Must match Module 1 distribution; no ROS 2 Jazzy content
3. **Docusaurus conventions** - Must follow `.md` files, `README.md` for indexes, no Mermaid
4. **4-Layer progression** - Must follow L1→L2→L3→L4 teaching framework
5. **Hardware tier awareness** - Every lesson must work for Tier 1 students; advanced features gated

---

## Non-Goals

1. **Full Unity integration** - Unity is mentioned conceptually but no hands-on Unity exercises
2. **Advanced physics tuning** - Beyond basic friction/collision; advanced physics is Module 3 content
3. **Multi-robot simulation** - Focus is single robot; swarm simulation is advanced topic
4. **Real hardware deployment** - Sim-to-real transfer discussed but not executed (Module 4 content)
5. **Custom Gazebo plugins** - Using existing plugins only; writing C++ plugins is advanced content
6. **MoveIt 2 integration** - Motion planning is Module 3+ content
7. **Navigation stack (Nav2)** - Full navigation is separate learning path, though basic motion is covered

---

## Chapter Architecture

### Chapter 8: Why Simulate? (Conceptual Foundation)
- **Layer**: L1 (Manual Foundation)
- **Tier**: 1 (Cloud)
- **Proficiency**: A2
- **Lessons**: 3
  - 8.1 The Digital Twin Concept
  - 8.2 Simulation-First Development
  - 8.3 Meet Gazebo Harmonic

### Chapter 9: Robot Description Formats
- **Layer**: L1 → L2
- **Tier**: 1
- **Proficiency**: A2
- **Lessons**: 4
  - 9.1 Understanding URDF
  - 9.2 Building Your First Robot
  - 9.3 Adding Physical Properties
  - 9.4 URDF with AI (L2: Three Roles)

### Chapter 10: Building Simulation Worlds
- **Layer**: L1 → L2
- **Tier**: 1-2
- **Proficiency**: A2 → B1
- **Lessons**: 4
  - 10.1 SDF World Basics
  - 10.2 Adding Models from Fuel
  - 10.3 Physics Configuration
  - 10.4 World Building with AI (L2: Three Roles)

### Chapter 11: Sensors in Simulation
- **Layer**: L1 → L2
- **Tier**: 1-2
- **Proficiency**: B1
- **Lessons**: 4
  - 11.1 Camera Simulation
  - 11.2 LIDAR Simulation
  - 11.3 IMU and Contact Sensors
  - 11.4 Sensor Debugging and Visualization

### Chapter 12: ROS 2 + Gazebo Integration
- **Layer**: L2 → L3
- **Tier**: 1-2
- **Proficiency**: B1
- **Lessons**: 4
  - 12.1 The ros_gz Bridge
  - 12.2 Spawning Robots from ROS 2
  - 12.3 Closed-Loop Control
  - 12.4 Creating ros_gz Skills (L3: Intelligence Design)

### Chapter 13: Module 2 Capstone
- **Layer**: L4 (Spec-Driven)
- **Tier**: 1-2
- **Proficiency**: B1
- **Lessons**: 3
  - 13.1 Capstone Specification
  - 13.2 Building the Simulation
  - 13.3 Testing, Validation, and Sim-to-Real Preview

---

## Dependencies

### Prerequisites
- Module 1: ROS 2 Foundations (all 7 chapters, 25 lessons)
- Basic XML understanding (covered briefly in Module 1)
- Command line proficiency (Ubuntu/WSL)

### Downstream
- Module 3: NVIDIA Isaac Sim (builds on simulation concepts)
- Module 4: VLA/Embodied AI (uses sim-to-real concepts)

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| TheConstruct cloud unavailable | Low | High | Document alternative cloud options; provide local setup guide |
| Gazebo Harmonic API changes | Low | Medium | Pin to specific Gazebo version; document version in lessons |
| URDF/SDF learning curve too steep | Medium | Medium | Progressive difficulty; heavy scaffolding in early lessons |
| ros_gz_bridge complexity | Medium | Medium | Provide working YAML configs; step-by-step debugging guide |
| Students skip Module 1 | Medium | High | Clear prerequisites; diagnostic quiz at module start |
