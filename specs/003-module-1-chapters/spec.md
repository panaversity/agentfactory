# Feature Specification: Module 1 Chapter Architecture

**Feature Branch**: `003-module-1-chapters`
**Created**: 2025-11-29
**Status**: Draft
**Input**: Module 1 chapter architecture for Physical AI textbook - 7 chapters covering Physical AI foundations and ROS 2 fundamentals across 5 weeks (25 lessons)

## Overview

Design and implement the complete chapter architecture for Module 1: "The Robotic Nervous System (ROS 2)" of the Physical AI & Humanoid Robotics textbook. This module spans Weeks 1-5 of the 13-week curriculum and serves as the foundational learning experience for all students.

The architecture is based on comprehensive research across:
- Learning sciences (cognitive load theory, chunking, scaffolding, mastery learning)
- Official ROS 2 documentation structure
- Industry patterns from MIT, Stanford, and NVIDIA robotics curricula
- The Panaversity 4-Layer Teaching Method

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Module 1 Content (Priority: P1)

A beginner student (A2 proficiency) with Python knowledge but no robotics background navigates through Module 1 to learn Physical AI concepts and ROS 2 fundamentals.

**Why this priority**: This is the primary use case - students must be able to progress through all 7 chapters and 25 lessons to build ROS 2 competency.

**Independent Test**: Can be tested by having a student complete Chapter 1-7 sequentially, with mastery validation at each chapter boundary.

**Acceptance Scenarios**:

1. **Given** a student on the Module 1 landing page, **When** they click "Start Learning", **Then** they begin Chapter 1 Lesson 1 with clear learning objectives visible
2. **Given** a student completing Chapter 3, **When** they finish the mastery check, **Then** they can proceed to Chapter 4 (first coding chapter)
3. **Given** a Tier 1 (laptop-only) student, **When** they access any lesson, **Then** all exercises are completable via cloud ROS 2 or MockROS
4. **Given** a student in Chapter 7 Capstone, **When** they complete the project, **Then** they have a working multi-node ROS 2 system

---

### User Story 2 - Instructor Reviews Curriculum (Priority: P2)

An instructor or curriculum planner reviews Module 1 to understand the teaching arc, time requirements, and assessment points for institutional adoption.

**Why this priority**: Instructors need clear visibility into chapter structure, learning objectives, and assessment strategies to plan courses.

**Independent Test**: Can be tested by having an instructor extract a 5-week teaching schedule from Module 1 chapter READMEs.

**Acceptance Scenarios**:

1. **Given** an instructor reviewing Module 1, **When** they read all 7 chapter READMEs, **Then** they can map each chapter to specific weeks (1-5)
2. **Given** an instructor planning assessments, **When** they review chapter structure, **Then** they find clear mastery gates and the capstone project specification
3. **Given** an institution with hardware constraints, **When** reviewing hardware requirements, **Then** Tier 1 fallback paths are documented for every chapter

---

### User Story 3 - Author Extends Module Structure (Priority: P3)

A content author uses the Module 1 chapter architecture as a template for creating additional modules or books on the platform.

**Why this priority**: The chapter architecture should be reusable as a platform pattern for future content.

**Independent Test**: Can be tested by using Module 1 structure to outline Module 2 chapters.

**Acceptance Scenarios**:

1. **Given** an author creating Module 2, **When** they reference Module 1 architecture, **Then** they have a clear template for chapter structure, lesson count, and layer progression
2. **Given** a platform admin, **When** reviewing Module 1 implementation, **Then** reusable skills are documented and accessible

---

### Edge Cases

- What happens when a student skips Chapter 3 (CLI concepts) and jumps to Chapter 4 (coding)?
  - System shows prerequisite warning; chapter navigation enforces sequential progression
- How does system handle a Tier 1 student attempting Tier 2+ optional content?
  - Content marked with `<HardwareGate>` shows cloud fallback instructions
- What if a student fails the Chapter 7 capstone mastery check?
  - System provides remediation path with specific skill gaps identified

## Requirements *(mandatory)*

### Functional Requirements

**Chapter Structure Requirements:**

- **FR-001**: Module 1 MUST contain exactly 7 chapters with the following structure:
  - Chapter 1: What is Physical AI? (Week 1, 3 lessons)
  - Chapter 2: The Robot System (Week 1-2, 4 lessons)
  - Chapter 3: Meet ROS 2 (Week 2, 4 lessons)
  - Chapter 4: Your First ROS 2 Code (Week 3, 4 lessons)
  - Chapter 5: Communication Mastery (Week 3-4, 4 lessons)
  - Chapter 6: Building Robot Systems (Week 4-5, 3 lessons)
  - Chapter 7: Capstone - Robot Controller (Week 5, 3 lessons)

- **FR-002**: Each chapter MUST contain a README.md with:
  - Chapter overview (2-3 paragraphs)
  - Learning objectives (3-5 per chapter, action verbs)
  - Lesson index with titles, duration, and core concepts
  - Hardware tier requirements with Tier 1 fallback
  - 4-Layer Teaching Method progression indicators
  - Prerequisites (prior chapters required)
  - Navigation (previous/next chapter, first lesson CTA)

- **FR-003**: Each lesson MUST be 45-60 minutes in duration and cover 1-2 core concepts maximum

- **FR-004**: Module 1 MUST be completable by Tier 1 (laptop/cloud only) students for ALL required content

**Pedagogical Requirements:**

- **FR-005**: Layer progression MUST follow this pattern across chapters:
  - Chapters 1-2: L1 dominant (85-100% manual foundation)
  - Chapter 3: L1→L2 transition (60% manual, 40% AI collaboration)
  - Chapters 4-5: L2 dominant with L3 introduction (40-50% AI collab, 10-40% intelligence design)
  - Chapters 6-7: L3→L4 dominant (20-50% intelligence design, 20-80% spec-driven)

- **FR-006**: Each chapter MUST include a mastery gate before students proceed to the next chapter

- **FR-007**: Chapters 1-2 MUST NOT require any coding (conceptual foundation only)

- **FR-008**: Chapter 3 MUST teach ROS 2 concepts through CLI exploration BEFORE coding in Chapter 4

- **FR-009**: Chapter 7 capstone MUST be achievable using only concepts from Chapters 1-6 (no URDF, Actions, or tf2)

**Content Quality Requirements:**

- **FR-010**: All ROS 2 technical claims MUST cite official ROS 2 Humble documentation
- **FR-011**: Each chapter MUST include at least one Mermaid diagram visualizing key concepts
- **FR-012**: Every lesson MUST follow the worked example → guided practice → independent practice pattern
- **FR-013**: AI collaboration sections (L2) MUST demonstrate Three Roles framework WITHOUT exposing framework labels

**Reusable Intelligence Requirements:**

- **FR-014**: Module 1 MUST create these reusable skills:
  - `ros2-publisher-subscriber` (Chapter 4)
  - `ros2-service-pattern` (Chapter 5)
  - `ros2-custom-interfaces` (Chapter 5)
  - `ros2-launch-system` (Chapter 6)

### Key Entities

- **Module**: Top-level curriculum unit containing chapters (Module 1 = Weeks 1-5)
- **Chapter**: Thematic grouping of lessons (7 chapters in Module 1)
- **Lesson**: Single learning session (45-60 min, 1-2 concepts)
- **Learning Objective**: Measurable skill using action verbs (Bloom's taxonomy)
- **Hardware Tier**: Equipment classification (Tier 1-4) with fallback paths
- **Mastery Gate**: Assessment checkpoint between chapters
- **Reusable Skill**: Platform-level intelligence asset created during learning

## Module 1 Chapter Specifications

### Chapter 1: What is Physical AI?
- **Week**: 1
- **Lessons**: 3 (45 min each)
- **Layer Mix**: L1: 100%
- **Hardware Tier**: Tier 1 (conceptual, no coding)
- **Core Concepts**: Embodied intelligence, digital→physical AI, humanoid landscape

**Lesson Breakdown**:
1. From ChatGPT to Walking Robots (digital vs physical AI)
2. Embodied Intelligence (physical constraints: gravity, latency, safety)
3. The Humanoid Revolution (industry landscape: Unitree, Figure, Tesla Bot, NVIDIA)

**Learning Objectives**:
- Distinguish software AI from embodied AI
- Explain why physical world constraints matter for robotics
- Survey the humanoid robotics ecosystem and key players

---

### Chapter 2: The Robot System
- **Week**: 1-2
- **Lessons**: 4 (45 min each)
- **Layer Mix**: L1: 85%, L2: 15%
- **Hardware Tier**: Tier 1 (conceptual diagrams, interactive tier selector)
- **Core Concepts**: Sensors, actuators, middleware, hardware tiers

**Lesson Breakdown**:
1. How Robots See: Sensors (LIDAR, cameras, IMU, force/torque)
2. How Robots Move: Actuators (motors, joints, control loops)
3. The Glue: Why Middleware Exists (the problem ROS solves)
4. Your Hardware Tier (self-assessment, learning paths per tier)

**Learning Objectives**:
- Categorize sensor types (proprioceptive vs exteroceptive)
- Explain the role of actuators and control systems
- Understand why middleware is essential for robotics
- Identify personal hardware tier and appropriate learning path

---

### Chapter 3: Meet ROS 2
- **Week**: 2
- **Lessons**: 4 (45-60 min each)
- **Layer Mix**: L1: 60%, L2: 40%
- **Hardware Tier**: Tier 1 (cloud ROS 2 or local)
- **Core Concepts**: Environment setup, turtlesim, nodes, topics, services (CLI only)

**Lesson Breakdown**:
1. Setting Up Your ROS 2 Environment (installation OR cloud setup, sourcing)
2. Turtlesim: ROS in Action (visual feedback, ros2 run, rqt)
3. Nodes & Topics: The Communication Model (ros2 node list, ros2 topic echo)
4. Services & Parameters: Request/Response (ros2 service call, ros2 param)

**Learning Objectives**:
- Configure ROS 2 environment (local or cloud)
- Use turtlesim to visualize ROS concepts
- Explore nodes and topics using ros2 CLI commands
- Understand services and parameters through CLI exploration

**Critical Note**: NO Python coding in this chapter - CLI exploration only

---

### Chapter 4: Your First ROS 2 Code
- **Week**: 3
- **Lessons**: 4 (45-60 min each)
- **Layer Mix**: L1: 40%, L2: 50%, L3: 10%
- **Hardware Tier**: Tier 1 (cloud ROS 2 recommended)
- **Core Concepts**: Workspaces, packages, publisher, subscriber (Python)

**Lesson Breakdown**:
1. Workspaces & Packages (colcon, workspace structure, package.xml)
2. Writing a Publisher (rclpy node, Timer, publishing messages)
3. Writing a Subscriber (callbacks, message handling)
4. Try With AI: Extend Your Nodes (AI collaboration to add features)

**Learning Objectives**:
- Build and manage ROS 2 packages with colcon
- Create a proper ROS 2 workspace structure
- Write a publisher node in Python (rclpy)
- Write a subscriber node in Python (rclpy)

**Reusable Skill Created**: `ros2-publisher-subscriber`

---

### Chapter 5: Communication Mastery
- **Week**: 3-4
- **Lessons**: 4 (45-60 min each)
- **Layer Mix**: L2: 50%, L3: 40%, L4: 10%
- **Hardware Tier**: Tier 1 (cloud ROS 2)
- **Core Concepts**: Services (Python), custom messages, topic vs service design

**Lesson Breakdown**:
1. Writing a Service Server (service definition, server implementation)
2. Writing a Service Client (async calls, handling responses)
3. Custom Messages & Services (.msg/.srv files, interface packages)
4. Design Patterns: Topics vs Services (when to use each, architecture decisions)

**Learning Objectives**:
- Implement service servers and clients in Python
- Create custom message and service interfaces
- Design appropriate communication patterns for different scenarios
- Write mini-specifications before implementation (L4 preview)

**Reusable Skills Created**: `ros2-service-pattern`, `ros2-custom-interfaces`

---

### Chapter 6: Building Robot Systems
- **Week**: 4-5
- **Lessons**: 3 (45-60 min each)
- **Layer Mix**: L2: 30%, L3: 50%, L4: 20%
- **Hardware Tier**: Tier 1 (cloud ROS 2)
- **Core Concepts**: Parameters, launch files, multi-node debugging

**Lesson Breakdown**:
1. Parameters: Configurable Nodes (declaring, reading, setting parameters)
2. Launch Files: Starting Systems (Python launch files, multi-node startup)
3. Debugging Multi-Node Systems (ros2doctor, rqt_graph, logging)

**Learning Objectives**:
- Add configurable parameters to Python nodes
- Write Python launch files to start multiple nodes
- Use ros2doctor and rqt_graph to diagnose issues
- Debug multi-node systems systematically

**Reusable Skill Created**: `ros2-launch-system`

---

### Chapter 7: Capstone - Robot Controller
- **Week**: 5
- **Lessons**: 3 (45-90 min each)
- **Layer Mix**: L3: 20%, L4: 80%
- **Hardware Tier**: Tier 1 (turtlesim/MockROS simulation)
- **Core Concepts**: Full system integration, spec-driven development

**Lesson Breakdown**:
1. Capstone Specification (write spec FIRST, define interfaces)
2. Building the Controller (implement from spec, guided integration)
3. Testing, Validation & Reflection (debug, validate, preview Module 2)

**Capstone Deliverable**:
- Multi-node system (3+ nodes)
- Publisher/subscriber for continuous data
- Service for commands
- Parameters for configuration
- Launch file to start everything
- Controls turtlesim OR MockROS robot

**What's NOT in Capstone** (deferred to Module 2):
- URDF (robot description)
- Actions (goal-based communication)
- tf2 (transforms)
- Gazebo integration

**Learning Objectives**:
- Design a multi-node system from specification
- Integrate nodes, topics, services, parameters, and launch files
- Debug and validate a complete ROS 2 system
- Apply spec-driven development methodology

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can complete all 25 lessons in Module 1 within 25-30 hours of study time
- **SC-002**: 90% of Tier 1 (laptop-only) students can complete the Chapter 7 capstone without hardware upgrades
- **SC-003**: Students demonstrate mastery progression from L1 to L4 by achieving 80%+ on each chapter's mastery gate
- **SC-004**: Chapter 7 capstone produces a working multi-node ROS 2 system that responds to commands
- **SC-005**: All 7 chapter READMEs contain the required sections (overview, objectives, lessons, hardware, navigation)
- **SC-006**: 4 reusable skills are created and documented in `.claude/skills/` by end of module
- **SC-007**: Instructor can map Module 1 to a 5-week teaching schedule using chapter READMEs alone
- **SC-008**: Content renders correctly in Docusaurus with proper MDX formatting and Mermaid diagrams

## Constraints

- **C-001**: All content MUST be written in MDX format compatible with Docusaurus 3.x
- **C-002**: All diagrams MUST use Mermaid syntax (no external image dependencies)
- **C-003**: Every chapter MUST have Tier 1 (laptop/cloud) fallback - no content requires physical hardware
- **C-004**: Chapter READMEs MUST NOT contain actual lesson content (structure and navigation only)
- **C-005**: No Actions, URDF, or tf2 content in Module 1 (deferred to Module 2)
- **C-006**: Lesson duration MUST NOT exceed 60 minutes for cognitive load management
- **C-007**: Each lesson MUST cover maximum 2 core concepts

## Non-Goals

- **NG-001**: This specification does NOT include individual lesson MDX content (separate implementation task)
- **NG-002**: This specification does NOT implement interactive components (PythonRunner, MockROS)
- **NG-003**: This specification does NOT include Urdu translations (separate localization feature)
- **NG-004**: This specification does NOT cover Modules 2-4 chapter architecture
- **NG-005**: This specification does NOT implement the RAG chatbot integration
- **NG-006**: This specification does NOT include video content or NotebookLM slides

## Assumptions

- **A-001**: Students have Python programming knowledge (functions, classes, loops)
- **A-002**: Students have basic Linux/terminal familiarity (ls, cd, pwd)
- **A-003**: Cloud ROS 2 environment (TheConstruct or similar) is available for Tier 1 students
- **A-004**: Docusaurus project structure exists at `docs/` directory
- **A-005**: The 4-Layer Teaching Method is documented in constitution and understood by validators
- **A-006**: Module README (from spec 002-module-content) will be created separately
- **A-007**: Official ROS 2 Humble documentation is the authoritative source for all technical claims

## Dependencies

- **D-001**: Constitution v1.0.0 defines 4-Layer Teaching Method and Hardware Tiers
- **D-002**: Spec 002-module-content defines Module README template and requirements
- **D-003**: Phase 1 infrastructure (Docusaurus setup) is complete
- **D-004**: ROS 2 Humble is the target distribution for all content

## Deliverables

### Chapter READMEs (7 files)
1. `docs/module-1-ros2/chapter-1-physical-ai/index.md`
2. `docs/module-1-ros2/chapter-2-robot-system/index.md`
3. `docs/module-1-ros2/chapter-3-meet-ros2/index.md`
4. `docs/module-1-ros2/chapter-4-first-code/index.md`
5. `docs/module-1-ros2/chapter-5-communication/index.md`
6. `docs/module-1-ros2/chapter-6-building-systems/index.md`
7. `docs/module-1-ros2/chapter-7-capstone/index.md`

### Supporting Artifacts
- Module 1 layer progression diagram (Mermaid)
- Chapter navigation sidebar configuration
- Reusable skill templates (4 skills documented)

## Research Sources

All technical content verified against:
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials - Beginner CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS 2 Tutorials - Beginner Client Libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)
- Learning sciences research on cognitive load, chunking, and scaffolding
- MIT/Stanford/NVIDIA robotics curriculum patterns
