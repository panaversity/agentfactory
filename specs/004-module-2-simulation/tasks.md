# Tasks: Module 2 - Gazebo/Unity Simulation

**Input**: Design documents from `/specs/004-module-2-simulation/`
**Prerequisites**: spec.md (user stories), plan.md (chapter architecture)

**Tests**: Not explicitly requested - validation is via Docusaurus build and content quality checks.

**Organization**: Tasks grouped by user story to enable independent implementation. Each chapter maps to specific user stories.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)
- All paths relative to `robolearn-interface/docs/module-2-simulation/`

## Path Conventions

```
robolearn-interface/docs/module-2-simulation/
├── README.md                           # Module overview
├── chapter-8-why-simulate/             # US1: Simulation Fundamentals
├── chapter-9-robot-description/        # US2: URDF Robot Models
├── chapter-10-simulation-worlds/       # US3: World Building
├── chapter-11-sensors-simulation/      # US4: Sensor Simulation
├── chapter-12-ros2-gazebo-integration/ # US5: ROS 2 + Gazebo
└── chapter-13-capstone/                # US6: Capstone Project
```

---

## Phase 1: Setup (Module Infrastructure)

**Purpose**: Create module structure and chapter scaffolding

- [ ] T001 Create module directory structure per plan.md at `robolearn-interface/docs/module-2-simulation/`
- [ ] T002 [P] Create module README.md with overview, prerequisites, learning outcomes at `README.md`
- [ ] T003 [P] Create chapter-8-why-simulate/ directory with README.md
- [ ] T004 [P] Create chapter-9-robot-description/ directory with README.md
- [ ] T005 [P] Create chapter-10-simulation-worlds/ directory with README.md
- [ ] T006 [P] Create chapter-11-sensors-simulation/ directory with README.md
- [ ] T007 [P] Create chapter-12-ros2-gazebo-integration/ directory with README.md
- [ ] T008 [P] Create chapter-13-capstone/ directory with README.md
- [ ] T009 Verify Docusaurus sidebar configuration includes Module 2 chapters

**Checkpoint**: Module structure ready - lesson implementation can begin

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Ensure lesson-generator skill and docusaurus-conventions are understood

**CRITICAL**: No lesson content can be written until author understands these patterns

- [ ] T010 Read `.claude/skills/authoring/lesson-generator/SKILL.md` and understand metadata requirements
- [ ] T011 Read `.claude/skills/authoring/docusaurus-conventions/SKILL.md` and verify all constraints
- [ ] T012 Read Module 1 reference lesson at `robolearn-interface/docs/module-1-ros2/chapter-1-physical-ai/01-digital-to-physical.md` for quality standard
- [ ] T013 Verify TheConstruct cloud environment supports Gazebo Harmonic + ROS 2 Humble (Tier 1 path)
- [ ] T014 Document Tier 1 cloud execution path in each chapter README

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learn Simulation Fundamentals (Priority: P1)

**Goal**: Student understands WHY simulation matters and Gazebo's architecture

**Independent Test**: Student can explain simulation-first workflow and identify 3 reasons why simulation precedes physical deployment

**Chapter**: 8 (Why Simulate?) | **Layer**: L1 | **Proficiency**: A2

### Implementation for User Story 1

- [ ] T015 [P] [US1] Create lesson 8.1 "Digital Twin Concept" at `chapter-8-why-simulate/01-digital-twin-concept.md`
  - Metadata: id, title, duration_minutes: 45, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: What is a digital twin, benefits in robotics, examples from industry
  - End with "Try With AI" section using ChatGPT web (L1, no AI tools onboarded yet)

- [ ] T016 [P] [US1] Create lesson 8.2 "Simulation-First Development" at `chapter-8-why-simulate/02-simulation-first.md`
  - Metadata: duration_minutes: 45, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: Why simulate before physical deployment, cost/safety/iteration benefits
  - Case studies: Tesla Bot, Boston Dynamics, Waymo simulation
  - End with "Try With AI" section

- [ ] T017 [P] [US1] Create lesson 8.3 "Meet Gazebo Harmonic" at `chapter-8-why-simulate/03-meet-gazebo.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: Gazebo Harmonic (gz-sim) architecture, client-server model, plugin system
  - Tier 1 path: Access via TheConstruct cloud environment
  - End with "Try With AI" section

- [ ] T018 [US1] Update chapter-8 README.md with learning objectives and lesson links
- [ ] T019 [US1] Verify all Chapter 8 lessons have correct frontmatter metadata
- [ ] T020 [US1] Run Docusaurus build to validate Chapter 8 content

**Checkpoint**: User Story 1 complete - Student understands simulation fundamentals

---

## Phase 4: User Story 2 - Create Robot Models with URDF (Priority: P1)

**Goal**: Student can create valid URDF robot models with links, joints, and physical properties

**Independent Test**: Student creates URDF for 2-wheel mobile robot that loads in Gazebo without errors

**Chapter**: 9 (Robot Description Formats) | **Layer**: L1→L2 | **Proficiency**: A2

### Implementation for User Story 2

- [ ] T021 [P] [US2] Create lesson 9.1 "Understanding URDF" at `chapter-9-robot-description/01-understanding-urdf.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: URDF basics, XML structure, links and joints explained
  - Code examples with expected output (error messages, successful parse)
  - End with "Try With AI" section

- [ ] T022 [P] [US2] Create lesson 9.2 "Building Your First Robot" at `chapter-9-robot-description/02-building-first-robot.md`
  - Metadata: duration_minutes: 75, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: Step-by-step URDF creation, chassis + 2 wheels, joint types
  - Hands-on: Create simple_robot.urdf from scratch
  - Tier 1: Execute in TheConstruct
  - End with "Try With AI" section

- [ ] T023 [P] [US2] Create lesson 9.3 "Adding Physical Properties" at `chapter-9-robot-description/03-adding-physical-properties.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: Mass, inertia, collision geometry, why physics properties matter
  - Exercise: Add physics to simple_robot.urdf, test in Gazebo
  - End with "Try With AI" section

- [ ] T024 [P] [US2] Create lesson 9.4 "URDF with AI" at `chapter-9-robot-description/04-urdf-with-ai.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L2, hardware_tier: 1
  - Content: AI-assisted URDF generation, evaluating AI output, correcting errors
  - THREE ROLES (invisible): AI generates URDF → Student evaluates → Iterate
  - NO framework labels exposed (Three Roles framework INVISIBLE)
  - End with "Try With AI" section (now students can use preferred AI tool)

- [ ] T025 [US2] Update chapter-9 README.md with learning objectives and lesson links
- [ ] T026 [US2] Verify Three Roles framework is INVISIBLE in lesson 9.4 (grep for forbidden labels)
- [ ] T027 [US2] Create urdf-robot-model skill at `.claude/skills/authoring/urdf-robot-model/SKILL.md`
  - Structure: Persona + Questions + Principles
  - Scope: URDF design, links, joints, physical properties, xacro patterns

- [ ] T028 [US2] Run Docusaurus build to validate Chapter 9 content

**Checkpoint**: User Story 2 complete - Student can create URDF robot models

---

## Phase 5: User Story 3 - Build Simulation Worlds (Priority: P2)

**Goal**: Student can create SDF world files with ground, obstacles, and physics configuration

**Independent Test**: Student creates SDF world with ground plane, 3+ obstacles, correct physics that loads in Gazebo

**Chapter**: 10 (Building Simulation Worlds) | **Layer**: L1→L2 | **Proficiency**: A2→B1

### Implementation for User Story 3

- [ ] T029 [P] [US3] Create lesson 10.1 "SDF World Basics" at `chapter-10-simulation-worlds/01-sdf-world-basics.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: SDF format, world structure, ground plane, lighting
  - Difference from URDF, when to use each
  - End with "Try With AI" section

- [ ] T030 [P] [US3] Create lesson 10.2 "Adding Models from Fuel" at `chapter-10-simulation-worlds/02-adding-models-from-fuel.md`
  - Metadata: duration_minutes: 60, proficiency_level: A2, layer: L1, hardware_tier: 1
  - Content: Gazebo Fuel repository, downloading models, placing in world
  - Exercise: Import table, chair, box from Fuel
  - End with "Try With AI" section

- [ ] T031 [P] [US3] Create lesson 10.3 "Physics Configuration" at `chapter-10-simulation-worlds/03-physics-configuration.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L1, hardware_tier: 1-2
  - Content: Physics engines, gravity, friction, collision parameters
  - Debugging: Unstable physics, parameter tuning
  - HardwareGate: Mark Tier 2 content for local GPU
  - End with "Try With AI" section

- [ ] T032 [P] [US3] Create lesson 10.4 "World Building with AI" at `chapter-10-simulation-worlds/04-world-building-with-ai.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L2, hardware_tier: 1
  - Content: AI-assisted world generation, describing environments, iterating on SDF
  - THREE ROLES (invisible): Student describes → AI generates → Student refines
  - End with "Try With AI" section

- [ ] T033 [US3] Update chapter-10 README.md with learning objectives and lesson links
- [ ] T034 [US3] Verify Three Roles framework is INVISIBLE in lesson 10.4
- [ ] T035 [US3] Create gazebo-world-builder skill at `.claude/skills/authoring/gazebo-world-builder/SKILL.md`
  - Structure: Persona + Questions + Principles
  - Scope: World design, physics config, Fuel models, environment construction

- [ ] T036 [US3] Run Docusaurus build to validate Chapter 10 content

**Checkpoint**: User Story 3 complete - Student can build simulation worlds

---

## Phase 6: User Story 4 - Simulate Robot Sensors (Priority: P2)

**Goal**: Student can add camera, LIDAR, IMU sensors to robot and view sensor output

**Independent Test**: Student configures camera and LIDAR sensor, views images and point clouds in simulation

**Chapter**: 11 (Sensors in Simulation) | **Layer**: L1→L2 | **Proficiency**: B1

### Implementation for User Story 4

- [ ] T037 [P] [US4] Create lesson 11.1 "Camera Simulation" at `chapter-11-sensors-simulation/01-camera-simulation.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L1, hardware_tier: 1-2
  - Content: Camera sensor SDF configuration, resolution, FOV, image topics
  - Exercise: Add camera to robot, view images in gz-gui
  - HardwareGate for Tier 2 GPU rendering
  - End with "Try With AI" section

- [ ] T038 [P] [US4] Create lesson 11.2 "LIDAR Simulation" at `chapter-11-sensors-simulation/02-lidar-simulation.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L1, hardware_tier: 1-2
  - Content: LIDAR sensor configuration, range, resolution, noise models
  - Exercise: Add LIDAR to robot, visualize point cloud
  - End with "Try With AI" section

- [ ] T039 [P] [US4] Create lesson 11.3 "IMU and Contact Sensors" at `chapter-11-sensors-simulation/03-imu-contact-sensors.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L1, hardware_tier: 1
  - Content: IMU configuration, acceleration/gyro data, contact sensors
  - Exercise: Add IMU to robot, monitor orientation during movement
  - End with "Try With AI" section

- [ ] T040 [P] [US4] Create lesson 11.4 "Sensor Debugging and Visualization" at `chapter-11-sensors-simulation/04-sensor-debugging-visualization.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L2, hardware_tier: 1-2
  - Content: Debugging sensor output, visualization tools, noise analysis
  - THREE ROLES (invisible): Student identifies issue → AI suggests fixes → Iterate
  - End with "Try With AI" section

- [ ] T041 [US4] Update chapter-11 README.md with learning objectives and lesson links
- [ ] T042 [US4] Verify Three Roles framework is INVISIBLE in lesson 11.4
- [ ] T043 [US4] Create sensor-simulation skill at `.claude/skills/authoring/sensor-simulation/SKILL.md`
  - Structure: Persona + Questions + Principles
  - Scope: Sensor config, SDF plugins, noise models, visualization

- [ ] T044 [US4] Run Docusaurus build to validate Chapter 11 content

**Checkpoint**: User Story 4 complete - Student can simulate robot sensors

---

## Phase 7: User Story 5 - Connect ROS 2 to Gazebo (Priority: P1)

**Goal**: Student can bridge Gazebo topics to ROS 2, control simulated robot with ROS 2 nodes

**Independent Test**: Student sends cmd_vel from ROS 2, robot moves in Gazebo; sensor data flows to ROS 2 topics

**Chapter**: 12 (ROS 2 + Gazebo Integration) | **Layer**: L2→L3 | **Proficiency**: B1

### Implementation for User Story 5

- [ ] T045 [P] [US5] Create lesson 12.1 "The ros_gz Bridge" at `chapter-12-ros2-gazebo-integration/01-ros-gz-bridge.md`
  - Metadata: duration_minutes: 75, proficiency_level: B1, layer: L2, hardware_tier: 1-2
  - Content: ros_gz_bridge architecture, topic syntax `/TOPIC@ROS_MSG@GZ_MSG`
  - YAML configuration, direction (GZ_TO_ROS, ROS_TO_GZ, BIDIRECTIONAL)
  - End with "Try With AI" section

- [ ] T046 [P] [US5] Create lesson 12.2 "Spawning Robots from ROS 2" at `chapter-12-ros2-gazebo-integration/02-spawning-robots.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L2, hardware_tier: 1-2
  - Content: ros_gz spawn service, URDF→SDF conversion, robot_state_publisher
  - Launch file integration with spawn
  - End with "Try With AI" section

- [ ] T047 [P] [US5] Create lesson 12.3 "Closed-Loop Control" at `chapter-12-ros2-gazebo-integration/03-closed-loop-control.md`
  - Metadata: duration_minutes: 75, proficiency_level: B1, layer: L2, hardware_tier: 1-2
  - Content: Velocity commands via cmd_vel, feedback from sensors, control loop
  - Exercise: Drive robot with teleop, subscribe to sensor topics
  - End with "Try With AI" section

- [ ] T048 [P] [US5] Create lesson 12.4 "Creating ros_gz Skills" at `chapter-12-ros2-gazebo-integration/04-creating-ros-gz-skills.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L3, hardware_tier: 1
  - Content: Crystallizing ros_gz patterns into reusable skills
  - L3: Intelligence Design - student creates their own skill
  - End with "Try With AI" section

- [ ] T049 [US5] Update chapter-12 README.md with learning objectives and lesson links
- [ ] T050 [US5] Verify ros_gz_bridge syntax matches official docs (`/TOPIC@ROS_MSG@GZ_MSG`)
- [ ] T051 [US5] Create ros2-gazebo-bridge skill at `.claude/skills/authoring/ros2-gazebo-bridge/SKILL.md`
  - Structure: Persona + Questions + Principles
  - Scope: Topic mapping, message conversion, spawn orchestration, debugging

- [ ] T052 [US5] Run Docusaurus build to validate Chapter 12 content

**Checkpoint**: User Story 5 complete - Student can integrate ROS 2 with Gazebo

---

## Phase 8: User Story 6 - Complete Capstone Project (Priority: P3)

**Goal**: Student demonstrates integrated learning with complete simulation project

**Independent Test**: Student delivers working simulation that meets specification they wrote

**Chapter**: 13 (Module 2 Capstone) | **Layer**: L4 | **Proficiency**: B1

**Dependency**: Requires US1-US5 completion (all skills accumulated)

### Implementation for User Story 6

- [ ] T053 [P] [US6] Create lesson 13.1 "Capstone Specification" at `chapter-13-capstone/01-capstone-specification.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L4, hardware_tier: 1
  - Content: Spec-driven development, writing requirements for simulation project
  - Student writes specification: robot requirements, world setup, sensors, success criteria
  - End with "Try With AI" section

- [ ] T054 [P] [US6] Create lesson 13.2 "Building the Simulation" at `chapter-13-capstone/02-building-simulation.md`
  - Metadata: duration_minutes: 90, proficiency_level: B1, layer: L4, hardware_tier: 1-2
  - Content: Implementing capstone using accumulated skills (URDF, SDF, sensors, ros_gz)
  - L4: Composing skills from Chapters 9-12
  - End with "Try With AI" section

- [ ] T055 [P] [US6] Create lesson 13.3 "Testing, Validation, and Sim-to-Real Preview" at `chapter-13-capstone/03-testing-validation-preview.md`
  - Metadata: duration_minutes: 60, proficiency_level: B1, layer: L4, hardware_tier: 1-2
  - Content: Validating against specification, sim-to-real gap concepts, Module 3 preview
  - Student evaluates their capstone against their own spec
  - End with "Try With AI" section

- [ ] T056 [US6] Update chapter-13 README.md with learning objectives and lesson links
- [ ] T057 [US6] Run Docusaurus build to validate Chapter 13 content

**Checkpoint**: User Story 6 complete - Student has completed capstone project

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Quality validation across all user stories

- [ ] T058 [P] Verify all 22 lessons have complete frontmatter metadata
- [ ] T059 [P] Verify all lessons end with "Try With AI" section (no trailing summaries)
- [ ] T060 [P] Run grep check for Three Roles framework labels (should find ZERO matches)
  - Command: `grep -ri "AI as Teacher\|AI as Student\|AI as Co-Worker\|What to notice\|AI learned\|AI now knows" chapter-*/`
  - Expected: No matches
- [ ] T061 [P] Verify cognitive load limits: A2 lessons ≤7 concepts, B1 lessons ≤10 concepts
- [ ] T062 [P] Verify all Tier 2+ content has `<HardwareGate>` markers
- [ ] T063 [P] Verify all lessons have Tier 1 (cloud) execution path documented
- [ ] T064 [P] Verify all code examples include expected output within 5 lines
- [ ] T065 Run full Docusaurus build: `cd robolearn-interface && npm run build`
- [ ] T066 Verify 4 skills created and documented in `.claude/skills/authoring/`
  - urdf-robot-model, gazebo-world-builder, sensor-simulation, ros2-gazebo-bridge
- [ ] T067 Update Module 2 placeholder at `robolearn-interface/docs/module-2-simulation/README.md` with final content
- [ ] T068 Create PHR for implementation completion

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phases 3-8)**: All depend on Foundational phase completion
  - US1, US2, US5 are P1 priority - implement first
  - US3, US4 are P2 priority - implement after P1
  - US6 is P3 priority - implement last (depends on US1-US5)
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Priority | Chapter | Dependencies | Can Parallelize With |
|-------|----------|---------|--------------|---------------------|
| US1 | P1 | 8 | None | US2, US5 |
| US2 | P1 | 9 | None | US1, US5 |
| US5 | P1 | 12 | US2 (for robot to bridge) | US1 |
| US3 | P2 | 10 | US2 (for robot in world) | US4 |
| US4 | P2 | 11 | US2 (for robot with sensors) | US3 |
| US6 | P3 | 13 | US1-US5 (all skills) | None |

### Within Each User Story

1. Create lessons in parallel (all marked [P])
2. Update chapter README after lessons complete
3. Create skill (if applicable) after lessons
4. Run Docusaurus build validation
5. Move to next user story

### Parallel Opportunities

**Fully parallel (no dependencies)**:
- T002-T008: All chapter directory creation
- T015-T017: All Chapter 8 lessons (US1)
- T021-T024: All Chapter 9 lessons (US2)
- T029-T032: All Chapter 10 lessons (US3)
- T037-T040: All Chapter 11 lessons (US4)
- T045-T048: All Chapter 12 lessons (US5)
- T053-T055: All Chapter 13 lessons (US6)
- T058-T066: All Polish phase checks

---

## Parallel Example: User Story 2 (URDF)

```bash
# Launch all Chapter 9 lessons in parallel:
Task T021: "Create lesson 9.1 Understanding URDF"
Task T022: "Create lesson 9.2 Building Your First Robot"
Task T023: "Create lesson 9.3 Adding Physical Properties"
Task T024: "Create lesson 9.4 URDF with AI"

# Then sequentially:
Task T025: "Update chapter-9 README.md"
Task T026: "Verify Three Roles invisible"
Task T027: "Create urdf-robot-model skill"
Task T028: "Run Docusaurus build"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 5)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Chapter 8 - Why Simulate?)
4. Complete Phase 4: User Story 2 (Chapter 9 - URDF)
5. Complete Phase 7: User Story 5 (Chapter 12 - ROS 2 Integration)
6. **STOP and VALIDATE**: Test MVP independently
   - Student can understand simulation-first philosophy
   - Student can create URDF robot
   - Student can bridge to ROS 2
7. Deploy/demo MVP if ready

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (Ch8) → Test independently → Deploy (understands simulation)
3. Add US2 (Ch9) → Test independently → Deploy (creates robots)
4. Add US5 (Ch12) → Test independently → Deploy (ROS 2 integration) **← MVP COMPLETE**
5. Add US3 (Ch10) → Test independently → Deploy (builds worlds)
6. Add US4 (Ch11) → Test independently → Deploy (simulates sensors)
7. Add US6 (Ch13) → Test independently → Deploy (capstone) **← FULL MODULE**

### Single Author Strategy

1. Complete all phases sequentially
2. Focus on P1 stories first (US1, US2, US5)
3. Add P2 stories (US3, US4)
4. Complete P3 story (US6)
5. Polish phase validates all content

---

## Lesson Author Policy

**CRITICAL**: Within each chapter, every lesson MUST end with a single final section titled "Try With AI":

- **Before AI tools taught (Chapter 8)**: Use ChatGPT web interface
- **After AI tools introduced (Chapters 9+)**: Instruct learners to use their preferred AI companion tool (Gemini CLI, Claude CLI, etc.)
- **No "Key Takeaways" or "What's Next" sections after "Try With AI"**

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- All lessons follow lesson-generator skill metadata requirements
- Three Roles framework MUST be INVISIBLE in L2 lessons
- Every lesson MUST have Tier 1 (cloud) execution path
- Verify Docusaurus build after each chapter completion
- Commit after each task or logical group
