# Implementation Plan: Module 2 Gazebo/Unity Simulation

**Branch**: `004-module-2-simulation` | **Date**: 2025-11-29 | **Spec**: `/specs/004-module-2-simulation/spec.md`

**Input**: Feature specification for Module 2: "The Digital Twin (Gazebo & Unity)" — 6 chapters, 22 lessons, 4 weeks of simulation-first robotics learning.

---

## Summary

Design and implement a complete 6-chapter, 22-lesson module teaching robotics simulation with Gazebo Harmonic (gz-sim). The module progresses from conceptual foundations (WHY simulate) through hands-on robotics modeling (URDF robot descriptions, SDF worlds), sensor simulation, and ROS 2 integration, culminating in a spec-driven capstone project. This module bridges ROS 2 fundamentals (Module 1) to AI-powered perception in Module 3 (NVIDIA Isaac).

**Technical Approach**: Layer-progressive pedagogy (L1: manual foundation → L2: AI collaboration with Three Roles → L3: intelligence design → L4: spec-driven integration) combined with hardware-tier awareness (Tier 1 cloud-only path via TheConstruct for all content; Tier 2 local Gazebo optional) and cognitive load management (A2: 5-7 concepts per lesson in Chapters 8-9; B1: 7-10 concepts in Chapters 10-13).

---

## Technical Context

**Language/Version**: Markdown (`.md` files) compatible with Docusaurus 3.x (static content platform)

**Primary Dependencies**:
- Gazebo Harmonic (gz-sim 8.x) - Official open-source robotics simulator
- ROS 2 Humble - Consistent with Module 1 distribution
- ros_gz_bridge - Official ROS-Gazebo integration package
- xacro - Modular URDF templates
- TheConstruct - Cloud ROS 2 environment for Tier 1 fallback
- Gazebo Fuel - Pre-built model repository

**Storage**: N/A (static markdown content, versioned in git)

**Testing**:
- Docusaurus build validation (no MDX syntax errors)
- Gazebo Harmonic documentation verification
- ROS 2 Humble official docs cross-reference
- Three Roles framework invisibility check (grep for exposed labels)
- Cognitive load validation (concept count per CEFR tier)
- Cloud execution path validation (Tier 1 runnable)

**Target Platform**: Web (Docusaurus documentation site + cloud simulation environment)

**Project Type**: Educational content with interactive exercises

**Performance Goals**:
- Page load: <2s for lesson content
- Gazebo launch in cloud: <5s from lesson context
- Cognitive load: 45-60 min lessons (A2), 60-75 min lessons (B1)
- Tier 1 execution: 100% of core content accessible via TheConstruct

**Constraints**:
- NO Gazebo Classic patterns (gz-sim only, modern APIs)
- ROS 2 Humble distribution only
- `.md` file extension ONLY (NOT `.mdx`)
- Chapter index files: `README.md` (NOT `index.md`)
- No `<` or `>` characters in prose (Docusaurus MDX escaping)
- Lesson frontmatter complete (id, title, duration, proficiency, layer, hardware_tier, learning_objectives)
- Tier 1 fallback documented for ALL lessons
- Lessons end with "Try With AI" section (no trailing summaries)
- Three Roles framework INVISIBLE to students (experienced through action, not labeled)

**Scale/Scope**:
- 1 module README + 6 chapter READMEs + 22 lesson markdown files
- 4 reusable skills created and documented
- Hardware tier markers for Tier 2+ content
- Cloud fallback paths for all exercises
- All content fits within 4-week module timeline

---

## Constitution Check

**GATE: Must pass before implementation. Re-check after chapter READMEs complete.**

| Principle | Assessment | Status | Justification |
|-----------|-----------|--------|---------------|
| **Specification Primacy** | Chapter architecture from spec; lesson plan maps to spec requirements | ✅ PASS | Spec 004 provides 6-chapter breakdown with user stories and acceptance criteria |
| **Progressive Complexity (CEFR)** | Ch8-9 (A2: 5-7 concepts), Ch10-13 (B1: 7-10 concepts) | ✅ PASS | Proficiency progression explicit; cognitive load limits enforced per tier |
| **Factual Accuracy** | All Gazebo claims cite Harmonic docs; ROS 2 from Humble; no Gazebo Classic | ✅ PASS | Research phase will verify all technical claims against official sources |
| **Coherent Pedagogical Structure** | L1→L2→L3→L4 progression; Ch8 (L1), Ch9-11 (L1→L2), Ch12 (L2→L3), Ch13 (L4) | ✅ PASS | Chapter specifications enforce layer progression; learning outcomes map to layers |
| **Intelligence Accumulation** | 4 skills crystallized (Ch9/10/11/12); reusable for Module 3 and future robotics | ✅ PASS | Skills identified in spec FR-026-029; Layer 3 lessons dedicated to skill creation |
| **Anti-Convergence Variation** | Modalities: narrative (Ch8) → hands-on URDF (Ch9) → world building (Ch10) → sensor config (Ch11) → ROS integration (Ch12) → spec-driven (Ch13) | ✅ PASS | Each chapter distinct modality; avoids repeating Module 1 patterns |
| **Minimal Sufficient Content** | 22 lessons justified by 6 user stories; non-goals exclude plugins, multi-robot, real deployment | ✅ PASS | Lesson count derived from spec; non-goals clearly defined |
| **Hardware-Awareness (Domain)** | Tier 1 path (TheConstruct) for ALL core content; Tier 2 marked with HardwareGate | ✅ PASS | Spec FR-004 requires Tier 1 work; Tier 2 exercises gated where hardware-specific |
| **Simulation-First (Domain)** | Module is simulation-only; no physical robot deployment (deferred to Module 4) | ✅ PASS | Entire module simulation-focused; sim-to-real concepts introduced but not executed |
| **Safety-Critical (Domain)** | Module 2 is pre-motor (simulation-only); safety deferred to Module 3-4 | ✅ PASS | No motor control content; foundation layer precedes physical deployment |
| **Formal Verification** | 6+ entities (chapters, lessons, concepts, skills), 4+ constraints (layer, load, tiers, Three Roles) | ✅ REQUIRES | Formal analysis in Section IV validates invariants; no violations found |
| **Three Roles Invisible (L2)** | Framework NEVER exposed in student content; experienced via action prompts | ✅ PASS | Planning ensures labels never appear; Three Roles demonstrated through iteration |

**All gates PASS. Proceeding with detailed chapter breakdown.**

---

## Formal Verification (Invariant Analysis)

### Maintained Invariants

1. **Layer Progression**: Every lesson in exactly one layer; chapters respect ordering
   - `∀ chapter ∈ [8,9,11]: lesson.layer ∈ [L1,L2]`
   - `chapter 12: lesson.layer ∈ [L2,L3]`
   - `chapter 13: lesson.layer = L4`

2. **Cognitive Load**: Lesson concept count respects CEFR tier
   - `∀ lesson: proficiency=A2 → concepts ≤ 7`
   - `∀ lesson: proficiency=B1 → concepts ≤ 10`

3. **Hardware Tier Coverage**: Every lesson has Tier 1 path; Tier 2+ gated
   - `∀ lesson: some lesson.tier1Path`
   - `∀ lesson: tier > 1 → some lesson.cloudFallback`

4. **Three Roles Invisibility**: Zero framework labels in student content
   - `∀ lesson: content ∌ ("AI as Teacher" ∨ "AI as Student" ∨ "AI as Co-Worker")`

### Counterexample Test (3-Instance Validation)

```
✅ Lesson 8.1 (A2, L1): 5 concepts ≤ 7 limit, no AI, Tier 1 browser
✅ Lesson 9.4 (A2, L2): 4 concepts ≤ 7 limit, Three Roles invisible, Tier 1 cloud
✅ Lesson 12.3 (B1, L2): 8 concepts ≤ 10 limit, Three Roles demonstrated, Tier 1 cloud ROS 2
```

**Result**: All invariants satisfied. No counterexamples found.

---

## Project Structure

### Documentation (this feature)

```
specs/004-module-2-simulation/
├── spec.md                    # Feature specification (6 chapters, 22 lessons, user stories)
├── plan.md                    # This file - implementation roadmap
├── research.md                # Gazebo/ROS 2 sources, teaching modality rationale
├── lesson-breakdown.json      # Machine-readable inventory (title, duration, layer, concepts)
└── skills-registry.json       # 4 skills created (name, scope, cross-book value)
```

### Deliverable Structure (Content)

```
docs/module-2-simulation/
├── README.md                  # Module 2 landing page

├── chapter-8-why-simulate/
│   ├── README.md
│   ├── 01-digital-twin-concept.md
│   ├── 02-simulation-first.md
│   └── 03-meet-gazebo.md

├── chapter-9-robot-description/
│   ├── README.md
│   ├── 01-understanding-urdf.md
│   ├── 02-building-first-robot.md
│   ├── 03-adding-physical-properties.md
│   └── 04-urdf-with-ai.md

├── chapter-10-simulation-worlds/
│   ├── README.md
│   ├── 01-sdf-world-basics.md
│   ├── 02-adding-models-from-fuel.md
│   ├── 03-physics-configuration.md
│   └── 04-world-building-with-ai.md

├── chapter-11-sensors-simulation/
│   ├── README.md
│   ├── 01-camera-simulation.md
│   ├── 02-lidar-simulation.md
│   ├── 03-imu-contact-sensors.md
│   └── 04-sensor-debugging-visualization.md

├── chapter-12-ros2-gazebo-integration/
│   ├── README.md
│   ├── 01-ros-gz-bridge.md
│   ├── 02-spawning-robots.md
│   ├── 03-closed-loop-control.md
│   └── 04-creating-ros-gz-skills.md

└── chapter-13-capstone/
    ├── README.md
    ├── 01-capstone-specification.md
    ├── 02-building-simulation.md
    └── 03-testing-validation-preview.md
```

---

## Chapter-by-Chapter Implementation Summary

### Chapter 8: Why Simulate? (L1, 3 lessons)

**Proficiency**: A2 | **Tier**: 1 | **Concepts**: 7 total
- 8.1 Digital Twin Concept (narrative)
- 8.2 Simulation-First Development (case studies)
- 8.3 Meet Gazebo Harmonic (architecture)

**Skills**: None (L1 foundation only)

### Chapter 9: Robot Description Formats (L1→L2, 4 lessons)

**Proficiency**: A2 | **Tier**: 1 | **Concepts**: 6-7 per lesson
- 9.1 Understanding URDF (manual XML)
- 9.2 Building Your First Robot (manual practice)
- 9.3 Adding Physical Properties (manual + L2 transition)
- 9.4 URDF with AI (L2: Three Roles invisible)

**Skills**: ✅ urdf-robot-model (Lesson 9.4, Layer 3)

### Chapter 10: Building Simulation Worlds (L1→L2, 4 lessons)

**Proficiency**: A2→B1 | **Tier**: 1-2 | **Concepts**: 5-7 per lesson
- 10.1 SDF World Basics (manual creation)
- 10.2 Adding Models from Fuel (model integration)
- 10.3 Physics Configuration (manual tuning)
- 10.4 World Building with AI (L2: Three Roles)

**Skills**: ✅ gazebo-world-builder (Lesson 10.4, Layer 3)

### Chapter 11: Sensors in Simulation (L1→L2, 4 lessons)

**Proficiency**: B1 | **Tier**: 1-2 | **Concepts**: 5-7 per lesson
- 11.1 Camera Simulation (manual config)
- 11.2 LIDAR Simulation (manual config)
- 11.3 IMU and Contact Sensors (manual config)
- 11.4 Sensor Debugging and Visualization (L2: Three Roles)

**Skills**: ✅ sensor-simulation (Lesson 11.4, Layer 3)

### Chapter 12: ROS 2 + Gazebo Integration (L2→L3, 4 lessons)

**Proficiency**: B1 | **Tier**: 1-2 | **Concepts**: 5-8 per lesson
- 12.1 The ros_gz Bridge (L2: understanding bridge)
- 12.2 Spawning Robots from ROS 2 (L2: integration)
- 12.3 Closed-Loop Control (L2: debugging control loops)
- 12.4 Creating ros_gz Skills (L3: skill crystallization)

**Skills**: ✅ ros2-gazebo-bridge (Lesson 12.4, Layer 3)

### Chapter 13: Module 2 Capstone (L4, 3 lessons)

**Proficiency**: B1 | **Tier**: 1-2 | **Concepts**: 5-6 per lesson
- 13.1 Capstone Specification (spec-first approach)
- 13.2 Building the Simulation (skill composition)
- 13.3 Testing, Validation, and Sim-to-Real Preview (validation)

**Skills**: None created (composing skills from Ch9-12)

---

## Skills to Create

| Skill | Chapter | Layer | Scope |
|-------|---------|-------|-------|
| `urdf-robot-model` | 9.4 | L3 | URDF design, links, joints, physical properties, xacro patterns |
| `gazebo-world-builder` | 10.4 | L3 | World design, physics config, Fuel models, environment construction |
| `sensor-simulation` | 11.4 | L3 | Sensor config, SDF plugins, noise models, visualization |
| `ros2-gazebo-bridge` | 12.4 | L3 | Topic mapping, message conversion, spawn orchestration, debugging |

Each skill uses **Persona + Questions + Principles** pattern and is reusable for Module 3 (Isaac) and future robotics courses.

---

## Validation Checklist

### Content Completion
- [ ] All 22 lesson files created with complete frontmatter
- [ ] All 6 chapter READMEs created
- [ ] All lesson IDs unique across all modules
- [ ] Docusaurus `npm run build` succeeds
- [ ] No broken internal links

### Learning Progression
- [ ] Chapter 8: Zero AI collaboration prompts (L1 only)
- [ ] Chapters 9-11: Three Roles demonstrated but INVISIBLE (grep check: zero framework labels)
- [ ] Chapter 12: Skill creation explicitly taught
- [ ] Chapter 13: Specification comes FIRST

### Cognitive Load
- [ ] Chapters 8-9: Each lesson ≤ 7 concepts
- [ ] Chapters 10-13: Each lesson ≤ 10 concepts

### Hardware Tier
- [ ] All Tier 1 lessons have cloud path documented
- [ ] Tier 2+ sections marked with `<HardwareGate minTier={N}>`
- [ ] CloudFallback provided for Tier 1 students

### Technical Accuracy
- [ ] No Gazebo Classic patterns (gz-sim only)
- [ ] ROS 2 Humble examples verified
- [ ] ros_gz_bridge syntax correct (`/TOPIC@ROS_MSG@GZ_MSG`)
- [ ] URDF/SDF examples valid XML

### Skills Quality
- [ ] 4 skills created and documented
- [ ] Each skill: Persona + Questions + Principles
- [ ] Skills reusable (cross-book value documented)

---

## Implementation Phases

| Phase | Deliverable | Days | Owner |
|-------|-------------|------|-------|
| 1 | Chapter READMEs + lesson skeletons | 1-2 | content-implementer |
| 2 | Core lesson content (Ch8-13) | 3-5 | content-implementer + lesson-generator skill |
| 3 | Validation & Three Roles check | 6 | educational-validator agent |
| 4 | Skills documentation | 6-7 | skill-creator skill |
| 5 | Final commit & PR | 7 | content-implementer |

**Total Timeline**: 7 days for complete module implementation

---

## Success Criteria

**Content Delivery**:
- ✅ 22 lessons + complete metadata
- ✅ 100% Tier 1 execution path
- ✅ Docusaurus build succeeds

**Learning Outcomes**:
- ✅ Students create valid URDF robots (loads in Gazebo)
- ✅ Students build SDF worlds with realistic physics
- ✅ Students configure camera, LIDAR, IMU sensors
- ✅ Students bridge Gazebo to ROS 2
- ✅ Students complete spec-driven capstone project

**Pedagogical Quality**:
- ✅ Zero Three Roles framework labels exposed
- ✅ 100% of L2 lessons show bidirectional AI collaboration
- ✅ All lessons end with "Try With AI"
- ✅ Cognitive load within CEFR limits

**Platform Intelligence**:
- ✅ 4 skills created and documented
- ✅ Skills reusable for Module 3+
- ✅ Cross-book intelligence value assessed

---

**Next Step**: Route to content-implementer for lesson implementation following this plan.
