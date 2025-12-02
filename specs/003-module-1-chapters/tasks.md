# Tasks: Module 1 Chapter Architecture

**Input**: Design documents from `/specs/003-module-1-chapters/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md (complete)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Content**: `docs/module-1-ros2/` for all chapter content
- **Skills**: `.claude/skills/authoring/` for ROS 2 skills
- **Specs**: `specs/003-module-1-chapters/` for design artifacts

---

## Phase 1: Setup (Infrastructure)

**Purpose**: Directory structure and configuration for Module 1 content

- [ ] T001 Create Module 1 directory structure in docs/module-1-ros2/
- [ ] T002 [P] Create chapter-1-physical-ai/ directory structure
- [ ] T003 [P] Create chapter-2-robot-system/ directory structure
- [ ] T004 [P] Create chapter-3-meet-ros2/ directory structure
- [ ] T005 [P] Create chapter-4-first-code/ directory structure
- [ ] T006 [P] Create chapter-5-communication/ directory structure
- [ ] T007 [P] Create chapter-6-building-systems/ directory structure
- [ ] T008 [P] Create chapter-7-capstone/ directory structure
- [ ] T009 Configure Docusaurus sidebar for Module 1 in sidebars.js

---

## Phase 2: Foundational (Chapter READMEs)

**Purpose**: Chapter landing pages that MUST be complete before lesson content

**⚠️ CRITICAL**: Chapter READMEs define navigation and structure for all lessons

- [ ] T010 [P] Create Chapter 1 README in docs/module-1-ros2/chapter-1-physical-ai/index.md
- [ ] T011 [P] Create Chapter 2 README in docs/module-1-ros2/chapter-2-robot-system/index.md
- [ ] T012 [P] Create Chapter 3 README in docs/module-1-ros2/chapter-3-meet-ros2/index.md
- [ ] T013 [P] Create Chapter 4 README in docs/module-1-ros2/chapter-4-first-code/index.md
- [ ] T014 [P] Create Chapter 5 README in docs/module-1-ros2/chapter-5-communication/index.md
- [ ] T015 [P] Create Chapter 6 README in docs/module-1-ros2/chapter-6-building-systems/index.md
- [ ] T016 [P] Create Chapter 7 README in docs/module-1-ros2/chapter-7-capstone/index.md
- [ ] T017 Create Module 1 README in docs/module-1-ros2/index.md (references spec 002)
- [ ] T018 Add Mermaid layer progression diagram to Module 1 README

**Checkpoint**: All 7 chapter READMEs + Module README complete - lesson implementation can begin

---

## Phase 3: User Story 1 - Student Learns Content (Priority: P1) 🎯 MVP

**Goal**: Student can navigate Module 1 and complete all 25 lessons to learn ROS 2 fundamentals

**Independent Test**: Student completes Chapter 1-7 sequentially with mastery gates

### Chapter 1: What is Physical AI? (3 lessons, L1 100%)

- [ ] T019 [P] [US1] Create Lesson 1.1: From ChatGPT to Walking Robots in docs/module-1-ros2/chapter-1-physical-ai/01-digital-to-physical.mdx
- [ ] T020 [P] [US1] Create Lesson 1.2: Embodied Intelligence in docs/module-1-ros2/chapter-1-physical-ai/02-embodied-intelligence.mdx
- [ ] T021 [P] [US1] Create Lesson 1.3: The Humanoid Revolution in docs/module-1-ros2/chapter-1-physical-ai/03-humanoid-revolution.mdx
- [ ] T022 [US1] Add Mermaid AI spectrum diagram to Lesson 1.1
- [ ] T023 [US1] Add hardware tier interactive selector concept to Lesson 1.3

### Chapter 2: The Robot System (4 lessons, L1 85%, L2 15%)

- [ ] T024 [P] [US1] Create Lesson 2.1: How Robots See (Sensors) in docs/module-1-ros2/chapter-2-robot-system/01-how-robots-see.mdx
- [ ] T025 [P] [US1] Create Lesson 2.2: How Robots Move (Actuators) in docs/module-1-ros2/chapter-2-robot-system/02-how-robots-move.mdx
- [ ] T026 [P] [US1] Create Lesson 2.3: The Glue (Middleware) in docs/module-1-ros2/chapter-2-robot-system/03-why-middleware.mdx
- [ ] T027 [P] [US1] Create Lesson 2.4: Your Hardware Tier in docs/module-1-ros2/chapter-2-robot-system/04-your-hardware-tier.mdx
- [ ] T028 [US1] Add sensor types Mermaid diagram to Lesson 2.1
- [ ] T029 [US1] Add control loop Mermaid diagram to Lesson 2.2
- [ ] T030 [US1] Add ROS 2 architecture Mermaid diagram to Lesson 2.3

### Chapter 3: Meet ROS 2 (4 lessons, L1 60%, L2 40%) - CLI ONLY

- [ ] T031 [P] [US1] Create Lesson 3.1: Setting Up Your ROS 2 Environment in docs/module-1-ros2/chapter-3-meet-ros2/01-setup-environment.mdx
- [ ] T032 [P] [US1] Create Lesson 3.2: Turtlesim in Action in docs/module-1-ros2/chapter-3-meet-ros2/02-turtlesim-action.mdx
- [ ] T033 [P] [US1] Create Lesson 3.3: Nodes & Topics (CLI Exploration) in docs/module-1-ros2/chapter-3-meet-ros2/03-nodes-topics.mdx
- [ ] T034 [P] [US1] Create Lesson 3.4: Services & Parameters (CLI) in docs/module-1-ros2/chapter-3-meet-ros2/04-services-parameters.mdx
- [ ] T035 [US1] Add cloud ROS 2 setup instructions (Tier 1 fallback) to Lesson 3.1
- [ ] T036 [US1] Add rqt_graph node visualization Mermaid diagram to Lesson 3.3
- [ ] T037 [US1] Verify NO Python coding in Chapter 3 lessons (CLI only constraint)

### Chapter 4: Your First ROS 2 Code (4 lessons, L2 50%, L3 10%)

- [ ] T038 [P] [US1] Create Lesson 4.1: Workspaces & Packages in docs/module-1-ros2/chapter-4-first-code/01-workspaces-packages.mdx
- [ ] T039 [P] [US1] Create Lesson 4.2: Writing a Publisher in docs/module-1-ros2/chapter-4-first-code/02-writing-publisher.mdx
- [ ] T040 [P] [US1] Create Lesson 4.3: Writing a Subscriber in docs/module-1-ros2/chapter-4-first-code/03-writing-subscriber.mdx
- [ ] T041 [P] [US1] Create Lesson 4.4: Try With AI in docs/module-1-ros2/chapter-4-first-code/04-try-with-ai.mdx
- [ ] T042 [US1] Reference ros2-publisher-subscriber skill in Lesson 4.2-4.3 code examples
- [ ] T043 [US1] Implement Three Roles framework INVISIBLY in Lesson 4.4 (no framework labels)
- [ ] T044 [US1] Verify Tier 1 cloud fallback instructions in all Chapter 4 lessons

### Chapter 5: Communication Mastery (4 lessons, L2 50%, L3 40%, L4 10%)

- [ ] T045 [P] [US1] Create Lesson 5.1: Writing a Service Server in docs/module-1-ros2/chapter-5-communication/01-service-server.mdx
- [ ] T046 [P] [US1] Create Lesson 5.2: Writing a Service Client in docs/module-1-ros2/chapter-5-communication/02-service-client.mdx
- [ ] T047 [P] [US1] Create Lesson 5.3: Custom Messages & Services in docs/module-1-ros2/chapter-5-communication/03-custom-messages.mdx
- [ ] T048 [P] [US1] Create Lesson 5.4: Design Patterns (Topics vs Services) in docs/module-1-ros2/chapter-5-communication/04-design-patterns.mdx
- [ ] T049 [US1] Reference ros2-service-pattern skill in Lesson 5.1-5.2 code examples
- [ ] T050 [US1] Reference ros2-custom-interfaces skill in Lesson 5.3 code examples
- [ ] T051 [US1] Add topics vs services decision tree Mermaid diagram to Lesson 5.4

### Chapter 6: Building Robot Systems (3 lessons, L3 50%, L4 20%)

- [ ] T052 [P] [US1] Create Lesson 6.1: Parameters (Configurable Nodes) in docs/module-1-ros2/chapter-6-building-systems/01-parameters.mdx
- [ ] T053 [P] [US1] Create Lesson 6.2: Launch Files (Multi-Node Startup) in docs/module-1-ros2/chapter-6-building-systems/02-launch-files.mdx
- [ ] T054 [P] [US1] Create Lesson 6.3: Debugging Multi-Node Systems in docs/module-1-ros2/chapter-6-building-systems/03-debugging-systems.mdx
- [ ] T055 [US1] Reference ros2-launch-system skill in Lesson 6.2 code examples
- [ ] T056 [US1] Add debugging workflow Mermaid diagram to Lesson 6.3

### Chapter 7: Capstone - Robot Controller (3 lessons, L4 80%)

- [ ] T057 [P] [US1] Create Lesson 7.1: Capstone Specification in docs/module-1-ros2/chapter-7-capstone/01-capstone-spec.mdx
- [ ] T058 [P] [US1] Create Lesson 7.2: Building the Controller in docs/module-1-ros2/chapter-7-capstone/02-building-controller.mdx
- [ ] T059 [P] [US1] Create Lesson 7.3: Testing, Validation & Reflection in docs/module-1-ros2/chapter-7-capstone/03-testing-validation.mdx
- [ ] T060 [US1] Add capstone system architecture Mermaid diagram to Lesson 7.1
- [ ] T061 [US1] Verify capstone uses ONLY Chapter 1-6 concepts (no URDF/Actions/tf2)
- [ ] T062 [US1] Add Module 2 preview section to Lesson 7.3

**Checkpoint**: User Story 1 complete - All 25 lessons created, student can learn ROS 2 fundamentals

---

## Phase 4: User Story 2 - Instructor Reviews Curriculum (Priority: P2)

**Goal**: Instructor can extract 5-week teaching schedule from Module 1 chapter READMEs

**Independent Test**: Instructor maps chapters to weeks 1-5 using READMEs alone

- [ ] T063 [US2] Add week mapping table to each chapter README (T010-T016)
- [ ] T064 [US2] Add assessment points section to each chapter README
- [ ] T065 [US2] Create instructor guide summary in docs/module-1-ros2/instructor-guide.md
- [ ] T066 [US2] Add hardware requirements matrix (Tier 1-4) to Module 1 README
- [ ] T067 [US2] Add mastery gate descriptions to each chapter README
- [ ] T068 [P] [US2] Verify all lesson durations documented (45-60 min constraint)

**Checkpoint**: User Story 2 complete - Instructor can plan 5-week course from READMEs

---

## Phase 5: User Story 3 - Author Extends Module Structure (Priority: P3)

**Goal**: Authors can use Module 1 as template for Module 2+ chapters

**Independent Test**: Author successfully outlines Module 2 using Module 1 structure

- [ ] T069 [US3] Document chapter architecture pattern in specs/003-module-1-chapters/architecture-pattern.md
- [ ] T070 [US3] Document lesson template structure with all required sections
- [ ] T071 [US3] Verify all 4 ROS 2 skills documented in .claude/skills/authoring/
- [ ] T072 [US3] Create skills registry JSON in specs/003-module-1-chapters/skills-registry.json
- [ ] T073 [US3] Document layer progression pattern (L1→L2→L3→L4) for reuse

**Checkpoint**: User Story 3 complete - Authors have reusable template for future modules

---

## Phase 6: Polish & Validation

**Purpose**: Quality assurance across all content

- [ ] T074 [P] Run educational-validator on all 25 lesson files
- [ ] T075 [P] Verify Three Roles framework invisible (grep for forbidden labels)
- [ ] T076 [P] Verify all Mermaid diagrams render correctly in Docusaurus
- [ ] T077 [P] Verify all code examples cite ROS 2 Humble documentation
- [ ] T078 Verify MDX frontmatter in all lesson files
- [ ] T079 Test chapter navigation (previous/next links)
- [ ] T080 Create lesson-breakdown.json machine-readable inventory
- [ ] T081 Final constitutional compliance check (11 gates)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all lesson content
- **User Story 1 (Phase 3)**: Depends on Foundational - creates all 25 lessons
- **User Story 2 (Phase 4)**: Can start after Phase 2, integrates with Phase 3
- **User Story 3 (Phase 5)**: Depends on Phase 3 completion (needs lessons as examples)
- **Polish (Phase 6)**: Depends on all user stories complete

### User Story Dependencies

- **User Story 1 (P1)**: Core learning path - independent after Foundational
- **User Story 2 (P2)**: Instructor view - can work in parallel with US1 chapters
- **User Story 3 (P3)**: Author template - needs US1 complete as reference

### Within User Story 1 (Lesson Creation)

Each chapter can be implemented in parallel:
- Chapter 1-2: No dependencies (conceptual)
- Chapter 3: No dependencies (CLI exploration)
- Chapter 4-6: Can reference skills but lessons are independent
- Chapter 7: Should be last (references all previous chapters)

### Parallel Opportunities

**Phase 1**: All T002-T008 can run in parallel (directory creation)

**Phase 2**: All T010-T016 can run in parallel (chapter READMEs)

**Phase 3 within chapters**:
```bash
# Chapter 1 lessons (parallel):
Task T019, T020, T021

# Chapter 2 lessons (parallel):
Task T024, T025, T026, T027

# Chapter 3 lessons (parallel):
Task T031, T032, T033, T034

# etc.
```

**Phase 4-5**: Can run in parallel with Phase 3 chapter completion

**Phase 6**: All validation tasks can run in parallel

---

## Parallel Example: Chapter 4 Implementation

```bash
# Launch all Chapter 4 lessons in parallel:
Task: "T038 Create Lesson 4.1: Workspaces & Packages"
Task: "T039 Create Lesson 4.2: Writing a Publisher"
Task: "T040 Create Lesson 4.3: Writing a Subscriber"
Task: "T041 Create Lesson 4.4: Try With AI"

# Then sequentially:
Task: "T042 Reference skill in code examples"
Task: "T043 Implement Three Roles invisibly"
Task: "T044 Verify Tier 1 fallback"
```

---

## Implementation Strategy

### MVP First (User Story 1 - Chapters 1-4)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Chapter READMEs (T010-T018)
3. Complete Chapters 1-4 lessons (T019-T044) - Core foundation
4. **STOP and VALIDATE**: Test first 4 chapters independently
5. Student can learn Physical AI + ROS 2 basics with pub/sub

### Incremental Delivery

1. Setup + Foundational → Structure ready
2. Chapters 1-2 → Conceptual foundation testable
3. Chapter 3 → CLI exploration testable
4. Chapter 4 → First code testable (pub/sub)
5. Chapters 5-6 → Services + launch testable
6. Chapter 7 → Capstone integration testable
7. US2 + US3 → Instructor/author features

### Content-Implementer Agent Strategy

For efficient parallel implementation:
1. Route all Chapter 1 lessons to content-implementer in parallel
2. Route all Chapter 2 lessons in parallel
3. Continue chapter by chapter
4. Run validation tasks after each chapter completes

---

## Task Summary

| Phase | Tasks | Parallel Opportunities |
|-------|-------|------------------------|
| Setup | 9 | 7 parallel (T002-T008) |
| Foundational | 9 | 7 parallel (T010-T016) |
| US1: Chapters 1-7 | 44 | 25 lesson tasks parallel by chapter |
| US2: Instructor | 6 | 1 parallel task |
| US3: Author | 5 | 0 parallel (sequential documentation) |
| Polish | 8 | 4 parallel validation tasks |
| **Total** | **81** | **~44 parallel opportunities** |

---

## Policy Notes

### Lesson Ending Convention

Within Module 1, each lesson MUST end with a single final section titled **"Try With AI"**:
- NO "Key Takeaways" sections
- NO "What's Next" sections
- NO "Summary" sections
- "Try With AI" contains reflection + AI collaboration prompt
- For Chapters 1-2 (pre-AI): Use "Reflect" section instead

### Three Roles Invisibility

All L2+ lessons MUST:
- Experience Three Roles through action prompts
- NEVER expose framework labels ("AI as Teacher/Student/Co-Worker")
- Validation: `grep -i "AI as\|Three Roles\|Role 1\|Role 2\|Role 3" [file]` returns 0 matches

### Hardware Tier Compliance

All lessons MUST include Tier 1 fallback:
- Cloud ROS 2 (TheConstruct) instructions
- MockROS browser simulation where applicable
- `<HardwareGate>` components for Tier 2+ content

---

**Tasks generated**: 2025-11-29
**Total tasks**: 81
**MVP scope**: Phase 1-2 + Chapters 1-4 (Tasks T001-T044)
**Parallel opportunities**: ~44 tasks
