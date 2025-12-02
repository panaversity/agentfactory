# Tasks: Module Content Architecture

**Input**: Design documents from `/specs/002-module-content/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not requested - focusing on content creation tasks.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Content**: `robolearn-interface/docs/` (Docusaurus content directory)
- **Modules**: `module-N-slug/index.md` per plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project structure and prepare for content creation

- [x] T001 Verify Docusaurus project structure exists at robolearn-interface/docs/
- [x] T002 [P] Create module-1-ros2/ directory in robolearn-interface/docs/
- [x] T003 [P] Create module-2-simulation/ directory in robolearn-interface/docs/
- [x] T004 [P] Create module-3-isaac/ directory in robolearn-interface/docs/
- [x] T005 [P] Create module-4-vla/ directory in robolearn-interface/docs/
- [x] T006 Verify Mermaid diagram support in Docusaurus configuration (pending: needs @docusaurus/theme-mermaid)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create reusable template and verify research sources before content creation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Create MODULE_TEMPLATE.md in specs/002-module-content/ with common structure from plan.md
- [x] T008 [P] Verify ROS 2 Humble documentation citations are accessible (docs.ros.org/en/humble/)
- [x] T009 [P] Verify Gazebo documentation citations are accessible (gazebosim.org/docs)
- [x] T010 [P] Verify NVIDIA Isaac documentation citations are accessible (docs.isaacsim.omniverse.nvidia.com/)
- [x] T011 [P] Verify Isaac Lab documentation citations are accessible (isaac-sim.github.io/IsaacLab/)
- [x] T012 Compile hardware tier fallback paths for Tier 1 (cloud options) from research

**Checkpoint**: Foundation ready - module README creation can now begin

---

## Phase 3: User Story 1 - Student Discovers Module Content (Priority: P1) 🎯 MVP

**Goal**: Students can navigate to any module README and understand what they will learn, prerequisites, hardware requirements, and chapter structure within 2 minutes.

**Independent Test**: Open any module README and verify:
1. Overview clearly explains module's role (2-3 paragraphs)
2. Learning objectives use action verbs (4-6 objectives)
3. Hardware tiers displayed with Tier 1 fallback visible
4. 4-Layer Teaching Method progression indicated
5. Navigation to first chapter present

### Implementation for User Story 1

#### Module 1: The Robotic Nervous System (ROS 2)

- [x] T013 [US1] Create index.md in robolearn-interface/docs/module-1-ros2/
- [x] T014 [US1] Write module overview (2-3 paragraphs) explaining ROS 2's role in Physical AI curriculum
- [x] T015 [US1] Define 6 learning objectives with action verbs (understand, recognize, explain, write, build, debug)
- [x] T016 [US1] Create chapter index: Ch1 Intro to Physical AI (8hrs), Ch2 ROS 2 Fundamentals (10hrs), Ch3 Building with ROS 2 (7hrs)
- [x] T017 [US1] Add hardware tier table (Tier 1: MockROS/Pyodide, Tier 2+: local ROS 2 Humble)
- [x] T018 [US1] Document 4-Layer progression: L1 (Ch1) → L1-L2-L3 (Ch2) → L1-L4 (Ch3 capstone)
- [x] T019 [US1] Create Mermaid diagram showing Chapter 1→2→3 layer progression
- [x] T020 [US1] Add prerequisites section (none - this is Module 1)
- [x] T021 [US1] Describe capstone: Multi-node ROS 2 system with publisher/subscriber
- [x] T022 [US1] Add "Start Learning" CTA linking to first chapter
- [x] T023 [US1] Add navigation: Next module → Module 2

#### Module 2: The Digital Twin (Gazebo & Unity)

- [x] T024 [P] [US1] Create index.md in robolearn-interface/docs/module-2-simulation/
- [x] T025 [US1] Write module overview (2-3 paragraphs) explaining simulation's role in Physical AI
- [x] T026 [US1] Define 4-5 learning objectives with action verbs (setup, build, integrate, design)
- [x] T027 [US1] Create chapter index: Ch4 Gazebo Simulation (6hrs), Ch5 Unity Integration (6hrs)
- [x] T028 [US1] Add hardware tier table (Tier 1: cloud Gazebo, Tier 2+: local install)
- [x] T029 [US1] Document 4-Layer progression: L1-L2-L3 (Ch4) → L1-L4 (Ch5 capstone)
- [x] T030 [US1] Create Mermaid diagram showing Gazebo→Unity→Digital Twin flow
- [x] T031 [US1] Add prerequisites section (Module 1 complete)
- [x] T032 [US1] Describe capstone: Digital Twin (Gazebo ↔ ROS 2 ↔ Unity visualization)
- [x] T033 [US1] Add navigation: ← Module 1, Module 3 →

#### Module 3: The AI-Robot Brain (NVIDIA Isaac)

- [x] T034 [P] [US1] Create index.md in robolearn-interface/docs/module-3-isaac/
- [x] T035 [US1] Write module overview (2-3 paragraphs) explaining Isaac's role in perception and training
- [x] T036 [US1] Define 5-6 learning objectives with action verbs (configure, deploy, train, integrate)
- [x] T037 [US1] Create chapter index: Ch6 Isaac SDK (5hrs), Ch7 Isaac ROS (7hrs), Ch8 RL optional (6-8hrs)
- [x] T038 [US1] Add hardware tier table (Tier 1: cloud Omniverse, Tier 2: RTX GPU, Tier 3: Jetson)
- [x] T039 [US1] Document 4-Layer progression: L1-L3 (Ch6) → L2-L4 preview (Ch7) → L2-L4 (Ch8)
- [x] T040 [US1] Create Mermaid diagram showing Isaac SDK→ROS→RL progression
- [x] T041 [US1] Add prerequisites section (Modules 1-2 complete)
- [x] T042 [US1] Describe capstone: AI-powered navigation with VSLAM
- [x] T043 [US1] Add navigation: ← Module 2, Module 4 →

#### Module 4: Vision-Language-Action (VLA)

- [x] T044 [P] [US1] Create index.md in robolearn-interface/docs/module-4-vla/
- [x] T045 [US1] Write module overview (2-3 paragraphs) explaining VLA and humanoid robotics
- [x] T046 [US1] Define 5-6 learning objectives with action verbs (design, implement, orchestrate, deploy)
- [x] T047 [US1] Create chapter index: Ch9 Humanoid Kinematics (6hrs), Ch10 Conversational Robotics (5hrs), Ch11 Capstone (7-8hrs)
- [x] T048 [US1] Add hardware tier table (Tier 1: simulation+cloud voice, Tier 4: physical robot)
- [x] T049 [US1] Document 4-Layer progression: L1-L3 (Ch9) → L2-L4 preview (Ch10) → L3-L4 (Ch11)
- [x] T050 [US1] Create Mermaid diagram showing Kinematics→Voice→Autonomous Humanoid
- [x] T051 [US1] Add prerequisites section (Modules 1-3 complete)
- [x] T052 [US1] Describe capstone: Autonomous Humanoid (voice command → plan → navigate → manipulate)
- [x] T053 [US1] Add navigation: ← Module 3, Graduation 🎓

**Checkpoint**: All 4 module READMEs created with student discovery features

---

## Phase 4: User Story 2 - Instructor Plans Curriculum (Priority: P2)

**Goal**: Instructors can extract a 13-week teaching schedule from module READMEs with equipment requirements clear.

**Independent Test**: Review all 4 READMEs and verify:
1. Week breakdown maps to Weeks 1-13
2. Time estimates per chapter are visible
3. Hardware lab requirements are documented
4. Assessment points (capstones) identified

### Implementation for User Story 2

- [x] T054 [US2] Add week range to Module 1 frontmatter and overview (Weeks 1-5)
- [x] T055 [P] [US2] Add week range to Module 2 frontmatter and overview (Weeks 6-7)
- [x] T056 [P] [US2] Add week range to Module 3 frontmatter and overview (Weeks 8-10)
- [x] T057 [P] [US2] Add week range to Module 4 frontmatter and overview (Weeks 11-13)
- [x] T058 [US2] Enhance hardware tier tables with institutional procurement notes
- [x] T059 [US2] Add assessment section per module identifying capstone as primary evaluation
- [ ] T060 [US2] Add "Instructor Notes" admonition with lab scheduling tips per module (deferred)

**Checkpoint**: Instructors can plan 13-week curriculum from READMEs

---

## Phase 5: User Story 3 - Author Expands Content (Priority: P3)

**Goal**: Authors can use existing module README structure as template for future modules/books.

**Independent Test**: Use Module 1 README structure to outline a hypothetical Module 5 and verify:
1. All required sections present
2. Consistent structure across modules
3. Template is extractable

### Implementation for User Story 3

- [x] T061 [US3] Ensure consistent MDX frontmatter across all 4 modules (id, title, sidebar_position, description, keywords)
- [x] T062 [US3] Verify section ordering is identical across all modules
- [ ] T063 [P] [US3] Add inline comments marking reusable template sections in Module 1 (deferred)
- [x] T064 [US3] Create AUTHORING_GUIDE.md in specs/002-module-content/ documenting module structure (via MODULE_TEMPLATE.md)

**Checkpoint**: Authors can reference Module 1 as template

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Validate all requirements and ensure quality

- [x] T065 Validate FR-001 through FR-012 compliance using specs/002-module-content/checklists/requirements.md
- [ ] T066 [P] Verify all Mermaid diagrams render correctly in Docusaurus (pending: needs @docusaurus/theme-mermaid)
- [ ] T067 [P] Verify all internal links are functional (chapter links, module navigation)
- [x] T068 [P] Verify all citation links are valid (ROS 2, NVIDIA, Gazebo docs)
- [x] T069 Run educational-validator on all 4 module READMEs for constitutional compliance (PASS after fixes)
- [x] T070 Verify MDX syntax compatibility with Docusaurus 3.x
- [x] T071 Final review: 4-Layer Teaching Method visibility in each module

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in priority order (P1 → P2 → P3)
  - Within US1: All 4 modules can be created in parallel
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Primary content creation
- **User Story 2 (P2)**: Can start after US1 - Enhances existing READMEs
- **User Story 3 (P3)**: Can start after US1 - Documents template pattern

### Within User Story 1 (Module Creation)

Modules can be created in parallel (T013, T024, T034, T044 are all [P]):
- Module 1 tasks: T013-T023
- Module 2 tasks: T024-T033
- Module 3 tasks: T034-T043
- Module 4 tasks: T044-T053

### Parallel Opportunities

- All module directory creation tasks (T002-T005) can run in parallel
- All citation verification tasks (T008-T011) can run in parallel
- All 4 module README creations can run in parallel
- US2 week range additions (T055-T057) can run in parallel
- All polish validation tasks can run in parallel

---

## Parallel Example: User Story 1 - Module Creation

```bash
# Launch all 4 module index.md creations in parallel:
Task: "Create index.md in robolearn-interface/docs/module-1-ros2/"
Task: "Create index.md in robolearn-interface/docs/module-2-simulation/"
Task: "Create index.md in robolearn-interface/docs/module-3-isaac/"
Task: "Create index.md in robolearn-interface/docs/module-4-vla/"

# Then populate each module's content (can be parallelized across 4 modules)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (directory structure)
2. Complete Phase 2: Foundational (template, citations)
3. Complete Phase 3: User Story 1 (all 4 module READMEs)
4. **STOP and VALIDATE**: Test each README independently
5. Deploy to Docusaurus and verify rendering

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → All 4 READMEs → Test → Deploy (MVP!)
3. Add User Story 2 → Instructor enhancements → Test → Deploy
4. Add User Story 3 → Template documentation → Test → Deploy
5. Polish phase → Final validation → Production ready

### Content-Implementer Handoff

After Phase 2 (Foundational) completion:
1. content-implementer reads plan.md in full
2. Uses MODULE_TEMPLATE.md for consistent structure
3. Follows layer progression from plan.md
4. Cites research sources from spec.md
5. Validates against requirements.md checklist

---

## Summary

| Phase | Tasks | Parallel Tasks | Purpose |
|-------|-------|----------------|---------|
| 1. Setup | T001-T006 | 4 | Directory structure |
| 2. Foundational | T007-T012 | 5 | Template + citations |
| 3. US1 (P1) | T013-T053 | 4 | 4 module READMEs |
| 4. US2 (P2) | T054-T060 | 3 | Instructor features |
| 5. US3 (P3) | T061-T064 | 1 | Author template |
| 6. Polish | T065-T071 | 4 | Validation |
| **Total** | **71 tasks** | **21** | |

**MVP Scope**: Phases 1-3 (User Story 1) = 53 tasks → 4 complete module READMEs

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each module README should be independently completable
- Commit after each module completion
- Stop at any checkpoint to validate independently
- Use content-implementer agent for actual README generation
