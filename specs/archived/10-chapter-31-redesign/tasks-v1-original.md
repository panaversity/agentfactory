---
title: "Chapter 31 Implementation Tasks"
feature: "chapter-31-redesign"
date_created: "2025-11-05"
date_updated: "2025-11-05"
status: "Ready for Implementation"
total_tasks: 52
---

# Implementation Tasks: Chapter 31 — Spec-Kit Plus Hands-On

**Project**: CoLearning Python Book, Part 5: Spec-Kit Plus Methodology (Chapters 30-32)
**Feature**: Chapter 31 — Spec-Kit Plus Hands-On Workflow with Human Control & Checkpoint Pattern
**Branch**: `010-chapter-31-redesign`
**Duration**: ~16-20 hours (8 lessons + validation)

---

## Overview

This document breaks down Chapter 31 implementation into atomic, independently testable tasks organized by **user story** (learning objective). Each task is small enough for one person to complete in 2-4 hours and includes explicit acceptance criteria.

**Critical Updates** (2025-11-05):
- Human Control & Checkpoint Pattern emphasis integrated throughout
- Part 5 structure corrected (Chapters 30-32, not 25-27)
- ADR governance applied (ADR-001, ADR-002, ADR-003)
- Workflow isomorphism pattern (lessons mirror actual Spec-Kit Plus phases)
- CEFR proficiency scaffolding (A2→B1→B2 progression)

**Key Principles**:
- **User-Story-First Organization**: Tasks grouped by what students will learn (4 user stories from spec.md)
- **Independent Testing**: Each story's tasks can be completed and validated independently
- **Human Control Emphasis**: YOU control workflow; AI orchestrator collaborates; Spec-Kit Plus provides structure
- **Checkpoint Pattern**: Agent completes set → human reviews/commits → continues (not autonomous loops)
- **Atomic Tasks**: Small, reviewable, testable increments guided by human validation
- **Try With AI**: Every lesson ends with "Try With AI" activity (no "Key Takeaways" or "Next Steps" sections)

---

## Task Organization

- **Phase 1: Setup** (2 tasks) — Initialize branch, prepare directory structure
- **Phase 2: Foundational Documents** (6 tasks) — Chapter README, templates, references
- **Phase 3: User Story 1 — SMART Acceptance Criteria** (6 tasks) — Lesson 1 content & exercises
- **Phase 4: User Story 2 — Spec-Kit Plus Project Structure** (8 tasks) — Lessons 2-3 (Installation, Constitution, Specify)
- **Phase 5: User Story 3 — Calculator Workflow Cascade** (12 tasks) — Lessons 4-6 (Clarify, Plan, Tasks with checkpoint pattern)
- **Phase 6: User Story 4 — Implementation with Validation** (8 tasks) — Lesson 7 (Implement + Validation)
- **Phase 7: Capstone Integration** (4 tasks) — Lesson 8 (Complete workflow)
- **Phase 8: Validation & Polish** (6 tasks) — Technical review, ADR alignment, publication readiness

---

## Phase 1: Setup

- [ ] T001 Create chapter directory structure at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/` with subdirectories: `examples/`, `exercises/`, `ai-prompts/`
- [ ] T002 Verify feature branch `010-chapter-31-redesign` is active and all artifacts (spec.md, plan.md, tasks.md, ADR-001, ADR-002, ADR-003) are in `/specs/010-chapter-31-redesign/` and `/history/adr/`

**Gate**: Directory structure created; branch verified; ADRs present

---

## Phase 2: Foundational Documents

### Context
Create shared foundation documents that all 8 lessons reference. These establish chapter structure, terminology, and assessment criteria.

- [ ] T003 [P] Create Chapter README at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/README.md` with: chapter overview, learning journey (8 lessons), prerequisites (Chapter 30 completion), calculator project introduction, Part 5 context (Chapters 30-32), links to all 8 lessons

- [ ] T004 [P] Create lesson template at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/_lesson-template.md` with standard sections: title, duration, proficiency target (CEFR + Bloom's), learning objectives, skills taught, key concepts, content outline, key insights, code examples, "Try With AI" (final section)

- [ ] T005 [P] Create glossary at `/specs/010-chapter-31-redesign/glossary.md` defining: Spec-Kit Plus, Horizontal Intelligence (ADRs+PHRs), Vertical Intelligence (orchestrator+subagents), Checkpoint Pattern, Atomic Tasks, Cascade Effect, AIDD Paradigm, Human Control, Validation-First

- [ ] T006 [P] Create spec template reference at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/spec-template-reference.md` showing example of each section with annotations explaining purpose

- [ ] T007 [P] Create validation checklist template at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/validation-checklist-template.md` showing how to map acceptance criteria to testable conditions (pass/fail)

- [ ] T008 [P] Create chapter assessment rubric at `/specs/010-chapter-31-redesign/assessment-rubric.md` defining pass/fail criteria for each lesson (proficiency level achieved, skills demonstrated, deliverables completed) and capstone (B2 proficiency, complete workflow, working code)

**Gate**: All foundational documents created; chapter structure ready for lesson content

---

## Phase 3: User Story 1 — Student Writes SMART Acceptance Criteria

### Context
**User Story 1** (P1): Developer learning to write clear, testable requirements. This is THE key skill for AIDD: poor criteria → poor code; clear criteria → usable code.

**Independent Test**: Student can convert vague requirements into SMART criteria that AI can build from without interpretation.

- [ ] T009 [US1] Write Lesson 1 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/01-installation-and-setup.md`
  - Duration: 1.5 hours | Proficiency: A2 (Procedural) | Bloom's: Remember + Understand
  - Learning Objective: Install Spec-Kit Plus + AI tool (Claude Code OR Gemini CLI), verify commands work, understand Horizontal (ADRs+PHRs) + Vertical Intelligence (orchestrator+subagents)
  - Key Concepts (max 5): (1) Spec-Kit Plus = opinionated toolkit, (2) Horizontal Intelligence = ADRs+PHRs, (3) Vertical Intelligence = orchestrator→subagents, (4) AI tool options (Claude Code/Gemini CLI), (5) Project structure (.specify/, specs/, history/)
  - Content: Part A (AI-Native SDD Toolkit - 10min), Part B (Install Spec-Kit Plus - 20min), Part C (AI Tool Setup - 20min), Part D (Verify Commands - 15min), Part E (Initialize Calculator Project - 20min), Part F (Test Workflow - 5min)
  - End with: "Try With AI" (test `/sp.*` commands with your AI tool)

- [ ] T010 [P] [US1] Create Lesson 1 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-1-installation/`
  - Example 1: Installation commands (pip/uvx)
  - Example 2: Project structure diagram (`.specify/`, `specs/`, `history/`)
  - Example 3: Command verification output (running `/sp.specify --help`)
  - Example 4: Initialized calculator project structure

- [ ] T011 [P] [US1] Create Lesson 1 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/exercises/lesson-1-exercises.md`
  - Exercise 1: Complete installation and verify with `/sp.specify --version`
  - Exercise 2: Initialize calculator project and explore directory structure
  - Exercise 3: Explain Horizontal vs. Vertical Intelligence in your own words

- [ ] T012 [US1] Create Lesson 1 assessment at `/specs/010-chapter-31-redesign/lesson-1-assessment.md`
  - Pass criteria: Working Spec-Kit Plus installation, AI tool configured, `/sp.*` commands verified, calculator project initialized, can explain Horizontal + Vertical Intelligence

- [ ] T013 [P] [US1] Create "Try With AI" prompts for Lesson 1 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/ai-prompts/lesson-1-try-with-ai.md`
  - Prompt 1: "Test `/sp.specify --help` in your AI tool (Claude Code or Gemini CLI)"
  - Prompt 2: "Ask your AI tool to explain Vertical Intelligence architecture"
  - Prompt 3: "Initialize a test project and verify all commands work"

- [ ] T014 [P] [US1] Create Claude Code/Gemini CLI setup guide at `/specs/010-chapter-31-redesign/ai-tool-setup-guide.md` explaining installation, configuration, command usage patterns for both tools

**Story Completion Gate**: Lesson 1 complete; installation verified; assessment criteria ready

---

## Phase 4: User Story 2 — Spec-Kit Plus Project Structure & Foundation (Lessons 2-3)

### Context
**User Story 2** (P1): Developer setting up first Spec-Kit Plus project, understanding structure enforces workflow, creating Constitution and initial specification.

**Independent Test**: Student can explain why structure enforces Spec → Plan → Tasks cascade and create Constitution + complete specification.

- [ ] T015 [US2] Write Lesson 2 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/02-constitution-phase.md`
  - Duration: 1.5 hours | Proficiency: A2 (Basic Application) | Bloom's: Understand + Apply
  - Learning Objective: Create Constitution once per project; understand Constitution → Commit → Feature loop; recognize Constitution as project governance
  - Key Concepts (max 5): (1) Constitution = project governance, (2) Created once (not per feature), (3) Commit pattern (Constitution → git commit → features), (4) Defines principles/standards/quality, (5) Template location (`.specify/templates/`)
  - Content: Part A (What is Constitution - 15min), Part B (Create Calculator Constitution - 40min), Part C (Commit Pattern - 10min), Part D (Review & Commit - 25min)
  - **Human Control Emphasis**: "YOU create Constitution; AI helps articulate YOUR principles"
  - End with: "Try With AI" (create Constitution for calculator project)

- [ ] T016 [US2] Write Lesson 3 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/03-specify-phase.md`
  - Duration: 2-2.5 hours | Proficiency: A2 (Basic Application) | Bloom's: Understand + Apply
  - Learning Objective: Write complete specification with all components; understand spec as source of truth; use `/sp.specify` for guided workflow
  - Key Concepts (max 5): (1) Specification = complete requirements document, (2) Spec template sections (Overview, Scope, Requirements, Acceptance Criteria, Constraints), (3) `/sp.specify` command launches specification workflow, (4) Spec quality → everything downstream, (5) Cascade principle intro
  - Content: Part A (Spec Components - 20min), Part B (Use `/sp.specify` - 40min), Part C (Calculator Spec - 60min), Part D (Completeness Check - 20min)
  - **Human Control Emphasis**: "YOU define requirements; AI helps structure and validate completeness"
  - End with: "Try With AI" (use `/sp.specify` to create calculator spec)

- [ ] T017 [P] [US2] Create Lesson 2 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-2-constitution/`
  - Example 1: Sample Constitution structure
  - Example 2: Calculator project Constitution (principles, standards, quality criteria)
  - Example 3: Git commit pattern demonstration

- [ ] T018 [P] [US2] Create Lesson 3 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-3-specification/`
  - Example 1: Incomplete spec (missing sections)
  - Example 2: Complete calculator spec with all sections
  - Example 3: Side-by-side showing impact of completeness on clarity

- [ ] T019 [P] [US2] Create Lessons 2-3 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/exercises/lesson-2-3-exercises.md`
  - Lesson 2 Exercises: (1) Create Constitution for calculator, (2) Commit Constitution to git, (3) Explain why Constitution is created once
  - Lesson 3 Exercises: (1) Use `/sp.specify` to start calculator spec, (2) Fill all required sections, (3) Review completeness against template

- [ ] T020 [US2] Create Lessons 2-3 assessment at `/specs/010-chapter-31-redesign/lessons-2-3-assessment.md`
  - Lesson 2: Pass criteria - Constitution created and committed, can explain Constitution → Commit → Feature pattern
  - Lesson 3: Pass criteria - Complete spec with all sections filled, can explain cascade principle, spec is "ready for planning"

- [ ] T021 [P] [US2] Create "Try With AI" prompts for Lessons 2-3 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/ai-prompts/lesson-2-3-try-with-ai.md`
  - Lesson 2: "Create Constitution for calculator project with AI help"
  - Lesson 3: "Use `/sp.specify` to create complete calculator specification"

- [ ] T022 [P] [US2] Create Constitution template reference at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/constitution-template-reference.md` showing example Constitution structure with annotations

**Story Completion Gate**: Lessons 2-3 complete; Constitution + Spec workflow demonstrated; cascade principle introduced

---

## Phase 5: User Story 3 — Calculator Workflow Cascade with Checkpoint Pattern (Lessons 4-6)

### Context
**User Story 3** (P2): Developer experiences complete SDD workflow with calculator project. Demonstrates cascade: clear spec → clear plan → clear tasks. **CRITICAL**: Checkpoint pattern emphasized throughout.

**Independent Test**: Student can describe how spec quality determines plan/task quality AND implement checkpoint pattern (agent completes set → human reviews/commits → continues).

- [ ] T023 [US3] Write Lesson 4 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/04-clarify-phase.md`
  - Duration: 1.5 hours | Proficiency: B1 (Intermediate Application) | Bloom's: Apply + Analyze
  - Learning Objective: Use `/sp.clarify` to identify spec gaps/ambiguities; iterate on spec based on AI feedback; understand refinement improves downstream quality
  - Key Concepts (max 7): (1) `/sp.clarify` analyzes spec for gaps, (2) Gap types (missing edge cases, vague terms, undefined behavior), (3) Iteration cycle (clarify → refine → re-clarify), (4) Spec refinement improves plan quality, (5) Human decides which gaps to address, (6) AI provides analysis, YOU make decisions, (7) Checkpoint: Review clarifications before proceeding
  - Content: Part A (`/sp.clarify` Overview - 15min), Part B (Run Clarification - 30min), Part C (Gap Analysis - 25min), Part D (Refinement Iteration - 20min)
  - **Human Control Emphasis**: "AI identifies gaps; YOU decide which to address and how"
  - **Checkpoint Pattern**: "Review clarifications → decide to proceed or refine more → then continue to planning"
  - End with: "Try With AI" (run `/sp.clarify` on your calculator spec, iterate)

- [ ] T024 [US3] Write Lesson 5 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/05-plan-phase.md`
  - Duration: 2 hours | Proficiency: B1 (Intermediate Application) | Bloom's: Apply + Analyze
  - Learning Objective: Use `/sp.plan` to generate implementation plan; understand phase structure and dependencies; create ADRs for significant decisions
  - Key Concepts (max 7): (1) `/sp.plan` generates architecture from spec, (2) Plan structure (phases, components, dependencies), (3) ADRs document significant decisions, (4) Plan quality flows from spec quality (cascade), (5) Human reviews and approves plan, (6) AI generates, YOU validate and decide, (7) Checkpoint: Review plan before tasks
  - Content: Part A (`/sp.plan` Overview - 20min), Part B (Generate Plan - 40min), Part C (Review Plan Structure - 30min), Part D (Create ADRs - 30min)
  - **Human Control Emphasis**: "AI generates plan; YOU review architecture and make final decisions"
  - **Checkpoint Pattern**: "Review plan → decide architecture is sound → THEN proceed to tasks (don't skip)"
  - End with: "Try With AI" (run `/sp.plan` for calculator, review plan quality)

- [ ] T025 [US3] Write Lesson 6 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/06-tasks-phase.md`
  - Duration: 1.5 hours | Proficiency: B1 (Intermediate Application) | Bloom's: Apply + Analyze
  - Learning Objective: Use `/sp.tasks` to decompose plan into atomic work units; understand checkpoint pattern (agent completes Phase 1 → YOU review/commit → continue to Phase 2); trace task lineage back to spec
  - Key Concepts (max 7): (1) `/sp.tasks` decomposes plan into atomic tasks, (2) Atomic task = 1-2 hour completion with clear acceptance criteria, (3) **Checkpoint Pattern: Request "Generate Phase 1 tasks, pause for review"**, (4) Agent completes set → YOU review → YOU commit → YOU continue, (5) Task dependencies and sequencing, (6) Lineage tracing (spec → plan → task), (7) Human guides progression (not autonomous execution)
  - Content: Part A (`/sp.tasks` Overview - 20min), Part B (**Checkpoint Pattern Demonstration** - 40min showing Phase 1 pause), Part C (Task Structure Analysis - 20min), Part D (Lineage Tracing - 10min)
  - **Human Control Emphasis**: "YOU control progression; agent completes sets as YOU request (not autonomously)"
  - **Checkpoint Pattern CRITICAL**: Demonstrate: "Generate Phase 1 (add, subtract) tasks → PAUSE → Review → Commit → Request Phase 2 (multiply, divide, power)"
  - End with: "Try With AI" (run `/sp.tasks` with checkpoints for calculator)

- [ ] T026 [P] [US3] Create Lesson 4 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-4-clarify/`
  - Example 1: Running `/sp.clarify` output showing gaps
  - Example 2: Spec before clarification (with gaps)
  - Example 3: Spec after clarification (gaps addressed)
  - Example 4: Cascade demonstration (better spec → better feedback)

- [ ] T027 [P] [US3] Create Lesson 5 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-5-plan/`
  - Example 1: Running `/sp.plan` output
  - Example 2: Generated plan structure (phases, components)
  - Example 3: Sample ADR for calculator (e.g., "Use Python Decimal for precision")
  - Example 4: Cascade demonstration (clear spec → clear plan structure)

- [ ] T028 [P] [US3] Create Lesson 6 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-6-tasks/`
  - Example 1: **Checkpoint pattern command: "Generate Phase 1 tasks, pause"**
  - Example 2: Phase 1 tasks output (add, subtract)
  - Example 3: **Checkpoint: Human reviews → commits → requests Phase 2**
  - Example 4: Phase 2 tasks output (multiply, divide, power)
  - Example 5: Lineage diagram (spec requirement → plan phase → task)

- [ ] T029 [P] [US3] Create Lessons 4-6 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/exercises/lesson-4-6-exercises.md`
  - Lesson 4: (1) Run `/sp.clarify`, (2) Document gaps, (3) Refine spec, (4) Re-run and verify improvement
  - Lesson 5: (1) Run `/sp.plan`, (2) Review plan structure, (3) Identify dependencies, (4) Create 1 ADR
  - Lesson 6: (1) **Use checkpoint pattern: Request Phase 1 only**, (2) Review tasks, (3) Commit, (4) Request Phase 2, (5) Trace task lineage

- [ ] T030 [US3] Create Lessons 4-6 assessment at `/specs/010-chapter-31-redesign/lessons-4-6-assessment.md`
  - Lesson 4: Pass criteria - Can run `/sp.clarify`, interpret gaps, refine spec, articulate improvement
  - Lesson 5: Pass criteria - Can run `/sp.plan`, understand plan structure, create ADR, explain cascade
  - Lesson 6: Pass criteria - **Can implement checkpoint pattern**, understand atomic tasks, trace lineage

- [ ] T031 [P] [US3] Create "Try With AI" prompts for Lessons 4-6 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/ai-prompts/lesson-4-6-try-with-ai.md`
  - Lesson 4: "Run `/sp.clarify` on calculator spec, iterate until gaps resolved"
  - Lesson 5: "Run `/sp.plan` for calculator, create ADR for precision handling"
  - Lesson 6: **"Use checkpoint pattern: Generate Phase 1 tasks → pause → review → continue"**

- [ ] T032 [P] [US3] Create checkpoint pattern reference at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/checkpoint-pattern-reference.md` showing:
  - Command format: "Generate Phase 1 tasks, then pause for my review"
  - Agent completes → returns to YOU → YOU review/commit → YOU continue
  - Why checkpoints matter (prevents autonomous loops, keeps YOU in control)
  - Professional practice alignment (validation-first safety)

- [ ] T033 [P] [US3] Create task lineage reference at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/task-lineage-reference.md` showing calculator example:
  - Spec requirement: "Calculator must handle division by zero safely"
  - Plan phase: "Error Handling Component"
  - Task: "Implement division function with ZeroDivisionError handling"
  - Code: Actual implementation with try/except

- [ ] T034 [P] [US3] Create ADR creation guide at `/specs/010-chapter-31-redesign/adr-creation-guide.md` explaining significance test (impact + alternatives + scope) and when to create ADRs vs. PHRs

**Story Completion Gate**: Lessons 4-6 complete; checkpoint pattern demonstrated; cascade effect visible; ADR workflow practiced

---

## Phase 6: User Story 4 — Implementation with Validation (Lesson 7)

### Context
**User Story 4** (P2): Developer implements using `/sp.implement`, learns validation as core skill, sees complete AIDD loop. **CRITICAL**: Checkpoint pattern applies to implementation (implement Phase 1 → pause → review → continue).

**Independent Test**: Student can run `/sp.implement` with checkpoints, validate code against acceptance criteria, distinguish spec ambiguity from code error.

- [ ] T035 [US4] Write Lesson 7 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/07-implement-phase.md`
  - Duration: 2.5-3 hours | Proficiency: B1-B2 (Intermediate to Advanced) | Bloom's: Apply + Analyze + Evaluate
  - Learning Objective: Use `/sp.implement` with checkpoint pattern; validate generated code against acceptance criteria; understand AIDD cycle (intent → generation → validation → refinement); provide feedback for iteration
  - Key Concepts (max 10): (1) `/sp.implement` orchestrates code generation, (2) **Checkpoint pattern: "Implement Phase 1 tasks, pause for review"**, (3) Agent generates → YOU validate → YOU commit, (4) AIDD loop: Intent (YOU) → Generation (AI) → Validation (YOU) → Decision (YOU), (5) Validation checklist from acceptance criteria, (6) Pass/fail per criterion, (7) Distinguish spec ambiguity vs. code error, (8) Feedback for refinement, (9) Never accept code you don't understand, (10) Human guides implementation progression
  - Content: Part A (AIDD Loop Overview - 20min), Part B (**Checkpoint Implementation Demo** - 60min showing Phase 1 pause), Part C (Validation Mastery - 50min), Part D (Refinement Cycle - 30min)
  - **Human Control Emphasis**: "YOU validate every output; AI generates code, YOU decide if it's correct"
  - **Checkpoint Pattern CRITICAL**: Demonstrate: "Implement Phase 1 (add, subtract) → PAUSE → Review code → Test → Commit → Request Phase 2 (multiply, divide, power)"
  - End with: "Try With AI" (implement calculator with checkpoints, validate each phase)

- [ ] T036 [P] [US4] Create Lesson 7 code examples at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/examples/lesson-7-implement/`
  - Example 1: **Checkpoint command: "Implement Phase 1 tasks, pause"**
  - Example 2: Generated code for Phase 1 (add, subtract functions)
  - Example 3: Validation checklist (acceptance criteria → pass/fail)
  - Example 4: Code that passes criteria
  - Example 5: Code that fails criterion (demonstrates feedback loop)
  - Example 6: **Checkpoint: Review → Test → Commit → Request Phase 2**
  - Example 7: Complete calculator implementation (all phases)

- [ ] T037 [P] [US4] Create Lesson 7 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/exercises/lesson-7-exercises.md`
  - Exercise 1: **Use checkpoint pattern: Request Phase 1 implementation only**
  - Exercise 2: Create validation checklist from acceptance criteria
  - Exercise 3: Test Phase 1 code against each criterion (pass/fail)
  - Exercise 4: Commit Phase 1, request Phase 2
  - Exercise 5: If code fails, provide feedback and request refinement
  - Exercise 6: Distinguish spec ambiguity from code error

- [ ] T038 [US4] Create Lesson 7 assessment at `/specs/010-chapter-31-redesign/lesson-7-assessment.md`
  - Pass criteria: **Can implement checkpoint pattern**, generates code with `/sp.implement`, validates against criteria, identifies pass/fail, provides feedback, distinguishes spec vs. code issues

- [ ] T039 [P] [US4] Create "Try With AI" prompts for Lesson 7 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/ai-prompts/lesson-7-try-with-ai.md`
  - Prompt 1: **"Implement Phase 1 tasks (add, subtract), pause for my review"**
  - Prompt 2: "Create validation checklist from acceptance criteria"
  - Prompt 3: "If code fails criterion, explain why and request refinement"

- [ ] T040 [P] [US4] Create validation mastery guide at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/validation-mastery-guide.md` showing:
  - How to create validation checklist from acceptance criteria
  - Pass/fail testing per criterion
  - Distinguishing spec ambiguity (go back to spec) vs. code error (provide feedback)
  - Professional validation checklist (technical accuracy, security, performance, maintainability)

**Story Completion Gate**: Lesson 7 complete; checkpoint pattern applied to implementation; validation demonstrated; AIDD loop visible

---

## Phase 7: Capstone Integration (Lesson 8)

### Context
**Lesson 8** consolidates all 7 phases into complete end-to-end workflow. Students apply everything learned to build working calculator from scratch.

**Independent Test**: Student completes Constitution → Specify → Clarify → Plan → Tasks → Implement → Validate workflow independently, producing working calculator with all artifacts.

- [ ] T041 Write Lesson 8 content at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/08-capstone-integration.md`
  - Duration: 3-4 hours | Proficiency: B2 (Advanced) | Bloom's: Create + Evaluate
  - Learning Objective: Complete full Spec-Kit Plus workflow independently; apply checkpoint pattern throughout; produce working calculator with complete artifacts; reflect on learning
  - Key Concepts: (1) Complete workflow integration, (2) Independent application, (3) Checkpoint pattern throughout, (4) Artifact completeness (Constitution, Spec, Plan, Tasks, ADRs, PHRs, Code), (5) Reflection and meta-learning
  - Content: Part A (Capstone Overview - 15min), Part B (Complete Workflow Execution - 180min with checkpoints), Part C (Validation & Testing - 45min), Part D (Reflection - 30min)
  - **Human Control Emphasis**: "YOU own the complete workflow; YOU make all decisions; AI collaborates"
  - **Checkpoint Pattern**: Applied at every phase (Clarify → review → Plan → review → Tasks → review → Implement Phase 1 → review → Phase 2 → review → etc.)
  - Deliverables: Working calculator + complete artifact set + reflection document (500-1000 words)
  - End with: "Try With AI" (complete independent calculator project)

- [ ] T042 [P] Create Lesson 8 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/exercises/lesson-8-exercises.md`
  - Exercise: Complete calculator project independently
  - Reflection prompts: (1) What surprised you? (2) How did spec quality affect outcomes? (3) How did checkpoint pattern help? (4) What would you do differently?

- [ ] T043 Create Lesson 8 assessment at `/specs/010-chapter-31-redesign/lesson-8-assessment.md`
  - Pass criteria: Complete workflow executed, working calculator with all 5 operations (add, subtract, multiply, divide, power), all artifacts present (Constitution, Spec, Plan, Tasks, ADRs, PHRs, Code), checkpoint pattern demonstrated, reflection shows learning

- [ ] T044 [P] Create capstone rubric at `/specs/010-chapter-31-redesign/capstone-rubric.md`
  - Specification: Complete? Clear? Testable? (Excellent/Good/Satisfactory/Needs Work)
  - Plan: Logical phases? Dependencies identified? (Excellent/Good/Satisfactory/Needs Work)
  - Tasks: Atomic? Well-sequenced? Checkpoint pattern used? (Excellent/Good/Satisfactory/Needs Work)
  - Code: Works? Passes acceptance criteria? Clean? (Excellent/Good/Satisfactory/Needs Work)
  - Reflection: Insightful? Demonstrates meta-learning? (Excellent/Good/Satisfactory/Needs Work)

**Story Completion Gate**: Lesson 8 complete; capstone demonstrated; complete workflow validated

---

## Phase 8: Validation & Polish

- [ ] T045 [P] Run constitutional alignment check: Verify all lessons follow ADR-001 (workflow isomorphism), ADR-002 (CEFR proficiency scaffolding), ADR-003 (human control & checkpoint pattern). Document findings at `/specs/010-chapter-31-redesign/constitutional-alignment-report.md`

- [ ] T046 [P] Run technical review: Verify all code examples run correctly, all `/sp.*` commands are shown correctly (within AI tool context, not standalone terminal), checkpoint pattern demonstrated properly. Fix issues and document at `/specs/010-chapter-31-redesign/technical-review-report.md`

- [ ] T047 [P] Run proficiency validation: Verify CEFR levels match cognitive load (A2: max 5 concepts, B1: max 7, B1-B2: max 10), Bloom's levels align with proficiency. Document findings at `/specs/010-chapter-31-redesign/proficiency-validation-report.md`

- [ ] T048 Validate "Try With AI" sections: Confirm all 8 lessons end with "Try With AI" (NO "Key Takeaways" or "Next Steps"). Lessons 1-3 use ChatGPT web; Lessons 4-8 use Claude Code or Gemini CLI. Document compliance at `/specs/010-chapter-31-redesign/try-with-ai-compliance.md`

- [ ] T049 Create chapter completion checklist at `/specs/010-chapter-31-redesign/chapter-completion-checklist.md` confirming:
  - [ ] All 8 lessons written (01-08.md files)
  - [ ] All lessons follow ADR-001 (workflow isomorphism)
  - [ ] All lessons follow ADR-002 (CEFR proficiency)
  - [ ] All lessons follow ADR-003 (human control & checkpoint pattern)
  - [ ] All "Try With AI" sections present (no "Key Takeaways")
  - [ ] Checkpoint pattern demonstrated in Lessons 6-8
  - [ ] All code examples tested and working
  - [ ] All `/sp.*` commands shown in AI tool context (not terminal)
  - [ ] Constitutional alignment verified
  - [ ] Proficiency levels validated
  - [ ] Capstone rubric complete

- [ ] T050 Create final publication checklist at `/specs/010-chapter-31-redesign/publication-checklist.md` confirming:
  - [ ] Technical review PASSED
  - [ ] Constitutional alignment PASSED
  - [ ] Proficiency validation PASSED
  - [ ] All exercises have solutions
  - [ ] All assessments have rubrics
  - [ ] README.md accurate and complete
  - [ ] All file paths correct
  - [ ] No broken links
  - [ ] Ready for learner testing

**Gate**: All validation complete; chapter ready for publication

---

## Task Dependency Graph

```
Phase 1 (Setup)
  ├─ T001-T002 (sequential)
  └─ GATE: Directory + branch ready

Phase 2 (Foundational)
  ├─ T003-T008 (parallel - all independent)
  └─ GATE: Foundation documents ready

Phase 3 (US1 - Installation)
  ├─ T009-T014 (T009 first, rest parallel)
  └─ GATE: Lesson 1 ready

Phase 4 (US2 - Constitution & Specify)
  ├─ T015-T022 (T015, T016 first; rest parallel)
  └─ GATE: Lessons 2-3 ready

Phase 5 (US3 - Clarify/Plan/Tasks with Checkpoints)
  ├─ T023-T034 (T023, T024, T025 sequential; examples/exercises parallel)
  └─ GATE: Lessons 4-6 ready with checkpoint pattern

Phase 6 (US4 - Implement & Validate)
  ├─ T035-T040 (T035 first; rest parallel)
  └─ GATE: Lesson 7 ready with validation

Phase 7 (Capstone)
  ├─ T041-T044 (T041 first; rest parallel)
  ├─ (Depends on: All Lessons 1-7 complete)
  └─ GATE: Lesson 8 ready

Phase 8 (Validation)
  ├─ T045-T050 (T045-T048 parallel; T049-T050 sequential after)
  ├─ (Depends on: All content created)
  └─ GATE: Chapter ready for publication
```

---

## Parallel Execution Strategy

**High Parallelism Opportunities**:

1. **Phase 2** (Foundational): T003-T008 all independent (6 tasks parallel)
2. **Phase 3** (US1): T010-T014 parallel after T009 (5 tasks parallel)
3. **Phase 4** (US2): T017-T022 parallel after T015-T016 (6 tasks parallel)
4. **Phase 5** (US3): T026-T034 parallel after T023-T025 (9 tasks parallel)
5. **Phase 6** (US4): T036-T040 parallel after T035 (5 tasks parallel)
6. **Phase 7** (Capstone): T042-T044 parallel after T041 (3 tasks parallel)
7. **Phase 8** (Validation): T045-T048 parallel (4 tasks parallel)

**Recommended Execution Order**:
1. Phase 1 (Setup) - 30 min
2. Phase 2 (Foundational) - 2 hours (parallelize all 6)
3. Phase 3 (US1) - 3 hours (T009 → rest parallel)
4. Phase 4 (US2) - 4 hours (T015-T016 → rest parallel)
5. Phase 5 (US3) - 6 hours (T023-T025 → rest parallel)
6. Phase 6 (US4) - 4 hours (T035 → rest parallel)
7. Phase 7 (Capstone) - 3 hours (T041 → rest parallel)
8. Phase 8 (Validation) - 2 hours (T045-T048 parallel → T049-T050)

**Total Estimated Time**: 18-22 hours

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)
**Deliverable**: Lessons 1-3 complete + foundation documents
- **Focus**: Installation, Constitution, Specify phases
- **Time**: 8-10 hours
- **Rationale**: Establishes foundation before tool-assisted workflow
- **Tasks**: T001-T022

### Phase 2: Tool-Assisted Workflow
**Deliverable**: Lessons 4-7 complete with checkpoint pattern
- **Focus**: Clarify, Plan, Tasks, Implement phases with human control
- **Time**: 10-12 hours
- **Prerequisite**: MVP complete
- **Tasks**: T023-T040

### Phase 3: Capstone & Validation
**Deliverable**: Lesson 8 + complete validation
- **Focus**: End-to-end integration, validation, publication readiness
- **Time**: 4-6 hours
- **Prerequisite**: Lessons 1-7 complete
- **Tasks**: T041-T050

---

## Success Criteria

### Chapter Completion
✅ All 8 lessons written and validated
✅ Human control & checkpoint pattern emphasized throughout (ADR-003 compliance)
✅ Workflow isomorphism demonstrated (ADR-001 compliance)
✅ CEFR proficiency scaffolding validated (ADR-002 compliance)
✅ All "Try With AI" sections present (no "Key Takeaways")
✅ Checkpoint pattern demonstrated in Lessons 6-8 (Tasks, Implement, Capstone)
✅ All code examples tested and working
✅ All `/sp.*` commands shown in AI tool context (Claude Code/Gemini CLI)
✅ Constitutional alignment verified
✅ Capstone project rubric complete
✅ Ready for publication

---

## Files to Create (Summary)

### Lesson Content Files (8 files)
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/01-installation-and-setup.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/02-constitution-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/03-specify-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/04-clarify-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/05-plan-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/06-tasks-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/07-implement-phase.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on/08-capstone-integration.md`

### Support Files (30+ files)
- README.md, lesson template, glossary, spec template reference, validation checklist template, assessment rubric
- Code examples (per lesson: 8 directories)
- Exercises (8 files)
- Assessments (8 files)
- "Try With AI" prompts (8 files)
- Reference documents (checkpoint pattern, task lineage, ADR creation guide, Constitution template, validation mastery guide)

### Spec/Plan Support Files (in `/specs/010-chapter-31-redesign/`)
- glossary.md, lesson-1-assessment.md through lesson-8-assessment.md, capstone-rubric.md, ai-tool-setup-guide.md, adr-creation-guide.md, constitutional-alignment-report.md, technical-review-report.md, proficiency-validation-report.md, try-with-ai-compliance.md, chapter-completion-checklist.md, publication-checklist.md

---

## Policy Note for Lesson Authors

**"Try With AI" Section Requirement**:
- **REQUIRED**: Every lesson MUST end with a single final section titled "Try With AI"
- **PROHIBITED**: Do NOT include "Key Takeaways" or "What's Next" sections
- **Before AI Tools Taught** (Lessons 1-3): Use ChatGPT web in "Try With AI" section
- **After Tool Onboarding** (Lessons 4-8): Instruct learners to use preferred AI tool (Claude Code or Gemini CLI); optionally provide CLI and web variants

---

**Tasks updated: 2025-11-05 to incorporate ADR governance, human control emphasis, checkpoint pattern, and Part 5 chapter structure corrections (30-32).**
