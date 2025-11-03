---
title: "Chapter 31 Implementation Tasks"
feature: "chapter-31-redesign"
date_created: "2025-11-03"
status: "Ready for Implementation"
total_tasks: 47
---

# Implementation Tasks: Chapter 31 — SpecifyPlus Hands-On

**Project**: CoLearning Python Book, Part 5: Specification-Driven Development
**Feature**: Redesign Chapter 31 with correct SpecifyPlus workflow, eliminate hallucinations
**Branch**: `010-chapter-31-redesign`
**Duration**: ~16-20 hours (7 lessons + capstone + validation)

---

## Overview

This document breaks down the Chapter 31 redesign into atomic, independently testable tasks organized by **user story** (learning objective). Each task is small enough for one person to complete in 2-4 hours and includes explicit acceptance criteria.

**Key Principles**:
- **User-Story-First Organization**: Tasks grouped by what students will learn (5 user stories)
- **Independent Testing**: Each story's tasks can be completed and validated independently
- **Spec-to-Content Flow**: Each lesson content flows directly from plan.md specifications
- **Hands-On Projects**: Two real projects (calculator, grading system) thread through lessons
- **Try With AI**: Every lesson ends with "Try With AI" activity (no "Key Takeaways" or "Next Steps" sections)

---

## Task Organization

- **Phase 1: Setup** (2 tasks) — Initialize branch, prepare templates
- **Phase 2: Foundational Lessons** (6 tasks) — Lessons 1-3 (manual thinking skills)
- **Phase 3: User Story 1 - SMART Criteria** (5 tasks) — Lesson 1 content
- **Phase 4: User Story 2 - Project Structure** (5 tasks) — Lesson 2 content
- **Phase 5: User Story 3 - Complete Specs** (6 tasks) — Lesson 3 content
- **Phase 6: User Story 4 - Spec Refinement** (6 tasks) — Lesson 4 content
- **Phase 7: User Story 5 - Planning & Tasks** (8 tasks) — Lessons 5-6 content
- **Phase 8: Implementation & Validation** (6 tasks) — Lesson 7 content
- **Phase 9: Capstone Project** (2 tasks) — Full workflow project
- **Phase 10: Polish & Validation** (3 tasks) — Final review, testing, publication

---

## Phase 1: Setup

- [ ] T001 Create lesson content templates directory structure at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/`
- [ ] T002 Initialize branch `010-chapter-31-redesign`, verify all spec.md, plan.md, and this tasks.md are in `/specs/010-chapter-31-redesign/`

**Gate**: All artifacts present and branch is clean

---

## Phase 2: Foundational Lessons Architecture

### Context
These tasks create the **lesson template structure** that all 7 lessons will follow. This is a shared foundation.

- [ ] T003 [P] Create lesson template with standard sections (Learning Objectives, Key Concepts, Content Outline, Key Insight, Code Examples, Try With AI) at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/_lesson-template.md`

- [ ] T004 [P] Create README.md for Chapter 31 with chapter overview, learning journey, prerequisites, and links to all 7 lessons at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/README.md`

- [ ] T005 Document the "Try With AI" activity pattern (end every lesson with interactive AI exercise, use ChatGPT web OR student's preferred CLI tool after onboarding) in implementation guide at `/specs/010-chapter-31-redesign/implementation-guide.md`

- [ ] T006 [P] Create code examples repository structure: `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/` with subdirectories for lesson-specific code, grading system, calculator

- [ ] T007 Create assessment rubric document at `/specs/010-chapter-31-redesign/assessment-rubric.md` defining pass/fail criteria for each lesson and capstone (SMART criteria validation, spec completeness, tool usage, code validation, reflection quality)

- [ ] T008 [P] Create glossary/reference document at `/specs/010-chapter-31-redesign/glossary.md` defining key terms (SMART, cascade, spec-first, AIDD, validation, co-learning) for easy student reference

**Gate**: All foundational documents created; structure ready for lesson content

---

## Phase 3: User Story 1 — Student Writes SMART Acceptance Criteria

### Context
Students learn to write clear, testable acceptance criteria using SMART framework. This is the foundation for all other lessons.

**Independent Test**: Student can convert vague requirements into 3+ SMART criteria, each with testable pass/fail conditions.

- [ ] T009 [US1] Write Lesson 1 content: "SMART Acceptance Criteria" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/01-smart-acceptance-criteria.md`
  - Duration: 2 hours
  - Sections: Learning objectives, Key Concepts (5 max), Content Outline (Part A: Problem, Part B: Framework, Part C: Practice)
  - Code examples: Vague requirement → bad code (demo), SMART criteria → good code (comparison)
  - Exercises: 3 practice exercises (identify vague terms, convert to SMART, peer review)
  - Assessment: Student produces SMART criteria template + 3 refined examples
  - End with: "Try With AI" activity (ask ChatGPT web to help convert vague requirements)

- [ ] T010 [P] [US1] Create code examples for Lesson 1 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-1-smart/`
  - Example 1: Vague requirement code (what AI generates from "helpful feedback")
  - Example 2: SMART criteria code (same requirement, now specific)
  - Example 3: Comparison showing difference (side-by-side)
  - Each with: purpose, complexity level, expected output

- [ ] T011 [P] [US1] Create Lesson 1 exercises document at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-1-exercises.md`
  - Exercise 1: Identify vague terms in 5 sample requirements
  - Exercise 2: Write 5 SMART acceptance criteria from vague feature description
  - Exercise 3: Critique peer's SMART criteria using framework
  - Solutions provided with explanations

- [ ] T012 [US1] Create Lesson 1 assessment checklist at `/specs/010-chapter-31-redesign/lesson-1-assessment.md`
  - Pass criteria: SMART criteria template completed, 3 refined examples with measurable conditions
  - Rubric: Each criterion is Specific (clear), Measurable (has numbers/metrics), Achievable (realistic), Relevant (matters), Time-bound (scoped)

- [ ] T013 [US1] Create "Try With AI" prompt templates for Lesson 1 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-1-try-with-ai.md`
  - Prompt 1: ChatGPT web - Help refine vague requirement
  - Prompt 2: Claude CLI - Review SMART criteria completeness
  - Prompt 3: Student-generated prompts (example format provided)

**Story Completion Gate**: Lesson 1 complete, assessment checklist ready, "Try With AI" verified

---

## Phase 4: User Story 2 — Student Understands SpecifyPlus Project Structure

### Context
Students learn why the Spec → Plan → Tasks cascade enforces clarity. This lesson explains the "structure teaches workflow" principle.

**Independent Test**: Student can explain the 3 directories (.specify/, specs/, history/), why spec.md/plan.md/tasks.md exist in sequence, and why you can't skip directly to tasks.

- [ ] T014 [US2] Write Lesson 2 content: "SpecifyPlus Project Structure & Cascade" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/02-specifyplus-structure.md`
  - Duration: 1.5 hours
  - Sections: Learning objectives, Key Concepts (5 max), Content Outline (Part A: Explore structure, Part B: Ask AI why, Part C: Map to SMART)
  - Initialize project demo: `pip install specifyplus` or `uvx specifyplus init grading-system`
  - Explain: .specify/ (templates, constitution, config), specs/ (spec.md, plan.md, tasks.md), history/ (prompts, decisions)
  - Show cascade: Spec quality → Plan quality → Task quality
  - Activities: Explore real project structure, ask AI companion why structure matters
  - Assessment: Explain cascade principle, describe what forces Spec → Plan → Tasks workflow
  - End with: "Try With AI" activity (ask AI to explain why structure enforces workflow)

- [ ] T015 [P] [US2] Create code examples for Lesson 2 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-2-structure/`
  - Example 1: Folder structure diagram (ASCII or visual)
  - Example 2: spec.md excerpt (what goes here)
  - Example 3: plan.md excerpt (shows how it flows from spec)
  - Example 4: tasks.md excerpt (shows how it flows from plan)

- [ ] T016 [P] [US2] Create Lesson 2 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-2-exercises.md`
  - Exercise 1: Initialize a SpecifyPlus project, explore structure, describe what you see
  - Exercise 2: Map Lesson 1's SMART criteria to spec template sections
  - Exercise 3: Predict: What would happen if you tried to write tasks without a plan?

- [ ] T017 [US2] Create Lesson 2 assessment at `/specs/010-chapter-31-redesign/lesson-2-assessment.md`
  - Pass criteria: Student can explain cascade principle, describe purpose of each directory, explain why structure enforces workflow

- [ ] T018 [US2] Create "Try With AI" prompts for Lesson 2 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-2-try-with-ai.md`
  - Prompt 1: ChatGPT web - Explain why SpecifyPlus structure forces spec-first workflow
  - Prompt 2: Gemini CLI example - Show how to initialize project and explore

**Story Completion Gate**: Lesson 2 complete, cascade principle explained, exercises ready

---

## Phase 5: User Story 3 — Student Writes Complete Specifications

### Context
Students write full specifications with all components (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria). This lesson integrates SMART thinking into formal template.

**Independent Test**: Student completes a full specification.md with all sections filled for grading system or calculator.

- [ ] T019 [US3] Write Lesson 3 content: "Complete Specification Writing" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/03-complete-specification.md`
  - Duration: 2 hours
  - Sections: Learning objectives, Key Concepts (6 max), Content Outline (Part A: Components, Part B: Relationships, Part C: Readiness check)
  - Map Lesson 1 (SMART) to formal template sections
  - Explain: Overview, Scope (In/Out), Requirements, Acceptance Criteria, Constraints, Success Criteria
  - Component relationships: How they work together
  - Readiness criteria: What makes a spec "ready for planning"
  - Project: Grading system specification (students fill in all sections)
  - Activities: Write full spec, peer review, iterate based on feedback
  - Assessment: Complete specification.md with all sections filled
  - End with: "Try With AI" activity (ask AI to validate spec completeness)

- [ ] T020 [P] [US3] Create code examples for Lesson 3 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-3-specification/`
  - Example 1: Incomplete spec (missing sections) - what it looks like
  - Example 2: Complete spec (grading system) - with all sections
  - Example 3: Side-by-side comparison showing flow from Lesson 1 (SMART) to formal template

- [ ] T021 [P] [US3] Create Lesson 3 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-3-exercises.md`
  - Exercise 1: Fill in grading system specification from provided outline
  - Exercise 2: Identify missing sections in incomplete spec, add them
  - Exercise 3: Peer review: Is this spec complete and ready for planning?

- [ ] T022 [US3] Create Lesson 3 assessment at `/specs/010-chapter-31-redesign/lesson-3-assessment.md`
  - Pass criteria: Complete specification.md with all sections (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria) filled clearly

- [ ] T023 [US3] Create "Try With AI" prompts for Lesson 3 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-3-try-with-ai.md`
  - Prompt 1: ChatGPT web - Review spec completeness
  - Prompt 2: Claude - Identify gaps in student's specification
  - Prompt 3: Gemini CLI - Validate that spec is "ready for planning"

- [ ] T024 [US3] Create specification template reference at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/spec-template-reference.md` showing example of each section (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria)

**Story Completion Gate**: Lesson 3 complete, full spec template reference ready, assessment checklist defined

---

## Phase 6: User Story 4 — Student Refines Specs with /sp.specify

### Context
Students use the actual SpecifyPlus `/sp.specify` command within Claude Code to analyze and refine specifications iteratively. This is where tool-assisted thinking begins.

**Independent Test**: Student can run `/sp.specify`, understand the feedback, iterate on spec, and articulate which gaps they filled and why each matters.

- [ ] T025 [US4] Write Lesson 4 content: "Refining Specs with /sp.specify" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/04-refining-specs-specify.md`
  - Duration: 1.5 hours
  - Sections: Learning objectives, Key Concepts (7 max), Content Outline (Part A: Tool overview, Part B: Running command, Part C: Iteration cycle)
  - Explain: `/sp.specify` analyzes spec, gives feedback on gaps/ambiguities
  - **CRITICAL**: Show `/sp.specify` within Claude Code context, NOT as standalone terminal command
  - Demo: Run `/sp.specify` on sample spec, explain output
  - Iteration: Address gaps, re-run `/sp.specify` to see improvement
  - Show cascade: Better spec → better feedback → clearer understanding
  - Activities: Students run `/sp.specify` on their grading system spec from Lesson 3, iterate 2-3 times
  - Assessment: Refined spec with documented gap resolution
  - End with: "Try With AI" activity (run `/sp.specify` on student's own spec, iterate)

- [ ] T026 [P] [US4] Create code examples for Lesson 4 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-4-specify/`
  - Example 1: Running `/sp.specify` within Claude Code (screenshot/code demo)
  - Example 2: Sample output showing gaps identified
  - Example 3: Refined spec after addressing gaps
  - Example 4: Comparison showing cascade effect (spec improved → feedback changed)

- [ ] T027 [P] [US4] Create Lesson 4 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-4-exercises.md`
  - Exercise 1: Run `/sp.specify` on provided incomplete spec, document gaps
  - Exercise 2: Refine spec to address critical gaps, re-run `/sp.specify`
  - Exercise 3: Explain which gaps were most important and why

- [ ] T028 [US4] Create Lesson 4 assessment at `/specs/010-chapter-31-redesign/lesson-4-assessment.md`
  - Pass criteria: Student can run `/sp.specify`, interpret output, refine spec, articulate gap resolution
  - Rubric: Can identify gaps? Can prioritize (critical/important/nice-to-have)? Can refine spec?

- [ ] T029 [US4] Create "Try With AI" prompts for Lesson 4 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-4-try-with-ai.md`
  - Prompt 1: "Run `/sp.specify` on your grading system spec within Claude Code"
  - Prompt 2: "Analyze the feedback and identify critical vs. nice-to-have gaps"
  - Prompt 3: "Ask Claude: Why is this gap important?"

- [ ] T030 [US4] Create Claude Code setup guide at `/specs/010-chapter-31-redesign/claude-code-setup.md` explaining how students should initialize Claude Code, navigate to project, run `/sp.specify`, etc.

**Story Completion Gate**: Lesson 4 complete, `/sp.specify` workflow correct (within Claude Code), iteration pattern demonstrated

---

## Phase 7: User Story 5 — Student Plans & Decomposes with /sp.plan and /sp.tasks

### Context
Students use `/sp.plan` to generate implementation plans and `/sp.tasks` to decompose plans into atomic work units. This demonstrates the cascade effect: spec quality → plan quality → task quality.

**Independent Test**: Student can run `/sp.plan` to generate plan, understand phase structure, run `/sp.tasks` to decompose tasks, and trace lineage backwards from task to plan to spec.

- [ ] T031 [US5] Write Lesson 5 content: "Planning from Specification (/sp.plan)" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/05-planning-sp-plan.md`
  - Duration: 1.5 hours
  - Sections: Learning objectives, Key Concepts (7 max), Content Outline (Part A: Plan structure, Part B: Generation demo, Part C: Dependency analysis)
  - Explain: `/sp.plan` generates implementation plan from specification
  - **CRITICAL**: Show `/sp.plan` within Claude Code context, NOT standalone terminal
  - Demo: Run `/sp.plan`, examine output (phases, lessons, dependencies, risks)
  - Show cascade: Improved spec → better plan structure
  - Activities: Students run `/sp.plan` on their refined spec from Lesson 4
  - Assessment: Understand plan structure, identify dependencies, explain phase sequencing
  - End with: "Try With AI" activity (run `/sp.plan`, ask Claude to explain why plan flows from spec)

- [ ] T032 [P] [US5] Write Lesson 6 content: "Decomposing Plans into Tasks (/sp.tasks)" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/06-decomposing-tasks-sp-tasks.md`
  - Duration: 1.5 hours
  - Sections: Learning objectives, Key Concepts (7 max), Content Outline (Part A: Task structure, Part B: Decomposition demo, Part C: Lineage tracing)
  - Explain: `/sp.tasks` breaks plan into atomic work units (2-4 hour tasks)
  - **CRITICAL**: Show `/sp.tasks` within Claude Code context, NOT standalone
  - Demo: Run `/sp.tasks`, examine output (tasks with ID, description, criteria, dependencies)
  - Show full lineage: Spec requirement → Plan phase → Task unit → Code
  - Activities: Students run `/sp.tasks`, trace backwards to plan and spec
  - Assessment: Understand task decomposition, recognize dependencies, predict consequences of out-of-order execution
  - End with: "Try With AI" activity (run `/sp.tasks`, trace lineage backwards)

- [ ] T033 [P] [US5] Create code examples for Lessons 5-6 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-5-6-plan-tasks/`
  - Example 1: Running `/sp.plan` within Claude Code
  - Example 2: Sample plan output (phases, lessons, dependencies)
  - Example 3: Running `/sp.tasks` within Claude Code
  - Example 4: Sample tasks output with ID, criteria, dependencies
  - Example 5: Lineage diagram (Spec requirement → Plan phase → Task → Code)

- [ ] T034 [P] [US5] Create Lessons 5-6 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-5-6-exercises.md`
  - Exercise 1 (Lesson 5): Run `/sp.plan`, identify phases and explain sequencing
  - Exercise 2 (Lesson 5): Understand why Phase 1 must complete before Phase 2
  - Exercise 3 (Lesson 6): Run `/sp.tasks`, examine task structure
  - Exercise 4 (Lesson 6): Trace 3 tasks backwards to plan and spec
  - Exercise 5: Predict what breaks if you try tasks in wrong order

- [ ] T035 [US5] Create Lessons 5-6 assessment at `/specs/010-chapter-31-redesign/lessons-5-6-assessment.md`
  - Lesson 5: Pass criteria - Understand plan structure, identify dependencies, explain phase sequencing
  - Lesson 6: Pass criteria - Decompose plan into tasks, understand dependencies, trace lineage

- [ ] T036 [US5] Create "Try With AI" prompts for Lessons 5-6 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-5-6-try-with-ai.md`
  - Lesson 5: Run `/sp.plan`, ask Claude to explain phase dependencies
  - Lesson 6: Run `/sp.tasks`, trace lineage with Claude's help

- [ ] T037 [US5] Create task lineage reference at `/specs/010-chapter-31-redesign/task-lineage-reference.md` showing example of how Spec requirement maps to Plan phase to Task to Code

- [ ] T038 [US5] Create `/sp.tasks` and `/sp.plan` usage guide at `/specs/010-chapter-31-redesign/sp-plan-tasks-guide.md` explaining correct Claude Code usage, NOT standalone terminal

**Story Completion Gate**: Lessons 5-6 complete, `/sp.plan` and `/sp.tasks` workflow correct (within Claude Code), full lineage demonstrated

---

## Phase 8: Implementation & Validation with /sp.implement

### Context
Students use `/sp.implement` to orchestrate AI-driven code generation and learn validation as a core skill. This is the final lesson where students see the complete AIDD loop in action.

**Independent Test**: Student can run `/sp.implement`, generate code, validate against acceptance criteria, identify when criteria are met/unmet, and provide feedback for refinement.

- [ ] T039 [US4] Write Lesson 7 content: "Implementation & Validation (/sp.implement)" at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/07-implementation-sp-implement.md`
  - Duration: 2.5 hours
  - Sections: Learning objectives, Key Concepts (8 max), Content Outline (Part A: AIDD loop, Part B: Code generation, Part C: Validation mastery)
  - Explain: `/sp.implement` orchestrates code generation from tasks
  - **CRITICAL**: Show `/sp.implement` within Claude Code context
  - Explain AIDD cycle: Intent (spec) → Generation (code) → Validation (your job) → Feedback → Refinement
  - Demo: Run `/sp.implement` on sample task, examine generated code
  - Validation focus: Compare code against acceptance criteria (pass/fail checklist)
  - Show cascade complete: Spec quality → Plan quality → Task quality → Code quality
  - Activities: Students run `/sp.implement` on calculator or grading system tasks, validate each criterion
  - Assessment: Generate code, create validation checklist, identify when all criteria pass
  - End with: "Try With AI" activity (generate code, validate, ask Claude for explanation if criteria fail)

- [ ] T040 [P] [US4] Create code examples for Lesson 7 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/examples/lesson-7-implement/`
  - Example 1: Running `/sp.implement` within Claude Code
  - Example 2: Generated code sample (calculator function or grading algorithm)
  - Example 3: Validation checklist (acceptance criteria → pass/fail)
  - Example 4: Code that passes criteria vs. code that partially misses
  - Example 5: Feedback loop (code misses criterion → refinement request → improved code)

- [ ] T041 [P] [US4] Create Lesson 7 exercises at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/exercises/lesson-7-exercises.md`
  - Exercise 1: Run `/sp.implement`, examine generated code
  - Exercise 2: Create validation checklist from acceptance criteria
  - Exercise 3: Test code against each criterion (pass/fail)
  - Exercise 4: If code fails criterion, provide feedback and ask for refinement
  - Exercise 5: Recognize when failure is spec ambiguity vs. code error

- [ ] T042 [US4] Create Lesson 7 assessment at `/specs/010-chapter-31-redesign/lesson-7-assessment.md`
  - Pass criteria: Student generates code with `/sp.implement`, validates against acceptance criteria, identifies pass/fail, provides feedback
  - Rubric: Can create validation checklist? Can identify failures? Can distinguish spec ambiguity from code error?

- [ ] T043 [P] [US4] Create "Try With AI" prompts for Lesson 7 at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/ai-prompts/lesson-7-try-with-ai.md`
  - Prompt 1: Run `/sp.implement` to generate code
  - Prompt 2: Create validation checklist for acceptance criteria
  - Prompt 3: If code fails criterion, ask Claude why and request refinement

- [ ] T044 [US4] Create validation checklist template at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/validation-checklist-template.md`
  - Template showing how to map acceptance criteria to testable conditions
  - Examples: "Feedback is 50-200 words" → Test: Count words, pass/fail. "Response time < 1 second" → Test: Time execution, pass/fail.

**Story Completion Gate**: Lesson 7 complete, `/sp.implement` workflow correct (within Claude Code), validation mastery demonstrated

---

## Phase 9: Capstone Project

### Context
Students complete a full SpecifyPlus workflow end-to-end: Specify → Clarify → Plan → Tasks → Implement. This consolidates all 7 lessons into one integrated project.

**Options**: Python Calculator (simpler, 3h) OR Grading System (complex, 4h)

**Independent Test**: Student produces working code that passes all acceptance criteria, with complete specification, plan, tasks, and reflection document.

- [ ] T045 Write capstone project guide at `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/capstone-project-guide.md`
  - Option A: Python Calculator - Simple domain, clear scope
  - Option B: Grading System - Complex, real-world, builds on earlier examples
  - Full workflow: Spec → Clarify → Plan → Tasks → Implement → Validate → Reflect
  - Deliverables: spec.md, plan.md, tasks.md, working code with tests, reflection (500-1000 words)
  - Reflection prompts: What surprised you? What would you do differently? How did spec quality affect code?

- [ ] T046 [P] Create capstone rubric at `/specs/010-chapter-31-redesign/capstone-rubric.md`
  - Specification: Complete? Clear? Testable?
  - Plan: Logical phases? Clear dependencies?
  - Tasks: Atomic? Well-sequenced?
  - Code: Works? Passes acceptance criteria?
  - Reflection: Insightful? Demonstrates learning?
  - Scoring: Excellent/Good/Satisfactory/Needs Work for each category

**Story Completion Gate**: Capstone project complete with all deliverables and passing rubric

---

## Phase 10: Polish & Validation

- [ ] T047 [P] Run final validation check: Review all 7 lessons for consistency, proper "Try With AI" formatting (no "Key Takeaways" or "Next Steps"), correct SpecifyPlus command usage (all within Claude Code context, not terminal). Document any fixes needed.

- [ ] T048 [P] Technical review: Verify all code examples run correctly, all acceptance criteria are testable, all cascades are demonstrated. Fix any issues.

- [ ] T049 Create completion checklist at `/specs/010-chapter-31-redesign/chapter-completion-checklist.md` confirming:
  - [ ] All 7 lessons written (01-07 markdown files)
  - [ ] All "Try With AI" activities present (no "Key Takeaways" or "Next Steps")
  - [ ] All `/sp.*` commands shown within Claude Code context (not terminal)
  - [ ] All hallucinations eliminated
  - [ ] All code examples tested
  - [ ] All assessments defined
  - [ ] Capstone project ready
  - [ ] Constitutional alignment verified

**Gate**: All tasks complete, chapter ready for publication

---

## Task Dependency Graph

```
Phase 1 (Setup)
  ├─ T001, T002
  └─ GATE: Branch ready

Phase 2 (Foundational)
  ├─ T003-T008 (parallel)
  └─ GATE: Structure ready

Phase 3 (US1: SMART Criteria)
  ├─ T009-T013 (can be parallel)
  └─ GATE: Lesson 1 ready

Phase 4 (US2: Project Structure)
  ├─ T014-T018 (can be parallel)
  └─ GATE: Lesson 2 ready

Phase 5 (US3: Complete Specs)
  ├─ T019-T024 (can be parallel)
  └─ GATE: Lesson 3 ready

Phase 6 (US4: /sp.specify Refinement)
  ├─ T025-T030 (can be parallel)
  └─ GATE: Lesson 4 ready

Phase 7 (US5: /sp.plan & /sp.tasks)
  ├─ T031-T038 (can be parallel, Lesson 5 before Lesson 6)
  └─ GATE: Lessons 5-6 ready

Phase 8 (Implementation /sp.implement)
  ├─ T039-T044 (can be parallel)
  └─ GATE: Lesson 7 ready

Phase 9 (Capstone)
  ├─ T045-T046 (parallel)
  ├─ (Depends on: All lessons 1-7 complete)
  └─ GATE: Capstone ready

Phase 10 (Validation)
  ├─ T047-T049 (parallel)
  ├─ (Depends on: All content created)
  └─ GATE: Chapter ready for publication
```

---

## Parallel Execution Strategy

**Parallel Opportunities** (can execute simultaneously):

1. **Phase 2 (Foundational)**: T003-T008 are all independent (templates, guides, rubric, glossary)
2. **Phase 3 (SMART Lesson)**: T009-T013 can be parallel (content, examples, exercises, assessment, prompts)
3. **Phase 4-7**: Within each lesson, content (T0XX) and examples (T0XX+1) can be parallel, but exercises follow content
4. **Capstone**: T045-T046 (guide and rubric) can be parallel

**Recommended Execution Order** (sequential for quality):
1. Phase 1 (Setup) - 30 min
2. Phase 2 (Foundational) - 1 hour (parallel)
3. Phase 3-8 (Lessons 1-7) - 2-3 hours each (sequential, can parallelize within phase)
4. Phase 9 (Capstone) - 1 hour
5. Phase 10 (Validation) - 1 hour

**Total Estimated Time**: 16-20 hours

---

## Implementation Strategy

### MVP (Minimum Viable Product)
**Scope**: Lessons 1-3 complete + simple capstone (calculator)
- Focus: Manual thinking skills (SMART criteria, project structure, spec writing)
- Time: 8-10 hours
- Rationale: Proves spec-first thinking before introducing tools

### Phase 2: Tool Integration
**Scope**: Lessons 4-7
- Focus: SpecifyPlus tools (`/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`)
- Time: 8-10 hours
- Prerequisite: MVP lessons complete

### Phase 3: Capstone & Validation
**Scope**: Capstone projects + full validation
- Options: Simple (calculator) or complex (grading system)
- Time: 3-5 hours
- Prerequisite: All 7 lessons complete

### Rollout Strategy
1. **Week 1**: MVP (Lessons 1-3) + simple capstone
2. **Week 2**: Tool integration (Lessons 4-7)
3. **Week 3**: Complex capstone + validation
4. **Week 4**: Publication & feedback loop

---

## Files to Create (Summary)

### Lesson Content Files (7 files)
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/01-smart-acceptance-criteria.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/02-specifyplus-structure.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/03-complete-specification.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/04-refining-specs-specify.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/05-planning-sp-plan.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/06-decomposing-tasks-sp-tasks.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/07-implementation-sp-implement.md`

### Support Files (7+ files)
- README.md (chapter overview)
- Capstone project guide
- All exercises (7 files)
- All assessment checklists (7 files)
- All "Try With AI" prompts (7 files)
- Code examples (per lesson)
- Reference documents (glossary, spec template reference, lineage, validation template, etc.)

### Spec/Plan Support Files (in `/specs/010-chapter-31-redesign/`)
- implementation-guide.md
- assessment-rubric.md
- glossary.md
- lesson-1-assessment.md through lesson-7-assessment.md
- capstone-rubric.md
- claude-code-setup.md
- sp-plan-tasks-guide.md
- task-lineage-reference.md
- chapter-completion-checklist.md

---

## Success Criteria

### Chapter Completion
✅ All 7 lessons written and validated
✅ All "Try With AI" activities present (ChatGPT web for Lessons 1-3, CLI tools for Lessons 4-7)
✅ All `/sp.*` commands shown within Claude Code context (NOT terminal)
✅ All hallucinations eliminated
✅ All code examples tested and working
✅ Cascade effect demonstrated at every level
✅ AIDD paradigm reflected throughout
✅ Capstone project ready and rubric-validated
✅ Constitutional alignment verified
✅ Ready for publication

---

