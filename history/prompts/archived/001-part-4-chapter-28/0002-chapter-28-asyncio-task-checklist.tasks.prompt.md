---
id: 0002
title: Chapter 28 Asyncio Task Checklist
stage: tasks
date: 2025-11-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-part-4-chapter-28
branch: 001-part-4-chapter-28
user: mjs
command: /sp.tasks
labels: ["chapter-28", "asyncio", "task-generation", "lesson-implementation", "user-story-mapping"]
links:
  spec: specs/001-part-4-chapter-28/spec.md
  plan: specs/001-part-4-chapter-28/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-part-4-chapter-28/tasks.md (created, 381 lines, 23KB)
tests:
  - Task format validation (all tasks follow - [ ] [ID] [P?] [Story] pattern)
  - User story mapping (6 lessons → 6 user stories)
  - Dependencies validated (sequential: US4→US1, US5→US2+US4, US6→US1-5)
  - Parallel opportunities identified (45 tasks marked [P])
---

## Prompt

User requested continuation of /sp.python-chapter workflow for Chapter 28: Asyncio after Phase 2 (planning) completed.

**Command**: /sp.tasks

**Context**:
- Approved specification (533 lines, 4 user stories P1/P2)
- Completed lesson plan (1,035 lines, 6 lessons with CEFR B1-B2 mapping)
- 6 lessons: Foundations → Concurrent Tasks → Advanced Patterns → CPU-Bound Work → Hybrid Workloads → Capstone
- 29 code examples across all lessons
- User stories map to lessons (US1=Lesson 1, US2=Lesson 2, etc.)

**Task**: Generate actionable task checklist organized by user story/lesson for implementation.

**Requirements**:
- Map 4 spec user stories to 6 lesson implementation phases
- Create tasks following strict checklist format: - [ ] [ID] [P?] [Story] Description with file path
- Organize by phase: Setup → Foundation → US1-6 → Polish
- Mark parallelizable tasks with [P]
- Include all 29 code examples as implementation tasks
- Validate lesson closure pattern ("Try With AI" ONLY)
- Include compliance checks (Python 3.14+, type hints, CoLearning elements)

## Response snapshot

Successfully generated comprehensive task checklist:

**File**: specs/001-part-4-chapter-28/tasks.md (381 lines, 23KB)

**Task Organization**:
- **Phase 1 (Setup)**: 6 tasks - chapter directory structure, dependencies, prerequisites
- **Phase 2 (Foundation)**: 5 tasks - shared resources (deprecation list, validation script, benchmark utility, pedagogy guidelines)
- **Phase 3 (US1 - Lesson 1)**: 17 tasks - Asyncio Foundations (7 concepts, 5 code examples, CoLearning elements)
- **Phase 4 (US2 - Lesson 2)**: 19 tasks - Concurrent Tasks (7 concepts, 6 code examples)
- **Phase 5 (US3 - Lesson 3)**: 19 tasks - Advanced Patterns (9 concepts, 6 code examples)
- **Phase 6 (US4 - Lesson 4)**: 19 tasks - CPU-Bound Work & GIL (8 concepts, 6 code examples)
- **Phase 7 (US5 - Lesson 5)**: 19 tasks - Hybrid Workloads (8 concepts, 6 code examples)
- **Phase 8 (US6 - Lesson 6)**: 20 tasks - Capstone AI Agent (integration, all patterns)
- **Phase 9 (Polish)**: 12 tasks - Cross-cutting concerns, validation, chapter completion

**Total**: 136 tasks

**Key Features**:
- All tasks follow strict format: `- [ ] T### [P?] [Story?] Description with file/path`
- 45 tasks marked [P] for parallel execution (code examples, independent writing)
- Dependencies documented: US4→US1, US5→US2+US4, US6→US1-5
- MVP scope identified: Lessons 1-2 (foundational + concurrent tasks)
- Incremental delivery: Week 1 (MVP), Week 2 (Lessons 3-4), Week 3 (Lessons 5-6)
- Validation checklist per lesson (YAML, type hints, compliance, CoLearning, closure pattern)
- Policy note for lesson authors (closure pattern, AI tool usage)

**Parallel Execution Examples**:
- Within Lesson 2: 6 code examples can be created in parallel (T033, T035, T037, T038, T039, T040)
- Across lessons: Lessons 1-3 can be written in parallel after Foundation phase

**Code Examples Breakdown**:
- Lesson 1: 5 examples (T016-T022)
- Lesson 2: 6 examples (T033, T035, T037-T040)
- Lesson 3: 6 examples (T052-T061, skip T053)
- Lesson 4: 6 examples (T073-T080)
- Lesson 5: 6 examples (T090-T099)
- Lesson 6: 2 examples (skeleton + solution) (T111, T116)
- **Total**: 29 code examples (all Python 3.14+, full type hints)

## Outcome

- ✅ Impact: Complete actionable task checklist ready for lesson-by-lesson implementation
- 🧪 Tests: Format validation (all 136 tasks correct), dependency validation (sequential order correct), parallel opportunities identified
- 📁 Files: tasks.md (381 lines, 23KB, 136 tasks across 9 phases)
- 🔁 Next prompts: /sp.implement to begin lesson implementation (or /sp.analyze for cross-artifact validation)
- 🧠 Reflection: Task generation successfully mapped spec user stories → lesson plan → actionable checklist; all constitutional requirements encoded in tasks (Python 3.14+ compliance, CoLearning pedagogy, lesson closure pattern)

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation completed successfully with correct format
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checklist format), Dependency validation PASS (sequential order correct)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.implement for content-implementer agent execution
