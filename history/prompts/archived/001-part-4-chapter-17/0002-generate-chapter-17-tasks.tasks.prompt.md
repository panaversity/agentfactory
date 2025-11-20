---
id: 0002
title: generate-chapter-17-tasks
stage: tasks
date: 2025-11-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-part-4-chapter-17
branch: 001-part-4-chapter-17
user: system
command: /sp.tasks
labels: ["python-fundamentals", "control-flow", "loops", "tasks", "part-4"]
links:
  spec: specs/001-part-4-chapter-17/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-part-4-chapter-17/tasks.md
tests:
  - None (task planning phase - no code tests)
---

## Prompt

User invoked `/sp.tasks` command after approving the detailed lesson plan (plan.md).

**Context**:
- Feature: 001-part-4-chapter-17 (Chapter 17: Control Flow and Loops)
- Branch: 001-part-4-chapter-17
- Input documents: spec.md (351 lines), plan.md (408 lines)
- User approval: "proceed" (after reviewing plan.md completion)

**Workflow executed**:
1. Ran check-prerequisites.sh to get feature directory
2. Loaded spec.md (13 success evals, 5 lesson outlines, acceptance criteria)
3. Loaded plan.md (detailed lesson-by-lesson implementation plan with CEFR proficiency mapping)
4. Loaded tasks-template.md to understand structure
5. Generated tasks.md organized by lesson (treating each lesson as a "user story")

**Task generation approach**:
- Organized tasks by lesson (L1-L5) for independent implementation
- 98 total tasks across 8 phases (Setup, Foundational, 5 Lessons, Polish)
- Each lesson has 15-16 implementation tasks following consistent structure
- Parallel opportunities identified (11 tasks marked [P])
- MVP scope defined (Lesson 1 only = 24 tasks)

## Response snapshot

**Tasks file created successfully**: specs/001-part-4-chapter-17/tasks.md (387 lines)

**Task organization**:
- **Phase 1**: Setup (4 tasks) - chapter directory, README, prerequisites verification, Python 3.14 environment
- **Phase 2**: Foundational (5 tasks) - shared standards (Try With AI format, CoLearning elements, lesson closure, Graduated Teaching Pattern, code testing checklist)
- **Phase 3**: Lesson 1 - Making Decisions with Conditionals (15 tasks, Priority P1, MVP)
- **Phase 4**: Lesson 2 - Pattern Matching with match-case (15 tasks, Priority P2)
- **Phase 5**: Lesson 3 - Repetition with Loops (15 tasks, Priority P3)
- **Phase 6**: Lesson 4 - Controlling Loops (15 tasks, Priority P4)
- **Phase 7**: Lesson 5 - Nested Control Structures (16 tasks, Priority P5)
- **Phase 8**: Polish & Cross-Chapter Validation (13 tasks)

**Key task structure for each lesson**:
1. Create lesson file
2. Write YAML frontmatter
3. Write content sections (review, main content, AI Companion sections)
4. Write Red Flags section
5. Write "Try With AI" section (4 prompts, Bloom's progression)
6. Test all code examples on Python 3.14
7. Validate lesson closure (ONLY "Try With AI")
8. Validate cognitive load (CEFR limits)
9. Validate reading level (Grade 7-9)

**Parallel opportunities**:
- Lessons 2 and 3 can be written in parallel (after Lesson 1 complete)
- All setup tasks marked [P]
- All foundational tasks marked [P]
- All polish tasks marked [P]

**MVP scope**: Lesson 1 only (24 tasks total: 4 setup + 5 foundational + 15 lesson 1 implementation)

## Outcome

- ✅ Impact: Complete task breakdown (98 tasks) for implementing Chapter 17 with clear lesson-by-lesson structure, parallel opportunities, and MVP scope defined
- 🧪 Tests: No code tests (task planning phase) - code example testing tasks included within lesson implementation tasks
- 📁 Files: specs/001-part-4-chapter-17/tasks.md (387 lines)
- 🔁 Next prompts: Human approval of tasks.md → then `/sp.implement` to execute lesson writing (likely starting with MVP: Lesson 1 only)
- 🧠 Reflection: Tasks organized by lesson (not by type) enables independent lesson implementation and validation. Each lesson follows identical 15-task structure (create file → write sections → test code → validate closure/cognitive load/reading level). Foundational phase critical (defines shared standards before any lesson writing begins). MVP scope (Lesson 1 = 24 tasks) allows fast validation of teaching approach before full chapter commitment.

## Evaluation notes (flywheel)

- Failure modes observed: None (tasks.md generated successfully on first attempt)
- Graders run and results (PASS/FAIL): N/A (task planning phase - validation occurs during /sp.implement)
- Prompt variant (if applicable): N/A (standard /sp.tasks workflow)
- Next experiment (smallest change to try): Execute MVP scope (Lesson 1 only) via `/sp.implement` to validate: (1) task granularity appropriate for LLM execution, (2) lesson structure meets constitution requirements, (3) code examples test successfully on Python 3.14
