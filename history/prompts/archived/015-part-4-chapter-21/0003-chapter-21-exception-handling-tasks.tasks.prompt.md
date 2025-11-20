---
id: 0003
title: Chapter 21 Exception Handling Tasks Generation
stage: tasks
date: 2025-11-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 015-part-4-chapter-21
branch: 015-part-4-chapter-21
user: mjs
command: /sp.tasks 015-part-4-chapter-21
labels: ["python-chapter", "exception-handling", "task-generation", "lesson-implementation", "intermediate"]
links:
  spec: /specs/015-part-4-chapter-21/spec.md
  plan: /specs/015-part-4-chapter-21/plan.md
  tasks: /specs/015-part-4-chapter-21/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/015-part-4-chapter-21/tasks.md (591 lines)
tests:
  - Task format validation (all 119 tasks follow checkbox + ID + description format): PASS
  - Lesson organization (tasks grouped by lesson L1-L5): PASS
  - Sequential dependencies validated (lessons build on prior concepts): PASS
  - Parallel opportunities identified (74/119 tasks = 62%): PASS
  - Independent test criteria (each lesson has acceptance test): PASS
---

## Prompt

Generate comprehensive task checklist (tasks.md) from approved spec.md and plan.md for Chapter 21: Exception Handling.

Input documents:
- spec.md: 3 user stories (P1: Fundamentals, P2: Raising Exceptions, P3: Capstone)
- plan.md: 5 lessons with CEFR levels, cognitive load validation, concept distribution

Requirements:
1. Organize tasks by lesson (map user stories → lessons)
2. All tasks follow format: `- [ ] [ID] [P?] [Lesson] Description with file path`
3. Sequential dependencies enforced (Lesson N builds on Lessons 1-N-1)
4. Identify parallel opportunities within lessons
5. Each lesson has independent test criteria
6. Include implementation checklist for content-implementer
7. Validate lesson boundaries (no forward references)
8. Include policy notes (lesson closure pattern, no SDD terminology in Part 4)

Output: Immediately executable tasks.md with 119 atomic, testable tasks across 7 phases.

## Response snapshot

✅ **PHASE 3 COMPLETE: Task Checklist Generated**

**Tasks File**: `/specs/015-part-4-chapter-21/tasks.md` (591 lines)

**Task Distribution**:
| Phase | Tasks | Parallelizable | Sequential | Focus |
|-------|-------|---------------|-----------|-------|
| 1: Setup | 4 | 2 | 2 | Chapter infrastructure |
| 2: Lesson 1 (A2) | 21 | 15 | 6 | Exception fundamentals |
| 3: Lesson 2 (A2-B1) | 20 | 14 | 6 | Multi-block handling |
| 4: Lesson 3 (B1) | 19 | 13 | 6 | Raising/custom exceptions |
| 5: Lesson 4 (B1) | 20 | 14 | 6 | Error handling strategies |
| 6: Lesson 5 (B1) | 22 | 4 | 18 | Capstone CSV parser |
| 7: Polish | 13 | 12 | 1 | Validation & publication |
| **TOTAL** | **119** | **74 (62%)** | **45** | **5 lessons + polish** |

**Lesson Mapping** (User Stories → Lessons):
- User Story 1 (P1 - Fundamentals) → Lessons 1-2 (A2 → A2-B1)
- User Story 2 (P2 - Raising Exceptions) → Lessons 3-4 (B1)
- User Story 3 (P3 - Capstone) → Lesson 5 (B1 integration)

**Dependencies Validated**:
- Strict sequential: Phase 1 → 2 → 3 → 4 → 5 → 6 → 7
- Within-lesson parallelization: Code examples, exercises, CoLearning elements
- Lesson boundaries enforced: Lesson N uses only concepts 1-N (no forward references)

**Independent Test Criteria** (Each Lesson):
- Lesson 1: Student writes try/except catching 2+ exception types
- Lesson 2: Student writes code with try/except/else/finally, predicts control flow
- Lesson 3: Student writes function raising custom exception + caller catching it
- Lesson 4: Student applies appropriate error handling strategy to scenario
- Lesson 5: Parser handles 4+ error types, provides helpful feedback, reports summary

**Policy Notes Included**:
- Lesson closure pattern: "Try With AI" ONLY (no summaries/checklists after)
- No SDD terminology in Part 4 (use "describe intent" not "write specification")
- AI tool references: ChatGPT (Part 1) → Claude Code/Gemini CLI (Part 2+)
- Zero forward references to Chapters 22+ (chapter scope only)

**Format Validation**:
✅ All 119 tasks follow strict checklist format: `- [ ] [ID] [P?] [Lesson] Description`
✅ All tasks include file paths (book-source/docs/04-Part-4-Python-Fundamentals/21-exception-handling/)
✅ All parallelizable tasks marked with [P]
✅ All lesson tasks marked with [L1], [L2], [L3], [L4], or [L5]
✅ Setup and Polish tasks have no lesson label (correct)

## Outcome

- ✅ Impact: Tasks.md provides immediately executable implementation plan. Lesson-writer can execute with high confidence. Every task is atomic and testable.
- 🧪 Tests: 5 independent test criteria (one per lesson). All tasks mapped to lesson acceptance criteria. No ambiguity in validation.
- 📁 Files: tasks.md created (591 lines, 119 tasks). PHR recorded (0003-...).
- 🔁 Next prompts: /sp.implement (invoke content-implementer with tasks.md checklist), /sp.analyze (validate spec ↔ plan ↔ tasks consistency)
- 🧠 Reflection: Task generation demonstrates complete pedagogical planning. 62% parallelization opportunity (74/119 tasks). Sequential dependencies clear (lessons build on prior concepts). MVP scope defined (Lesson 1 only). Incremental delivery strategy: 5 sprints (one per lesson) + polish. Tasks ready for content-implementer subagent execution.

## Evaluation notes (flywheel)

- Failure modes observed: None. Task generation followed spec user stories and plan lesson structure faithfully.
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checkbox + ID + description). Lesson organization PASS (tasks grouped by lesson). Dependencies validated PASS (sequential progression). Independent test criteria PASS (each lesson has acceptance test).
- Prompt variant (if applicable): N/A - standard /sp.tasks flow adapted for educational content (lessons instead of features)
- Next experiment (smallest change to try): No changes needed. Tasks complete and ready for implementation. Next: /sp.implement to invoke content-implementer subagent with this checklist.
