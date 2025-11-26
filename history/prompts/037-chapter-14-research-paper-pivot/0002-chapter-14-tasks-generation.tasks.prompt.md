---
id: "0002"
title: "Chapter 14 Tasks Generation"
stage: tasks
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 037-chapter-14-research-paper-pivot
branch: 037-chapter-14-research-paper-pivot
user: human
command: /sp.tasks
labels: [chapter-14, spec-kit-plus, tasks, lesson-rewrite]
links:
  spec: specs/037-chapter-14-research-paper-pivot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/037-chapter-14-research-paper-pivot/tasks.md
tests:
  - format validation: PASS (all tasks follow checklist format)
---

## Prompt

/sp.tasks - Generate implementation tasks for Chapter 14 Spec-Kit Plus Hands-On rewrite.

Context: spec.md and plan.md define 12 lessons teaching Spec-Kit Plus workflow. Each lesson focuses on ONE command/concept. Research paper is practice vehicle only.

User Stories:
- US1 (P1): Master Spec-Kit Plus Workflow (lessons 01-08)
- US2 (P1): Create Reusable Intelligence (lessons 09-10)
- US3 (P2): Experience Intelligence Acceleration (lesson 11)

## Response snapshot

Generated tasks.md with 33 tasks organized by user story:

**Phase 1 (Setup)**: 3 tasks - backup, review, document
**Phase 2 (Foundational)**: 1 task - README update
**Phase 3 (US1)**: 8 tasks - rewrite lessons 01-08 with Spec-Kit Plus focus
**Phase 4 (US2)**: 2 tasks - rewrite lessons 09-10 for skill creation
**Phase 5 (US3)**: 1 task - rewrite capstone for acceleration proof
**Phase 6 (Assessment)**: 1 task - rewrite quiz
**Phase 7 (Summaries)**: 11 tasks - update all summary files
**Phase 8 (Validation)**: 6 tasks - quality checks

Key constraints enforced in each task:
- Command is the lesson, paper is just practice
- Remove all video/MCP/Playwright references
- End with "Try With AI" section only
- Max 7 concepts per lesson (A2-B1)

## Outcome

- Impact: Task breakdown enables parallel implementation of lesson rewrites
- Tests: Format validation PASS
- Files: tasks.md (33 tasks, 8 phases)
- Next prompts: /sp.implement 037-chapter-14-research-paper-pivot
- Reflection: Breaking by user story shows clear MVP path (US1 = lessons 01-08)

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS
- Prompt variant (if applicable): N/A
- Next experiment: Proceed to implementation phase
